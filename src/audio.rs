//! USB Full-Speed の再生専用 USB Audio Class 2.0 インターフェイスと共有制御状態。
//!
//! 対応形式は左右 2 チャネルの PCM Format I、16-bit（2 byte/sample）または
//! packed 24-bit（3 byte/sample）、44.1 / 48 / 88.2 / 96 kHz。既定値は 16-bit / 48 kHz。
//! AudioControl の信号経路は USB Streaming Input Terminal → Feature Unit →
//! Speaker Output Terminal で、両 Terminal が同じ内部可変 Clock Source を参照する。
//! Feature Unit は互換性のために列挙するが、音量・ミュート制御は公開しない。
//! AudioStreaming の Alt 0 は帯域を使わない停止状態、Alt 1 は 16-bit、Alt 2 は 24-bit。
//! 各有効 Alt は独立した非同期等時 OUT と明示的フィードバック IN を持ち、
//! 1 ms の最大 OUT 容量は 96 kHz を基準に 384 / 576 byte として確保する。
//!
//! 現実装のフィードバック転送は 4 byte で、下位 3 byte にリトルエンディアンの
//! 10.14 固定小数点値（ステレオフレーム数/ms）、第 4 byte に 0 を格納する。
//! これは 16.16 形式ではない。I2S 実クロック由来の基準値を呼び出し元から受け取り、
//! FIFO のワード水位に応じた上限付き比例補正と段階的な追従でホストの送出量を調整する。
//!
//! 制御要求は AudioControl の Clock Source、マスターチャネルを対象とする。
//! 周波数 SET_CUR は先頭 4 byte の Hz 値を検証し、GET_CUR は現在の Hz 値を返す。
//! GET_RANGE は対応する 4 レートを離散範囲として返し、Clock Validity GET_CUR は
//! 常に有効を返す。対象外要求は委譲（None）または拒否し、短いバッファは拒否する。
//!
//! 再生有効状態、周波数、ビット幅、設定世代、基準/補正済みフィードバック値を
//! Relaxed なアトミック変数で共有する。各値の読み書きは不可分だが、複数値の
//! 一括スナップショットや他のメモリ操作との同期は保証しない。世代番号は周波数・
//! ビット幅の変更を通知し、再生有効状態だけの変更では増えない。
//! 本モジュールはディスクリプタ生成・エンドポイント確保・要求処理・補正計算を担う。
//! PCM の受信/変換、FIFO 管理、I2S/DMA の再設定、フィードバックの実送信は呼び出し元が担う。
//! USB リセットは停止・既定形式へ戻すが、補正値のリセットは別途呼び出し元が行う。

use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

use embassy_usb::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::descriptor::{SynchronizationType, UsageType};
use embassy_usb::driver::{Driver, Endpoint, EndpointIn, EndpointOut};
use embassy_usb::types::InterfaceNumber;
use embassy_usb::{Builder, Handler};

// 起動時と無効な設定からの復帰時に用いる既定サンプルレート（Hz）。
pub const SAMPLE_RATE_HZ: u32 = 48_000;
// PCM のチャネル数。1 ステレオフレームは左右 2 サンプルからなる。
pub const CHANNEL_COUNT: usize = 2;
// Alt 1 で公開する 1 チャネル当たりの有効ビット数。
pub const BITS_PER_SAMPLE_16: u8 = 16;
// Alt 2 で公開する 1 チャネル当たりの有効ビット数（USB では 3 byte に詰める）。
pub const BITS_PER_SAMPLE_24: u8 = 24;
// 16-bit / 96 kHz の 1 ms 等時 OUT 最大サイズ（384 byte）。
pub const USB_PACKET_SIZE_16: usize = usb_packet_size(BITS_PER_SAMPLE_16, 96_000);
// 24-bit / 96 kHz の 1 ms 等時 OUT 最大サイズ（576 byte）。
pub const USB_PACKET_SIZE_24: usize = usb_packet_size(BITS_PER_SAMPLE_24, 96_000);
// 最大レートの USB 1 パケットを I2S へ展開したサイズ（32-bit ワード 192 個）。
pub const MAX_I2S_PACKET_WORDS: usize = i2s_words_per_usb_packet(BITS_PER_SAMPLE_24, 96_000);

// Alternate Setting 1 / 2 の有効化状態を保持し、再生開始/停止を追跡する。
pub static STREAM_ACTIVE: AtomicBool = AtomicBool::new(false);
// USB 制御要求で選択されたサンプルレート（Hz）をタスク間で共有する。
static CURRENT_SAMPLE_RATE_HZ: AtomicU32 = AtomicU32::new(SAMPLE_RATE_HZ);
// 現在の 1 チャネル当たりのビット幅。停止中も最後の形式を保持する。
static CURRENT_BITS_PER_SAMPLE: AtomicU8 = AtomicU8::new(BITS_PER_SAMPLE_16);
// ストリーム設定の世代番号。再設定が必要なときに main 側が検出する。
static STREAM_CONFIG_VERSION: AtomicU32 = AtomicU32::new(0);
// I2S 実クロック由来の補正前基準値（フレーム/ms の 10.14）。初期値は公称 48 kHz。
static BASE_FEEDBACK_VALUE_10_14: AtomicU32 = AtomicU32::new(feedback_value_10_14(SAMPLE_RATE_HZ));
// ホストへ返す補正済みの値（フレーム/ms の 10.14）。初期値は公称 48 kHz。
static CURRENT_FEEDBACK_VALUE_10_14: AtomicU32 =
    AtomicU32::new(feedback_value_10_14(SAMPLE_RATE_HZ));

// USB の Audio インターフェイスクラスコード。
const USB_CLASS_AUDIO: u8 = 0x01;
// AudioControl インターフェイスのサブクラスコード。
const USB_SUBCLASS_AUDIO_CONTROL: u8 = 0x01;
// AudioStreaming インターフェイスのサブクラスコード。
const USB_SUBCLASS_AUDIO_STREAMING: u8 = 0x02;
// Audio Class 2.0 を示すインターフェイスプロトコルコード。
const USB_PROTOCOL_IP_02_00: u8 = 0x20;

// クラス固有インターフェイスディスクリプタの種別。
const CS_INTERFACE: u8 = 0x24;
// クラス固有エンドポイントディスクリプタの種別。
const CS_ENDPOINT: u8 = 0x25;

// AudioControl ヘッダーディスクリプタのサブタイプ。
const AC_HEADER: u8 = 0x01;
// AudioControl Input Terminal ディスクリプタのサブタイプ。
const AC_INPUT_TERM: u8 = 0x02;
// AudioControl Output Terminal ディスクリプタのサブタイプ。
const AC_OUTPUT_TERM: u8 = 0x03;
// AudioControl Feature Unit ディスクリプタのサブタイプ。
const AC_FEATURE_UNIT: u8 = 0x06;
// AudioControl Clock Source ディスクリプタのサブタイプ。
const AC_CLOCK_SOURCE: u8 = 0x0A;

// AudioStreaming General ディスクリプタのサブタイプ。
const AS_GENERAL: u8 = 0x01;
// AudioStreaming Format Type ディスクリプタのサブタイプ。
const AS_FORMAT_TYPE: u8 = 0x02;
// AudioStreaming クラス固有エンドポイントディスクリプタの General サブタイプ。
const EP_GENERAL: u8 = 0x01;

// 両 Terminal が参照し、クロック制御要求の宛先になるエンティティ ID。
const CLOCK_SOURCE_ID: u8 = 0x10;
// USB PCM 入力 Terminal のエンティティ ID。Streaming 側からも参照する。
const INPUT_TERM_ID: u8 = 0x11;
// 入出力 Terminal 間の Feature Unit のエンティティ ID。
const FEATURE_UNIT_ID: u8 = 0x12;
// スピーカー出力 Terminal のエンティティ ID。
const OUTPUT_TERM_ID: u8 = 0x13;

// Audio Function の用途をデスクトップスピーカーとして通知するカテゴリ。
const FUNCTION_CATEGORY_DESKTOP_SPEAKER: u8 = 0x01;
// USB ストリーミングを表す Terminal タイプコード。
const TERM_USB_STREAMING: u16 = 0x0101;
// スピーカーを表す Terminal タイプコード。
const TERM_SPEAKER: u16 = 0x0301;
// 前方左・前方右チャネルの配置ビットマップ。
const CHANNEL_CONFIG_FL_FR: u32 = 0x0000_0003;
// Format I の対応形式ビットマップにおける PCM ビット。
const PCM_FORMAT_I: u32 = 0x0000_0001;
// フィードバックディスクリプタの bRefresh 値。転送間隔 bInterval とは別のフィールド。
const FEEDBACK_REFRESH_PERIOD: u8 = 1;
// フィードバック IN の最大パケット長（byte）。第 4 byte は 0 として送る。
const FEEDBACK_PACKET_SIZE: u16 = 4;
// 1 回の制御更新で許す値の最大変化量（10.14 の最下位単位）。
const FEEDBACK_SMOOTHING_STEP_10_14: i32 = 16;
// 基準値に対する目標補正の絶対値上限（10.14 の最下位単位）。
const FEEDBACK_MAX_CORRECTION_10_14: i32 = 512;
// FIFO 誤差を 1 パケット分のフレーム数で正規化した比例ゲイン（10.14 単位）。
const FEEDBACK_ERROR_GAIN_10_14: i32 = 128;
// FIFO の目標蓄積量を最大 USB パケット換算で表した個数。
const FIFO_TARGET_PACKETS: usize = 12;
// 再生開始に必要な蓄積量を最大 USB パケット換算で表した個数。
const FIFO_START_PACKETS: usize = 6;
// 目標からこのパケット数以内の誤差では目標補正を 0 とする不感帯。
const FIFO_CONTROL_DEADBAND_PACKETS: usize = 1;

// UAC2 の現在値要求コード。転送方向により GET_CUR / SET_CUR を区別する。
const UAC2_CUR: u8 = 0x01;
// UAC2 の対応範囲取得要求コード。
const UAC2_GET_RANGE: u8 = 0x02;
// Clock Source のサンプル周波数制御セレクター。
const CLOCK_FREQUENCY_CONTROL_SELECTOR: u8 = 0x01;
// Clock Source の有効性制御セレクター。
const CLOCK_VALIDITY_CONTROL_SELECTOR: u8 = 0x02;
// 内部プログラマブルクロックを宣言する bmAttributes 値。
const CLOCK_SOURCE_ATTRIBUTES_INTERNAL_PROGRAMMABLE: u8 = 0x03;
// 周波数は読み書き可能、有効性は読み取り専用とする bmControls 値。
const CLOCK_SOURCE_CONTROLS_HOST_PROGRAMMABLE_FREQUENCY_RW_VALIDITY_RO: u8 = 0x07;
// Clock Source に特定の関連 Terminal を指定しない値。
const CLOCK_SOURCE_ASSOCIATION_TERMINAL_ID: u8 = 0x00;
// チャネル別ではなくクロック全体を指定する要求のチャネル番号。
const CLOCK_CONTROL_MASTER_CHANNEL: u8 = 0x00;
// Clock Validity GET_CUR で常に返す有効フラグ。
const CLOCK_VALIDITY_TRUE: u8 = 1;
// 周波数制御値の転送長（リトルエンディアン u32、4 byte）。
const CLOCK_FREQUENCY_BYTES: usize = core::mem::size_of::<u32>();
// クロック有効性の転送長（u8、1 byte）。
const CLOCK_VALIDITY_BYTES: usize = core::mem::size_of::<u8>();
// 16-bit / 24-bit の両形式で受け付け、GET_RANGE でも公開する周波数（Hz）。
const SUPPORTED_SAMPLE_RATES_HZ: [u32; 4] = [44_100, 48_000, 88_200, 96_000];

// USB 要求の振り分けに必要なインターフェイス番号を保持するハンドラー。
// エンドポイント自体は保持せず、生成時に呼び出し元へ返す。
pub struct UsbAudioClass {
    // Clock Source のクラス制御要求を受ける AudioControl インターフェイス番号。
    ac_interface: InterfaceNumber,
    // Alt Setting の変更で再生状態を更新する AudioStreaming インターフェイス番号。
    streaming_interface: InterfaceNumber,
}

// USB Audio の bSubslotSize を求めるため、ビット数から必要 byte 数へ丸める。
// 1 チャネル分の幅を入力し、端数を切り上げた byte 数を返す（0 bit は 0 byte）。
const fn bytes_per_sample(bits_per_sample: u8) -> usize {
    (bits_per_sample as usize).div_ceil(8)
}

// この実装では 16/24-bit どちらも I2S 32-bit スロットを 1 ワードで扱う。
// 引数のビット幅は検証せず、常に 1 チャネル当たりのワード数 1 を返す。
const fn i2s_words_per_sample(bits_per_sample: u8) -> usize {
    let _ = bits_per_sample;
    1
}

// Full-Speed 等時 OUT は 1ms ごとの最大転送量で wMaxPacketSize を決める。
// ビット幅と Hz から、フレーム数を切り上げたステレオ PCM の byte 数を返す。
const fn usb_packet_size(bits_per_sample: u8, sample_rate_hz: u32) -> usize {
    sample_rate_hz.div_ceil(1_000) as usize * CHANNEL_COUNT * bytes_per_sample(bits_per_sample)
}

// USB 1 パケットぶんが I2S 側で何ワードになるかを事前に計算しておく。
// ビット幅と Hz を受け取り、1 ms のフレーム数を切り上げた 32-bit ワード数を返す。
// 形式の対応可否は検証せず、0 Hz の場合は 0 を返す。
pub const fn i2s_words_per_usb_packet(bits_per_sample: u8, sample_rate_hz: u32) -> usize {
    sample_rate_hz.div_ceil(1_000) as usize * CHANNEL_COUNT * i2s_words_per_sample(bits_per_sample)
}

// 対応レート（Hz）をフレーム/ms の 10.14 値に変換する。整数除算の端数は切り捨てる。
const fn feedback_value_10_14(sample_rate_hz: u32) -> u32 {
    // Full-Speed の明示的フィードバックで使う 10.14 固定小数点へ変換する。
    (sample_rate_hz << 14) / 1_000
}

// Hz 単位の入力が公開する 4 種類の離散レートのいずれかなら true を返す。
fn supports_sample_rate(sample_rate_hz: u32) -> bool {
    SUPPORTED_SAMPLE_RATES_HZ.contains(&sample_rate_hz)
}

// Alt Setting ごとのビット幅とクロック設定の組み合わせが許容範囲か判定する。
// 1 チャネルのビット数と Hz を入力し、16/24-bit 以外または非対応レートなら false。
pub fn supports_stream_format(bits_per_sample: u8, sample_rate_hz: u32) -> bool {
    match bits_per_sample {
        BITS_PER_SAMPLE_16 => supports_sample_rate(sample_rate_hz),
        BITS_PER_SAMPLE_24 => supports_sample_rate(sample_rate_hz),
        _ => false,
    }
}

// 選択中の公称サンプルレート（Hz）を Relaxed 読み出しで返す。実クロック測定値ではない。
pub fn current_sample_rate_hz() -> u32 {
    CURRENT_SAMPLE_RATE_HZ.load(Ordering::Relaxed)
}

// 停止状態を問わず、保持中の 1 チャネル当たりビット数を Relaxed 読み出しで返す。
pub fn current_bits_per_sample() -> u8 {
    CURRENT_BITS_PER_SAMPLE.load(Ordering::Relaxed)
}

// 補正済み 10.14 値の下位 3 byte をリトルエンディアンで詰め、第 4 byte を 0 にする。
// 4 byte の送信用配列を返すだけで、エンドポイントへの書き込みは行わない。
pub fn current_feedback_packet() -> [u8; 4] {
    let bytes = current_feedback_value_10_14().to_le_bytes();
    [bytes[0], bytes[1], bytes[2], 0]
}

// 現在の形式・Hz を個別に読み、USB 1 ms 分の最大 I2S ワード数を返す。
// 設定変更と競合した場合、ビット幅と周波数の一括スナップショットにはならない。
pub fn current_i2s_packet_words() -> usize {
    i2s_words_per_usb_packet(current_bits_per_sample(), current_sample_rate_hz())
}

// FIFO は「何パケットぶん貯めたいか」で管理し、サンプルレート変更時も追従させる。
// 1 パケットの I2S ワード数から目標ワード数を返す。積が溢れる場合は usize::MAX。
pub fn feedback_target_level_words(packet_words: usize) -> usize {
    packet_words.saturating_mul(FIFO_TARGET_PACKETS)
}

// 1 パケットの I2S ワード数から再生開始しきい値を返す。積は usize::MAX に飽和する。
pub fn feedback_start_level_words(packet_words: usize) -> usize {
    packet_words.saturating_mul(FIFO_START_PACKETS)
}

// 設定変更検出用の世代番号を Relaxed 読み出しで返す。周回するため差の大小ではなく変化を見る。
pub fn stream_config_version() -> u32 {
    STREAM_CONFIG_VERSION.load(Ordering::Relaxed)
}

// 現在の送信候補値をフレーム/ms の 10.14 固定小数点で返す。状態は変更しない。
pub fn current_feedback_value_10_14() -> u32 {
    CURRENT_FEEDBACK_VALUE_10_14.load(Ordering::Relaxed)
}

// 送信候補値と基準値の差を符号付き 10.14 単位で返す（正なら送出増加方向）。
// 2 つの共有値は個別に読み出すため、同時更新中の差は一時的な値になり得る。
pub fn current_feedback_correction_10_14() -> i32 {
    current_feedback_value_10_14() as i32 - BASE_FEEDBACK_VALUE_10_14.load(Ordering::Relaxed) as i32
}

// サンプルレート切り替え直後や停止時に、補正の積み残しを消して基準値へ戻す。
// 入力はフレーム/ms の 10.14 値。基準値と送信候補を同値にし、設定世代は変更しない。
pub fn reset_feedback_control(base_feedback_value_10_14: u32) {
    BASE_FEEDBACK_VALUE_10_14.store(base_feedback_value_10_14, Ordering::Relaxed);
    CURRENT_FEEDBACK_VALUE_10_14.store(base_feedback_value_10_14, Ordering::Relaxed);
}

// FIFO 水位と 1 パケットの I2S ワード数、実クロック由来の 10.14 基準値から
// 次の送信候補を計算・保存して返す。ワード数と基準値は本機の通常動作範囲を前提とする。
// 不感帯内では目標補正を 0、帯域外では比例補正を ±512 に制限し、1 回に最大 16 だけ追従。
// packet_words が 0 でも除数は最低 1 フレームとし、目標フィードバックは負にしない。
// 基準値/候補の共有状態のみ更新し、実際の USB 送信や FIFO 操作は行わない。
pub fn update_feedback_control(
    fifo_level_words: usize,
    packet_words: usize,
    base_feedback_value_10_14: u32,
) -> u32 {
    // I2S 実クロックから得た基準値を毎回更新し、FIFO 水位ぶんだけ上下に補正する。
    BASE_FEEDBACK_VALUE_10_14.store(base_feedback_value_10_14, Ordering::Relaxed);

    let packet_frames = (packet_words / CHANNEL_COUNT).max(1) as i32;
    let target_words = feedback_target_level_words(packet_words);
    let deadband_words = packet_words.saturating_mul(FIFO_CONTROL_DEADBAND_PACKETS);
    let fifo_error_words = target_words as i32 - fifo_level_words as i32;

    // 目標水位より低ければホスト送出を少し速く、高ければ少し遅く誘導する。
    let target_correction = if fifo_error_words.unsigned_abs() as usize <= deadband_words {
        0
    } else {
        (fifo_error_words / CHANNEL_COUNT as i32) * FEEDBACK_ERROR_GAIN_10_14 / packet_frames
    }
    .clamp(
        -FEEDBACK_MAX_CORRECTION_10_14,
        FEEDBACK_MAX_CORRECTION_10_14,
    );

    let target_feedback = (base_feedback_value_10_14 as i32 + target_correction).max(0) as u32;
    let current_feedback = CURRENT_FEEDBACK_VALUE_10_14.load(Ordering::Relaxed);
    // 急激に値を変えるとホスト側の追従が不安定になるため、1 ステップずつ近づける。
    let next_feedback = if current_feedback < target_feedback {
        current_feedback.saturating_add(
            (target_feedback - current_feedback).min(FEEDBACK_SMOOTHING_STEP_10_14 as u32),
        )
    } else {
        current_feedback.saturating_sub(
            (current_feedback - target_feedback).min(FEEDBACK_SMOOTHING_STEP_10_14 as u32),
        )
    };

    CURRENT_FEEDBACK_VALUE_10_14.store(next_feedback, Ordering::Relaxed);
    next_feedback
}

// 制御要求から受け取ったサンプルレートを共有状態へ反映する。
// Hz 値を無検証で交換し、以前と異なる場合に true を返す。世代更新は呼び出し元の責務。
fn update_sample_rate(sample_rate_hz: u32) -> bool {
    let previous = CURRENT_SAMPLE_RATE_HZ.swap(sample_rate_hz, Ordering::Relaxed);
    previous != sample_rate_hz
}

// Alt Setting 切り替えに応じて 16-bit / 24-bit のどちらかを記録する。
// ビット数を無検証で交換し、変更の有無を返す。再生状態や世代番号は変更しない。
fn update_stream_format(bits_per_sample: u8) -> bool {
    let previous = CURRENT_BITS_PER_SAMPLE.swap(bits_per_sample, Ordering::Relaxed);
    previous != bits_per_sample
}

// main 側の再設定処理を起こすために世代番号をインクリメントする。
// Relaxed の原子的加算を用い、u32 の上限を越えると 0 に周回する。
fn note_stream_config_change() {
    STREAM_CONFIG_VERSION.fetch_add(1, Ordering::Relaxed);
}

impl UsbAudioClass {
    // Builder に UAC2 のディスクリプタと 2 形式分の等時エンドポイントを追加する。
    // 戻り値はハンドラー、16-bit OUT/feedback IN、24-bit OUT/feedback IN の順。
    // 共有状態の初期化やハンドラー登録、転送開始は行わず、確保処理の失敗は Builder 側に従う。
    pub fn new<'d, D: Driver<'d>>(
        builder: &mut Builder<'d, D>,
    ) -> (
        Self,
        D::EndpointOut,
        D::EndpointIn,
        D::EndpointOut,
        D::EndpointIn,
    )
    where
        D::EndpointOut: EndpointOut,
        D::EndpointIn: EndpointIn,
    {
        // 1 つの Audio Function の中に AudioControl と AudioStreaming を構成する。
        let mut func = builder.function(USB_CLASS_AUDIO, 0x00, USB_PROTOCOL_IP_02_00);

        let mut ac_interface = func.interface();
        let ac_interface_number = ac_interface.interface_number();
        let mut ac_alt = ac_interface.alt_setting(
            USB_CLASS_AUDIO,
            USB_SUBCLASS_AUDIO_CONTROL,
            USB_PROTOCOL_IP_02_00,
            None,
        );

        // AC ヘッダーから Output Terminal までのクラス固有ディスクリプタの合計長（byte）。
        const AC_TOTAL_LENGTH: u16 = 64;
        // AudioControl 側では Windows 互換性のため Feature Unit を含む最小構成で公開する。
        ac_alt.descriptor(
            CS_INTERFACE,
            &[
                AC_HEADER,
                0x00,
                0x02,
                FUNCTION_CATEGORY_DESKTOP_SPEAKER,
                (AC_TOTAL_LENGTH & 0xff) as u8,
                (AC_TOTAL_LENGTH >> 8) as u8,
                0x00,
            ],
        );
        ac_alt.descriptor(
            CS_INTERFACE,
            &[
                AC_CLOCK_SOURCE,
                CLOCK_SOURCE_ID,
                CLOCK_SOURCE_ATTRIBUTES_INTERNAL_PROGRAMMABLE,
                CLOCK_SOURCE_CONTROLS_HOST_PROGRAMMABLE_FREQUENCY_RW_VALIDITY_RO,
                CLOCK_SOURCE_ASSOCIATION_TERMINAL_ID,
                0x00,
            ],
        );
        ac_alt.descriptor(
            CS_INTERFACE,
            &[
                AC_INPUT_TERM,
                INPUT_TERM_ID,
                (TERM_USB_STREAMING & 0xff) as u8,
                (TERM_USB_STREAMING >> 8) as u8,
                0x00,
                CLOCK_SOURCE_ID,
                CHANNEL_COUNT as u8,
                (CHANNEL_CONFIG_FL_FR & 0xff) as u8,
                ((CHANNEL_CONFIG_FL_FR >> 8) & 0xff) as u8,
                ((CHANNEL_CONFIG_FL_FR >> 16) & 0xff) as u8,
                ((CHANNEL_CONFIG_FL_FR >> 24) & 0xff) as u8,
                0x00,
                0x00,
                0x00,
                0x00,
            ],
        );
        ac_alt.descriptor(
            CS_INTERFACE,
            &[
                AC_FEATURE_UNIT,
                FEATURE_UNIT_ID,
                INPUT_TERM_ID,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
            ],
        );
        ac_alt.descriptor(
            CS_INTERFACE,
            &[
                AC_OUTPUT_TERM,
                OUTPUT_TERM_ID,
                (TERM_SPEAKER & 0xff) as u8,
                (TERM_SPEAKER >> 8) as u8,
                0x00,
                FEATURE_UNIT_ID,
                CLOCK_SOURCE_ID,
                0x00,
                0x00,
                0x00,
            ],
        );

        let mut as_interface = func.interface();
        let as_interface_number = as_interface.interface_number();
        // Alt 0 は帯域未使用、Alt 1/2 で各形式の等時 OUT エンドポイントを有効化する。
        let _ = as_interface.alt_setting(
            USB_CLASS_AUDIO,
            USB_SUBCLASS_AUDIO_STREAMING,
            USB_PROTOCOL_IP_02_00,
            None,
        );

        let mut as_alt_16 = as_interface.alt_setting(
            USB_CLASS_AUDIO,
            USB_SUBCLASS_AUDIO_STREAMING,
            USB_PROTOCOL_IP_02_00,
            None,
        );
        // Alt 1 は 16-bit 用。96 kHz でも FS の 1023 byte 制限内に収まる。
        as_alt_16.descriptor(
            CS_INTERFACE,
            &[
                AS_GENERAL,
                INPUT_TERM_ID,
                0x00,
                0x01,
                (PCM_FORMAT_I & 0xff) as u8,
                ((PCM_FORMAT_I >> 8) & 0xff) as u8,
                ((PCM_FORMAT_I >> 16) & 0xff) as u8,
                ((PCM_FORMAT_I >> 24) & 0xff) as u8,
                CHANNEL_COUNT as u8,
                (CHANNEL_CONFIG_FL_FR & 0xff) as u8,
                ((CHANNEL_CONFIG_FL_FR >> 8) & 0xff) as u8,
                ((CHANNEL_CONFIG_FL_FR >> 16) & 0xff) as u8,
                ((CHANNEL_CONFIG_FL_FR >> 24) & 0xff) as u8,
                0x00,
            ],
        );
        as_alt_16.descriptor(
            CS_INTERFACE,
            &[
                AS_FORMAT_TYPE,
                0x01,
                bytes_per_sample(BITS_PER_SAMPLE_16) as u8,
                BITS_PER_SAMPLE_16,
            ],
        );

        let stream_endpoint_16 = as_alt_16.alloc_endpoint_out(
            embassy_usb_driver::EndpointType::Isochronous,
            None,
            USB_PACKET_SIZE_16 as u16,
            1,
        );
        // OUT と feedback を Alt 1/2 で分けることで 16-bit と 24-bit を独立して列挙する。
        let feedback_endpoint_16 = as_alt_16.alloc_endpoint_in(
            embassy_usb_driver::EndpointType::Isochronous,
            None,
            FEEDBACK_PACKET_SIZE,
            1,
        );
        // ストリーム OUT 側へ同期先のフィードバックエンドポイント番号を関連付ける。
        as_alt_16.endpoint_descriptor(
            stream_endpoint_16.info(),
            SynchronizationType::Asynchronous,
            UsageType::DataEndpoint,
            &[0x00, feedback_endpoint_16.info().addr.into()],
        );
        as_alt_16.descriptor(CS_ENDPOINT, &[EP_GENERAL, 0x00, 0x00, 0x00, 0x00, 0x00]);
        // Full-Speed の 10.14 フィードバック値を 4 byte で返す（4th byte は 0）。
        as_alt_16.endpoint_descriptor(
            feedback_endpoint_16.info(),
            SynchronizationType::NoSynchronization,
            UsageType::FeedbackEndpoint,
            &[FEEDBACK_REFRESH_PERIOD, 0x00],
        );

        let mut as_alt_24 = as_interface.alt_setting(
            USB_CLASS_AUDIO,
            USB_SUBCLASS_AUDIO_STREAMING,
            USB_PROTOCOL_IP_02_00,
            None,
        );
        // Alt 2 は 24-bit packed PCM 用。96 kHz でも 576 byte/frame で FS 制限内に収まる。
        as_alt_24.descriptor(
            CS_INTERFACE,
            &[
                AS_GENERAL,
                INPUT_TERM_ID,
                0x00,
                0x01,
                (PCM_FORMAT_I & 0xff) as u8,
                ((PCM_FORMAT_I >> 8) & 0xff) as u8,
                ((PCM_FORMAT_I >> 16) & 0xff) as u8,
                ((PCM_FORMAT_I >> 24) & 0xff) as u8,
                CHANNEL_COUNT as u8,
                (CHANNEL_CONFIG_FL_FR & 0xff) as u8,
                ((CHANNEL_CONFIG_FL_FR >> 8) & 0xff) as u8,
                ((CHANNEL_CONFIG_FL_FR >> 16) & 0xff) as u8,
                ((CHANNEL_CONFIG_FL_FR >> 24) & 0xff) as u8,
                0x00,
            ],
        );
        as_alt_24.descriptor(
            CS_INTERFACE,
            &[
                AS_FORMAT_TYPE,
                0x01,
                bytes_per_sample(BITS_PER_SAMPLE_24) as u8,
                BITS_PER_SAMPLE_24,
            ],
        );

        let stream_endpoint_24 = as_alt_24.alloc_endpoint_out(
            embassy_usb_driver::EndpointType::Isochronous,
            None,
            USB_PACKET_SIZE_24 as u16,
            1,
        );
        let feedback_endpoint_24 = as_alt_24.alloc_endpoint_in(
            embassy_usb_driver::EndpointType::Isochronous,
            None,
            FEEDBACK_PACKET_SIZE,
            1,
        );
        // 24-bit 側も 16-bit と同じく非同期 OUT + 明示的フィードバック構成にする。
        as_alt_24.endpoint_descriptor(
            stream_endpoint_24.info(),
            SynchronizationType::Asynchronous,
            UsageType::DataEndpoint,
            &[0x00, feedback_endpoint_24.info().addr.into()],
        );
        as_alt_24.descriptor(CS_ENDPOINT, &[EP_GENERAL, 0x00, 0x00, 0x00, 0x00, 0x00]);
        as_alt_24.endpoint_descriptor(
            feedback_endpoint_24.info(),
            SynchronizationType::NoSynchronization,
            UsageType::FeedbackEndpoint,
            &[FEEDBACK_REFRESH_PERIOD, 0x00],
        );

        (
            Self {
                ac_interface: ac_interface_number,
                streaming_interface: as_interface_number,
            },
            stream_endpoint_16,
            feedback_endpoint_16,
            stream_endpoint_24,
            feedback_endpoint_24,
        )
    }
}

impl Handler for UsbAudioClass {
    // Class/Interface 宛て OUT 要求を処理し、対象外インターフェイス等は None で委譲する。
    // 本 AC 宛てでは Clock Source のマスター周波数 SET_CUR だけを受け付ける。
    // buf の先頭 4 byte をリトルエンディアン Hz として読み、余剰 byte は無視する。
    // 短いデータ・未対応形式/要求は拒否し、受理した値が変わった場合だけ設定世代を進める。
    fn control_out(&mut self, req: Request, buf: &[u8]) -> Option<OutResponse> {
        // ホストからの SET_CUR は AudioControl Interface の Clock Source だけを受け付ける。
        if req.request_type != RequestType::Class || req.recipient != Recipient::Interface {
            return None;
        }

        if (req.index as u8) != u8::from(self.ac_interface) {
            return None;
        }

        if ((req.index >> 8) as u8) != CLOCK_SOURCE_ID || req.request != UAC2_CUR {
            return Some(OutResponse::Rejected);
        }

        if (req.value as u8) != CLOCK_CONTROL_MASTER_CHANNEL
            || ((req.value >> 8) as u8) != CLOCK_FREQUENCY_CONTROL_SELECTOR
            || buf.len() < CLOCK_FREQUENCY_BYTES
        {
            return Some(OutResponse::Rejected);
        }

        let sample_rate_hz = u32::from_le_bytes(buf[..CLOCK_FREQUENCY_BYTES].try_into().unwrap());
        let bits_per_sample = current_bits_per_sample();
        // 現在有効な Alt Setting で扱えないレートは拒否する。
        if !supports_stream_format(bits_per_sample, sample_rate_hz) {
            return Some(OutResponse::Rejected);
        }

        if update_sample_rate(sample_rate_hz) {
            note_stream_config_change();
        }
        Some(OutResponse::Accepted)
    }

    // Class/Interface 宛て IN 要求に buf を使って応答する。対象外要求は None で委譲する。
    // Clock Source のマスター以外のチャネルと応答バッファ不足は拒否する。
    // 周波数 CUR は 4 byte の Hz、RANGE は件数 u16 と 4 組の min/max/res u32（計 50 byte）、
    // 有効性 CUR は 1 byte を返す。各整数はリトルエンディアンで、離散範囲の res は 0。
    // Accepted は書き込んだ部分のスライスを借用し、共有設定は変更しない。
    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        // GET_CUR / GET_RANGE も同じ Clock Source にだけ応答する。
        if req.request_type != RequestType::Class || req.recipient != Recipient::Interface {
            return None;
        }

        if (req.index as u8) != u8::from(self.ac_interface) {
            return None;
        }

        if ((req.index >> 8) as u8) != CLOCK_SOURCE_ID {
            return None;
        }

        let control_selector = (req.value >> 8) as u8;
        if (req.value as u8) != CLOCK_CONTROL_MASTER_CHANNEL {
            return Some(InResponse::Rejected);
        }

        match (control_selector, req.request) {
            (CLOCK_FREQUENCY_CONTROL_SELECTOR, UAC2_CUR) => {
                // ホストへ現在選択中のサンプルレートを返す。
                if buf.len() < CLOCK_FREQUENCY_BYTES {
                    return Some(InResponse::Rejected);
                }
                let bytes = current_sample_rate_hz().to_le_bytes();
                buf[..bytes.len()].copy_from_slice(&bytes);
                Some(InResponse::Accepted(&buf[..bytes.len()]))
            }
            (CLOCK_FREQUENCY_CONTROL_SELECTOR, UAC2_GET_RANGE) => {
                // 両ビット幅で使える 44.1/48/88.2/96 kHz を min=max の離散範囲として返す。
                let mut response = [0u8; 2 + SUPPORTED_SAMPLE_RATES_HZ.len() * 12];
                if buf.len() < response.len() {
                    return Some(InResponse::Rejected);
                }
                response[0..2]
                    .copy_from_slice(&(SUPPORTED_SAMPLE_RATES_HZ.len() as u16).to_le_bytes());
                for (index, sample_rate_hz) in SUPPORTED_SAMPLE_RATES_HZ.iter().enumerate() {
                    let offset = 2 + index * 12;
                    response[offset..offset + 4].copy_from_slice(&sample_rate_hz.to_le_bytes());
                    response[offset + 4..offset + 8].copy_from_slice(&sample_rate_hz.to_le_bytes());
                    response[offset + 8..offset + 12].copy_from_slice(&0u32.to_le_bytes());
                }
                buf[..response.len()].copy_from_slice(&response);
                Some(InResponse::Accepted(&buf[..response.len()]))
            }
            (CLOCK_VALIDITY_CONTROL_SELECTOR, UAC2_CUR) => {
                if buf.len() < CLOCK_VALIDITY_BYTES {
                    return Some(InResponse::Rejected);
                }
                buf[0] = CLOCK_VALIDITY_TRUE;
                Some(InResponse::Accepted(&buf[..CLOCK_VALIDITY_BYTES]))
            }
            _ => None,
        }
    }

    // 本 Streaming インターフェイスの Alt 通知だけを処理し、それ以外は無視する。
    // Alt 1/2 でビット幅と再生有効状態を更新し、非対応レートなら既定 Hz に戻す。
    // その他の Alt は停止扱いで形式を保持する。形式/Hz が変わった場合だけ世代を進め、
    // 有効状態のみの変更では進めない。フィードバック値やハードウェアは直接変更しない。
    fn set_alternate_setting(&mut self, iface: InterfaceNumber, alternate: u8) {
        if iface == self.streaming_interface {
            let mut changed = false;
            // Alt 0=停止、Alt 1=16-bit、Alt 2=24-bit として扱う。
            let active = matches!(alternate, 1 | 2);

            if active {
                let bits_per_sample = if alternate == 2 {
                    BITS_PER_SAMPLE_24
                } else {
                    BITS_PER_SAMPLE_16
                };
                changed |= update_stream_format(bits_per_sample);

                if !supports_stream_format(bits_per_sample, current_sample_rate_hz()) {
                    // 24-bit/16-bit の切り替え後に無効なレートが残っていたら既定値へ戻す。
                    changed |= update_sample_rate(SAMPLE_RATE_HZ);
                }
            }

            STREAM_ACTIVE.store(active, Ordering::Relaxed);
            if changed {
                note_stream_config_change();
            }
        }
    }

    // USB バスリセット通知で再生を停止し、16-bit / 48 kHz の既定形式を保存する。
    // ビット幅または Hz に変化があれば世代を進める。補正値や FIFO はここではリセットしない。
    fn reset(&mut self) {
        // USB バスリセット後は列挙直後の既定状態へ戻す。
        STREAM_ACTIVE.store(false, Ordering::Relaxed);
        let mut changed = false;
        changed |= update_stream_format(BITS_PER_SAMPLE_16);
        changed |= update_sample_rate(SAMPLE_RATE_HZ);
        if changed {
            note_stream_config_change();
        }
    }
}
