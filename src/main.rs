#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::{join, join5};
use embassy_rp::bind_interrupts;
use embassy_rp::dma;
use embassy_rp::peripherals;
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_usb::driver::{Endpoint, EndpointError, EndpointIn, EndpointOut};
use embassy_usb::Builder;
use panic_probe as _;
use static_cell::StaticCell;

mod audio;
mod i2s;

// USB / PIO / DMA の割り込みを Embassy の型付きハンドラへ結び付ける。
bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<peripherals::USB>;
    PIO0_IRQ_0 => PioInterruptHandler<peripherals::PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<peripherals::DMA_CH0>;
});

// 1ms ごとの最大 USB パケットを 32 個ぶん貯められる深さをソフト FIFO に確保する。
const AUDIO_FIFO_CAPACITY_WORDS: usize = audio::MAX_I2S_PACKET_WORDS * 32;

static AUDIO_FIFO: Mutex<CriticalSectionRawMutex, AudioSampleFifo<AUDIO_FIFO_CAPACITY_WORDS>> =
    Mutex::new(AudioSampleFifo::new());
static AUDIO_HANDLER: StaticCell<audio::UsbAudioClass> = StaticCell::new();
static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static BOS_DESCRIPTOR: StaticCell<[u8; 64]> = StaticCell::new();
static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

// USB 受信と I2S 送信の間に挟む単純なリングバッファ。
// オーバーフロー時は古いサンプルを捨て、アンダーフロー時は呼び出し側で無音を補う。
struct AudioSampleFifo<const N: usize> {
    data: [u32; N],
    read: usize,
    len: usize,
}

impl<const N: usize> AudioSampleFifo<N> {
    const fn new() -> Self {
        Self {
            data: [0; N],
            read: 0,
            len: 0,
        }
    }

    fn clear(&mut self) {
        self.read = 0;
        self.len = 0;
    }

    fn push_slice(&mut self, words: &[u32]) {
        // 一度に容量以上が来た場合は最新の N ワードだけを残す。
        if words.len() >= N {
            self.clear();
            for &word in &words[words.len() - N..] {
                self.write_word(word);
            }
            return;
        }

        let missing_space = words.len().saturating_sub(N - self.len);
        if missing_space != 0 {
            self.discard_oldest(missing_space);
        }

        for &word in words {
            self.write_word(word);
        }
    }

    fn pop_slice(&mut self, out: &mut [u32]) -> usize {
        let count = out.len().min(self.len);
        for slot in out.iter_mut().take(count) {
            *slot = self.read_word();
        }
        count
    }

    fn discard_oldest(&mut self, count: usize) {
        let discard = count.min(self.len);
        self.read = (self.read + discard) % N;
        self.len -= discard;
    }

    fn len(&self) -> usize {
        self.len
    }

    fn write_word(&mut self, word: u32) {
        let write = (self.read + self.len) % N;
        self.data[write] = word;
        self.len += 1;
    }

    fn read_word(&mut self) -> u32 {
        let word = self.data[self.read];
        self.read = (self.read + 1) % N;
        self.len -= 1;
        word
    }
}

// USB の 24-bit packed little-endian PCM を、I2S 32-bit 左詰めスロットへ変換する。
fn pcm24_to_i2s_slot(sample_bytes: &[u8]) -> u32 {
    let sign = if (sample_bytes[2] & 0x80) != 0 {
        0xff
    } else {
        0x00
    };
    let sample = i32::from_le_bytes([sample_bytes[0], sample_bytes[1], sample_bytes[2], sign]);
    (sample << 8) as u32
}

// USB で受けた PCM フレーム列を、PIO へそのまま送れる I2S ワード列へ展開する。
fn bytes_to_i2s_words(bytes: &[u8], bits_per_sample: u8, out: &mut [u32]) -> usize {
    match bits_per_sample {
        audio::BITS_PER_SAMPLE_16 => {
            let mut count = 0;
            for frame_bytes in bytes.chunks_exact(4) {
                if count + 1 >= out.len() {
                    break;
                }

                out[count] = (u16::from_le_bytes([frame_bytes[0], frame_bytes[1]]) as u32) << 16;
                out[count + 1] =
                    (u16::from_le_bytes([frame_bytes[2], frame_bytes[3]]) as u32) << 16;
                count += 2;
            }
            count
        }
        audio::BITS_PER_SAMPLE_24 => {
            let mut count = 0;
            for frame_bytes in bytes.chunks_exact(6) {
                if count + 1 >= out.len() {
                    break;
                }

                out[count] = pcm24_to_i2s_slot(&frame_bytes[..3]);
                out[count + 1] = pcm24_to_i2s_slot(&frame_bytes[3..6]);
                count += 2;
            }
            count
        }
        _ => 0,
    }
}

#[embassy_executor::main(
    executor = "embassy_rp::executor::Executor",
    entry = "cortex_m_rt::entry"
)]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("boot");

    // USB デバイスと PIO ベースの I2S 送信器を初期化する。
    let driver = Driver::new(p.USB, Irqs);
    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);

    // UAC2 デバイスとしてホストへ見せる基本情報を設定する。
    let mut usb_config = embassy_usb::Config::new(0x1209, 0x2350);
    usb_config.manufacturer = Some("hirata-naoto");
    usb_config.product = Some("XIAO RP2350 USB Audio to I2S");
    usb_config.serial_number = Some("0001");
    usb_config.device_class = 0x00;
    usb_config.device_sub_class = 0x00;
    usb_config.device_protocol = 0x00;
    usb_config.composite_with_iads = false;
    usb_config.max_power = 100;
    usb_config.max_packet_size_0 = 64;

    let mut builder = Builder::new(
        driver,
        usb_config,
        CONFIG_DESCRIPTOR.init([0; 256]),
        BOS_DESCRIPTOR.init([0; 64]),
        &mut [],
        CONTROL_BUF.init([0; 64]),
    );

    let (
        audio_handler,
        mut stream_endpoint_16,
        mut feedback_endpoint_16,
        mut stream_endpoint_24,
        mut feedback_endpoint_24,
    ) = audio::UsbAudioClass::new(&mut builder);
    let audio_handler = AUDIO_HANDLER.init(audio_handler);
    builder.handler(audio_handler);

    let mut usb = builder.build();
    let mut i2s = i2s::I2sPioTx::new(
        &mut common,
        sm0,
        p.DMA_CH0,
        Irqs,
        p.PIN_26,
        p.PIN_27,
        p.PIN_28,
    );
    let mut silence = [0u32; audio::MAX_I2S_PACKET_WORDS];
    // 起動直後は現在設定のサンプルレートで I2S を回し、初期フィードバック値を合わせる。
    let mut i2s_timing = i2s.configure(audio::current_sample_rate_hz());
    audio::reset_feedback_control(i2s_timing.feedback_value_10_14);
    info!(
        "i2s init nominal={}Hz actual={}Hz bits={} feedback={}",
        audio::current_sample_rate_hz(),
        i2s_timing.actual_sample_rate_hz,
        audio::current_bits_per_sample(),
        i2s_timing.feedback_value_10_14
    );
    let initial_packet_words = audio::current_i2s_packet_words();
    i2s.prime(&silence[..initial_packet_words]);
    i2s.start();

    let usb_fut = usb.run();
    let receive_16_fut = async {
        let mut packet = [0u8; audio::USB_PACKET_SIZE_16];
        let mut words = [0u32; audio::MAX_I2S_PACKET_WORDS];

        loop {
            // Alternate Setting 1 が有効になるまで待ち、切り替え時に FIFO を空にする。
            stream_endpoint_16.wait_enabled().await;
            info!("stream16 enabled");
            {
                let mut fifo = AUDIO_FIFO.lock().await;
                fifo.clear();
            }

            loop {
                match stream_endpoint_16.read(&mut packet).await {
                    Ok(received) => {
                        // USB パケットを I2S ワードへ展開して共有 FIFO へ積む。
                        let word_count = bytes_to_i2s_words(
                            &packet[..received],
                            audio::BITS_PER_SAMPLE_16,
                            &mut words,
                        );
                        let mut fifo = AUDIO_FIFO.lock().await;
                        fifo.push_slice(&words[..word_count]);
                    }
                    Err(EndpointError::Disabled) => {
                        info!("stream16 disabled");
                        let mut fifo = AUDIO_FIFO.lock().await;
                        fifo.clear();
                        break;
                    }
                    // 等時転送では取りこぼしより継続動作を優先する。
                    Err(EndpointError::BufferOverflow) => {}
                }
            }
        }
    };
    let receive_24_fut = async {
        let mut packet = [0u8; audio::USB_PACKET_SIZE_24];
        let mut words = [0u32; audio::MAX_I2S_PACKET_WORDS];

        loop {
            // Alternate Setting 2 が有効になったら 24-bit packed PCM の受信を始める。
            stream_endpoint_24.wait_enabled().await;
            info!("stream24 enabled");
            {
                let mut fifo = AUDIO_FIFO.lock().await;
                fifo.clear();
            }

            loop {
                match stream_endpoint_24.read(&mut packet).await {
                    Ok(received) => {
                        let word_count = bytes_to_i2s_words(
                            &packet[..received],
                            audio::BITS_PER_SAMPLE_24,
                            &mut words,
                        );
                        let mut fifo = AUDIO_FIFO.lock().await;
                        fifo.push_slice(&words[..word_count]);
                    }
                    Err(EndpointError::Disabled) => {
                        info!("stream24 disabled");
                        let mut fifo = AUDIO_FIFO.lock().await;
                        fifo.clear();
                        break;
                    }
                    Err(EndpointError::BufferOverflow) => {}
                }
            }
        }
    };
    let playback_fut = async {
        let mut chunk = [0u32; audio::MAX_I2S_PACKET_WORDS];
        let mut config_version = audio::stream_config_version();
        let mut playback_started = false;
        let mut buffering = false;
        let mut diag_frames = 0u16;

        loop {
            let next_config_version = audio::stream_config_version();
            if next_config_version != config_version {
                // サンプルレートやビット幅が変わったら FIFO と I2S タイミングを同期し直す。
                config_version = next_config_version;
                let packet_words = audio::current_i2s_packet_words();
                let start_level_words = audio::feedback_start_level_words(packet_words);
                let target_level_words = audio::feedback_target_level_words(packet_words);
                info!(
                    "stream config changed nominal={}Hz bits={} i2s_words={} start={} target={}",
                    audio::current_sample_rate_hz(),
                    audio::current_bits_per_sample(),
                    packet_words,
                    start_level_words,
                    target_level_words
                );
                {
                    let mut fifo = AUDIO_FIFO.lock().await;
                    fifo.clear();
                }
                i2s_timing = i2s.configure(audio::current_sample_rate_hz());
                audio::reset_feedback_control(i2s_timing.feedback_value_10_14);
                info!(
                    "i2s timing actual={}Hz feedback={}",
                    i2s_timing.actual_sample_rate_hz, i2s_timing.feedback_value_10_14
                );
                silence[..packet_words].fill(0);
                i2s.prime(&silence[..packet_words]);
                i2s.start();
                playback_started = false;
                buffering = false;
                diag_frames = 0;
            }

            let packet_words = audio::current_i2s_packet_words();
            let start_level_words = audio::feedback_start_level_words(packet_words);
            let target_level_words = audio::feedback_target_level_words(packet_words);
            let active = audio::STREAM_ACTIVE.load(core::sync::atomic::Ordering::Relaxed);
            let mut started_this_cycle = false;
            let (fifo_level_words, written) = {
                let mut fifo = AUDIO_FIFO.lock().await;
                if active {
                    // 再生開始前は十分にバッファがたまるまで待ち、開始後は即座に取り出す。
                    let fifo_level_words = fifo.len();
                    let should_start = playback_started || fifo_level_words >= start_level_words;
                    let written = if should_start {
                        if !playback_started {
                            started_this_cycle = true;
                            playback_started = true;
                        }
                        fifo.pop_slice(&mut chunk[..packet_words])
                    } else {
                        0
                    };
                    (fifo_level_words, written)
                } else {
                    fifo.clear();
                    playback_started = false;
                    (0, 0)
                }
            };

            if active {
                // FIFO 水位に応じて明示的フィードバック値を微調整し、長期的なずれを吸収する。
                audio::update_feedback_control(
                    fifo_level_words,
                    packet_words,
                    i2s_timing.feedback_value_10_14,
                );

                if started_this_cycle {
                    buffering = false;
                    info!(
                        "playback start fifo={} start={} target={}",
                        fifo_level_words, start_level_words, target_level_words
                    );
                } else if !playback_started {
                    if !buffering {
                        info!(
                            "buffering fifo={} start={} target={}",
                            fifo_level_words, start_level_words, target_level_words
                        );
                        buffering = true;
                    }
                } else {
                    buffering = false;
                }

                diag_frames = diag_frames.saturating_add(1);
                if diag_frames >= 1_000 {
                    diag_frames = 0;
                    info!(
                        "playback diag fifo={} target={} feedback={} correction={}",
                        fifo_level_words,
                        target_level_words,
                        audio::current_feedback_value_10_14(),
                        audio::current_feedback_correction_10_14()
                    );
                }
            } else {
                // ストリーム停止中は診断状態を片付け、基準フィードバック値へ戻しておく。
                if buffering || diag_frames != 0 {
                    buffering = false;
                    diag_frames = 0;
                }
                audio::reset_feedback_control(i2s_timing.feedback_value_10_14);
            }

            // 足りないぶんは無音で埋め、I2S クロックは止めずに流し続ける。
            chunk[written..packet_words].fill(0);
            i2s.write_words(&chunk[..packet_words]).await;
        }
    };
    let feedback_16_fut = async {
        loop {
            // 16-bit ストリーム用の明示的フィードバックを 1ms 周期で返し続ける。
            feedback_endpoint_16.wait_enabled().await;

            loop {
                let feedback_packet = audio::current_feedback_packet();
                match feedback_endpoint_16.write(&feedback_packet).await {
                    Ok(()) => {}
                    Err(EndpointError::Disabled) => break,
                    Err(EndpointError::BufferOverflow) => {}
                }
            }
        }
    };
    let feedback_24_fut = async {
        loop {
            // 24-bit ストリーム側も同じ制御値を別エンドポイントから返す。
            feedback_endpoint_24.wait_enabled().await;

            loop {
                let feedback_packet = audio::current_feedback_packet();
                match feedback_endpoint_24.write(&feedback_packet).await {
                    Ok(()) => {}
                    Err(EndpointError::Disabled) => break,
                    Err(EndpointError::BufferOverflow) => {}
                }
            }
        }
    };

    join(
        usb_fut,
        join5(
            receive_16_fut,
            receive_24_fut,
            playback_fut,
            feedback_16_fut,
            feedback_24_fut,
        ),
    )
    .await;
    loop {}
}
