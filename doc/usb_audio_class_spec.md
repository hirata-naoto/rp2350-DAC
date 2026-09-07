# USB Audio Class 2.0 インターフェース仕様

## インターフェース構成

| # | インターフェース | Alt Setting | Class/Subclass/Protocol | 内容 |
|---|---|---|---|---|
| Function | Audio Function | - | 0x01 / 0x00 / 0x20 (IP_VERSION_02_00) | Desktop Speaker カテゴリ |
| IAD | Interface Association | IF0-1 | 0x01 / 0x00 / 0x20 | AudioControl + AudioStreaming を 1 つの Audio Function として束ねる |
| IF0 | AudioControl (AC) | Alt 0 | 0x01 / 0x01 / 0x20 | Clock Source, Input/Output Terminal |
| IF1 | AudioStreaming (AS) | Alt 0 | 0x01 / 0x02 / 0x20 | ゼロバンド幅（帯域未使用） |
| IF1 | AudioStreaming (AS) | Alt 1 | 0x01 / 0x02 / 0x20 | 16-bit PCM ストリーミング |
| IF1 | AudioStreaming (AS) | Alt 2 | 0x01 / 0x02 / 0x20 | 24-bit PCM ストリーミング |

デバイスディスクリプタは IAD 付き composite device として `0xEF / 0x02 / 0x01` を使用する。

## AudioControl エンティティ

| エンティティ | ID | 種別/接続先 |
|---|---|---|
| Clock Source | 0x10 | Internal Programmable、周波数RW/Validity RO |
| Input Terminal | 0x11 | USB Streaming (0x0101)、2ch (FL/FR)、Clock 0x10 |
| Output Terminal | 0x12 | Speaker (0x0301)、入力元 0x11、Clock 0x10 |

## エンドポイント（Alt 1 / Alt 2 共通構成）

| Alt | ビット幅 | wMaxPacketSize | エンドポイント | 方向/転送種別 | 同期 |
|---|---|---|---|---|---|
| 1 | 16-bit | 384 bytes (96kHz時) | Stream EP | OUT / Isochronous | Asynchronous |
| 1 | 16-bit | 3 bytes | Feedback EP | IN / Isochronous | Feedback (10.14固定小数点) |
| 2 | 24-bit | 576 bytes (96kHz時) | Stream EP | OUT / Isochronous | Asynchronous |
| 2 | 24-bit | 3 bytes | Feedback EP | IN / Isochronous | Feedback (10.14固定小数点) |

wMaxPacketSize の算出式: `ceil(sample_rate/1000) × channels(2) × bytes_per_sample`（96kHzを基準に確保）

## サポートするサンプルレート・パラメータ

| 項目 | 値 |
|---|---|
| 対応レート | 44,100 / 48,000 / 88,200 / 96,000 Hz |
| チャンネル数 | 2 (FL/FR) |
| ビット深度 | 16-bit / 24-bit（Alt Settingで切替） |
| Feedback更新周期 | 1ms (FEEDBACK_REFRESH_PERIOD) |
| FIFO目標水位 | 12パケット分（開始時は6パケット分） |

## 制御要求(Control Request)対応状況

| 対象 | Selector | Request | 対応 |
|---|---|---|---|
| Clock Source | Frequency (0x01) | SET_CUR | ✅ サンプルレート設定 |
| Clock Source | Frequency (0x01) | GET_CUR | ✅ 現在のサンプルレート取得 |
| Clock Source | Frequency (0x01) | GET_RANGE | ✅ 対応レート一覧返却 |
| Clock Source | Validity (0x02) | GET_CUR | ✅ 常にValid(1)を返却 |

Clock Sourceの制御はAudioControl Interfaceに対してのみ応答し、それ以外は非対応(None/Rejected)として扱われる。
