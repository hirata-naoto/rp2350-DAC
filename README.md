# XIAO RP2350 USB Audio DAC

XIAO RP2350 を USB Audio Class 2.0 のステレオ DAC として使い、I2S で外部 DAC へ出力するファームウェアです。

## 対応フォーマット

- 2ch PCM
- 16-bit / 24-bit
- 44.1 / 48 / 88.2 / 96 kHz

24-bit ストリームも 96 kHz まで受け付けます。I2S 出力は互換性を優先して常に 32-bit スロットで送出し、16-bit/24-bit PCM は左詰めで配置します。

## ピン配置

- DOUT: GPIO26
- BCLK: GPIO27
- LRCLK: GPIO28

対象ボードは **Seeed Studio XIAO RP2350** です。

## ビルド

Rust の ARM ターゲットを追加します。

```bash
rustup target add thumbv8m.main-none-eabihf
```

ビルド:

```bash
cargo build --release
```

`.cargo/config.toml` は `probe-rs run --chip RP235x` を runner に設定しています。必要に応じて書き換えてください。

## 実装概要

- USB: `embassy-usb` による UAC2 非同期等時 OUT + 明示的フィードバック
- I2S: `embassy-rp` の PIO を使った送信
- USB 受信データは FIFO に蓄積し、PIO へ DMA 転送

## 注意

- 24-bit PCM は USB から 3 byte packed little-endian で受け、I2S では 32-bit スロットへ左詰めして出力します。
- 接続する DAC 側は GPIO27/BCLK, GPIO28/LRCLK, GPIO26/DOUT のマスター送出を受けられる設定にしてください。
