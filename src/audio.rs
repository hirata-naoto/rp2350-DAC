use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

use embassy_usb::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::descriptor::{SynchronizationType, UsageType};
use embassy_usb::driver::{Driver, Endpoint, EndpointIn, EndpointOut};
use embassy_usb::types::InterfaceNumber;
use embassy_usb::{Builder, Handler};

pub const SAMPLE_RATE_HZ: u32 = 48_000;
pub const CHANNEL_COUNT: usize = 2;
pub const BITS_PER_SAMPLE_16: u8 = 16;
pub const BITS_PER_SAMPLE_24: u8 = 24;
pub const USB_PACKET_SIZE_16: usize = usb_packet_size(BITS_PER_SAMPLE_16, 96_000);
pub const USB_PACKET_SIZE_24: usize = usb_packet_size(BITS_PER_SAMPLE_24, 96_000);
pub const MAX_I2S_PACKET_WORDS: usize = i2s_words_per_usb_packet(BITS_PER_SAMPLE_24, 96_000);

// Alternate Setting 1 / 2 の有効化状態を保持し、再生開始/停止を追跡する。
pub static STREAM_ACTIVE: AtomicBool = AtomicBool::new(false);
static CURRENT_SAMPLE_RATE_HZ: AtomicU32 = AtomicU32::new(SAMPLE_RATE_HZ);
static CURRENT_BITS_PER_SAMPLE: AtomicU8 = AtomicU8::new(BITS_PER_SAMPLE_16);
static STREAM_CONFIG_VERSION: AtomicU32 = AtomicU32::new(0);

const USB_CLASS_AUDIO: u8 = 0x01;
const USB_SUBCLASS_AUDIO_CONTROL: u8 = 0x01;
const USB_SUBCLASS_AUDIO_STREAMING: u8 = 0x02;
const USB_PROTOCOL_IP_02_00: u8 = 0x20;

const CS_INTERFACE: u8 = 0x24;
const CS_ENDPOINT: u8 = 0x25;

const AC_HEADER: u8 = 0x01;
const AC_INPUT_TERM: u8 = 0x02;
const AC_OUTPUT_TERM: u8 = 0x03;
const AC_CLOCK_SOURCE: u8 = 0x0A;

const AS_GENERAL: u8 = 0x01;
const AS_FORMAT_TYPE: u8 = 0x02;
const EP_GENERAL: u8 = 0x01;

const CLOCK_SOURCE_ID: u8 = 0x10;
const INPUT_TERM_ID: u8 = 0x11;
const OUTPUT_TERM_ID: u8 = 0x12;

const FUNCTION_CATEGORY_DESKTOP_SPEAKER: u8 = 0x01;
const TERM_USB_STREAMING: u16 = 0x0101;
const TERM_SPEAKER: u16 = 0x0301;
const CHANNEL_CONFIG_FL_FR: u32 = 0x0000_0003;
const PCM_FORMAT_I: u32 = 0x0000_0001;
const FEEDBACK_REFRESH_PERIOD: u8 = 1;

const UAC2_CUR: u8 = 0x01;
const UAC2_GET_RANGE: u8 = 0x02;
const CLOCK_FREQUENCY_CONTROL_SELECTOR: u8 = 0x01;
const CLOCK_VALIDITY_CONTROL_SELECTOR: u8 = 0x02;
const CLOCK_SOURCE_ATTRIBUTES_INTERNAL_PROGRAMMABLE: u8 = 0x03;
const CLOCK_SOURCE_CONTROLS_HOST_PROGRAMMABLE_FREQUENCY_RW_VALIDITY_RO: u8 = 0x07;
const CLOCK_SOURCE_ASSOCIATION_TERMINAL_ID: u8 = 0x00;
const CLOCK_CONTROL_MASTER_CHANNEL: u8 = 0x00;
const CLOCK_VALIDITY_TRUE: u8 = 1;
const CLOCK_FREQUENCY_BYTES: usize = core::mem::size_of::<u32>();
const CLOCK_VALIDITY_BYTES: usize = core::mem::size_of::<u8>();
const SUPPORTED_SAMPLE_RATES_HZ: [u32; 4] = [44_100, 48_000, 88_200, 96_000];

pub struct UsbAudioClass {
    ac_interface: InterfaceNumber,
    streaming_interface: InterfaceNumber,
}

const fn bytes_per_sample(bits_per_sample: u8) -> usize {
    (bits_per_sample as usize).div_ceil(8)
}

const fn i2s_words_per_sample(bits_per_sample: u8) -> usize {
    let _ = bits_per_sample;
    CHANNEL_COUNT
}

const fn usb_packet_size(bits_per_sample: u8, sample_rate_hz: u32) -> usize {
    sample_rate_hz.div_ceil(1_000) as usize * CHANNEL_COUNT * bytes_per_sample(bits_per_sample)
}

pub const fn i2s_words_per_usb_packet(bits_per_sample: u8, sample_rate_hz: u32) -> usize {
    sample_rate_hz.div_ceil(1_000) as usize * CHANNEL_COUNT * i2s_words_per_sample(bits_per_sample)
}

const fn feedback_value_10_14(sample_rate_hz: u32) -> u32 {
    // Full-Speed の明示的フィードバックで使う 10.14 固定小数点へ変換する。
    (sample_rate_hz << 14) / 1_000
}

const fn feedback_packet_10_14(sample_rate_hz: u32) -> [u8; 3] {
    let bytes = feedback_value_10_14(sample_rate_hz).to_le_bytes();
    [bytes[0], bytes[1], bytes[2]]
}

fn supports_sample_rate(sample_rate_hz: u32) -> bool {
    SUPPORTED_SAMPLE_RATES_HZ.contains(&sample_rate_hz)
}

pub fn supports_stream_format(bits_per_sample: u8, sample_rate_hz: u32) -> bool {
    match bits_per_sample {
        BITS_PER_SAMPLE_16 => supports_sample_rate(sample_rate_hz),
        BITS_PER_SAMPLE_24 => supports_sample_rate(sample_rate_hz),
        _ => false,
    }
}

pub fn current_sample_rate_hz() -> u32 {
    CURRENT_SAMPLE_RATE_HZ.load(Ordering::Relaxed)
}

pub fn current_bits_per_sample() -> u8 {
    CURRENT_BITS_PER_SAMPLE.load(Ordering::Relaxed)
}

pub fn current_feedback_packet() -> [u8; 3] {
    feedback_packet_10_14(current_sample_rate_hz())
}

pub fn current_i2s_packet_words() -> usize {
    i2s_words_per_usb_packet(current_bits_per_sample(), current_sample_rate_hz())
}

pub fn stream_config_version() -> u32 {
    STREAM_CONFIG_VERSION.load(Ordering::Relaxed)
}

fn update_sample_rate(sample_rate_hz: u32) -> bool {
    let previous = CURRENT_SAMPLE_RATE_HZ.swap(sample_rate_hz, Ordering::Relaxed);
    previous != sample_rate_hz
}

fn update_stream_format(bits_per_sample: u8) -> bool {
    let previous = CURRENT_BITS_PER_SAMPLE.swap(bits_per_sample, Ordering::Relaxed);
    previous != bits_per_sample
}

fn note_stream_config_change() {
    STREAM_CONFIG_VERSION.fetch_add(1, Ordering::Relaxed);
}

impl UsbAudioClass {
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
        let mut func = builder.function(USB_CLASS_AUDIO, 0x00, USB_PROTOCOL_IP_02_00);

        let mut ac_interface = func.interface();
        let ac_interface_number = ac_interface.interface_number();
        let mut ac_alt = ac_interface.alt_setting(
            USB_CLASS_AUDIO,
            USB_SUBCLASS_AUDIO_CONTROL,
            USB_PROTOCOL_IP_02_00,
            None,
        );

        const AC_TOTAL_LENGTH: u16 = 46;
        // AudioControl 側ではクロック源と入出力ターミナルだけを最小構成で公開する。
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
                AC_OUTPUT_TERM,
                OUTPUT_TERM_ID,
                (TERM_SPEAKER & 0xff) as u8,
                (TERM_SPEAKER >> 8) as u8,
                0x00,
                INPUT_TERM_ID,
                CLOCK_SOURCE_ID,
                0x00,
                0x00,
                0x00,
            ],
        );

        let mut as_interface = func.interface();
        let as_interface_number = as_interface.interface_number();
        // Alt 0 は帯域未使用、Alt 1 で実際の等時 OUT エンドポイントを有効化する。
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
                0x00,
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
        let feedback_endpoint_16 =
            as_alt_16.alloc_endpoint_in(embassy_usb_driver::EndpointType::Isochronous, None, 4, 1);
        // ストリーム OUT 側へ同期先のフィードバックエンドポイント番号を関連付ける。
        as_alt_16.endpoint_descriptor(
            stream_endpoint_16.info(),
            SynchronizationType::Asynchronous,
            UsageType::DataEndpoint,
            &[0x00, feedback_endpoint_16.info().addr.into()],
        );
        as_alt_16.descriptor(CS_ENDPOINT, &[EP_GENERAL, 0x00, 0x00, 0x00, 0x00, 0x00]);
        // フィードバック値自体は 10.14 の 3 byte だが、最大長は余裕を見て 4 byte にする。
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
                0x00,
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
        let feedback_endpoint_24 =
            as_alt_24.alloc_endpoint_in(embassy_usb_driver::EndpointType::Isochronous, None, 4, 1);
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
    fn control_out(&mut self, req: Request, buf: &[u8]) -> Option<OutResponse> {
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
        if !supports_stream_format(bits_per_sample, sample_rate_hz) {
            return Some(OutResponse::Rejected);
        }

        if update_sample_rate(sample_rate_hz) {
            note_stream_config_change();
        }
        Some(OutResponse::Accepted)
    }

    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
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
                // 離散レート列として 44.1/48/88.2/96 kHz を返し、
                // 各 alternate setting の wMaxPacketSize で実際の組み合わせを絞り込む。
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

    fn set_alternate_setting(&mut self, iface: InterfaceNumber, alternate: u8) {
        if iface == self.streaming_interface {
            let mut changed = false;
            let active = matches!(alternate, 1 | 2);

            if active {
                let bits_per_sample = if alternate == 2 {
                    BITS_PER_SAMPLE_24
                } else {
                    BITS_PER_SAMPLE_16
                };
                changed |= update_stream_format(bits_per_sample);

                if !supports_stream_format(bits_per_sample, current_sample_rate_hz()) {
                    changed |= update_sample_rate(SAMPLE_RATE_HZ);
                }
            }

            STREAM_ACTIVE.store(active, Ordering::Relaxed);
            if changed {
                note_stream_config_change();
            }
        }
    }

    fn reset(&mut self) {
        STREAM_ACTIVE.store(false, Ordering::Relaxed);
        let mut changed = false;
        changed |= update_stream_format(BITS_PER_SAMPLE_16);
        changed |= update_sample_rate(SAMPLE_RATE_HZ);
        if changed {
            note_stream_config_change();
        }
    }
}
