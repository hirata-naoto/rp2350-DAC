#![no_std]
#![no_main]

use ch32_hal::otg_fs::{self, Driver};
use ch32_hal::usb::EndpointDataBuffer512;
use ch32_hal::{self as hal, bind_interrupts, peripherals, Config};
use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::{join, join5};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_usb::driver::{Endpoint, EndpointError, EndpointIn, EndpointOut};
use embassy_usb::Builder;
use panic_halt as _;
use static_cell::StaticCell;

mod audio;
mod i2s;

bind_interrupts!(struct Irq {
    OTG_FS => otg_fs::InterruptHandler<peripherals::OTG_FS>;
});

const AUDIO_FIFO_CAPACITY_WORDS: usize = audio::MAX_I2S_PACKET_WORDS * 32;
const I2S_DMA_BUFFER_WORDS: usize = audio::MAX_I2S_PACKET_WORDS * 4;

static AUDIO_FIFO: Mutex<CriticalSectionRawMutex, AudioSampleFifo<AUDIO_FIFO_CAPACITY_WORDS>> =
    Mutex::new(AudioSampleFifo::new());
static AUDIO_HANDLER: StaticCell<audio::UsbAudioClass> = StaticCell::new();
static I2S_DMA_BUFFER: StaticCell<[u16; I2S_DMA_BUFFER_WORDS]> = StaticCell::new();

struct AudioSampleFifo<const N: usize> {
    data: [u16; N],
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

    fn push_slice(&mut self, words: &[u16]) {
        if words.len() >= N {
            // FIFO 容量を超える入力は末尾の最新サンプルだけを残す。
            self.clear();
            for &word in &words[words.len() - N..] {
                self.write_word(word);
            }
            return;
        }

        let missing_space = words.len().saturating_sub(N - self.len);
        if missing_space != 0 {
            // USB 側が一時的に速かったときは古いサンプルを捨てて遅延の増えすぎを防ぐ。
            self.discard_oldest(missing_space);
        }

        for &word in words {
            self.write_word(word);
        }
    }

    fn pop_slice(&mut self, out: &mut [u16]) -> usize {
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

    fn write_word(&mut self, word: u16) {
        let write = (self.read + self.len) % N;
        self.data[write] = word;
        self.len += 1;
    }

    fn read_word(&mut self) -> u16 {
        let word = self.data[self.read];
        self.read = (self.read + 1) % N;
        self.len -= 1;
        word
    }
}

fn bytes_to_i2s_words(bytes: &[u8], bits_per_sample: u8, out: &mut [u16]) -> usize {
    match bits_per_sample {
        audio::BITS_PER_SAMPLE_16 => {
            let mut count = 0;
            for sample_bytes in bytes.chunks_exact(2).take(out.len()) {
                // USB から届く little-endian PCM16 を、そのまま I2S の 16-bit スロットへ並べる。
                out[count] = u16::from_le_bytes([sample_bytes[0], sample_bytes[1]]);
                count += 1;
            }
            count
        }
        audio::BITS_PER_SAMPLE_24 => {
            let mut count = 0;
            for sample_bytes in bytes.chunks_exact(3) {
                if count + 1 >= out.len() {
                    break;
                }

                // 24-bit PCM は 32-bit チャネル枠へ左詰めし、DMA からは上位 16 bit → 下位 16 bit の順で流す。
                let sign = if (sample_bytes[2] & 0x80) != 0 {
                    0xff
                } else {
                    0x00
                };
                let sample =
                    i32::from_le_bytes([sample_bytes[0], sample_bytes[1], sample_bytes[2], sign]);
                let aligned = ((sample << 8) as u32).to_be_bytes();
                out[count] = u16::from_be_bytes([aligned[0], aligned[1]]);
                out[count + 1] = u16::from_be_bytes([aligned[2], aligned[3]]);
                count += 2;
            }
            count
        }
        _ => 0,
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let config = Config {
        // USB/I2S のタイミング計算を 144 MHz 前提でそろえる。
        rcc: hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSI,
        ..Default::default()
    };
    let p = hal::init(config);
    info!("boot");

    // SPI2/I2S2 に割り当てるピンを確保して他用途への再利用を防ぐ。
    let _spi2 = p.SPI2;
    let _i2s_ws = p.PB12;
    let _i2s_ck = p.PB13;
    let _i2s_sd = p.PB15;
    let dma_buffer = I2S_DMA_BUFFER.init([0; I2S_DMA_BUFFER_WORDS]);

    let mut endpoint_buffers: [EndpointDataBuffer512; 4] =
        core::array::from_fn(|_| EndpointDataBuffer512::default());
    let driver = Driver::new(p.OTG_FS, p.PA12, p.PA11, &mut endpoint_buffers);

    let mut usb_config = embassy_usb::Config::new(0x1209, 0x3050);
    usb_config.manufacturer = Some("hirata-naoto");
    usb_config.product = Some("CH32V305 USB Audio to I2S");
    usb_config.serial_number = Some("0001");
    usb_config.device_class = 0x00;
    usb_config.device_sub_class = 0x00;
    usb_config.device_protocol = 0x00;
    usb_config.composite_with_iads = false;
    usb_config.max_power = 100;
    usb_config.max_packet_size_0 = 64;

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 64];
    let mut msos_descriptor = [0; 64];
    let mut control_buf = [0; 64];

    // USB Audio Class の各種記述子とエンドポイントを組み立てる。
    let mut builder = Builder::new(
        driver,
        usb_config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
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
    let mut i2s = i2s::I2s2Tx::new(p.DMA1_CH5, dma_buffer);
    let mut silence = [0u16; audio::MAX_I2S_PACKET_WORDS];
    i2s.configure(
        audio::current_sample_rate_hz(),
        audio::current_bits_per_sample(),
    );
    info!(
        "i2s init rate={}Hz bits={}",
        audio::current_sample_rate_hz(),
        audio::current_bits_per_sample()
    );
    let initial_packet_words = audio::current_i2s_packet_words();
    i2s.prime(&silence[..initial_packet_words]);
    i2s.start();

    let usb_fut = usb.run();
    let receive_16_fut = async {
        let mut packet = [0u8; audio::USB_PACKET_SIZE_16];
        let mut words = [0u16; audio::MAX_I2S_PACKET_WORDS];

        loop {
            stream_endpoint_16.wait_enabled().await;
            info!("stream16 enabled");
            {
                // 新しいストリーム開始時は前回の残りを捨てて先頭から再生し直す。
                let mut fifo = AUDIO_FIFO.lock().await;
                fifo.clear();
            }

            loop {
                match stream_endpoint_16.read(&mut packet).await {
                    Ok(received) => {
                        let word_count = bytes_to_i2s_words(
                            &packet[..received],
                            audio::BITS_PER_SAMPLE_16,
                            &mut words,
                        );
                        let mut fifo = AUDIO_FIFO.lock().await;
                        // USB 等時転送で受けた PCM を、I2S 側とは独立した FIFO へ積む。
                        fifo.push_slice(&words[..word_count]);
                    }
                    Err(EndpointError::Disabled) => {
                        info!("stream16 disabled");
                        let mut fifo = AUDIO_FIFO.lock().await;
                        fifo.clear();
                        break;
                    }
                    Err(EndpointError::BufferOverflow) => {}
                }
            }
        }
    };
    let receive_24_fut = async {
        let mut packet = [0u8; audio::USB_PACKET_SIZE_24];
        let mut words = [0u16; audio::MAX_I2S_PACKET_WORDS];

        loop {
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
        let mut chunk = [0u16; audio::MAX_I2S_PACKET_WORDS];
        let mut config_version = audio::stream_config_version();

        loop {
            let next_config_version = audio::stream_config_version();
            if next_config_version != config_version {
                config_version = next_config_version;
                let packet_words = audio::current_i2s_packet_words();
                info!(
                    "stream config changed rate={}Hz bits={} i2s_words={}",
                    audio::current_sample_rate_hz(),
                    audio::current_bits_per_sample(),
                    packet_words
                );
                {
                    let mut fifo = AUDIO_FIFO.lock().await;
                    fifo.clear();
                }
                // フォーマットやレートが切り替わったら I2S 分周とデータ長を更新し、古い残留データを無音で置き換える。
                i2s.configure(
                    audio::current_sample_rate_hz(),
                    audio::current_bits_per_sample(),
                );
                silence[..packet_words].fill(0);
                i2s.prime(&silence[..packet_words]);
            }

            let packet_words = audio::current_i2s_packet_words();
            let written = {
                let mut fifo = AUDIO_FIFO.lock().await;
                if audio::STREAM_ACTIVE.load(core::sync::atomic::Ordering::Relaxed) {
                    fifo.pop_slice(&mut chunk[..packet_words])
                } else {
                    fifo.clear();
                    0
                }
            };

            // FIFO が空のときは無音を補って、DMA の連続出力を途切れさせない。
            chunk[written..packet_words].fill(0);
            i2s.write_words(&chunk[..packet_words]).await;
        }
    };
    let feedback_16_fut = async {
        loop {
            feedback_endpoint_16.wait_enabled().await;

            loop {
                // 現在選択中のレートを 10.14 形式の明示的フィードバックで返し続ける。
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
