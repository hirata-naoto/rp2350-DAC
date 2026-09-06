use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::dma;
use embassy_rp::gpio::{Drive, SlewRate};
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::pio::{
    Common, Config, Direction, FifoJoin, Instance, LoadedProgram, PioPin, ShiftConfig,
    ShiftDirection, StateMachine,
};
use embassy_rp::pio_programs::clock_divider::calculate_pio_clock_divider;
use embassy_rp::Peri;

// I2S は左右それぞれ 32-bit スロット固定で送出する。
const I2S_SLOT_BITS: u32 = 32;
const I2S_CHANNEL_COUNT: u32 = 2;
// この PIO プログラムは 1 bit 出力に 2 サイクル使い、side-set で BCLK/LRCLK を同時駆動する。
const PIO_CYCLES_PER_BIT: u32 = 2;
const FEEDBACK_DENOMINATOR_HZ: u64 = 1_000;
const FEEDBACK_NUMERATOR_SCALE: u64 = 32_768;

// I2S 実効サンプルレートと、USB へ返す 10.14 フィードバック値をまとめる。
pub struct I2sTiming {
    pub actual_sample_rate_hz: u32,
    pub feedback_value_10_14: u32,
}

// PIO ステートマシンと DMA を組み合わせた I2S 送信器。
pub struct I2sPioTx<'d, PIO: Instance, const SM: usize> {
    dma: dma::Channel<'d>,
    sm: StateMachine<'d, PIO, SM>,
    data_pin: embassy_rp::pio::Pin<'d, PIO>,
    bit_clock_pin: embassy_rp::pio::Pin<'d, PIO>,
    lr_clock_pin: embassy_rp::pio::Pin<'d, PIO>,
    program: LoadedProgram<'d, PIO>,
    started: bool,
}

impl<'d, PIO: Instance, const SM: usize> I2sPioTx<'d, PIO, SM> {
    pub fn new<D: dma::ChannelInstance>(
        common: &mut Common<'d, PIO>,
        sm: StateMachine<'d, PIO, SM>,
        dma_ch: Peri<'d, D>,
        irq: impl Binding<D::Interrupt, dma::InterruptHandler<D>> + 'd,
        data_pin: Peri<'d, impl PioPin + 'd>,
        bit_clock_pin: Peri<'d, impl PioPin + 'd>,
        lr_clock_pin: Peri<'d, impl PioPin + 'd>,
    ) -> Self {
        let mut data_pin = common.make_pio_pin(data_pin);
        let mut bit_clock_pin = common.make_pio_pin(bit_clock_pin);
        let mut lr_clock_pin = common.make_pio_pin(lr_clock_pin);

        // 外付け DAC へ直接出すため、立ち上がりを優先して強めのドライブ設定にする。
        data_pin.set_drive_strength(Drive::_12mA);
        bit_clock_pin.set_drive_strength(Drive::_12mA);
        lr_clock_pin.set_drive_strength(Drive::_12mA);
        data_pin.set_slew_rate(SlewRate::Fast);
        bit_clock_pin.set_slew_rate(SlewRate::Fast);
        lr_clock_pin.set_slew_rate(SlewRate::Fast);

        // side-set の 2bit で BCLK/LRCLK を作りながら、32bit×2ch を順にシフトアウトする。
        let program = pio::pio_asm!(
            ".side_set 2",
            "    mov x, y           side 0b01",
            "left_data:",
            "    out pins, 1        side 0b00",
            "    jmp x-- left_data  side 0b01",
            "    out pins, 1        side 0b10",
            "    mov x, y           side 0b11",
            "right_data:",
            "    out pins, 1        side 0b10",
            "    jmp x-- right_data side 0b11",
            "    out pins, 1        side 0b00",
        );
        let program = common.load_program(&program.program);

        let mut this = Self {
            dma: dma::Channel::new(dma_ch, irq),
            sm,
            data_pin,
            bit_clock_pin,
            lr_clock_pin,
            program,
            started: false,
        };
        this.configure(48_000);
        this
    }

    pub fn configure(&mut self, sample_rate_hz: u32) -> I2sTiming {
        // 目標サンプルレートから必要な PIO ステートマシンクロックを逆算する。
        let target_sm_hz = sample_rate_hz * I2S_SLOT_BITS * I2S_CHANNEL_COUNT * PIO_CYCLES_PER_BIT;
        let clock_divider = calculate_pio_clock_divider(target_sm_hz);
        let divider_bits = clock_divider.to_bits() as u64;
        let feedback_denominator = divider_bits * FEEDBACK_DENOMINATOR_HZ;
        // 実クロックに基づく USB Audio 10.14 フィードバック値を丸め込みで求める。
        let feedback_value_10_14 = ((clk_sys_freq() as u64 * FEEDBACK_NUMERATOR_SCALE)
            + feedback_denominator / 2)
            / feedback_denominator;
        let actual_sample_rate_hz =
            ((feedback_value_10_14 * FEEDBACK_DENOMINATOR_HZ) + (1 << 13)) >> 14;
        let mut config = Config::default();
        config.use_program(&self.program, &[&self.bit_clock_pin, &self.lr_clock_pin]);
        config.set_out_pins(&[&self.data_pin]);
        config.clock_divider = clock_divider;
        config.shift_out = ShiftConfig {
            threshold: 32,
            direction: ShiftDirection::Left,
            auto_fill: true,
        };
        config.fifo_join = FifoJoin::TxOnly;

        // 設定変更時はいったん停止し、FIFO とカウンタを初期状態へ戻す。
        self.sm.set_enable(false);
        self.sm.set_config(&config);
        self.sm.set_pin_dirs(
            Direction::Out,
            &[&self.data_pin, &self.lr_clock_pin, &self.bit_clock_pin],
        );
        self.sm.clear_fifos();
        unsafe { self.sm.set_y(I2S_SLOT_BITS - 2) };
        self.started = false;

        I2sTiming {
            actual_sample_rate_hz: actual_sample_rate_hz as u32,
            feedback_value_10_14: feedback_value_10_14 as u32,
        }
    }

    pub fn prime(&mut self, data: &[u32]) {
        // DMA 開始前に数ワード先行投入し、送信開始直後のアンダーランを避ける。
        self.sm.clear_fifos();
        for &word in data.iter().take(8) {
            self.sm.tx().push(word);
        }
    }

    pub fn start(&mut self) {
        if self.started {
            return;
        }

        // prime 済みの FIFO を使ってステートマシンを走らせる。
        self.sm.set_enable(true);
        self.started = true;
    }

    pub async fn write_words(&mut self, data: &[u32]) {
        // 1 チャンクぶんを DMA で流し切るまで待つ。
        self.sm.tx().dma_push(&mut self.dma, data, false).await;
    }
}
