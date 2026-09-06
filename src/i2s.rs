use embassy_rp::dma;
use embassy_rp::gpio::{Drive, SlewRate};
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::pio::{
    Common, Config, Direction, FifoJoin, Instance, LoadedProgram, PioPin, ShiftConfig,
    ShiftDirection, StateMachine,
};
use embassy_rp::pio_programs::clock_divider::calculate_pio_clock_divider;
use embassy_rp::Peri;

const I2S_SLOT_BITS: u32 = 32;

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

        data_pin.set_drive_strength(Drive::_12mA);
        bit_clock_pin.set_drive_strength(Drive::_12mA);
        lr_clock_pin.set_drive_strength(Drive::_12mA);
        data_pin.set_slew_rate(SlewRate::Fast);
        bit_clock_pin.set_slew_rate(SlewRate::Fast);
        lr_clock_pin.set_slew_rate(SlewRate::Fast);

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

    pub fn configure(&mut self, sample_rate_hz: u32) {
        let mut config = Config::default();
        config.use_program(&self.program, &[&self.bit_clock_pin, &self.lr_clock_pin]);
        config.set_out_pins(&[&self.data_pin]);
        config.clock_divider = calculate_pio_clock_divider(sample_rate_hz * I2S_SLOT_BITS * 2 * 2);
        config.shift_out = ShiftConfig {
            threshold: 32,
            direction: ShiftDirection::Left,
            auto_fill: true,
        };
        config.fifo_join = FifoJoin::TxOnly;

        self.sm.set_enable(false);
        self.sm.set_config(&config);
        self.sm.set_pin_dirs(
            Direction::Out,
            &[&self.data_pin, &self.lr_clock_pin, &self.bit_clock_pin],
        );
        self.sm.clear_fifos();
        unsafe { self.sm.set_y(I2S_SLOT_BITS - 2) };
        self.started = false;
    }

    pub fn prime(&mut self, data: &[u32]) {
        self.sm.clear_fifos();
        for &word in data.iter().take(8) {
            self.sm.tx().push(word);
        }
    }

    pub fn start(&mut self) {
        if self.started {
            return;
        }

        self.sm.set_enable(true);
        self.started = true;
    }

    pub async fn write_words(&mut self, data: &[u32]) {
        self.sm.tx().dma_push(&mut self.dma, data, false).await;
    }
}
