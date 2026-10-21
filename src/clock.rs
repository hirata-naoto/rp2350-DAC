//! USB オーディオのサンプルレート系統に合わせて RP2350 のシステムクロックを設定する。

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_rp::clocks::{ClockConfig, PllConfig, SysClkConfig, SysClkSrc};
use embassy_rp::pac;

const XOSC_HZ: u32 = 12_000_000;
pub const CLOCK_48_FAMILY_HZ: u32 = 110_571_428;
pub const CLOCK_44_FAMILY_HZ: u32 = 101_600_000;
const SYS_CLOCK_48_FAMILY: u32 = 0;
const SYS_CLOCK_44_FAMILY: u32 = 1;

static CURRENT_SYS_CLOCK_HZ: AtomicU32 = AtomicU32::new(CLOCK_48_FAMILY_HZ);
static CURRENT_SYS_CLOCK_FAMILY: AtomicU32 = AtomicU32::new(SYS_CLOCK_48_FAMILY);

pub fn embassy_config() -> embassy_rp::config::Config {
    let mut clocks = ClockConfig::crystal(XOSC_HZ);
    clocks.xosc.as_mut().unwrap().sys_pll = Some(PllConfig {
        refdiv: 1,
        fbdiv: 129,
        post_div1: 7,
        post_div2: 2,
    });
    clocks.sys_clk = SysClkConfig {
        src: SysClkSrc::PllSys,
        div_int: 1,
        div_frac: 0,
    };
    embassy_rp::config::Config::new(clocks)
}

pub fn current_sys_clock_hz() -> u32 {
    CURRENT_SYS_CLOCK_HZ.load(Ordering::Relaxed)
}

pub fn set_for_sample_rate(sample_rate_hz: u32) -> u32 {
    let (family, pll, divider, sys_clock_hz) = match sample_rate_hz {
        44_100 | 88_200 => (
            SYS_CLOCK_44_FAMILY,
            PllConfig {
                refdiv: 1,
                fbdiv: 127,
                post_div1: 5,
                post_div2: 3,
            },
            (1, 0),
            CLOCK_44_FAMILY_HZ,
        ),
        _ => (
            SYS_CLOCK_48_FAMILY,
            PllConfig {
                refdiv: 1,
                fbdiv: 129,
                post_div1: 7,
                post_div2: 2,
            },
            (1, 0),
            CLOCK_48_FAMILY_HZ,
        ),
    };

    if CURRENT_SYS_CLOCK_FAMILY.load(Ordering::Relaxed) == family {
        return current_sys_clock_hz();
    }

    unsafe {
        let clocks = pac::CLOCKS;
        clocks
            .clk_sys_ctrl()
            .modify(|w| w.set_src(pac::clocks::vals::ClkSysCtrlSrc::CLK_REF));
        while clocks.clk_sys_selected().read() != pac::clocks::regs::ClkSysSelected(1) {}

        configure_sys_pll(pll);

        clocks.clk_sys_div().write(|w| {
            w.set_int(divider.0);
            w.set_frac(divider.1);
        });
        clocks.clk_sys_ctrl().modify(|w| {
            w.set_auxsrc(pac::clocks::vals::ClkSysCtrlAuxsrc::CLKSRC_PLL_SYS);
            w.set_src(pac::clocks::vals::ClkSysCtrlSrc::CLKSRC_CLK_SYS_AUX);
        });
        while clocks.clk_sys_selected().read() != pac::clocks::regs::ClkSysSelected(2) {}
    }

    CURRENT_SYS_CLOCK_HZ.store(sys_clock_hz, Ordering::Relaxed);
    CURRENT_SYS_CLOCK_FAMILY.store(family, Ordering::Relaxed);
    sys_clock_hz
}

unsafe fn configure_sys_pll(config: PllConfig) {
    let pll = pac::PLL_SYS;
    pll.pwr().write(|w| {
        w.set_pd(true);
        w.set_vcopd(true);
        w.set_postdivpd(true);
        w.set_dsmpd(true);
    });
    cortex_m::asm::delay(10);
    pll.cs().write(|w| w.set_refdiv(config.refdiv));
    pll.fbdiv_int().write(|w| w.set_fbdiv_int(config.fbdiv));
    pll.pwr().write(|w| {
        w.set_pd(false);
        w.set_vcopd(false);
        w.set_postdivpd(true);
        w.set_dsmpd(true);
    });

    let mut timeout = 1_000_000;
    while !pll.cs().read().lock() {
        timeout -= 1;
        assert!(timeout != 0, "PLL_SYS failed to lock");
    }

    pll.prim().write(|w| {
        w.set_postdiv1(config.post_div1);
        w.set_postdiv2(config.post_div2);
    });
    pll.pwr().modify(|w| w.set_postdivpd(false));
    cortex_m::asm::delay(100);
}
