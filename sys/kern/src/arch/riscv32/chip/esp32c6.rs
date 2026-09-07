// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! ESP32-C6 chip support: everything the RISC-V privileged spec leaves to
//! the implementation, as this chip implements it.
//!
//! See the module docs in `arch/riscv32.rs` for the surface this module
//! must export and the contract each item carries. The C6 is a single-core
//! chip; what it chose, and this module therefore owns:
//!
//! - **Interrupt delivery is two-level**: 77 peripheral interrupt *sources*
//!   are routed by an interrupt matrix onto 32 CPU interrupt *lines*. The
//!   line controller, though, is not a CLIC but the simpler Espressif
//!   "PLIC" (`SOC_INT_PLIC_SUPPORTED`): flat per-line enable/type/priority
//!   registers, and the line number delivered directly as the `mcause`
//!   exception code -- no id offset, no claim handshake. `mtvec` is
//!   vectored, the core forcing that mode (see `MTVEC_MODE`): an
//!   interrupt on line n arrives at slot n of the generic layer's jump
//!   table, every slot leads to the one kernel entry sequence, and
//!   `mcause` says why.
//! - **There is no CLINT and no `mtime`/`mtimecmp`**: the kernel tick runs
//!   on the SYSTIMER peripheral's TARGET0 comparator in periodic mode.
//!   Hubris is the only software on this single-core chip, so all three
//!   comparators are free and the tick takes the first.
//! - **One core, so no `mhartid` selection anywhere**: the matrix, the
//!   PLIC window and the reset register are all singular.
//! - **Bus-level permission filtering (APM) sits in front of everything**
//!   and is closed to U-mode at reset -- one HP_APM block covers SRAM and
//!   peripheral space alike, with no separate per-peripheral gate. It must
//!   be opened before any task can execute a single instruction. It is
//!   also where a DMA master could be confined, since the same block
//!   identifies bus masters. This chip support does not do that: it only
//!   opens the filter for the U-mode CPU (the `apm` module), so a
//!   peripheral that masters the bus is not held to its task's memory.
//! - **No coprocessors.** The core is RV32IMAC: no FPU (no `F` in the
//!   ISA string, no `SOC_CPU_HAS_FPU`), no vendor HWLOOP/PIE units or
//!   their CSRs. The generic layer's integer-only `SavedState` is correct
//!   by construction, with nothing to disable.
//! - **Reset** goes through LP_AON (CPU core software reset).
//!
//! Registers are reached through the `esp32c6` PAC (esp-rs/esp-pacs);
//! their semantics cite public esp-idf, `components/soc/esp32c6`.

use core::sync::atomic::Ordering;

use super::InterruptEvent;
use abi::{InterruptNum, UsageError};
use esp32c6 as pac;

// Register blocks. The kernel owns these peripherals outright, so it
// dereferences the PAC's fixed base pointers rather than taking a
// `Peripherals` singleton, as upstream Hubris does for its STM32 PACs.
macro_rules! register_blocks {
    ($($name:ident: $periph:ident, $module:ident;)*) => {$(
        fn $name() -> &'static pac::$module::RegisterBlock {
            // Safety: the PAC pointer is the block's fixed MMIO address,
            // valid for the life of the program.
            unsafe { &*pac::$periph::ptr() }
        }
    )*};
}

register_blocks! {
    timg0: TIMG0, timg0;
    lp_wdt: LP_WDT, lp_wdt;
    hp_apm: HP_APM, hp_apm;
    lp_apm: LP_APM, lp_apm;
    pcr: PCR, pcr;
    systimer: SYSTIMER, systimer;
    plic_mx: PLIC_MX, plic_mx;
    intmtx: INTERRUPT_CORE0, interrupt_core0;
    lp_aon: LP_AON, lp_aon;
}

/// `mtvec` mode bits: vectored -- not by choice, the MODE field is WARL
/// and this core forces it (a write of 0 reads back 1; esp-idf likewise
/// always installs vectored on this family). Exceptions arrive at the
/// base, interrupt line n at base + 4*n; the generic layer's trap vector
/// is a jump table shaped for exactly this. The core also hardwires
/// mtvec base bits [7:2] to zero -- the base must be 256-byte aligned or
/// it is silently truncated.
pub(super) const MTVEC_MODE: usize = 1;

/// PMP granularity is 4 bytes (`SOC_CPU_PMP_REGION_GRANULARITY` in public
/// soc_caps.h) -- full NAPOT resolution. Note the build system still
/// rounds regions to powers of two no smaller than 32 bytes
/// (build/xtask/src/config.rs `mpu_alignment`), which is merely
/// conservative on this chip.
pub(super) const PMP_GRANULARITY: u32 = 4;

/// Sixteen PMP entries, all programmed per task (`SOC_CPU_PMP_REGION_*`
/// count in public soc_caps.h). Twice the Cortex-M budget, and used: a
/// driver for a closed peripheral stack holds several register grants
/// on top of its own memory.
pub(super) const PMP_ENTRIES: usize = 16;

/// One-shot hardware init, called by `start_first_task` before the first
/// task runs. Per the chip contract this runs with `mstatus.MIE` clear and
/// the kernel's `mtvec` installed, so unmasking interrupt sources here is
/// safe: nothing can fire until the first `mret` drops into U-mode.
pub(super) fn init(tick_divisor: u32) {
    disable_flashboot_watchdogs();

    // This core implements the (long-retired) N extension's U-mode trap
    // machinery -- `ustatus`/`uie`/`utvec`, a U-mode PLIC bank, a U-mode
    // CLINT -- which esp-idf's esp_tee uses to hand interrupts to
    // unprivileged code. That means `medeleg` and `mideleg` exist, and if
    // anything is delegated, a U-mode trap (a task's ecall, a task's
    // fault, the tick) vectors through the *user* trap vector instead of
    // the kernel's mtvec and is simply gone: the symptom is a first task
    // that runs and a kernel that never hears from it again. Route every
    // trap to M-mode, unconditionally. (Numeric CSR ids: medeleg = 0x302,
    // mideleg = 0x303 -- this toolchain's `csrw` only knows them by
    // number.)
    unsafe {
        core::arch::asm!(
            "
            csrw 0x302, zero
            csrw 0x303, zero
            ",
            options(nostack),
        );
    }

    // Bus-level permission filtering (APM). The reset state is misleading:
    // region 0 spans the whole address space with filtering on and every
    // REE permission clear, so a U-mode instruction fetch returns zeros and
    // faults as an illegal instruction with mtval = 0 -- not as an access
    // fault. `apm::init` opens the map for U-mode CPU traffic; see that
    // module.
    apm::init();

    // No coprocessors to disable: RV32IMAC, no F extension and no vendor
    // units.

    // Unmask all PLIC priority levels: delivery requires a line's priority
    // to exceed the threshold, so park the threshold at 0 (its reset
    // value, written anyway so boot does not depend on it). Delivery stays
    // gated by mstatus.MIE, which the generic layer keeps clear in M-mode.
    // Safety: the threshold field takes any 8-bit level.
    plic_mx()
        .mxint_thresh()
        .write(|w| unsafe { w.cpu_mxint_thresh().bits(0) });

    // Ungate the SYSTIMER. The C6's PCR gate defaults to enabled and the
    // UNIT0 counter free-runs out of reset (UNIT0_WORK_EN default 1) -- but
    // set the gate explicitly anyway, so boot does not depend on reset
    // defaults.
    pcr().systimer_conf().modify(|_, w| {
        w.systimer_clk_en().set_bit().systimer_rst_en().clear_bit()
    });

    // Program the kernel tick: SYSTIMER TARGET0 in periodic mode fires
    // every `tick_divisor` ticks, forever, with no per-tick reprogramming.
    // The write starts from the register's reset value, zero, so the
    // counter select (bit 31) stays at UNIT0.
    let st = systimer();
    uassert!(tick_divisor <= SYSTIMER_TARGET_PERIOD_MASK);
    // Safety: the period was just checked against the field's width.
    st.target0_conf().write(|w| {
        unsafe { w.period().bits(tick_divisor) }
            .period_mode()
            .set_bit()
    });
    // Latch the comparator configuration.
    st.comp0_load().write(|w| w.load().set_bit());
    // Turn the comparator on and take its interrupt.
    st.conf().modify(|_, w| w.target0_work_en().set_bit());
    st.int_clr().write(|w| w.target0().clear_bit_by_one());
    st.int_ena().modify(|_, w| w.target0().set_bit());

    // Route the SYSTIMER_TARGET0 interrupt source through the interrupt
    // matrix to our reserved CPU line, then enable the line at the PLIC:
    // level-triggered (TYPE bit 0, the default), priority 1 so it clears
    // the threshold. The matrix takes the plain line number -- no CLIC-id
    // offset on this chip (public esp-idf, hal/interrupt_plic_ll.h).
    route(TICK_SOURCE, TICK_LINE);
    mie_set(1 << TICK_LINE);
    plic_enable_modify(|en| en | (1 << TICK_LINE));
}

/// Sets bits in the `mie` CSR. On this core the 32 CPU interrupt lines
/// occupy `mie`/`mip` bits 0..31 directly (nonstandard -- the standard
/// meanings of those bits do not apply), and a line delivers only if its
/// `mie` bit is set in addition to its PLIC enable. Mainline esp-idf never
/// touches `mie` because it runs everything in M-mode; the U-mode
/// precedent is esp_tee, whose PLIC rv_utils variant sets exactly this
/// (esp_tee_rv_utils.h).
fn mie_set(bits: u32) {
    // Safety: setting mie bits only unmasks interrupt lines; delivery is
    // still gated by mstatus.MIE / privilege, per the chip contract.
    unsafe {
        core::arch::asm!(
            "csrs mie, {}",
            in(reg) bits,
            options(nostack, preserves_flags),
        );
    }
}

/// Clears bits in the `mie` CSR; see [`mie_set`].
fn mie_clear(bits: u32) {
    // Safety: masking an interrupt line has no memory-safety effect.
    unsafe {
        core::arch::asm!(
            "csrc mie, {}",
            in(reg) bits,
            options(nostack, preserves_flags),
        );
    }
}

/// Turns off the watchdogs the boot ROM leaves running.
///
/// The timer-group watchdog and the LP watchdog both reset with their
/// *flashboot* mode enabled, obliging whatever boots from flash to switch
/// them off or be reset about a second in. The super watchdog cannot be
/// switched off, only auto-fed. All three sit behind the same
/// write-protect key.
fn disable_flashboot_watchdogs() {
    // Safety, for every key write below: each protect register is a
    // single 32-bit key field, and any value is a legal write.

    // Timer group 0's watchdog (timer_group_reg.h).
    let timg0 = timg0();
    timg0
        .wdtwprotect()
        .write(|w| unsafe { w.wdt_wkey().bits(WDT_WKEY) });
    timg0
        .wdtconfig0()
        .modify(|_, w| w.wdt_flashboot_mod_en().clear_bit());
    timg0
        .wdtwprotect()
        .write(|w| unsafe { w.wdt_wkey().bits(0) });

    // The LP watchdog (lp_wdt_reg.h).
    let lp_wdt = lp_wdt();
    lp_wdt
        .wdtwprotect()
        .write(|w| unsafe { w.wdt_wkey().bits(WDT_WKEY) });
    lp_wdt
        .wdtconfig0()
        .modify(|_, w| w.wdt_flashboot_mod_en().clear_bit());
    lp_wdt
        .wdtwprotect()
        .write(|w| unsafe { w.wdt_wkey().bits(0) });

    // The super watchdog: auto-feed, per the vendor bootloader's own
    // policy (lp_wdt_reg.h: SWD_CONFIG, AUTO_FEED_EN).
    lp_wdt
        .swd_wprotect()
        .write(|w| unsafe { w.swd_wkey().bits(WDT_WKEY) });
    lp_wdt
        .swd_conf()
        .modify(|_, w| w.swd_auto_feed_en().set_bit());
    lp_wdt
        .swd_wprotect()
        .write(|w| unsafe { w.swd_wkey().bits(0) });
}

/// Unlock key for the watchdogs' configuration (public esp-idf,
/// esp_hal_wdt esp32c6: `LP_WDT_WKEY_VALUE`, same value for mwdt).
const WDT_WKEY: u32 = 0x50D8_3AA1;

/// Classifies an interrupt trap. The `mcause` exception code of an
/// interrupt is the CPU line number, no offset involved.
pub(super) fn decode_interrupt(mcause: u32) -> Option<InterruptEvent> {
    let line = mcause & 0xFFF;
    if line == TICK_LINE {
        return Some(InterruptEvent::Tick);
    }
    if (line as usize) < CPU_INT_LINES {
        let source = IRQ_LINE_SOURCES[line as usize].load(Ordering::Relaxed);
        if source != NO_SOURCE {
            return Some(InterruptEvent::Peripheral(source));
        }
    }
    None
}

/// Clears the tick interrupt at its source. The PLIC pending state for a
/// level-triggered input follows the source, so this is the whole job.
pub(super) fn ack_tick() {
    systimer()
        .int_clr()
        .write(|w| w.target0().clear_bit_by_one());
}

// Register semantics below are from public esp-idf components/soc/esp32c6
// (register/soc/systimer_reg.h, pcr_reg.h, hp_apm_reg.h,
// interrupt_matrix_reg.h) and include/soc/plic_reg.h.

/// Bus-level permission filtering: the APM controller decides what each
/// security mode may touch, per address range, and the CPU is TEE in
/// M-mode and REE0 in U-mode. At reset region 0 spans the whole address
/// space with every REE permission clear, so a U-mode instruction fetch
/// returns zeros and faults as an illegal instruction with mtval = 0 --
/// not as an access fault. `init` opens both blocks for REE0; per-task
/// isolation is the PMP's. Public esp-idf `components/soc/esp32c6`
/// (`hp_apm_reg.h`, `lp_apm_reg.h`, `reg_base.h`).
///
/// This is the CPU side only. Confining DMA masters -- the same block
/// identifies them -- is not part of this chip support.
mod apm {
    use super::{hp_apm, lp_apm};

    /// HP_APM and LP_APM share a block shape -- REGION_FILTER_EN, then per
    /// region START, END (inclusive) and ATTR with R{0,1,2}_{X,W,R} at
    /// bits 0..2, 4..6, 8..10 -- but the PAC gives each its own types, so
    /// the sequence is spelled once as a macro. `REE_MASK` spans all
    /// three REE fields and the reserved bit after each, as the mask
    /// always has.
    const REE_MASK: u32 = 0xfff;

    /// Region 0 of a block: everything, for the U-mode CPU only. The ATTR
    /// value is masked and merged, not overwritten: whatever the block
    /// keeps above the REE fields stays as reset.
    macro_rules! open_for_ree0 {
        ($apm:expr) => {{
            let apm = $apm;
            let region = apm.region(0);
            // Safety: the address registers are single 32-bit fields, so
            // any value is a legal write; the filter-enable write below
            // only adds region 0 to the bits already set.
            region
                .addr_start()
                .write(|w| unsafe { w.addr_start().bits(0) });
            region
                .addr_end()
                .write(|w| unsafe { w.addr_end().bits(u32::MAX) });
            region.pms_attr().modify(|r, w| {
                // Safety: clears only the REE fields and their reserved
                // neighbours; everything above is written back as read.
                unsafe { w.bits(r.bits() & !REE_MASK) };
                w.r0_pms_x()
                    .set_bit()
                    .r0_pms_w()
                    .set_bit()
                    .r0_pms_r()
                    .set_bit()
            });
            apm.region_filter_en().modify(|r, w| unsafe {
                w.region_filter_en().bits(r.region_filter_en().bits() | 1)
            });
        }};
    }

    pub(super) fn init() {
        open_for_ree0!(hp_apm());
        open_for_ree0!(lp_apm());
    }
}

// SYSTIMER clock gate (pcr_reg.h: PCR_SYSTIMER_CONF): CLK_EN clocks the
// register interface (default 1 on this chip), RST_EN requests reset. The
// counter itself free-runs out of reset (systimer_reg.h: UNIT0_WORK_EN,
// default 1) at 16 MHz -- XTAL/2.5, per esp_hw_support's
// `systimer_ticks_to_us`. The kernel tick uses the TARGET0 comparator,
// whose low 26 bits hold the period in systimer ticks.

/// Largest period the TARGET0 comparator's 26-bit field holds.
const SYSTIMER_TARGET_PERIOD_MASK: u32 = 0x03FF_FFFF;

/// The SYSTIMER_TARGET0 interrupt source number (soc/interrupts.h:
/// ETS_SYSTIMER_TARGET0_INTR_SOURCE = 57).
const TICK_SOURCE: u32 = 57;

/// CPU interrupt line reserved for the kernel tick. Line 0 is left
/// unused, matching esp-idf convention where CPU interrupt 0 is treated
/// as invalid by the interrupt allocator.
const TICK_LINE: u32 = 1;

// The Espressif "PLIC" (PLIC_MX -- the M-mode bank; a parallel UX bank
// exists for U-mode delivery, unused here). Flat 32-bit-per-32-lines
// registers: ENABLE, TYPE (1 = edge), CLEAR (edge acknowledge), EIP
// status (RO), one 4-bit priority word per line, THRESH (plic_reg.h).

/// Read-modify-writes the whole PLIC enable word.
fn plic_enable_modify(f: impl FnOnce(u32) -> u32) {
    // Safety: the enable field is the full word, one bit per line.
    plic_mx().mxint_enable().modify(|r, w| unsafe {
        w.cpu_mxint_enable().bits(f(r.cpu_mxint_enable().bits()))
    });
}

/// Routes `source` onto CPU `line` through the interrupt matrix and gives
/// the line priority 1, above the (zero) threshold.
fn route(source: u32, line: u32) {
    // Safety: `line` is a CPU line number, below 32, so it fits the 5-bit
    // map field; 1 fits the 4-bit priority field.
    intmtx()
        .core_0_intr_map(source as usize)
        .write(|w| unsafe { w.map().bits(line as u8) });
    plic_mx()
        .mxint_pri(line as usize)
        .write(|w| unsafe { w.cpu_mxint_pri().bits(1) });
}

// ---- Task peripheral interrupts ----
//
// Interrupt delivery is two-level: a peripheral interrupt *source* (one
// per peripheral event, 77 of them) is routed by the interrupt matrix onto
// one of 32 CPU interrupt *lines*, which the PLIC then gates and delivers.
//
// A Hubris `InterruptNum` -- the `irq = N` value in chip.toml -- is the
// **source number** (the `ETS_*_INTR_SOURCE` index from public esp-idf
// `interrupts.h`; the matrix map registers are laid out in exactly that
// order, which the PAC's `core_0_intr_map(source)` array follows). CPU
// lines are an internal resource the kernel assigns: line 0 is left
// unused, line 1 is the kernel tick, and lines 2..31 are claimed lazily,
// first touch, permanently for the boot, recorded in `IRQ_LINE_SOURCES`
// for reverse lookup at dispatch time. Running out is a configuration bug
// and panics.

/// Number of interrupt-matrix map registers (soc/interrupts.h:
/// ETS_MAX_INTR_SOURCE; also the length of the PAC's map array).
const INTMTX_SOURCE_COUNT: u32 = 77;

const CPU_INT_LINES: usize = 32;
const FIRST_TASK_LINE: u32 = 2;
/// CPU interrupt lines the PLIC will not enable: writes to these bits of
/// PLIC_MXINT_ENABLE are ignored (readback mask 0xffffff66, measured on
/// silicon). They are the standard RISC-V CLINT positions in `mip` --
/// 0 USIP, 3 MSIP, 4 UTIP, 7 MTIP -- which the core's CLINT
/// (DR_REG_CLINT_M_BASE, reg_base.h) drives directly rather than the
/// PLIC. A source routed onto one of them by the interrupt matrix
/// silently never fires; the allocator skips them.
const CLINT_LINES: u32 = 1 << 0 | 1 << 3 | 1 << 4 | 1 << 7;
const NO_SOURCE: u32 = u32::MAX;

/// Which source each CPU interrupt line carries; `NO_SOURCE` = unclaimed.
/// Only written from non-preemptible kernel code.
static IRQ_LINE_SOURCES: [core::sync::atomic::AtomicU32; CPU_INT_LINES] = {
    #[allow(clippy::declare_interior_mutable_const)]
    const NONE: core::sync::atomic::AtomicU32 =
        core::sync::atomic::AtomicU32::new(NO_SOURCE);
    [NONE; CPU_INT_LINES]
};

/// Returns the CPU line carrying `source`, claiming and routing a fresh
/// line on first touch.
fn line_for_source(source: u32) -> Result<u32, UsageError> {
    if source >= INTMTX_SOURCE_COUNT {
        return Err(UsageError::NoIrq);
    }
    for line in FIRST_TASK_LINE..CPU_INT_LINES as u32 {
        if CLINT_LINES & (1 << line) != 0 {
            continue;
        }
        let cur = IRQ_LINE_SOURCES[line as usize].load(Ordering::Relaxed);
        if cur == source {
            return Ok(line);
        }
        if cur == NO_SOURCE {
            IRQ_LINE_SOURCES[line as usize].store(source, Ordering::Relaxed);
            // Route the source onto its new line and give the line a
            // priority above the (zero) threshold. The line's enable bit
            // is still clear, so nothing fires yet.
            route(source, line);
            return Ok(line);
        }
    }
    panic!("out of CPU interrupt lines");
}

/// Enables interrupt `n` (a source number). `also_clear_pending` has
/// nothing to clear for a level-triggered line -- pending follows the
/// source -- and every line this kernel routes is level-triggered, so it
/// is accepted and ignored.
pub fn enable_irq(n: u32, _also_clear_pending: bool) -> Result<(), UsageError> {
    let line = line_for_source(n)?;
    mie_set(1 << line);
    plic_enable_modify(|en| en | (1 << line));
    Ok(())
}

/// Disables interrupt `n`.
pub fn disable_irq(
    n: u32,
    _also_clear_pending: bool,
) -> Result<(), UsageError> {
    let line = line_for_source(n)?;
    plic_enable_modify(|en| en & !(1 << line));
    mie_clear(1 << line);
    Ok(())
}

/// Reports whether interrupt `n` is enabled and/or pending.
pub fn irq_status(n: u32) -> Result<abi::IrqStatus, UsageError> {
    let line = line_for_source(n)?;
    let mut status = abi::IrqStatus::empty();
    status.set(
        abi::IrqStatus::ENABLED,
        plic_mx().mxint_enable().read().cpu_mxint_enable().bits() & (1 << line)
            != 0,
    );
    status.set(
        abi::IrqStatus::PENDING,
        plic_mx().emip_status().read().cpu_eip_status().bits() & (1 << line)
            != 0,
    );
    Ok(status)
}

/// Pends interrupt `n` in software -- which this controller cannot do: the
/// PLIC's pending state is read-only (EIP) for level inputs and its CLEAR
/// register only clears edges; there is no set side. Refuse rather than
/// silently drop, so a future user of `sys_irq_control`'s pend flag gets an
/// error to investigate instead of a lost interrupt.
pub fn pend_software_irq(
    InterruptNum(n): InterruptNum,
) -> Result<(), UsageError> {
    let _ = line_for_source(n)?;
    Err(UsageError::NoIrq)
}

/// Resets the kernel's core. Never returns.
///
/// The CPU core software reset in LP_AON (lp_aon_reg.h: CPUCORE0_CFG,
/// SW_RESET) -- the same primitive esp-idf's esp_restart path uses on this
/// chip (hal/cpu_utility_ll.h). The core restarts into the ROM
/// bootloader. Peripherals are not reset; state a fresh boot doesn't
/// reprogram survives. An LP-watchdog-forced full-system reset is the
/// upgrade path if this proves insufficient.
pub fn reset() -> ! {
    // Set the reset bit and keep the rest of the register, as esp-idf's
    // `cpu_utility_ll_reset_cpu` does. The register also holds
    // STAT_VECTOR_SEL (reset value 1); writing the reset bit alone clears
    // it, and the core then does not come back up in the ROM.
    lp_aon()
        .cpucore0_cfg()
        .modify(|_, w| w.cpu_core0_sw_reset().set_bit());

    // The reset takes effect asynchronously; wait for it.
    loop {
        core::hint::spin_loop();
    }
}
