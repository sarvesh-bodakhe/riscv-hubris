// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Architecture support for 32-bit RISC-V.
//!
//! # Status
//!
//! This is a **skeleton**. It defines every name the portable kernel expects
//! from `arch::`, so that the kernel compiles for a `riscv32` target, but the
//! bodies are unimplemented. Nothing here has run on hardware.
//!
//! The intent is to establish the seam as a separate, reviewable step from
//! filling it in.
//!
//! # How this differs from `arm_m`
//!
//! Three differences drive most of the design:
//!
//! 1. **No hardware exception frame.** ARMv?-M pushes r0-r3, r12, lr, pc and
//!    xPSR onto the task stack automatically on exception entry, so
//!    `SavedState` only has to cover the *other* half of the register file.
//!    RISC-V pushes nothing: the trap handler saves and restores every
//!    register it touches. So `SavedState` here holds the whole integer file.
//!
//! 2. **A scratch register exists.** `arm_m` keeps the current task pointer in
//!    a global (`CURRENT_TASK_PTR`) because it has no register to spare on
//!    exception entry. RISC-V has `mscratch` for exactly this purpose. We still
//!    keep a global for now, for parity and for debugger visibility, but the
//!    trap handler will use `mscratch`.
//!
//! 3. **PMP, not MPU.** Regions are described by `pmpaddr`/`pmpcfg` pairs and
//!    encoded NAPOT (naturally-aligned power of two) rather than by an
//!    MPU-style base/size register pair.
//!
//! # Register conventions
//!
//! Syscall arguments and returns have to live in *some* registers, and the
//! choice is ours as long as the kernel and `sys/userlib`'s stubs agree. We use
//! the argument registers, which is what a RISC-V programmer will expect:
//!
//! | Slot | Register |
//! |---|---|
//! | `arg0` .. `arg6` | `a0` .. `a6` (`x10` .. `x16`) |
//! | syscall number | `a7` (`x17`) |
//! | `ret0` .. `ret5` | `a0` .. `a5` (`x10` .. `x15`) |
//!
//! Note this differs from `arm_m`, which uses `r4`-`r11` -- those are
//! callee-saved on ARM, which mattered there for exception-frame reasons that
//! do not apply to us.

use crate::descs::RegionAttributes;
use crate::task;
use crate::time::Timestamp;
use abi::{InterruptNum, UsageError};

/// Assertion macro used by portable kernel code (`umem.rs`) as well as by the
/// arch modules.
///
/// This lives in the arch module, rather than somewhere portable, because
/// `arch.rs` imports it with `#[macro_use]`. Any new arch backend has to
/// provide it. (Yes, this is a slightly odd place for it.)
macro_rules! uassert {
    ($cond : expr) => {
        if !$cond {
            panic!("Assertion failed!");
        }
    };
}

/// RISC-V integer registers that must be saved across a context switch.
///
/// Unlike ARMv?-M, the hardware saves nothing on trap entry, so this covers the
/// entire integer file except `x0` (hardwired zero, nothing to save).
///
/// Fields are in exact `x1`..`x31` order and must stay that way: the trap
/// handler spills with a mechanical `sw xN, (N-1)*4(sp)` sweep, so reordering
/// them for readability would silently corrupt every context switch.
///
/// # UNRESOLVED: coprocessor state is not saved here
///
/// A RISC-V core may carry state beyond the integer file: the F and D
/// extensions' floating-point registers, the V extension's vector state, or
/// vendor-specific units. None of it is in this struct. Two tasks using any
/// of it would corrupt each other silently.
///
/// So one of the following must be made true. Which one is an open decision:
///
/// 1. Tasks never touch the coprocessors -- soft-float ABI, no vector or
///    vendor-unit instructions -- enforced by the `-march`/`-mabi` the build
///    system passes, *not* by this comment. Then they hold no task state and
///    this struct is correct as written. This is the Hubris-shaped answer
///    (static, no per-task allocation), but it is not implemented or enforced
///    anywhere yet.
/// 2. This struct grows to cover them, and every context switch pays the cost
///    unconditionally.
/// 3. Lazy save on first use: leave `mstatus.FS`/`mstatus.VS` Off, take the
///    illegal-instruction trap on a task's first use, and save state only
///    for tasks that touch it. A core whose Off state fails to trap some
///    instruction would leave a hole in that detection.
///
/// # Note on `mstatus`/`mcause`
///
/// Deliberately absent, and this is not an oversight. The kernel is
/// non-preemptible (`mstatus.MIE` stays 0 for the whole trap handler and is
/// never re-enabled inside it), so kernel entries never nest and every
/// return is to U-mode. On a core with a CLIC, that also makes the previous
/// interrupt level in `mcause` 0 on every return path; `mret` restores it
/// in hardware. The kernel reads `mcause` to decode the trap cause and must
/// never write it.
#[repr(C)]
#[derive(Debug, Default)]
pub struct SavedState {
    // NOTE: the following fields must be kept contiguous and in register order;
    // the trap handler indexes them by offset.
    pub ra: u32,  // x1
    pub sp: u32,  // x2
    pub gp: u32,  // x3
    pub tp: u32,  // x4
    pub t0: u32,  // x5
    pub t1: u32,  // x6
    pub t2: u32,  // x7
    pub s0: u32,  // x8  (also fp)
    pub s1: u32,  // x9
    pub a0: u32,  // x10
    pub a1: u32,  // x11
    pub a2: u32,  // x12
    pub a3: u32,  // x13
    pub a4: u32,  // x14
    pub a5: u32,  // x15
    pub a6: u32,  // x16
    pub a7: u32,  // x17
    pub s2: u32,  // x18
    pub s3: u32,  // x19
    pub s4: u32,  // x20
    pub s5: u32,  // x21
    pub s6: u32,  // x22
    pub s7: u32,  // x23
    pub s8: u32,  // x24
    pub s9: u32,  // x25
    pub s10: u32, // x26
    pub s11: u32, // x27
    pub t3: u32,  // x28
    pub t4: u32,  // x29
    pub t5: u32,  // x30
    pub t6: u32,  // x31
    // NOTE: the above fields must be kept contiguous.
    /// Task PC, taken from `mepc` on trap entry and restored to it on exit.
    pub pc: u32,
}

/// Map the argument registers to (architecture-independent) syscall argument
/// and return slots. See the register-convention table above.
impl task::ArchState for SavedState {
    fn stack_pointer(&self) -> u32 {
        self.sp
    }

    fn arg0(&self) -> u32 {
        self.a0
    }
    fn arg1(&self) -> u32 {
        self.a1
    }
    fn arg2(&self) -> u32 {
        self.a2
    }
    fn arg3(&self) -> u32 {
        self.a3
    }
    fn arg4(&self) -> u32 {
        self.a4
    }
    fn arg5(&self) -> u32 {
        self.a5
    }
    fn arg6(&self) -> u32 {
        self.a6
    }

    fn syscall_descriptor(&self) -> u32 {
        self.a7
    }

    fn ret0(&mut self, x: u32) {
        self.a0 = x
    }
    fn ret1(&mut self, x: u32) {
        self.a1 = x
    }
    fn ret2(&mut self, x: u32) {
        self.a2 = x
    }
    fn ret3(&mut self, x: u32) {
        self.a3 = x
    }
    fn ret4(&mut self, x: u32) {
        self.a4 = x
    }
    fn ret5(&mut self, x: u32) {
        self.a5 = x
    }
}

/// Precomputed PMP register values for one memory region.
///
/// Hubris computes protection register contents at *compile* time (see
/// `compute_region_extension_data`) so that a context switch is a handful of
/// register writes rather than a bit-twiddling exercise. On RISC-V that means
/// a `pmpaddr` value and the matching byte of `pmpcfg`.
#[derive(Copy, Clone, Debug)]
#[repr(C)]
pub struct RegionDescExt {
    /// Value to load into the region's `pmpaddrN` register. For a NAPOT region
    /// this encodes both base and size.
    pub pmpaddr: u32,
    /// Configuration byte for this region: R/W/X bits plus the address-matching
    /// mode. Held as a `u32` for alignment convenience.
    pub pmpcfg: u32,
}

/// Precomputes the PMP register values for a region, at compile time.
///
/// TODO: unimplemented. Returns a deny-all placeholder.
///
/// This must stay a `const fn`: the build system generates a static task table
/// containing the results, so this is evaluated by the compiler, not at run
/// time. That also means it cannot panic on bad input in the final version
/// without failing the build -- which is, in fact, the desired behaviour for a
/// misaligned region.
///
/// The real implementation encodes NAPOT: for a region of size `2^n` at a
/// naturally aligned base, `pmpaddr = (base >> 2) | ((1 << (n - 3)) - 1)`, with
/// `pmpcfg.A = NAPOT`. The smallest region the PMP expresses is its
/// granularity, `2^(G+2)` bytes, which the privileged spec leaves to the
/// implementation.
pub const fn compute_region_extension_data(
    _base: u32,
    _size: u32,
    _attributes: RegionAttributes,
) -> RegionDescExt {
    RegionDescExt {
        pmpaddr: 0,
        pmpcfg: 0,
    }
}

/// Reprograms the PMP for `task`, called on every context switch.
///
/// TODO: unimplemented.
///
/// Note for the implementation: PMP entries are checked in priority order,
/// lowest index first, and an access that matches only some of its bytes
/// against an entry fails outright (privileged spec, PMP chapter). Region
/// layout has to respect that for accesses that straddle a boundary.
pub fn apply_memory_protection(_task: &task::Task) {
    todo!("PMP programming")
}

/// Resets a task's saved state so it will start from its entry point.
///
/// TODO: unimplemented.
pub fn reinitialize(_task: &mut task::Task) {
    todo!("task reinitialization")
}

/// Starts the kernel tick and drops into the first task. Never returns.
///
/// TODO: unimplemented.
pub fn start_first_task(_tick_divisor: u32, _task: &task::Task) -> ! {
    todo!("first task entry")
}

/// Records which task is currently running, for the trap handler's benefit.
///
/// TODO: unimplemented. Will write `mscratch`.
///
/// # Safety
///
/// Caller must ensure `task` points into the live task table.
pub unsafe fn set_current_task(_task: &task::Task) {
    todo!("current task pointer")
}

/// Reads the kernel tick counter.
///
/// TODO: unimplemented.
///
/// Note the privileged spec mandates no timer: `mtime`/`mtimecmp` exist on
/// many cores but not all, and some provide timekeeping only through a
/// vendor peripheral routed as an ordinary external interrupt. Where the
/// tick comes from is a per-chip decision, not an ISA one.
pub fn now() -> Timestamp {
    todo!("tick counter")
}

/// Records the clock frequency, in kHz, for the tick.
///
/// TODO: unimplemented.
///
/// # Safety
///
/// Caller must ensure this is called before the tick is started.
pub unsafe fn set_clock_freq(_tick_divisor: u32) {
    todo!("clock frequency")
}

/// Enables interrupt `n`, optionally clearing any pending instance first.
///
/// TODO: unimplemented. The interrupt controller is implementation-chosen:
/// CLINT, PLIC, CLIC or vendor-specific.
pub fn enable_irq(
    _n: u32,
    _also_clear_pending: bool,
) -> Result<(), UsageError> {
    todo!("interrupt enable")
}

/// Disables interrupt `n`, optionally clearing any pending instance.
///
/// TODO: unimplemented.
pub fn disable_irq(
    _n: u32,
    _also_clear_pending: bool,
) -> Result<(), UsageError> {
    todo!("interrupt disable")
}

/// Reports whether interrupt `n` is enabled, pending, and/or posted.
///
/// TODO: unimplemented.
pub fn irq_status(_n: u32) -> Result<abi::IrqStatus, UsageError> {
    todo!("interrupt status")
}

/// Pends interrupt `n` in software.
///
/// TODO: unimplemented.
pub fn pend_software_irq(_n: InterruptNum) -> Result<(), UsageError> {
    todo!("interrupt software pend")
}

/// Resets the chip. Never returns.
///
/// TODO: unimplemented.
pub fn reset() -> ! {
    todo!("system reset")
}
