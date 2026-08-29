// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Architecture support for 32-bit RISC-V.
//!
//! # Layout: ISA vs chip
//!
//! The RISC-V privileged architecture standardizes far less of a working
//! system than ARMv?-M does. A Cortex-M part comes with its vector table,
//! NVIC, SysTick and MPU defined by the architecture; a RISC-V
//! implementation chooses its own interrupt fabric (CLINT, PLIC, CLIC, or
//! something vendor-specific), its own timer, and its own reset and
//! bus-protection machinery. So this backend is split in two:
//!
//! - **this module**: everything the ISA and the privileged spec pin down --
//!   the register file and trap entry/exit, syscall and fault dispatch, PMP
//!   programming, task initialization, timekeeping, and the scheduling glue;
//! - **`riscv32/chip/<name>.rs`**: everything the implementation chose -- the
//!   tick timer, interrupt routing and control, any bus-level access gate,
//!   the coprocessor set, and reset.
//!
//! Exactly one chip module is selected by a kernel Cargo feature, named in
//! the app.toml (`[kernel] features = ["esp32c6"]`). Like the arch modules
//! themselves (see `arch.rs`), chip modules are duck-typed rather than
//! implementing a trait: each must export the same set of names. The
//! required surface is:
//!
//! | Name | Kind | Contract |
//! |---|---|---|
//! | `PMP_ENTRIES` | `const usize` | PMP entries the core implements and the kernel programs per task: 8 or 16. Must match the `[pmp] entries` the chip description gives the build (checked at compile time) |
//! | `PMP_GRANULARITY` | `const u32` | smallest NAPOT region the PMP expresses exactly; smaller access-granting regions are rejected at build time |
//! | `init(tick_divisor)` | `fn` | one-shot hardware init before the first task: bus gates, coprocessor disable, tick timer, interrupt routing. Called with `mstatus.MIE` clear and `mtvec` already installed, so it may unmask interrupt sources freely. |
//! | `enable_irq` / `disable_irq` / `irq_status` / `pend_software_irq` | `fn` | the kernel's IRQ interface, in Hubris `InterruptNum` numbering |
//! | `reset()` | `fn -> !` | restart the system |
//!
//! One contract item deserves emphasis: `SavedState` below holds only the
//! integer register file, so `chip::init` **must disable every coprocessor**
//! whose state a task could otherwise touch (FPU, vendor units). State that
//! is accessible but not context-switched is silently shared between tasks.
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
//!    keep a global, for parity and for debugger visibility, but the trap
//!    handler uses `mscratch`.
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

use core::ptr::null_mut;
use core::sync::atomic::{AtomicBool, AtomicPtr, Ordering};

use crate::atomic::AtomicExt;
use crate::descs::RegionAttributes;
use crate::startup::with_task_table;
use crate::task;
use crate::time::Timestamp;
use crate::umem::USlice;
use abi::{FaultInfo, FaultSource};
use unwrap_lite::UnwrapLite;

// The kernel requires an atomic swap operation, abstracted behind
// `crate::atomic::AtomicExt` because not every supported CPU has one in
// hardware: ARMv6-M doesn't, and its arch module substitutes an
// interrupt-masking load/store pair (a "polyfill").
//
// On RISC-V, atomic read-modify-write instructions come from the "A"
// (atomic) ISA extension. When it is present, Rust's native
// `AtomicBool::swap` compiles to a single `amoswap` instruction and the
// polyfill below is just a direct call to it. When it is absent, libcore
// does not offer `swap` at all, and this module would need an
// ARMv6-M-style interrupt-masking implementation instead.
//
// The kernel builds for the `riscv32imac` target, so A is available.
cfg_if::cfg_if! {
    if #[cfg(target_feature = "a")] {
        impl AtomicExt for AtomicBool {
            type Primitive = bool;

            #[inline(always)]
            fn swap_polyfill(
                &self,
                value: Self::Primitive,
                ordering: Ordering,
            ) -> Self::Primitive {
                self.swap(value, ordering)
            }
        }
    } else {
        compile_error!(
            "this RISC-V target lacks the A (atomic) extension; \
             AtomicExt needs an interrupt-masking polyfill here, \
             like the ARMv6-M one in arch/arm_m.rs"
        );
    }
}

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

/// Reads a CSR by name, e.g. `read_csr!("mcause")`.
macro_rules! read_csr {
    ($name:literal) => {{
        let value: u32;
        // Safety: reading these M-mode status CSRs has no side effects.
        unsafe {
            core::arch::asm!(
                concat!("csrr {}, ", $name),
                out(reg) value,
                options(nomem, nostack, preserves_flags),
            );
        }
        value
    }};
}

// Chip support module selection; see the module docs for the surface each
// chip module must export. NOTE: the macros above are visible to the chip
// module because they are defined before it -- keep it that way.
cfg_if::cfg_if! {
    if #[cfg(feature = "esp32c6")] {
        // Note the path is relative to this file's directory (arch/), not
        // the module's child directory (arch/riscv32/): a #[path] inside a
        // macro-generated block -- cfg_if! here -- resolves against the file.
        #[path = "riscv32/chip/esp32c6.rs"]
        mod chip;
    } else {
        compile_error!(
            "building for riscv32 requires a chip support module; \
             name its feature in the app.toml, e.g. \
             [kernel] features = [\"esp32c6\"]"
        );
    }
}

// The chip module implements these directly; they are part of the arch
// interface the portable kernel consumes.
pub use chip::{disable_irq, enable_irq, irq_status, pend_software_irq, reset};

/// RISC-V integer registers that must be saved across a context switch.
///
/// Unlike ARMv?-M, the hardware saves nothing on trap entry, so this covers the
/// entire integer file except `x0` (hardwired zero, nothing to save).
///
/// Fields are in exact `x1`..`x31` order and must stay that way: the trap
/// handler spills with a mechanical `sw xN, (N-1)*4(sp)` sweep, so reordering
/// them for readability would silently corrupt every context switch.
///
/// # Coprocessor state is deliberately absent
///
/// Only the integer file is saved. Any coprocessor state a task could reach
/// (FPU registers, vendor-unit state) would be silently shared across
/// context switches, so the chip layer is required to disable every
/// coprocessor before the first task runs -- see the chip surface contract
/// in the module docs. (Lazy, trap-on-first-use switching was considered
/// and rejected: it needs a per-task coprocessor save area the kernel
/// does not have, for a feature no task in this tree uses.)
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
/// This must stay a `const fn`: the build system generates a static task table
/// containing the results, so this is evaluated by the compiler, not at run
/// time. A `panic!` here is therefore a *build* failure, which is exactly what
/// we want for a region the PMP cannot represent.
///
/// Encoding is NAPOT (naturally-aligned power of two): for a region of size
/// `2^n` at a naturally aligned base, `pmpaddr = (base >> 2) | ((size >> 3) -
/// 1)` and `pmpcfg.A = NAPOT`. See the PMP chapter of the privileged spec.
///
/// PMP granularity is implementation-chosen, so it comes from the chip module
/// (`chip::PMP_GRANULARITY`): for a NAPOT region smaller than that, the
/// hardwired low `pmpaddr` bits silently grow it to the granularity. That is
/// fine for a region granting *no* access (the 32-byte null region -- growing
/// a deny region only denies more), but would be a silent protection hole
/// for any region granting access, so we reject those.
pub const fn compute_region_extension_data(
    base: u32,
    size: u32,
    attributes: RegionAttributes,
) -> RegionDescExt {
    if size < 4 || !size.is_power_of_two() {
        panic!("PMP regions must be power-of-two sized, >= 4 bytes");
    }
    if !base.is_multiple_of(size) {
        panic!("PMP regions must be naturally aligned");
    }

    let r = attributes.contains(RegionAttributes::READ);
    let w = attributes.contains(RegionAttributes::WRITE);
    let x = attributes.contains(RegionAttributes::EXECUTE);

    if size < chip::PMP_GRANULARITY && (r || w || x) {
        panic!("region smaller than PMP granularity grants access");
    }
    // The R=0, W=1 combination is reserved by the PMP spec (absent Smepmp
    // rules we don't use).
    if w && !r {
        panic!("PMP cannot express write-without-read");
    }

    // DEVICE and DMA select memory type and cache attributes on the ARM
    // MPU. The PMP has no equivalent, and neither role grants access, so
    // both are ignored here.

    // A 4-byte region has no NAPOT encoding (the smallest is 8 bytes);
    // the spec gives it its own mode, NA4, with pmpaddr the bare address.
    // Both are usable only when the granularity is 4 (chips with G > 0
    // hardwire the low pmpaddr bits and NA4 reads back as OFF), which the
    // granularity check above already guarantees for a 4-byte grant.
    let (pmpaddr, mode) = if size == 4 {
        (base >> 2, 0b10) // A = NA4
    } else {
        ((base >> 2) | ((size >> 3) - 1), 0b11) // A = NAPOT
    };
    let mut pmpcfg: u32 = mode << 3; // L = 0 (never locked)
    if r {
        pmpcfg |= 1 << 0;
    }
    if w {
        pmpcfg |= 1 << 1;
    }
    if x {
        pmpcfg |= 1 << 2;
    }

    RegionDescExt { pmpaddr, pmpcfg }
}

/// Regions per task: every PMP entry the core implements.
///
/// The privileged spec defines 0, 16 or 64 entries architecturally, but an
/// implementation exposes what it has and hardwires the rest to zero, and
/// eight is common in small cores (the same budget as the Cortex-M MPU).
/// The chip module states its count; the build system takes the same
/// number from the chip description and `descs.rs` checks the two agree.
pub const REGIONS_PER_TASK: usize = chip::PMP_ENTRIES;

const _: () = assert!(
    chip::PMP_ENTRIES == 8 || chip::PMP_ENTRIES == 16,
    "the riscv32 backend programs 8 or 16 PMP entries"
);

/// Reprograms the PMP for `task`, called on every context switch.
///
/// A task's regions map one-to-one onto PMP entries `0..PMP_ENTRIES`,
/// precomputed at build time (see [`compute_region_extension_data`]).
/// Entries are never locked (L=0), and unlocked PMP entries do not apply
/// to M-mode at all -- so the kernel is unaffected by whatever is
/// programmed here, and no enable/disable dance is needed around the
/// update. U-mode is not running while we're in here, so transient states
/// are unobservable.
pub fn apply_memory_protection(task: &task::Task) {
    // Sized for the larger configuration; on an 8-entry core the upper
    // half stays zero and is never written.
    let mut pmpcfg = [0u32; 4];
    let mut pmpaddr = [0u32; 16];
    for (i, region) in task.region_table().iter().enumerate() {
        let ext = &region.arch_data;
        pmpaddr[i] = ext.pmpaddr;
        pmpcfg[i / 4] |= (ext.pmpcfg & 0xFF) << ((i % 4) * 8);
    }

    // Safety: writing unlocked PMP entries has no effect on M-mode
    // execution; the worst a bad value can do is deny or grant U-mode
    // access, which is a correctness bug, not a memory-safety violation in
    // the kernel.
    unsafe {
        core::arch::asm!("
            csrw pmpaddr0, {addr0}
            csrw pmpaddr1, {addr1}
            csrw pmpaddr2, {addr2}
            csrw pmpaddr3, {addr3}
            csrw pmpaddr4, {addr4}
            csrw pmpaddr5, {addr5}
            csrw pmpaddr6, {addr6}
            csrw pmpaddr7, {addr7}
            csrw pmpcfg0, {cfg0}
            csrw pmpcfg1, {cfg1}
            ",
            addr0 = in(reg) pmpaddr[0],
            addr1 = in(reg) pmpaddr[1],
            addr2 = in(reg) pmpaddr[2],
            addr3 = in(reg) pmpaddr[3],
            addr4 = in(reg) pmpaddr[4],
            addr5 = in(reg) pmpaddr[5],
            addr6 = in(reg) pmpaddr[6],
            addr7 = in(reg) pmpaddr[7],
            cfg0 = in(reg) pmpcfg[0],
            cfg1 = in(reg) pmpcfg[1],
            options(nostack, preserves_flags),
        );
        // The upper eight exist only on a 16-entry core; on an 8-entry
        // core the CSRs may be hardwired to zero or absent entirely, and
        // writing them is at best pointless. Constant condition: the
        // branch folds away.
        if chip::PMP_ENTRIES == 16 {
            core::arch::asm!("
                csrw pmpaddr8, {addr8}
                csrw pmpaddr9, {addr9}
                csrw pmpaddr10, {addr10}
                csrw pmpaddr11, {addr11}
                csrw pmpaddr12, {addr12}
                csrw pmpaddr13, {addr13}
                csrw pmpaddr14, {addr14}
                csrw pmpaddr15, {addr15}
                csrw pmpcfg2, {cfg2}
                csrw pmpcfg3, {cfg3}
                ",
                addr8 = in(reg) pmpaddr[8],
                addr9 = in(reg) pmpaddr[9],
                addr10 = in(reg) pmpaddr[10],
                addr11 = in(reg) pmpaddr[11],
                addr12 = in(reg) pmpaddr[12],
                addr13 = in(reg) pmpaddr[13],
                addr14 = in(reg) pmpaddr[14],
                addr15 = in(reg) pmpaddr[15],
                cfg2 = in(reg) pmpcfg[2],
                cfg3 = in(reg) pmpcfg[3],
                options(nostack, preserves_flags),
            );
        }
    }
}

/// Resets a task's saved state so it will start from its entry point.
///
/// Much simpler than the ARM version: RISC-V has no hardware exception
/// frame, so there is nothing to fabricate on the task stack -- the entire
/// initial state lives in `SavedState` and is applied by the trap-return
/// path.
pub fn reinitialize(task: &mut task::Task) {
    *task.save_mut() = SavedState::default();
    let initial_stack = task.descriptor().initial_stack;

    // The RISC-V psABI requires 16-byte stack alignment at call boundaries.
    uassert!(initial_stack & 0xF == 0);

    task.save_mut().pc = task.descriptor().entry_point;
    task.save_mut().sp = initial_stack;

    // Paint the stack with a distinct pattern, for the benefit of stack
    // usage measurement (humility stackmargin); same value as arm_m. Start
    // from the region holding the stack's top word -- one word below the
    // initial stack pointer; a zero stack pointer saturates to an address
    // in no region, which skips the paint -- and paint from its base up to
    // the stack pointer. There is no exception frame to leave room for.
    if let Some((index, mut region)) = task
        .region_table()
        .iter()
        .copied()
        .enumerate()
        .find(|(_, r)| r.contains((initial_stack as usize).saturating_sub(4)))
    {
        // The stack may span several contiguous regions (the build chunks
        // a non-power-of-two allocation); walk back through the sorted
        // table to the first one.
        let mut okay = true;
        for prev in task.region_table()[..index].iter().rev() {
            // A descriptor that overflows a u32 means a corrupt table.
            let Some(prev_end) = prev.base.checked_add(prev.size) else {
                okay = false;
                break;
            };
            if prev_end != region.base {
                break;
            }
            region = *prev;
        }

        // This is a diagnostic: if the slice does not fit, skip the paint
        // rather than take the system down.
        if okay
            && let Some(len) =
                (initial_stack as usize).checked_sub(region.base as usize)
            && let Ok(mut uslice) =
                USlice::<u32>::from_raw(region.base as usize, len >> 2)
        {
            // Unwrap rather than tolerate failure: try_write failing would
            // mean the task's stack isn't writable by the task, which would
            // bite us later anyway.
            let zap = task.try_write(&mut uslice).unwrap_lite();
            for word in zap.iter_mut() {
                *word = 0xbaddcafe;
            }
        }
    }
}

/// Starts the kernel tick and the first task. Never returns.
///
/// `tick_divisor` is the number of tick-timer input ticks per kernel tick
/// (1 ms); the app supplies it, since the timer's input frequency is a
/// board/clock configuration question.
pub fn start_first_task(tick_divisor: u32, task: &task::Task) -> ! {
    // Take ownership of the trap path before enabling anything that can
    // raise an interrupt. Two ordering rules, both learned the hard way:
    // interrupt sources must not be enabled while mtvec still points at
    // whatever ran before us, and the kernel's non-preemptibility must be
    // established rather than assumed -- mstatus.MIE is not reliably 0
    // when the boot ROM hands over, so an already-pending source fires
    // the instant it is unmasked.
    unsafe {
        // Name the first task before the trap vector can be reached. The
        // trap entry spills the register file through mscratch before it
        // can tell where the trap came from, so mscratch must point at a
        // save area from the moment mtvec is ours. A kernel fault in the
        // chip's init is then reported as what it is; through whatever
        // the boot ROM left in mscratch, the spill itself may fault and
        // be reported in its place.
        //
        // Safety: `task` points into the live task table per our contract.
        set_current_task(task);

        // Kernel runs with M-mode interrupts off, forever. Tasks get
        // them because interrupts to a higher privilege mode are always
        // enabled while running in a lower one.
        core::arch::asm!(
            "csrci mstatus, 8", // clear MIE
            options(nostack, preserves_flags),
        );

        // Install the trap vector, direct mode (the low two bits of mtvec
        // are zero because _hubris_trap_entry is 4-byte aligned and mode
        // Direct is encoding 0): every trap lands at the same entry point.
        core::arch::asm!(
            "csrw mtvec, {}",
            in(reg) _hubris_trap_entry as usize,
            options(nostack, preserves_flags),
        );
    }

    // Chip-level hardware init: bus filters opened for U-mode, coprocessors
    // disabled, the tick timer programmed and routed. Runs with MIE clear
    // and our mtvec installed (see above), so it may unmask sources freely.
    chip::init(tick_divisor);

    unsafe {
        // Enter the task through the same register-restore path every trap
        // exit uses: mstatus.MPP is forced to U and MPIE to 1 there, so the
        // mret at its end is a drop into U-mode at the task's saved pc.
        // Global interrupts in M-mode (mstatus.MIE) remain 0 forever -- the
        // kernel is not preemptible.
        _hubris_task_return()
    }
}

/// Records which task is currently running, for the trap handler's benefit.
///
/// The trap handler finds the current task through `mscratch`. Because
/// `Task` is `repr(C)` with `save: SavedState` as its first field (the same
/// layout contract arm_m's assembly relies on), the task pointer doubles as
/// the pointer to its register save area.
///
/// The `CURRENT_TASK_PTR` static mirrors mscratch for the benefit of
/// debuggers (Humility reads it by symbol) and matches arm_m.
///
/// # Safety
///
/// Caller must ensure `task` points into the live task table, and must not
/// hold other references into it when the trap handler could run.
pub unsafe fn set_current_task(task: &task::Task) {
    CURRENT_TASK_PTR.store(task as *const _ as *mut _, Ordering::Relaxed);
    crate::profiling::event_context_switch(task as *const _ as usize);
    // Safety: writing mscratch has no side effect other than changing what
    // the next trap entry uses as its spill base.
    unsafe {
        core::arch::asm!(
            "csrw mscratch, {}",
            in(reg) task as *const task::Task,
            options(nostack, preserves_flags),
        );
    }
}

/// Mirror of `mscratch` for debugger consumption; see [`set_current_task`].
#[unsafe(no_mangle)]
static CURRENT_TASK_PTR: AtomicPtr<task::Task> = AtomicPtr::new(null_mut());

unsafe extern "C" {
    /// Trap entry point; only ever entered by the hardware via mtvec.
    fn _hubris_trap_entry();
    /// Register-restore path: resumes the task named by `mscratch`. Entered
    /// by falling out of the trap handler, or directly by
    /// [`start_first_task`].
    fn _hubris_task_return() -> !;
}

// The trap entry/exit path.
//
// What ARMvX-M did in hardware happens here in instructions: nothing is
// saved automatically, and the handler starts with the *task's* registers
// live -- including sp, which is task-controlled and must not be used. The
// escape hatch is mscratch, which holds the current Task pointer (== the
// SavedState pointer, offset 0): `csrrw` swaps it with sp atomically,
// giving us a trusted spill base without clobbering any task register.
//
// SavedState field offsets are load-bearing here (x1..x31 in order, pc at
// 124); the const assertions below pin them.
//
// The kernel is not preemptible: mstatus.MIE stays 0 for the entire time
// we are in M-mode (trap entry clears it; we never set it), so entries
// never nest and the kernel stack can start fresh at _stack_start on every
// entry.
core::arch::global_asm!(
    "
    .section .text.hubris_trap_entry
    .balign 4
    .global _hubris_trap_entry
    .global _hubris_task_return
_hubris_trap_entry:
    # sp <-> mscratch: sp now points at the current task's SavedState;
    # the task's sp is parked in mscratch.
    csrrw sp, mscratch, sp

    # Spill the integer file in SavedState field order, except sp itself.
    sw x1, 0(sp)
    sw x3, 8(sp)
    sw x4, 12(sp)
    sw x5, 16(sp)
    sw x6, 20(sp)
    sw x7, 24(sp)
    sw x8, 28(sp)
    sw x9, 32(sp)
    sw x10, 36(sp)
    sw x11, 40(sp)
    sw x12, 44(sp)
    sw x13, 48(sp)
    sw x14, 52(sp)
    sw x15, 56(sp)
    sw x16, 60(sp)
    sw x17, 64(sp)
    sw x18, 68(sp)
    sw x19, 72(sp)
    sw x20, 76(sp)
    sw x21, 80(sp)
    sw x22, 84(sp)
    sw x23, 88(sp)
    sw x24, 92(sp)
    sw x25, 96(sp)
    sw x26, 100(sp)
    sw x27, 104(sp)
    sw x28, 108(sp)
    sw x29, 112(sp)
    sw x30, 116(sp)
    sw x31, 120(sp)

    # Second swap: retrieve the task's sp (parking the Task pointer back
    # in mscratch, where the next trap needs it) and finish the save.
    csrrw t0, mscratch, sp
    sw t0, 4(sp)
    csrr t1, mepc
    sw t1, 124(sp)

    # Into Rust: a0 = task pointer, on a fresh kernel stack.
    mv a0, sp
    la sp, _stack_start
    call _hubris_trap_dispatch

    # Fall through: resume whichever task mscratch now names.
_hubris_task_return:
    csrr t0, mscratch

    # Return to U-mode with interrupts enabled there: MPP <- 00, MPIE <- 1.
    li t1, 0x1800
    csrc mstatus, t1
    li t1, 0x80
    csrs mstatus, t1

    lw t1, 124(t0)
    csrw mepc, t1

    # Restore everything except sp and t0 (x5), which we need as the base.
    lw x1, 0(t0)
    lw x3, 8(t0)
    lw x4, 12(t0)
    lw x6, 20(t0)
    lw x7, 24(t0)
    lw x8, 28(t0)
    lw x9, 32(t0)
    lw x10, 36(t0)
    lw x11, 40(t0)
    lw x12, 44(t0)
    lw x13, 48(t0)
    lw x14, 52(t0)
    lw x15, 56(t0)
    lw x16, 60(t0)
    lw x17, 64(t0)
    lw x18, 68(t0)
    lw x19, 72(t0)
    lw x20, 76(t0)
    lw x21, 80(t0)
    lw x22, 84(t0)
    lw x23, 88(t0)
    lw x24, 92(t0)
    lw x25, 96(t0)
    lw x26, 100(t0)
    lw x27, 104(t0)
    lw x28, 108(t0)
    lw x29, 112(t0)
    lw x30, 116(t0)
    lw x31, 120(t0)
    lw x2, 4(t0)
    # t0 last, through itself.
    lw x5, 16(x5)

    mret
    "
);

// Pin the SavedState offsets the assembly above spills to.
const _: () = {
    assert!(core::mem::offset_of!(SavedState, ra) == 0);
    assert!(core::mem::offset_of!(SavedState, sp) == 4);
    assert!(core::mem::offset_of!(SavedState, t0) == 16);
    assert!(core::mem::offset_of!(SavedState, a7) == 64);
    assert!(core::mem::offset_of!(SavedState, t6) == 120);
    assert!(core::mem::offset_of!(SavedState, pc) == 124);
};

/// Rust half of the trap path: called by `_hubris_trap_entry` with the
/// task's state already saved and the kernel stack live.
#[unsafe(no_mangle)]
unsafe extern "C" fn _hubris_trap_dispatch(task: *mut task::Task) {
    let mstatus = read_csr!("mstatus");
    // MPP != U means the trap came from the kernel itself. Note that by
    // this point the entry sequence has already overwritten the interrupted
    // task's SavedState with kernel register values -- acceptable only
    // because we are about to panic and never resume anything.
    if mstatus & 0x1800 != 0 {
        panic!(
            "kernel fault: mcause={:#010x} mepc={:#010x} mtval={:#010x}",
            read_csr!("mcause"),
            read_csr!("mepc"),
            read_csr!("mtval"),
        );
    }

    let mcause = read_csr!("mcause");
    if (mcause as i32) < 0 {
        // Interrupt. Nothing can be enabled yet (mie is never written), so
        // this is unreachable until a tick source and an interrupt
        // controller are wired up.
        // TODO: dispatch timer tick and external interrupts here.
        panic!("unexpected interrupt: mcause={:#010x}", mcause);
    }

    if mcause == 8 {
        // Environment call from U-mode: a syscall. mepc points at the ecall
        // itself (always 4 bytes -- not compressible); resume after it.
        // Safety: the entry sequence passed us a valid task pointer, and we
        // drop this reference before syscall_entry aliases the task table.
        let nr = unsafe {
            let save = (*task).save_mut();
            save.pc = save.pc.wrapping_add(4);
            save.a7
        };
        // Safety: state was saved by the entry sequence; we are the syscall
        // interrupt handler; the kernel does not nest.
        unsafe { crate::syscalls::syscall_entry(nr, task) };
    } else {
        // Safety: valid task pointer, per above.
        unsafe { handle_fault(task, mcause) };
    }
}

/// Delivers a fault taken in U-mode to the fault machinery, then picks a
/// new task to run. Mirrors the tail of arm_m's `handle_fault`.
unsafe fn handle_fault(task: *mut task::Task, mcause: u32) {
    // Safety: dereferencing the trusted task pointer; result is dropped
    // immediately so it doesn't alias the task table below.
    let idx = unsafe { usize::from((*task).descriptor().index) };
    let mtval = read_csr!("mtval");

    let fault = match mcause {
        // Instruction address misaligned / instruction access fault: the
        // task's pc left its executable regions (or PMP said no).
        0 | 1 => FaultInfo::IllegalText,
        2 => FaultInfo::IllegalInstruction,
        // Load/store address misaligned or access fault: mtval holds the
        // offending data address.
        4..=7 => FaultInfo::MemoryAccess {
            address: Some(mtval),
            source: FaultSource::User,
        },
        // Breakpoint (ebreak in task code) and anything unrecognized.
        _ => FaultInfo::InvalidOperation(mcause),
    };

    with_task_table(|tasks| {
        let next = match task::force_fault(tasks, idx, fault) {
            task::NextTask::Specific(i) => &tasks[i],
            task::NextTask::Other => task::select(idx, tasks),
            task::NextTask::Same => &tasks[idx],
        };

        if core::ptr::eq(next as *const _, task as *const _) {
            panic!("attempt to return to Task #{idx} after fault");
        }

        apply_memory_protection(next);
        // Safety: this leaks a pointer aliasing next into static scope, but
        // we're not going to read it back until the next kernel entry, so
        // we won't be aliasing/racing.
        unsafe {
            set_current_task(next);
        }
    });
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

/// Records the app's tick divisor under the name arm_m uses, which
/// debuggers read by symbol. On ARM the divisor is the CPU clock in kHz,
/// since SysTick counts that clock. Here it is the input rate of the
/// chip's tick timer in kHz, which need not be the CPU clock; it is the
/// value the chip's tick setup consumes.
///
/// # Safety
///
/// Caller must ensure this is called before the tick is started.
pub unsafe fn set_clock_freq(tick_divisor: u32) {
    CLOCK_FREQ_KHZ.store(tick_divisor, Ordering::Relaxed);
}

/// The tick timer's input rate in kHz, for debugger consumption; see
/// [`set_clock_freq`].
#[unsafe(no_mangle)]
static CLOCK_FREQ_KHZ: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
