// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Kernel entry for the minimal Espressif image. The image's first
//! instruction, `_start`, is the kernel's (arch/riscv32.rs); it sets up the
//! runtime and calls `main` below, which enters the kernel. The boot ROM's
//! loader has already placed the image's segments in SRAM and jumps there.

#![no_std]
#![no_main]

/// SYSTIMER ticks per kernel tick (1 ms): the counter runs at 16 MHz,
/// XTAL/2.5 (esp_hw_support's `systimer_ticks_to_us` in public esp-idf).
const CYCLES_PER_MS: u32 = 16_000;

#[unsafe(no_mangle)]
extern "C" fn main() -> ! {
    unsafe { kern::startup::start_kernel(CYCLES_PER_MS) }
}
