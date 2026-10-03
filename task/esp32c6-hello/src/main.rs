// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! A task that prints, by asking another task to do it.
//!
//! This exists to demonstrate the whole Hubris model end to end on a new
//! port: two unprivileged tasks, mutually distrusting and separately
//! linked, cooperating through the kernel. This task owns no peripheral at
//! all -- it has no `uses` grant, so its PMP regions cover only its own text
//! and RAM, and a store to the UART's registers from here would fault.
//! The only way it can reach the outside world is to ask the driver task,
//! which does hold that grant.
//!
//! The message carries its text as a **lease** rather than in the message
//! body: the bytes stay in this task's memory and the kernel copies them
//! out on the driver's behalf, checking every access against *this* task's
//! regions. A driver cannot be tricked into reading memory its caller
//! could not read itself.
//!
//! What happened to the victim is also recorded in a ring buffer in this
//! task's RAM, the way upstream tasks keep their own trace, so the story
//! can be read back through the debugger with `humility ringbuf`, with
//! no serial cable attached.

#![no_std]
#![no_main]

use drv_uart_api::Uart;
#[cfg(feature = "demo-victim")]
use ringbuf::{ringbuf, ringbuf_entry};
#[cfg(feature = "demo-victim")]
use task_esp32c6_demo_victim_api::Victim;
use userlib::*;

// Filled in at packaging time from the `task-slots` entry in app.toml, so
// this task refers to the driver by role rather than by index.
task_slot!(UART, uart_driver);
#[cfg(feature = "demo-victim")]
task_slot!(VICTIM, victim);

/// What the victim did when asked. `Alive` carries the generation that
/// answered, so the buffer shows it stepping as the supervisor restarts
/// the task.
#[cfg(feature = "demo-victim")]
#[derive(Copy, Clone, PartialEq)]
enum Trace {
    None,
    VictimAlive(u8),
    VictimNotAnswering,
    VictimPanicked,
    VictimSurvivedPanic,
    VictimForeignReadFaulted,
    VictimForeignReadSucceeded,
}

#[cfg(feature = "demo-victim")]
ringbuf!(Trace, 16, Trace::None);

#[unsafe(export_name = "main")]
fn main() -> ! {
    write(b"\r\n[hello task: talking to the uart driver over IPC]\r\n");

    let mut beat: u32 = 0;
    loop {
        // "tick <n>", assembled by hand: no formatting machinery in a task
        // this small.
        let mut line = [0u8; 16];
        let mut n = 0;
        for b in b"tick " {
            line[n] = *b;
            n += 1;
        }
        n += write_u32(&mut line[n..], beat);
        line[n] = b'\r';
        line[n + 1] = b'\n';
        write(&line[..n + 2]);

        #[cfg(feature = "demo-victim")]
        drive_victim(beat);

        beat = beat.wrapping_add(1);
        hl::sleep_for(1000);
    }
}

/// A few ticks in, asks the victim task to do the two forbidden things
/// and reports what happened, in the ring buffer and on the console.
/// Each request is expected to kill the victim: the reply comes back as
/// `ServerDeath`, the supervisor restarts it, and the next `alive` finds
/// the new generation answering. If a request succeeds instead, that is
/// reported too -- it would mean isolation is not enforced, which is the
/// one thing worth shouting.
#[cfg(feature = "demo-victim")]
fn drive_victim(beat: u32) {
    // The slot lookup refreshes the generation, so after a restart this
    // names the victim that is running now.
    let id = VICTIM.get_task_id();
    let generation = u8::from(id.generation());
    let victim = Victim::from(id);
    let (trace, text): (Trace, &[u8]) = match beat {
        2 => match victim.alive(0xa5) {
            Ok(0xa5) => (Trace::VictimAlive(generation), b"[victim: alive]\r\n"),
            _ => (Trace::VictimNotAnswering, b"[victim: NOT answering]\r\n"),
        },
        3 => match victim.panic_now() {
            Err(_) => (
                Trace::VictimPanicked,
                b"[victim: panicked; the supervisor restarted it]\r\n",
            ),
            Ok(()) => (
                Trace::VictimSurvivedPanic,
                b"[victim: survived a panic request (unexpected)]\r\n",
            ),
        },
        4 => match victim.alive(0xa5) {
            Ok(0xa5) => (
                Trace::VictimAlive(generation),
                b"[victim: alive again, new generation]\r\n",
            ),
            _ => (
                Trace::VictimNotAnswering,
                b"[victim: NOT back after the panic]\r\n",
            ),
        },
        // An address the victim holds no grant for: the start of the
        // peripheral space the kernel keeps to itself.
        5 => match victim.read_foreign(0x0200_0000) {
            Err(_) => (
                Trace::VictimForeignReadFaulted,
                b"[victim: foreign read faulted in hardware; restarted]\r\n",
            ),
            Ok(_) => (
                Trace::VictimForeignReadSucceeded,
                b"[victim: foreign read SUCCEEDED -- isolation not enforced]\r\n",
            ),
        },
        6 => match victim.alive(0xa5) {
            Ok(0xa5) => (
                Trace::VictimAlive(generation),
                b"[victim: alive again after the fault]\r\n",
            ),
            _ => (
                Trace::VictimNotAnswering,
                b"[victim: NOT back after the fault]\r\n",
            ),
        },
        _ => return,
    };
    ringbuf_entry!(trace);
    write(text);
}

/// Sends `bytes` to the UART driver and panics if it refuses.
///
/// A failed write here means the driver died mid-call -- a genuine
/// event worth surfacing -- and panicking hands the decision to jefe
/// rather than silently dropping output.
fn write(bytes: &[u8]) {
    Uart::from(UART.get_task_id()).write(bytes).unwrap_lite();
}

/// Writes `value` as decimal into `out`, returning how many bytes it took.
fn write_u32(out: &mut [u8], value: u32) -> usize {
    if value == 0 {
        out[0] = b'0';
        return 1;
    }
    // Render backwards into a scratch buffer, then reverse: 10 digits is
    // the most a u32 can need.
    let mut digits = [0u8; 10];
    let mut n = 0;
    let mut v = value;
    while v != 0 {
        digits[n] = b'0' + (v % 10) as u8;
        v /= 10;
        n += 1;
    }
    for i in 0..n {
        out[i] = digits[n - 1 - i];
    }
    n
}
