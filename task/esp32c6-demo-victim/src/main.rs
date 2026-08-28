// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! A task whose whole purpose is to be broken on request.
//!
//! Two of Hubris's central claims are only convincing when watched
//! happening: a task that dies is contained and restarted by the
//! supervisor rather than taking the system with it, and a task cannot
//! read memory it was not granted. Both are demonstrated by asking this
//! task to do the forbidden thing and observing that everything else,
//! including whatever asked, keeps running.
//!
//! The caller sees the death as `ServerDeath` and the task's generation
//! number stepping.

#![no_std]
#![no_main]

use idol_runtime::{NotificationHandler, RequestError};
use userlib::*;

struct ServerImpl;

impl idl::InOrderVictimImpl for ServerImpl {
    fn alive(
        &mut self,
        _: &RecvMessage,
        token: u32,
    ) -> Result<u32, RequestError<core::convert::Infallible>> {
        Ok(token)
    }

    fn panic_now(
        &mut self,
        _: &RecvMessage,
    ) -> Result<(), RequestError<core::convert::Infallible>> {
        panic!("demo: asked to panic");
    }

    fn read_foreign(
        &mut self,
        _: &RecvMessage,
        addr: u32,
    ) -> Result<u32, RequestError<core::convert::Infallible>> {
        // Not a wild pointer bug: a deliberate load from an address this
        // task holds no region for. The PMP rejects it and the kernel
        // faults the task -- no fault handler here can suppress that,
        // which is the point.
        Ok(unsafe { core::ptr::read_volatile(addr as *const u32) })
    }
}

impl NotificationHandler for ServerImpl {
    fn current_notification_mask(&self) -> u32 {
        0
    }

    fn handle_notification(&mut self, _bits: NotificationBits) {}
}

#[unsafe(export_name = "main")]
fn main() -> ! {
    let mut incoming = [0u8; idl::INCOMING_SIZE];
    let mut server = ServerImpl;
    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

mod idl {
    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
