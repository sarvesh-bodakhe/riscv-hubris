// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Client API for the UART console drivers. The interface is
//! chip-neutral; each chip's UART driver task serves it.
//!
//! See `idl/uart.idol`. `Uart::write` blocks until the driver
//! has pushed every byte into the TX FIFO; the only error is the
//! driver restarting mid-call. Logging paths that have nowhere to
//! report a failure drop the result deliberately.

#![no_std]

use userlib::sys_send;

include!(concat!(env!("OUT_DIR"), "/client_stub.rs"));
