// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Console driver task for the ESP32-C6's UART0, minimal form: the boot
//! ROM's configuration is inherited (115200 8N1, the ROM's own console),
//! and only the FIFOs and the receive interrupt are touched. Registers
//! are reached through the chip's PAC (public esp-idf,
//! `register/soc/uart_reg.h` in `components/soc/esp32c6`).
//!
//! Serves the chip-neutral `idl/uart.idol`: one `write` over a read
//! lease. Received bytes are echoed from the interrupt path, so the IRQ
//! plumbing is observable from a terminal.

#![no_std]
#![no_main]

use esp32c6 as pac;

use idol_runtime::{Leased, LenLimit, NotificationHandler, R, RequestError};
use userlib::*;

/// STATUS: RXFIFO_CNT in the low bits, TXFIFO_CNT from bit 16. The
/// fields are 8 bits wide; the mask is wider and reads them correctly.
const FIFO_CNT_MASK: u32 = 0x3FF;

/// The TX FIFO is 128 bytes deep; stay well clear of the limit.
const TX_FIFO_HEADROOM: u32 = 100;

fn uart() -> &'static pac::uart0::RegisterBlock {
    // Safety: the block is granted to this task by the app's
    // `uses = ["uart0"]`, and nothing else in the image drives it.
    unsafe { &*pac::UART0::ptr() }
}

fn init() {
    let uart = uart();
    // CONF1: the RXFIFO_FULL threshold. The reset value suits bulk
    // transfer; a console wants a single keystroke to wake it. Written
    // through its field, so the TXFIFO_EMPTY threshold beside it in the
    // register is left as it was.
    //
    // Safety: 1 is within the field's range.
    uart.conf1()
        .modify(|_, w| unsafe { w.rxfifo_full_thrhd().bits(1) });
    uart.int_clr().write(|w| unsafe { w.bits(0x000F_FFFF) });
    // Unmask the receive interrupt at the device: RXFIFO_FULL fires once
    // the RX FIFO holds at least CONF1's threshold bytes. Nothing is
    // delivered until sys_irq_control opens the kernel's side as well.
    uart.int_ena().write(|w| w.rxfifo_full().set_bit());
}

fn tx_fifo_count() -> u32 {
    (uart().status().read().bits() >> 16) & FIFO_CNT_MASK
}

fn rx_fifo_count() -> u32 {
    uart().status().read().bits() & FIFO_CNT_MASK
}

/// Pushes bytes into the TX FIFO, spinning politely while it is nearly
/// full: at 115200 baud a full FIFO drains in about 11 ms.
fn write_bytes(bytes: &[u8]) {
    for &b in bytes {
        while tx_fifo_count() > TX_FIFO_HEADROOM {
            hl::sleep_for(1);
        }
        // Writing a word to FIFO pushes one byte.
        uart().fifo().write(|w| unsafe { w.bits(u32::from(b)) });
    }
}

/// Drains the RX FIFO, echoing what arrived. Clear first, then drain:
/// the other order loses a byte that lands between the last read and
/// the clear.
fn echo_pending_input() {
    uart()
        .int_clr()
        .write(|w| w.rxfifo_full().clear_bit_by_one());
    while rx_fifo_count() > 0 {
        let byte = uart().fifo().read().rxfifo_rd_byte().bits();
        if byte == b'\r' {
            write_bytes(b"\r\n");
        } else {
            write_bytes(&[byte]);
        }
    }
}

struct ServerImpl;

impl idl::InOrderUartImpl for ServerImpl {
    fn write(
        &mut self,
        _: &RecvMessage,
        source: LenLimit<Leased<R, [u8]>, 1024>,
    ) -> Result<(), RequestError<core::convert::Infallible>> {
        let mut chunk = [0u8; 32];
        let total = source.len();
        let mut sent = 0;
        while sent < total {
            let n = (total - sent).min(chunk.len());
            source
                .read_range(sent..sent + n, &mut chunk[..n])
                .map_err(|_| RequestError::went_away())?;
            write_bytes(&chunk[..n]);
            sent += n;
        }
        Ok(())
    }
}

impl NotificationHandler for ServerImpl {
    fn current_notification_mask(&self) -> u32 {
        notifications::UART_IRQ_MASK
    }

    fn handle_notification(&mut self, bits: NotificationBits) {
        if bits.check_notification_mask(notifications::UART_IRQ_MASK) {
            echo_pending_input();
            sys_irq_control(notifications::UART_IRQ_MASK, true);
        }
    }
}

#[unsafe(export_name = "main")]
fn main() -> ! {
    init();
    sys_irq_control(notifications::UART_IRQ_MASK, true);
    write_bytes(b"\r\n[uart drv up]\r\n");

    let mut incoming = [0u8; idl::INCOMING_SIZE];
    let mut server = ServerImpl;
    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

mod idl {
    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}

include!(concat!(env!("OUT_DIR"), "/notifications.rs"));
