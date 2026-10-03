# riscv-hubris

A RISC-V (riscv32) port of [Hubris](https://github.com/oxidecomputer/hubris).

| Chip                   | Status      |
|------------------------|-------------|
| ESP32-C6               | Supported   |
| ESP32-C3               | In progress |
| RP2350 (Hazard3 core)  | In progress |

On the ESP32-C6, tasks are isolated from each other by the PMP, the RISC-V
counterpart of the Arm MPU that upstream Hubris uses. DMA is not confined
yet: the PMP restricts only the CPU.

Start at [`app/demo-esp32c6`](../app/demo-esp32c6/README.md). For flashing and
debugging use [riscv-humility](https://github.com/sarvesh-bodakhe/riscv-humility).
