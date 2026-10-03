# ESP32-C6 demo application

The smallest image that shows Hubris working on the ESP32-C6: five
tasks, isolated from each other by the PMP, one of which is broken on
purpose so you can watch the kernel contain it and the supervisor
restart it.

| Task          | Crate                        | Role                                                        |
|---------------|------------------------------|-------------------------------------------------------------|
| `jefe`        | `task-jefe` (upstream)       | supervisor: restarts tasks the kernel faults                |
| `uart_driver` | `drv-esp32c6-uart`           | the only task granted UART0; serves `write` requests        |
| `victim`      | `task-esp32c6-demo-victim`   | breaks itself on request: a panic, then a load it may not do |
| `hello`       | `task-esp32c6-hello`         | prints through the driver over IPC and runs the experiment  |
| `idle`        | `task-idle` (upstream)       | runs when nothing else can                                  |

The `esp32c6-` crates arrived with this port; everything else is
upstream Hubris. `hello` holds no peripheral: its
only way to the outside world is a message to the driver, carrying the
text as a lease the kernel copies on the driver's behalf, checked
against hello's own memory.

## What it does

Once a second `hello` sends `tick <n>` to the driver. On ticks 2 to 6
it asks the victim, over IPC, to do the following and reports what
came back:

```
tick 2   [victim: alive]
tick 3   [victim: panicked; the supervisor restarted it]
tick 4   [victim: alive again, new generation]
tick 5   [victim: foreign read faulted in hardware; restarted]
tick 6   [victim: alive again after the fault]
```

Tick 3 is containment in software: the victim panics, hello's call
returns `ServerDeath` rather than hanging, `jefe` restarts the victim.
Tick 5 is containment in hardware: the victim loads from an address it
holds no PMP region for, the load faults, the kernel stops the task; no
code inside the victim can catch that. Tick 6 shows the restart took.
What the PMP enforces is the CPU's own accesses: a peripheral that
masters the bus (DMA) is not bound by it, and this chip support does
not confine DMA (see the chip module).
If either forbidden request *succeeded*, hello would say so loudly.

## Building and flashing

```console
$ cargo xtask dist app/demo-esp32c6/app.toml
$ cargo xtask flash app/demo-esp32c6/app.toml
```

`dist` wraps the linked image in the boot ROM's format and puts it in
the archive; `flash` runs `humility flash` over the chip's built-in
USB-Serial-JTAG, the port labelled **USB** on the devkit. It needs a
Humility that knows this target (the riscv32 port of Humility); point
`HUBRIS_HUMILITY_PATH` at it if it is not the `humility` on your path.

## Watching it

Everything the demo proves is readable through the same USB cable
that flashed it. The task table shows the victim's generation stepping
and, if you catch it between restarts, the fault the kernel recorded:

```console
$ humility -a target/demo-esp32c6/dist/default/build-demo-esp32c6-image-default.zip tasks
ID TASK                       GEN PRI STATE
 0 jefe                         0   0 recv, notif: fault timer(T+13)
 1 uart_driver                  0   2 recv, notif: uart-irq(irq43)
 2 victim                       2   5 recv
 3 hello                        0   6 notif: bit31(T+123)
 4 idle                         0   7 RUNNING
```

`hello` also records each outcome of the experiment, with the
generation of the victim that answered, in a ring buffer in its own
RAM, the way upstream tasks keep their trace:

```console
$ humility -a <archive> ringbuf
humility: ring buffer task_esp32c6_hello::__RINGBUF in hello:
 NDX LINE      GEN    COUNT PAYLOAD
   0  162        1        1 VictimAlive(0x0)
   1  162        1        1 VictimPanicked
   2  162        1        1 VictimAlive(0x1)
   3  162        1        1 VictimForeignReadFaulted
   4  162        1        1 VictimAlive(0x2)
```

The text itself goes out on UART0, which keeps the boot ROM's settings
(115200 8N1). On the devkit that is the port labelled **UART**; open it
with any serial terminal to see the `tick` lines, and type into it to
see the driver echo your keystrokes from its interrupt path. The kernel
prints nothing.
