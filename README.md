## Raspberry PI Pico

This is a Symmetric multiprocessing (SMP) demo for Raspberry Pi Pico.

The MDEPX rtos is used in this demo.

The main() function of this app creates a few threads that are running on different CPUs of RP2040 SoC.

CPUs are selected by MDX scheduler in round-robin fashion.

## Build instructions on Linux
    $ export CROSS_COMPILE=arm-none-eabi-
    $ make

## Program

You will need two pico boards: one acts as a programmer (using PicoProbe), another is a target pico.

Download PicoProbe UF2 file from [Utilities section](https://www.raspberrypi.org/documentation/rp2040/getting-started/#board-specifications).

Drop the PicoProbe UF2 file onto the RPI-RP2 volume of your programmer pico and reboot the pico, so it is now an SWD programmer.

Connect to each other two picos using jumper wires:

| Programmer Pico | Target Pico                |
| --------------- | -------------------------- |
| GP2             | SWD header SWCLK pin       |
| Any GND pin     | SWD header GND pin         |
| GP3             | SWD header SWDIO pin       |
| GP4             | GP1 (uart0 RX)             |
| GP5             | GP0 (uart0 TX)             |
| VSYS            | VSYS                       |
| GND             | GND                        |

Also connect the programmer pico to your PC using USB-C cable.

#### OpenOCD

You will need an openocd application in order to communicate to the programmer.

Build and install it from [raspberrypi fork](https://github.com/raspberrypi/openocd/). I used picoprobe branch.

Program the result file (obj/rpi-pico.elf) to your target pico:

```
sudo openocd -f interface/picoprobe.cfg -f target/rp2040.cfg -s tcl -c 'program obj/rpi-pico.elf reset exit
```

## Connect serial console

You should see something like this:

```
cpu0: test_thr0
cpu1: test_thr2
cpu0: test_thr0
cpu1: Hello world
cpu0: test_thr1
```

![pico](https://raw.githubusercontent.com/machdep/raspberrypi-pico/master/images/pico.jpg)
