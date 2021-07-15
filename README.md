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

#### OpenOCD

You will need an openocd application in order to communicate to the programmer.

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
