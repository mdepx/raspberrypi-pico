## Raspberry PI Pico

This is a Symmetric multiprocessing (SMP) demo for Raspberry Pi Pico.

The MDEPX rtos is used in this demo.

The main() function of this app creates a few threads that are running on different CPUs of RP2040 SoC.

CPUs are selected by MDX scheduler in round-robin fashion.

## Build instructions on Linux
    $ export CROSS_COMPILE=arm-none-eabi-
    $ make

## Program

Program the result file (obj/rpi-pico.elf) to your pico.

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
