## Raspberry PI Pico

This is a demo app for Raspberry Pi Pico.

The MDEPX rtos is used in this demo.

The main() function of this app creates a few threads that are running on cpu0 of the pico.

![pico](https://raw.githubusercontent.com/machdep/raspberrypi-pico/master/images/pico.png)

## Build instructions on Linux
    $ sudo apt install gcc-arm-embedded
    $ make

## Program

You will need two pico boards: one acts as a programmer (using PicoProbe), another is a target pico.

Download PicoProbe UF2 file from [Raspberry Pi website](https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html#debugging-using-another-raspberry-pi-pico)

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

Also connect the programmer pico to your PC using USB cable.

#### OpenOCD

You will need an openocd application in order to communicate to the programmer.

Build and install it from raspberrypi [openocd fork](https://github.com/raspberrypi/openocd/).

Program the result file (obj/raspberry-pico.elf) to your target pico:

```
sudo openocd -f interface/picoprobe.cfg -f target/rp2040.cfg \
    -s tcl -c 'program /path/to/obj/raspberry-pico.elf reset exit'
```

## Connect serial console

It is /dev/ttyACMx on Linux, and /dev/ttyUSBx on FreeBSD.

You can use cu(1) this way:
```
sudo cu -l /dev/ttyACM0 -s 115200
```

You should see something like this:

```
cpu0: test_thr0
cpu1: test_thr2
cpu0: test_thr0
cpu1: Hello world
cpu0: test_thr1
```

![pico](https://raw.githubusercontent.com/machdep/raspberrypi-pico/master/images/pico.jpg)
