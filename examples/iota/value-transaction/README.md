# IOTA example

## Overview

This example demonstrates generating and signing IOTA transactions in the Contiki-NG operating system.

*Disclaimer:* this is a proof-of-concept example only! Do not use it for real-world applications. I do not endorse IOTA.


## Getting started

To get started with Contiki-NG, follow the documentation in https://github.com/contiki-ng/contiki-ng/wiki

From the different approaches for setting up Contiki-NG programming environment, we recommened installing the Docker image.


## Supported platforms

This example is tested on `native` and `srf06-cc26xx` platforms.

### Building for the native platform

The `native` platform allows to run Contiki as a process on top of a host operating system. First, build the binary image by:

    $ make TARGET=native

Then you can run the image as a regular executable:

    $ ./main.native


### Building for the Texas Instruments CC26xx/CC13xx System-on-Chip platforms

There are multiple boards supported for the `srf06-cc26xx` platform. For example, if you have a CC2650 Launchpad device, you can build the firmware for it with 

    $ make TARGET=srf06-cc26xx BOARD=launchpad/cc2650

After the build process has successfully completed, you have two options for flashing the device:

1. Programming through JTAG
2. Programming through the on-board bootloader

For the former option, you can download and use [Uniflash](http://www.ti.com/tool/UNIFLASH). For the latter, the bootloader script (CC2538-bsl) that comes with Contiki-NG. To use the script, you first need to put the board in the bootload mode: press and hold the button 1, then press reset button, then release the button 1. See [here](http://pablocorbalan.com/getting-started-with-ti-cc2650-launchpad) for more detail. Then upload the image with:

    $ make main.upload PORT=/dev/ttyACM0 TARGET=srf06-cc26xx BOARD=launchpad/cc2650

To access serial output from the board, set `PORT` variable to the name of the tty device and connect with the `login` target:

    $ make login PORT=/dev/ttyACM0
