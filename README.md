# Two/three USB-to-serial bridges with build in Forth

This is merge of two projects:

- [Mecrisp-Stellaris](https://github.com/jjonethal/mecrisp-stellaris)
- [pill\_serial](https://github.com/satoshinm/pill_serial)

_Mecrisp-Stellaris_ is now a mature Forth system for a lot of ARM microcontrollers. It is especially usefull on popular and cheap STM32F103 devices. However, it uses USART1 on 115kbps as it's terminal and running on 8MHz only.
STM32F103 hw is quite capable - 72Mhz max, three USARTs, USB FS device with 7 IN/OUT endpoints.

There is already an extension in Forth [available](https://jeelabs.org/article/1718c/) to use USB as a Forth console.

On other side, the [pill\_serial](https://github.com/satoshinm/pill_serial) project goal was to create triple virtual COM USB devices and bridge them to to three F103 USARTs.

My question was: how about to use the first USB virtual COM as a Forth terminal and have even better "Swiss Army Knife" with two additional USB to serial bridges? This project is the answer to this question.

## Supported Boards

- [Blue Pill](https://wiki.stm32duino.com/index.php?title=Blue_Pill)
- [Maple Mini](https://wiki.stm32duino.com/index.php?title=Maple_Mini) clones

## Quick Start

Precompiled firmware can be found in bin directory. \_mm posfix firmware is for Maple Mini and the \_bp one is for Blue Pill. The one with mecrisp\_ prefix has build in Forth where the first wirtual COM is used as terminal. The firmware without mecrisp\_ prefix is just USB to triple UART bridge like the original [pill\_serial](https://github.com/satoshinm/pill_serial) with the folloving mapping (assume /dev/usbACM0-2 devices where not yet created):

| USB Virtual COM | USART          | TX pin | RX pin | Note                  |
| --------------- | -------------  | ------ | ------ | ----------------------
| /dev/usbACM0    | USART1         | PA9    | PA10   |                       |
| /dev/usbACM0    | Forth terminal | -      | -      | mecrisp\_... firmware |
| /dev/usbACM1    | USART2         | PA2    | PA3    |                       |
| /dev/usbACM2    | USART3         | PB10   | PB11   |                       |


If you have [st-link](https://www.st.com/en/development-tools/st-link-v2.html) or it's [clone](https://wiki.stm32duino.com/index.php?title=ST-LINK_clone)
the firmware can be flashed e.g. with the [st-flash](https://github.com/texane/stlink) utility:

```
st-flash erase
st-flash --reset write mecrisp_pillserial_bp.bin 0x8000000
```

Note both BOOT0 and BOOT1 must be set to 0 on the board. The Maple Mini does not have BOOT0 and BOOT1 jumpers but BOOT0 is pulled down by default so there is no need to do anything with it. Just connect GND, SWCLK (PA14) and SWDIO *PA13) to the st-link.

Alternative method is to flash firmware using serial bootloader. A possible utility for the task is [stm32flash](https://sourceforge.net/p/stm32flash/wiki/Home/). Any less then dollar cheap CP2102 usb to serial converter (note: not an RS232 one, Rx, Tx must be 3.3V) can be used to connect to USART1 (PA9, PA10) - Tx to Rx and Rx to Tx. Then swith BOOT0 jumper to 1 and press reset button to start the serial bootloader. BOOT0 jumper needs to be returned back to 0 after the firmware is flashed. On the Maple Mini BOOT0 is wired with the user button so press and hold it and press the reset button then.


## Usage

Usage of usb to uart bridges should be obvious. Just use it as any other usb to uart converter. Any usual baudrate should work (1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200bps). Higher baudrates should work as well (limit not tested yet but hardware should support up to 3Mbps).
Other settings like data bits, parity, stop bits (one, one and half, two) all should work as well.

### Mecrisp Stellaris Terminal

To use build in Forth terminal connect to first virtual COM port either by using your favorit virtual terminal or for more serious programming (loading, include, require of files) use a specialized terminal for various forth systems like [e4thcom](https://wiki.forth-ev.de/doku.php/en:projects:e4thcom). Unlike with a connection via UART there is no need to set bautrade, default (9600bps) is ok, still you will have full possible speed e.g.:

```
e4thcom -d ttyACM0 -t mecrisp-st
```

That's it, if connection is successfull, you should see a welcome message. Hit Enter and you should see `ok.` prompt there indicating everything works.

Now follow guides [here](http://mecrisp.sourceforge.net/) and [here](http://hightechdoc.net/mecrisp-stellaris/_build/html/index.html) and enjoy the [Forth System](https://forth-standard.org/) running on a powerfull embeded system and prepare to throw out all your arduinos!




## TODO

- USB suspend to decrease current consumption to comply with USB specs
- Check and test max and min baudrates and data bits, parity and so on

