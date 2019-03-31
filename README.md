# Two/three USB-to-serial bridges with build in Forth

This is merge of two projects:

- [Mecrisp-Stellaris](https://github.com/jjonethal/mecrisp-stellaris)
- [pill\_serial](https://github.com/satoshinm/pill_serial)

_Mecrisp-Stellaris_ is now a mature Forth system for a lot of ARM microcontrollers. It is especially usefull on popular and cheap STM32F103 devices. However, it uses USART1 on 115kbps as it's terminal and running on 8MHz only.
STM32F103 hw is quite capable - 72Mhz max, three USARTs, USB FS device with 7 IN/OUT endpoints.

There is already an extension in Forth [available](https://jeelabs.org/article/1718c/) to use USB as a Forth console.

On other side, the [pill\_serial](https://github.com/satoshinm/pill_serial) project tried (not succesfully) to create triple virtual COM USB devices and bridge them to to three F103 USARTs.

My question was: how about to use the first USB virtual COM as a Forth terminal and have even better "Swiss Army Knife" with two additional USB to serial bridges? This project is the answer to this question.

## Supported Boards

- [Blue Pill](https://wiki.stm32duino.com/index.php?title=Blue_Pill)
- [Maple Mini](https://wiki.stm32duino.com/index.php?title=Maple_Mini) clones

## Quick Start

Find a desired firmware in the bin directory. Choose one with \_mm posfix for Maple Mini or with \_bp for Blue Pill. The one with mecrisp\_ prefix has the Forth build in using first wirtual COM as terminal. The firmware without this pprefix is just USB to triple UART bridge just like original [pill\_serial](https://github.com/satoshinm/pill_serial) was intended with the folloving mapping (assume /dev/usbACM0-3 devices where not yet in use before blug in):

| USB Virtual COM | USART   | TX pin | RX pin |
| --------------- | ------  | ------ | ------ |
| /dev/usbACM0    | USART1* | PA9    | PA10   |
| /dev/usbACM1    | USART2  | PA2    | PA3    |
| /dev/usbACM2    | USART3  | PB10   | PB11   |

* Forth console instead if firmware with build in Forth


Flash the firmware e.g. with st-link and st-flash utility:

```
st-flash erase
st-flash --reset write mecrisp_pillserial_bp.bin 0x8000000
```


## TODO

- USB suspend to decrease current consumption to comply with USB specs

