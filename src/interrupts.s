@
@    Mecrisp-Stellaris - A native code Forth implementation for ARM-Cortex M microcontrollers
@    Copyright (C) 2013  Matthias Koch
@
@    pill_serial_mecrisp - pill_serial extension with mecrisp-stellaris
@    Copyright (C) 2019  Ivan Priesol
@
@    This program is free software: you can redistribute it and/or modify
@    it under the terms of the GNU General Public License as published by
@    the Free Software Foundation, either version 3 of the License, or
@    (at your option) any later version.
@
@    This program is distributed in the hope that it will be useful,
@    but WITHOUT ANY WARRANTY; without even the implied warranty of
@    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
@    GNU General Public License for more details.
@
@    You should have received a copy of the GNU General Public License
@    along with this program.  If not, see <http://www.gnu.org/licenses/>.
@

@ Routinen für die Interruptchandler, die zur Laufzeit neu gesetzt werden können.
@ Code for interruptc handlers that are exchangeable on the fly

@------------------------------------------------------------------------------
@ Alle Interruptchandler funktionieren gleich und werden komfortabel mit einem Makro erzeugt:
@ All interruptc handlers work the same way and are generated with a macro:
@------------------------------------------------------------------------------

@ vector table is handled by C so lables must be <name>_isr and set as global.
@ If WName is passed then hook word names are irq-<WName> otherwise irq-<Name>
@ Handler is optional initial irq handler instead of "unhandled"	

@ effective load 32 bit constant into register	
.macro ldrconst Reg, Const
  .ifdef m0core
  ldr \Reg, =\Const
  .else
@ This is recommended over ldr unless next instriction is not another load constant instruction 
    movw \Reg, #:lower16:\Const
    movt \Reg, #:upper16:\Const
  .endif
.endm

	
.macro interruptc Name:req, WName
@------------------------------------------------------------------------------
  .ifb \WName
    Wortbirne Flag_visible|Flag_variable, "irq-\Name"  @ ( -- addr )
  .else	
    Wortbirne Flag_visible|Flag_variable, "irq-\WName" @ ( -- addr )
  .endif
	
  CoreVariable irq_hook_\Name
@------------------------------------------------------------------------------  
  pushdatos
  ldr tos, =irq_hook_\Name
  bx lr
  .word unhandled  @ Startwert für unbelegte Interruptcs   Start value for unused interrupts
	
  .global \Name\()_isr
\Name\()_isr:
  ldrconst r0, irq_hook_\Name
  ldr r0, [r0]  @ Cannot ldr to PC directly, as this would require bit 0 to be set accordingly.
  mov pc, r0    @ No need to make bit[0] uneven as 16-bit Thumb "mov" to PC ignores bit 0.
.endm


.macro interrupte Name:req, WName
@------------------------------------------------------------------------------
  .ifb \WName
    Wortbirne Flag_visible|Flag_variable, "irq-\Name"  @ ( -- addr )
  .else	
    Wortbirne Flag_visible|Flag_variable, "irq-\WName" @ ( -- addr )
  .endif
@------------------------------------------------------------------------------  
  pushdatos
  ldrconst tos, irq_hook_\Name
  bx lr	
.endm
	
	
	                @ changes over orig mecrisp
interruptc wwdg		@ new
interruptc systick	@ new
	
interruptc rtc
interruptc exti0
interruptc exti1
interruptc exti2
interruptc exti3
interruptc exti4
interruptc adc1_2, adc

interruptc dma1_channel1, dma1 @ new (all dma)
interruptc dma1_channel2, dma2
interruptc dma1_channel3, dma3
interruptc dma1_channel4, dma4
interruptc dma1_channel5, dma5
interruptc dma1_channel6, dma6
interruptc dma1_channel7, dma7

interruptc can_tx, cantx    @ new (all can)
interruptc can_rx0, canrx0
interruptc can_rx1, canrx1
interruptc can_sce, cansce
	
interruptc exti9_5, exti5
interruptc tim1_brk, tim1brk
interruptc tim1_up, tim1up
interruptc tim1_trg_com, tim1trg
interruptc tim1_cc, tim1cc
interruptc tim2
interruptc tim3

interrupte tim4	

interruptc i2c1_ev, i2c1ev
interruptc i2c1_er, i2c1er
interruptc i2c2_ev, i2c2ev
interruptc i2c2_er, i2c2er
interruptc spi1
interruptc spi2

interruptc usart1
	
interrupte usart2
interrupte usart3
interruptc exti15_10, exti10
interruptc rtc_alarm, rtcalarm
interruptc usb_wakeup, usbwkup

interrupte usb_lp_can_rx0, usbfs

@ available on >= 256kB flash devices only:
@ interruptc tim5   
@ interruptc tim6
@ interruptc tim7
@ interruptc spi3
@ interruptc uart4
@ interruptc uart5
	
	
@------------------------------------------------------------------------------
