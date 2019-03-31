@
@    Mecrisp-Stellaris - A native code Forth implementation for ARM-Cortex M microcontrollers
@    Copyright (C) 2013  Matthias Koch

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

	
@ Flag: If set let USB (third ACM port) to take control of configuration
@       of usart1	
.globl usb_usart1_ctl
Variable "usb-usart1", usb_usart1_ctl, 0


.equ FIFO_SIZE, 128
	

@ -----------------------------------------------------------------------------
  Wortbirne Flag_visible, "usb-emit"
usb_emit: @ ( c -- ) Emit one character
@ -----------------------------------------------------------------------------
   push {lr}

1: bl usb_qemit
   cmp tos, #0
   drop
   beq 1b

   mov r0, tos
   bl usb_emit_c	
   drop

   pop {pc}

@ -----------------------------------------------------------------------------
  Wortbirne Flag_visible, "usb-key"
usb_key: @ ( -- c ) Receive one character
@ -----------------------------------------------------------------------------
   push {lr}

1: bl usb_qkey
   cmp tos, #0
   drop
   beq 1b

   pushdatos
   bl usb_key_c
   mov tos, r0	

1: pop {pc}

@ -----------------------------------------------------------------------------
  Wortbirne Flag_visible, "usb-emit?"
usb_qemit:  @ ( -- ? ) Ready to send a character ?
@ -----------------------------------------------------------------------------
   push {lr}
   bl pause

   pushdaconst FIFO_SIZE
   ldr r0, =forth_emit_buf
   ldrb r0, [r0]  @ length of buffer content

   cmp r0, tos
   mov tos, #0	
   beq 1f
	
   mvns tos, tos	
1: pop {pc}

@ -----------------------------------------------------------------------------
  Wortbirne Flag_visible, "usb-key?"
usb_qkey:  @ ( -- ? ) Is there a key press ?
@ -----------------------------------------------------------------------------
   push {lr}
   bl pause

   pushdatos
   ldr tos, =forth_key_buf
   ldrb tos, [tos]
   subs tos, #1	    		@ 0<>
   sbcs tos, tos
   mvns tos, tos
	
   pop {pc}

