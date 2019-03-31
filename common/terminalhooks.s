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

@ Terminal redirection hooks. 
@ Defaults for pill_serial_mecrisp: use usb communication by default


.macro Variable Name, Label, Init

@------------------------------------------------------------------------------
  Wortbirne Flag_visible|Flag_variable, \Name @ ( -- addr )
  CoreVariable \Label
@------------------------------------------------------------------------------
  pushdatos
  ldr tos, =\Label
  bx lr
  .word \Init
	
.endm
	
.include "usb-terminal.s"

@ Variable "hook-emit", hook_emit, serial_emit
@ Variable "hook-key", hook_key, serial_key
@ Variable "hook-qemit", hook_qemit, serial_qemit
@ Variable "hook-qkey", hook_qkey, serial_qkey
	
Variable "hook-emit", hook_emit, usb_emit
Variable "hook-key", hook_key, usb_key
Variable "hook-qemit", hook_qemit, usb_qemit
Variable "hook-qkey", hook_qkey, usb_qkey

Variable "hook-pause", hook_pause, nop_vektor @ No Pause defined for default


@------------------------------------------------------------------------------
  Wortbirne Flag_visible, "emit" @ ( c -- )
emit:
@------------------------------------------------------------------------------
  push {r0, r1, r2, r3, lr} @ Used in core, registers have to be saved !
  ldr r0, =hook_emit
  bl hook_intern
  pop {r0, r1, r2, r3, pc}

@------------------------------------------------------------------------------
  Wortbirne Flag_visible, "key" @ ( -- c )
key:
@------------------------------------------------------------------------------
  push {r0, r1, r2, r3, lr} @ Used in core, registers have to be saved !
  ldr r0, =hook_key
  bl hook_intern
  pop {r0, r1, r2, r3, pc}

@------------------------------------------------------------------------------
  Wortbirne Flag_visible, "emit?" @ ( -- ? )
@------------------------------------------------------------------------------
  ldr r0, =hook_qemit
  ldr r0, [r0]
  mov pc, r0

@------------------------------------------------------------------------------
  Wortbirne Flag_visible, "key?" @ ( -- ? )
@------------------------------------------------------------------------------
  ldr r0, =hook_qkey
  ldr r0, [r0]
  mov pc, r0

@------------------------------------------------------------------------------
  Wortbirne Flag_visible, "pause" @ ( -- ? )
pause:
@------------------------------------------------------------------------------
  ldr r0, =hook_pause
hook_intern:
  ldr r0, [r0]
  mov pc, r0




.macro Debug_Terminal_Init

  @ A special initialisation sequence intended for debugging
  @ Necessary when you wish to use the terminal before running catchflashpointers.

  @ Kurzschluss-Initialisierung für die Terminalvariablen

   @ Return stack pointer already set up. Time to set data stack pointer !
   @ Normaler Stackpointer bereits gesetzt. Setze den Datenstackpointer:
   ldr psp, =datenstackanfang

   @ TOS setzen, um Pufferunterläufe gut erkennen zu können
   @ TOS magic number to see spurious stack underflows in .s
   @ ldr tos, =0xAFFEBEEF
   movs tos, #42

   ldr r1, =serial_emit
   ldr r0, =hook_emit
   str r1, [r0]

   ldr r1, =serial_qemit
   ldr r0, =hook_qemit
   str r1, [r0]

   ldr r1, =serial_key
   ldr r0, =hook_key
   str r1, [r0]

   ldr r1, =serial_qkey
   ldr r0, =hook_qkey
   str r1, [r0]

   ldr r1, =nop_vektor
   ldr r0, =hook_pause
   str r1, [r0]

.endm
