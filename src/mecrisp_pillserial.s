@
@    Mecrisp-Stellaris - A native code Forth implementation for ARM-Cortex M microcontrollers
@    Copyright (C) 2013  Matthias Koch
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
	
.syntax unified
@ .cpu cortex-m3
.thumb

@ -----------------------------------------------------------------------------
@ Swiches for capabilities of this chip
@ -----------------------------------------------------------------------------

.equ registerallocator, 1
@ Not available: .equ charkommaavailable, 1

@ -----------------------------------------------------------------------------
@ Start with some essential macro definitions
@ -----------------------------------------------------------------------------

.include "../common/datastackandmacros.s"

@ -----------------------------------------------------------------------------
@ Speicherkarte für Flash und RAM
@ Memory map for Flash and RAM
@ -----------------------------------------------------------------------------

@ Konstanten für die Größe des Ram-Speichers
	
@ TODO: how to correctly compute this from _ebss
@ .equ RamAnfang, _ebss        @ Start of RAM          Porting: Change this !
.equ RamAnfang, 0x20000600     @ Start of RAM          Porting: Change this !
@ .equ RamEnde,   0x20005000     @ End   of RAM.  20 kb. Porting: Change this !

@ Konstanten für die Größe und Aufteilung des Flash-Speichers

@ TODO: how to correctly compute this from _etext
@ .equ Kernschutzadresse,     _etext + 0x40     @ Darunter wird niemals etwas geschrieben ! Mecrisp core never writes flash below this address.
@ .equ FlashDictionaryAnfang, _etext + 0x40     @ Used by core + USB driver.
.equ Kernschutzadresse,     0x00007800 @ Darunter wird niemals etwas geschrieben ! Mecrisp core never writes flash below this address.
.equ FlashDictionaryAnfang, 0x00007800 @ Used by core + USB driver.
@ .equ FlashDictionaryEnde,   0x00010000 @ 34 kb Platz für das Flash-Dictionary       34 kb Flash available. Porting: Change this !
.equ Backlinkgrenze,        RamAnfang  @ Ab dem Ram-Start.


@ -----------------------------------------------------------------------------
@ Anfang im Flash - Interruptvektortabelle ganz zu Beginn
@ Flash start - Vector table has to be placed here
@ -----------------------------------------------------------------------------
.text    @ Hier beginnt das Vergnügen mit der Stackadresse und der Einsprungadresse

@ interrupts handled by C code in this project
@ .include "vectors.s" @ You have to change vectors for Porting !
	
@ -----------------------------------------------------------------------------
@ Include the Forth core of Mecrisp-Stellaris
@ -----------------------------------------------------------------------------

.include "../common/forth-core.s"

@ -----------------------------------------------------------------------------
.globl ForthReset
ForthReset: @ Einsprung zu Beginn
@ -----------------------------------------------------------------------------
   @ Initialisierungen der Hardware, habe und brauche noch keinen Datenstack dafür
   @ Initialisations for Terminal hardware, without Datastack.
   bl uart_init

   @ Catch the pointers for Flash dictionary
   .include "../common/catchflashpointers.s"

   @ bl platform_init	

   .if CpuId == 103
   welcome " for STM32F103 by Matthias Koch"
   .elseif CpuId == 405
   welcome " for STM32F4 by Matthias Koch"
   .endif	

   @ Ready to fly !
   .include "../common/boot.s"
