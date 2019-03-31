.cpu cortex-m3

.equ CpuId, 103
.equ RamEnde, 0x20005000             @ End of RAM   20 kb
.equ FlashDictionaryEnde,0x00010000  @ End of FLASH 64 kb

.include "mecrisp_pillserial.s"
