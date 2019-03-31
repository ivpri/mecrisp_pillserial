/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __GENERAL_H
#define __GENERAL_H

#define _GNU_SOURCE
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>
#include <inttypes.h>

#include "platform.h"

#define RELOC_ISR(name)					\
  void name ## _isrr(void);				\
  void (* irq_hook_ ## name)() = &name ## _isrr;	\
  void name ## _isr(void) { (* irq_hook_ ## name)(); }	\
  void name ## _isrr(void)



#ifndef DEBUG
#include <stdio.h>
#define DEBUG	printf
#endif

#endif

