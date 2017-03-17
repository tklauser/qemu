/*
 * Altera UART emulation
 *
 * Copyright (c) 2012 Chris Wulff <crwulff@gmail.com>
 * Copyright (c) 2017 Tobias Klauser <tklauser@distanz.ch>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ALTERA_UART_H
#define ALTERA_UART_H

#include "hw/sysbus.h"
#include "sysemu/char.h"

#define ALTERA_UART_R_MAX	6

typedef struct AlteraUARTState {
    SysBusDevice busdev;
    MemoryRegion mmio;
    CharBackend chr;
    qemu_irq irq;

    uint32_t regs[ALTERA_UART_R_MAX];
} AlteraUARTState;

void altera_uart_create(int channel, const hwaddr addr, qemu_irq irq);

#endif /* ALTERA_UART_H */
