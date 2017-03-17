/*
 * QEMU model of the Altera UART.
 *
 * Copyright (c) 2012 Chris Wulff <crwulff@gmail.com>
 * Copyright (c) 2017 Tobias Klauser <tklauser@distanz.ch>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/lgpl-2.1.html>
 *
 * The Altera UART hardware registers are described in:
 * https://www.altera.com/en_US/pdfs/literature/ug/ug_embedded_ip.pdf
 * (In particular "Register Map" on page 76)
 */

#include "qemu/osdep.h"
#include "hw/char/altera_uart.h"
#include "hw/sysbus.h"
#include "sysemu/char.h"
#include "sysemu/sysemu.h"
#include "qemu/error-report.h"

#define R_RXDATA        0
#define R_TXDATA        1
#define R_STATUS        2
#define R_CONTROL       3
#define R_DIVISOR       4
#define R_ENDOFPACKET   5

#define STATUS_PE        0x0001
#define STATUS_FE        0x0002
#define STATUS_BRK       0x0004
#define STATUS_ROE       0x0008
#define STATUS_TOE       0x0010
#define STATUS_TMT       0x0020
#define STATUS_TRDY      0x0040
#define STATUS_RRDY      0x0080
#define STATUS_E         0x0100
#define STATUS_DTCS      0x0400
#define STATUS_CTS       0x0800
#define STATUS_EOP       0x1000

#define CONTROL_IPE      0x0001
#define CONTROL_IFE      0x0002
#define CONTROL_IBRK     0x0004
#define CONTROL_IROE     0x0008
#define CONTROL_ITOE     0x0010
#define CONTROL_ITMT     0x0020
#define CONTROL_ITRDY    0x0040
#define CONTROL_IRRDY    0x0080
#define CONTROL_IE       0x0100
#define CONTROL_TBRK     0x0200
#define CONTROL_IDTCS    0x0400
#define CONTROL_RTS      0x0800
#define CONTROL_IEOP     0x1000

#define TYPE_ALTERA_UART "ALTR.uart"
#define ALTERA_UART(obj) \
    OBJECT_CHECK(AlteraUARTState, (obj), TYPE_ALTERA_UART)

#define ALTERA_UART_REGS_MEM_SIZE  (ALTERA_UART_R_MAX * sizeof(uint32_t))

static void altera_uart_update_irq(AlteraUARTState *s)
{
    unsigned int irq;

    irq = (s->regs[R_STATUS] & s->regs[R_CONTROL] &
          (STATUS_PE | STATUS_FE | STATUS_BRK | STATUS_ROE | STATUS_TOE |
           STATUS_TMT | STATUS_TRDY | STATUS_RRDY | STATUS_E | STATUS_DTCS));
    qemu_set_irq(s->irq, !!irq);
}

static uint64_t altera_uart_read(void *opaque, hwaddr addr, unsigned int size)
{
    AlteraUARTState *s = opaque;
    uint32_t r = 0;
    addr >>= 2;
    addr &= 0x7;
    switch (addr) {
    case R_RXDATA:
        r = s->regs[R_RXDATA];
        s->regs[R_STATUS] &= ~STATUS_RRDY;
        altera_uart_update_irq(s);
        break;

    case R_STATUS:
        r = s->regs[R_STATUS];
        s->regs[R_STATUS] &= ~(STATUS_PE | STATUS_FE | STATUS_BRK |
                               STATUS_ROE | STATUS_TOE | STATUS_E |
                               STATUS_DTCS);
        altera_uart_update_irq(s);
        break;

    default:
        if (addr < ARRAY_SIZE(s->regs)) {
            r = s->regs[addr];
        }
        break;
    }

    return r;
}

static void altera_uart_write(void *opaque, hwaddr addr, uint64_t val64,
		              unsigned int size)
{
    AlteraUARTState *s = opaque;
    uint32_t value = val64;
    unsigned char ch = value;

    addr >>= 2;
    addr &= 0x7;

    switch (addr) {
    case R_TXDATA:
        qemu_chr_fe_write(&s->chr, &ch, 1);

        s->regs[addr] = value;
        break;

    case R_RXDATA:
    case R_STATUS:
        /* No writeable bits */
        break;

    default:
        s->regs[addr] = value;
        break;
    }
    altera_uart_update_irq(s);
}

static void altera_uart_receive(void *opaque, const uint8_t *buf, int size)
{
    AlteraUARTState *s = opaque;

    s->regs[R_RXDATA] = *buf;
    s->regs[R_STATUS] |= STATUS_RRDY;

    altera_uart_update_irq(s);
}

static int altera_uart_can_receive(void *opaque)
{
    AlteraUARTState *s = opaque;
    return ((s->regs[R_STATUS] & STATUS_RRDY) == 0);
}

static const MemoryRegionOps uart_ops = {
    .read = altera_uart_read,
    .write = altera_uart_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static void altera_uart_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    AlteraUARTState *s = ALTERA_UART(obj);

    memory_region_init_io(&s->mmio, OBJECT(s), &uart_ops, s,
                          TYPE_ALTERA_UART, ALTERA_UART_REGS_MEM_SIZE);
    sysbus_init_mmio(sbd, &s->mmio);
    sysbus_init_irq(sbd, &s->irq);
}

void altera_uart_create(int channel, const hwaddr addr, qemu_irq irq)
{
    DeviceState *dev;
    SysBusDevice *bus;
    Chardev *chr;
    const char chr_name[] = "uart";
    char label[ARRAY_SIZE(chr_name) + 1];

    dev = qdev_create(NULL, TYPE_ALTERA_UART);

    if (channel >= MAX_SERIAL_PORTS) {
        error_report("Only %d serial ports are supported by QEMU",
                     MAX_SERIAL_PORTS);
        exit(1);
    }

    chr = serial_hds[channel];
    if (!chr) {
        snprintf(label, ARRAY_SIZE(label), "%s%d", chr_name, channel);
        chr = qemu_chr_new(label, "null");
        if (!chr) {
            error_report("Failed to assign serial port to altera-uart %s", label);
            exit(1);
        }
    }
    qdev_prop_set_chr(dev, "chardev", chr);

    bus = SYS_BUS_DEVICE(dev);
    qdev_init_nofail(dev);

    if (addr != (hwaddr)-1) {
        sysbus_mmio_map(bus, 0, addr);
    }

    sysbus_connect_irq(bus, 0, irq);
}

static const VMStateDescription vmstate_altera_uart = {
    .name = "altera-uart" ,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AlteraUARTState, ALTERA_UART_R_MAX),
        VMSTATE_END_OF_LIST()
    }
};

static void altera_uart_reset(DeviceState *d)
{
    AlteraUARTState *s = ALTERA_UART(d);

    s->regs[R_RXDATA] = 0;
    s->regs[R_TXDATA] = 0;
    s->regs[R_STATUS] = STATUS_TMT | STATUS_TRDY; /* Always ready to tx */
    s->regs[R_CONTROL] = 0;
}

static void altera_uart_realize(DeviceState *dev, Error **errp)
{
    AlteraUARTState *s = ALTERA_UART(dev);
    qemu_chr_fe_set_handlers(&s->chr, altera_uart_can_receive,
                             altera_uart_receive, NULL, s, NULL, true);
}

static void altera_uart_unrealize(DeviceState *dev, Error **errp)
{
}

static Property altera_uart_properties[] = {
    DEFINE_PROP_CHR("chardev", AlteraUARTState, chr),
    DEFINE_PROP_END_OF_LIST(),
};

static void altera_uart_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = altera_uart_realize;
    dc->unrealize = altera_uart_unrealize;
    dc->props = altera_uart_properties;
    dc->vmsd = &vmstate_altera_uart;
    dc->reset = altera_uart_reset;
    dc->desc = "Altera UART";
}

static const TypeInfo altera_uart_info = {
    .name          = TYPE_ALTERA_UART,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AlteraUARTState),
    .instance_init = altera_uart_init,
    .class_init    = altera_uart_class_init,
};

static void altera_uart_register(void)
{
    type_register_static(&altera_uart_info);
}

type_init(altera_uart_register)
