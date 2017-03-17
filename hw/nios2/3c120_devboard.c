/*
 * Altera 3C120 Nios2 GHRD
 *
 * Copyright (c) 2017 Tobias Klauser <tklauser@distanz.ch>
 *
 * Based on 10m50_devboard which is:
 *
 * Copyright (c) 2016 Marek Vasut <marek.vasut@gmail.com>
 * Copyright (c) 2012 Chris Wulff <crwulff@gmail.com>
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
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "cpu.h"

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/char/serial.h"
#include "hw/char/altera_juart.h"
#include "hw/char/altera_uart.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "qemu/config-file.h"

#include "boot.h"

#define BINARY_DEVICE_TREE_FILE    "3c120-devboard.dtb"

static void nios2_3c120_ghrd_init(MachineState *machine)
{
    Nios2CPU *cpu;
    DeviceState *dev;
    MemoryRegion *address_space_mem = get_system_memory();
    MemoryRegion *phys_tcm = g_new(MemoryRegion, 1);
    MemoryRegion *phys_tcm_alias = g_new(MemoryRegion, 1);
    MemoryRegion *phys_ram = g_new(MemoryRegion, 1);
    MemoryRegion *phys_ram_alias = g_new(MemoryRegion, 1);
    ram_addr_t tcm_base = 0x07fff400;
    ram_addr_t tcm_size = 0x1000;    /* 1 kiB, but QEMU limit is 4 kiB */
    ram_addr_t ram_base = 0x10000000;
    ram_addr_t ram_size = 0x08000000;
    qemu_irq *cpu_irq, irq[32];
    int i;

    /* Physical TCM (tb_ram_1k) with alias at 0xc0000000 */
    memory_region_init_ram(phys_tcm, NULL, "nios2.tcm", tcm_size, &error_abort);
    memory_region_init_alias(phys_tcm_alias, NULL, "nios2.tcm.alias",
                             phys_tcm, 0, tcm_size);
    vmstate_register_ram_global(phys_tcm);
    memory_region_add_subregion(address_space_mem, tcm_base, phys_tcm);
    memory_region_add_subregion(address_space_mem, 0xc0000000 + tcm_base,
                                phys_tcm_alias);

    /* Physical DRAM with alias at 0xc0000000 */
    memory_region_init_ram(phys_ram, NULL, "nios2.ram", ram_size, &error_abort);
    memory_region_init_alias(phys_ram_alias, NULL, "nios2.ram.alias",
                             phys_ram, 0, ram_size);
    vmstate_register_ram_global(phys_ram);
    memory_region_add_subregion(address_space_mem, ram_base, phys_ram);
    memory_region_add_subregion(address_space_mem, 0xc0000000 + ram_base,
                                phys_ram_alias);

    /* Create CPU -- FIXME */
    cpu = cpu_nios2_init("nios2");

    /* Register: CPU interrupt controller (PIC) */
    cpu_irq = nios2_cpu_pic_init(cpu);

    /* Register: Internal Interrupt Controller (IIC) */
    dev = qdev_create(NULL, "altera,iic");
    qdev_prop_set_ptr(dev, "cpu", cpu);
    qdev_init_nofail(dev);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, cpu_irq[0]);
    for (i = 0; i < 32; i++) {
        irq[i] = qdev_get_gpio_in(dev, i);
    }

    /* Register: Altera JTAG UART */
    altera_juart_create(1, 0xf0004d50, irq[1], ALTERA_JUART_DEFAULT_FIFO_SIZE);

    /* Register: Altera UART */
    altera_uart_create(1, 0xf0004c80, irq[10]);

    /* Register: Timer sys_clk_timer  */
    dev = qdev_create(NULL, "ALTR.timer");
    qdev_prop_set_uint32(dev, "clock-frequency", 125 * 1000000);
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0xf0400000);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq[11]);

    /* Register: Timer sys_clk_timer_1  */
    dev = qdev_create(NULL, "ALTR.timer");
    qdev_prop_set_uint32(dev, "clock-frequency", 125 * 1000000);
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0xe0008000);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq[5]);

    /* Configure new exception vectors and reset CPU for it to take effect. */
    cpu->reset_addr = 0xc2800000;
    cpu->exception_addr = 0xd0000020;
    cpu->fast_tlb_miss_addr = 0xc7fff400;

    nios2_load_kernel(cpu, ram_base, ram_size, machine->initrd_filename,
                      BINARY_DEVICE_TREE_FILE, NULL);
}

static void nios2_3c120_ghrd_machine_init(struct MachineClass *mc)
{
    mc->desc = "Altera 3C120 GHRD Nios II design";
    mc->init = nios2_3c120_ghrd_init;
    mc->is_default = 1;
}

DEFINE_MACHINE("3c120-ghrd", nios2_3c120_ghrd_machine_init);
