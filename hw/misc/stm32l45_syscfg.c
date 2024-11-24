#include "qemu/osdep.h"
#include "qemu/log.h"
#include "trace.h"
#include "hw/qdev-clock.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "hw/misc/stm32l45_syscfg.h"

static void stm32l45_syscfg_reset(DeviceState *dev)
{
    STM32L45SyscfgState *s = STM32L45_SYSCFG(dev);

    s->syscfg_memrmp = 0x00000000;
    s->syscfg_pmc = 0x00000000;
    s->syscfg_exticr[0] = 0x00000000;
    s->syscfg_exticr[1] = 0x00000000;
    s->syscfg_exticr[2] = 0x00000000;
    s->syscfg_exticr[3] = 0x00000000;
    s->syscfg_scsr = 0x00000000;
    s->syscfg_cfgr2 = 0x00000000;
    s->syscfg_swpr = 0x00000000;
    s->syscfg_skr = 0x00000000;
}

static void stm32l45_syscfg_set_irq(void *opaque, int irq, int level)
{
    STM32L45SyscfgState *s = opaque;
    int icrreg = irq / 4;
    int startbit = (irq & 3) * 4;
    uint8_t config = irq / 16;

    if (icrreg < SYSCFG_NUM_EXTICR) {
        if (extract32(s->syscfg_exticr[icrreg], startbit, 4) == config) {
            qemu_set_irq(s->gpio_out[irq], level);
        }
    }
}

static uint64_t stm32l45_syscfg_read(void *opaque, hwaddr addr, unsigned int size)
{
    STM32L45SyscfgState *s = opaque;

    switch (addr) {
    case SYSCFG_MEMRMP:
        return s->syscfg_memrmp;
    case SYSCFG_PMC:
        return s->syscfg_pmc;
    case SYSCFG_EXTICR1...SYSCFG_EXTICR4:
        return s->syscfg_exticr[addr / 4 - SYSCFG_EXTICR1 / 4];
    case SYSCFG_SCSR:
        return s->syscfg_scsr;
    case SYSCFG_CFGR2:
        return s->syscfg_cfgr2;
    case SYSCFG_SWPR:
        return s->syscfg_swpr;
    case SYSCFG_SKR:
        return s->syscfg_skr;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, addr);
        return 0;
    }
}

static void stm32l45_syscfg_write(void *opaque, hwaddr addr,
                                 uint64_t val64, unsigned int size)
{
    STM32L45SyscfgState *s = opaque;
    uint32_t value = val64;

    switch (addr) {
    case SYSCFG_MEMRMP:
        s->syscfg_memrmp = value & 0x3;  // Only bits [1:0] are writable
        qemu_log_mask(LOG_UNIMP,
                     "STM32L45 SYSCFG: Memory remap not implemented\n");
        break;
    case SYSCFG_PMC:
        s->syscfg_pmc = value & 0x7F;    // Bits [6:0] are writable
        break;
    case SYSCFG_EXTICR1...SYSCFG_EXTICR4:
        s->syscfg_exticr[addr / 4 - SYSCFG_EXTICR1 / 4] = value & 0xFFFF;
        break;
    case SYSCFG_SCSR:
        // SRAM2 busy flag is read-only, only write to writable bits
        s->syscfg_scsr = (s->syscfg_scsr & 0x1) | (value & ~0x1);
        break;
    case SYSCFG_CFGR2:
        s->syscfg_cfgr2 = value & 0x3F;  // Bits [5:0] are writable
        break;
    case SYSCFG_SWPR:
        s->syscfg_swpr = value;          // All bits are writable
        break;
    case SYSCFG_SKR:
        s->syscfg_skr = value & 0xFF;    // Bits [7:0] are writable
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                     "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, addr);
    }
}

static const MemoryRegionOps stm32l45_syscfg_ops = {
    .read = stm32l45_syscfg_read,
    .write = stm32l45_syscfg_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

//static void stm32l45_syscfg_init(Object *obj)
//{
//    STM32L45SyscfgState *s = STM32L45_SYSCFG(obj);
//
//    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
//
//    memory_region_init_io(&s->mmio, obj, &stm32l45_syscfg_ops, s,
//                         TYPE_STM32L45_SYSCFG, 0x400);
//    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
//
//    /* 16 possible EXTI lines for 9 ports (A through I) */
//    qdev_init_gpio_in(DEVICE(obj), stm32l45_syscfg_set_irq, 16 * 9);
//    qdev_init_gpio_out(DEVICE(obj), s->gpio_out, 16);
//}


static void stm32l45_syscfg_init(Object *obj)
{
    STM32L45SyscfgState *s = STM32L45_SYSCFG(obj);

    /* Add clock initialization */
    qdev_init_clock_in(DEVICE(s), "clk", NULL, NULL, 0);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    memory_region_init_io(&s->mmio, obj, &stm32l45_syscfg_ops, s,
                         TYPE_STM32L45_SYSCFG, 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);

    /* 16 possible EXTI lines for 9 ports (A through I) */
    qdev_init_gpio_in(DEVICE(obj), stm32l45_syscfg_set_irq, 16 * 9);
    qdev_init_gpio_out(DEVICE(obj), s->gpio_out, 16);
}

static const VMStateDescription vmstate_stm32l45_syscfg = {
    .name = TYPE_STM32L45_SYSCFG,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(syscfg_memrmp, STM32L45SyscfgState),
        VMSTATE_UINT32(syscfg_pmc, STM32L45SyscfgState),
        VMSTATE_UINT32_ARRAY(syscfg_exticr, STM32L45SyscfgState,
                            SYSCFG_NUM_EXTICR),
        VMSTATE_UINT32(syscfg_scsr, STM32L45SyscfgState),
        VMSTATE_UINT32(syscfg_cfgr2, STM32L45SyscfgState),
        VMSTATE_UINT32(syscfg_swpr, STM32L45SyscfgState),
        VMSTATE_UINT32(syscfg_skr, STM32L45SyscfgState),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32l45_syscfg_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = stm32l45_syscfg_reset;
    dc->vmsd = &vmstate_stm32l45_syscfg;
}

static const TypeInfo stm32l45_syscfg_info = {
    .name          = TYPE_STM32L45_SYSCFG,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32L45SyscfgState),
    .instance_init = stm32l45_syscfg_init,
    .class_init    = stm32l45_syscfg_class_init,
};

static void stm32l45_syscfg_register_types(void)
{
    type_register_static(&stm32l45_syscfg_info);
}

type_init(stm32l45_syscfg_register_types)
