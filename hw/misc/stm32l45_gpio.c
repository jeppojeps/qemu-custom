#include "qemu/osdep.h"
#include "hw/misc/stm32l45_gpio.h"
#include "qemu/log.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
static uint64_t stm32l45_gpio_read(void *opaque, hwaddr addr,
                                  unsigned int size)
{
    STM32L45GPIOState *s = (STM32L45GPIOState *)opaque;
    uint32_t r = 0;

    switch (addr) {
    case 0x00: /* MODER */
        r = s->MODER;
        break;
    case 0x04: /* OTYPER */
        r = s->OTYPER;
        break;
    case 0x08: /* OSPEEDR */
        r = s->OSPEEDR;
        break;
    case 0x0C: /* PUPDR */
        r = s->PUPDR;
        break;
    case 0x10: /* IDR */
        r = s->IDR;
        break;
    case 0x14: /* ODR */
        r = s->ODR;
        break;
    case 0x18: /* BSRR */
        r = s->BSRR;
        break;
    case 0x1C: /* LCKR */
        r = s->LCKR;
        break;
    case 0x20: /* AFRL */
        r = s->AFR[0];
        break;
    case 0x24: /* AFRH */
        r = s->AFR[1];
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                    "STM32L45 GPIO: Read access to unknown register 0x%"
                    HWADDR_PRIx "\n", addr);
    }

    return r;
}

static void stm32l45_gpio_write(void *opaque, hwaddr addr,
                               uint64_t value, unsigned int size)
{
    STM32L45GPIOState *s = (STM32L45GPIOState *)opaque;

    switch (addr) {
    case 0x00: /* MODER */
        s->MODER = value;
        break;
    case 0x04: /* OTYPER */
        s->OTYPER = value;
        break;
    case 0x08: /* OSPEEDR */
        s->OSPEEDR = value;
        break;
    case 0x0C: /* PUPDR */
        s->PUPDR = value;
        break;
    case 0x14: /* ODR */
        s->ODR = value;
        qemu_log_mask(LOG_UNIMP, "GPIO%c ODR write: 0x%08" PRIx32 "\n",
                     'A' + s->port_id, (uint32_t)value);
        break;
    case 0x18: /* BSRR */
        /* Upper half clears bits, lower half sets bits */
        s->ODR &= ~((value >> 16) & 0xFFFF);
        s->ODR |= (value & 0xFFFF);
        qemu_log_mask(LOG_UNIMP, "GPIO%c BSRR write: 0x%08" PRIx32 " -> ODR: 0x%08"
                     PRIx32 "\n", 'A' + s->port_id, (uint32_t)value, s->ODR);
        break;
    case 0x1C: /* LCKR */
        s->LCKR = value;
        break;
    case 0x20: /* AFRL */
        s->AFR[0] = value;
        break;
    case 0x24: /* AFRH */
        s->AFR[1] = value;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                    "STM32L45 GPIO: Write access to unknown register 0x%"
                    HWADDR_PRIx "\n", addr);
    }
}

static const MemoryRegionOps stm32l45_gpio_ops = {
    .read = stm32l45_gpio_read,
    .write = stm32l45_gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


static void stm32l45_gpio_init(Object *obj)
{
    STM32L45GPIOState *s = STM32L45_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &stm32l45_gpio_ops, s,
                         TYPE_STM32L45_GPIO, 0x400);
    sysbus_init_mmio(sbd, &s->mmio);
}

static void stm32l45_gpio_reset(DeviceState *dev)
{
    STM32L45GPIOState *s = STM32L45_GPIO(dev);
    
    s->MODER = 0;
    s->OTYPER = 0;
    s->OSPEEDR = 0;
    s->PUPDR = 0;
    s->IDR = 0;
    s->ODR = 0;
    s->BSRR = 0;
    s->LCKR = 0;
    s->AFR[0] = 0;
    s->AFR[1] = 0;
}

static Property stm32l45_gpio_properties[] = {
    DEFINE_PROP_UINT8("port-id", STM32L45GPIOState, port_id, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void stm32l45_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = stm32l45_gpio_reset;
    device_class_set_props(dc, stm32l45_gpio_properties);
}

static const TypeInfo stm32l45_gpio_info = {
    .name          = TYPE_STM32L45_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32L45GPIOState),
    .instance_init = stm32l45_gpio_init,
    .class_init    = stm32l45_gpio_class_init,
};

static void stm32l45_gpio_register_types(void)
{
    type_register_static(&stm32l45_gpio_info);
}

type_init(stm32l45_gpio_register_types)
