#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "migration/vmstate.h"
#include "hw/misc/stm32l45_rcc.h"
#include "hw/clock.h"
#include "hw/irq.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/registerfields.h"
#include "trace.h"


static uint64_t stm32_nvic_read(void *opaque, hwaddr addr, unsigned size)
{
    STM32NVICState *s = (STM32NVICState *)opaque;
    
    switch (addr) {
        case NVIC_AIRCR_OFFSET:
            // Return current AIRCR value with correct key
            return s->aircr | AIRCR_VECTKEY_READ;
            
        case NVIC_VTOR_OFFSET:
            return s->vtor;
            
        case NVIC_ICSR_OFFSET:
            return s->icsr;
            
        case NVIC_SHPR1_OFFSET:
            return s->shpr[0];
            
        case NVIC_SHPR2_OFFSET:
            return s->shpr[1];
            
        case NVIC_SHPR3_OFFSET:
            return s->shpr[2];
            
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                         "stm32_nvic_read: Bad offset %x\n", (int)addr);
            return 0;
    }
}

static void stm32_nvic_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    STM32NVICState *s = (STM32NVICState *)opaque;
    
    switch (addr) {
        case NVIC_AIRCR_OFFSET:
            // Only accept writes with correct key
            if ((value & AIRCR_VECTKEY_MASK) == AIRCR_VECTKEY_WRITE) {
                s->aircr = (value & ~AIRCR_VECTKEY_MASK) | AIRCR_VECTKEY_READ;
            }
            break;
            
        case NVIC_VTOR_OFFSET:
            s->vtor = value & 0xFFFFFF80; // Aligned to 128-byte boundary
            break;
            
        case NVIC_ICSR_OFFSET:
            // Only allow writing to certain bits
            s->icsr = (s->icsr & 0xFFFF0000) | (value & 0x0000FFFF);
            break;
            
        case NVIC_SHPR1_OFFSET:
            s->shpr[0] = value;
            break;
            
        case NVIC_SHPR2_OFFSET:
            s->shpr[1] = value;
            break;
            
        case NVIC_SHPR3_OFFSET:
            s->shpr[2] = value;
            break;
            
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                         "stm32_nvic_write: Bad offset %x\n", (int)addr);
    }
}

static const MemoryRegionOps stm32_nvic_ops = {
    .read = stm32_nvic_read,
    .write = stm32_nvic_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void stm32_nvic_init(Object *obj)
{
    STM32NVICState *s = STM32_NVIC(obj);
    
    memory_region_init_io(&s->iomem, obj, &stm32_nvic_ops, s,
                         TYPE_STM32_NVIC, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    
    // Initialize IRQ lines
    qdev_init_gpio_out(DEVICE(obj), s->irq, 240);
    
    // Set default register values
    s->aircr = 0x0300; // PRIGROUP = 3 by default
    s->vtor = 0;
    s->icsr = 0;
    memset(s->shpr, 0, sizeof(s->shpr));
    memset(s->irq_state, 0, sizeof(s->irq_state));
}

// External API implementation
void stm32_nvic_set_pending(void *opaque, int irq)
{
    STM32NVICState *s = (STM32NVICState *)opaque;
    if (irq >= 0 && irq < 240) {
        s->irq_state[irq] |= 0x01;
    }
}

void stm32_nvic_clear_pending(void *opaque, int irq)
{
    STM32NVICState *s = (STM32NVICState *)opaque;
    if (irq >= 0 && irq < 240) {
        s->irq_state[irq] &= ~0x01;
    }
}

int stm32_nvic_acknowledge_irq(void *opaque)
{
    STM32NVICState *s = (STM32NVICState *)opaque;
    // Basic priority-based IRQ acknowledgment
    for (int i = 0; i < 240; i++) {
        if (s->irq_state[i] & 0x01) {
            return i;
        }
    }
    return -1;
}

void stm32_nvic_complete_irq(void *opaque, int irq)
{
    STM32NVICState *s = (STM32NVICState *)opaque;
    if (irq >= 0 && irq < 240) {
        s->irq_state[irq] &= ~0x01;
    }
}

static const TypeInfo stm32_nvic_info = {
    .name          = TYPE_STM32_NVIC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32NVICState),
    .instance_init = stm32_nvic_init,
};

static void stm32_nvic_register_types(void)
{
    type_register_static(&stm32_nvic_info);
}

type_init(stm32_nvic_register_types)