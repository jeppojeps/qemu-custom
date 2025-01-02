/*
 * STM32L4 SysTick Timer 
 *
 * Implementation of the ARM Cortex-M SysTick timer
 * configured for 4MHz clock
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "migration/vmstate.h"
#include "trace.h"

#define TYPE_STM32L4_SYSTICK "stm32l4-systick"
#define STM32L4_SYSTICK(obj) \
    OBJECT_CHECK(STM32L4SysTickState, (obj), TYPE_STM32L4_SYSTICK)

/* Register offsets */
#define SYSTICK_CSR    0x00  /* Control and Status Register */
#define SYSTICK_RVR    0x04  /* Reload Value Register */
#define SYSTICK_CVR    0x08  /* Current Value Register */
#define SYSTICK_CALIB  0x0C  /* Calibration Register */

/* CSR bit masks */
#define SYSTICK_CSR_ENABLE     (1 << 0)
#define SYSTICK_CSR_TICKINT    (1 << 1)
#define SYSTICK_CSR_CLKSOURCE  (1 << 2)
#define SYSTICK_CSR_COUNTFLAG  (1 << 16)

typedef struct STM32L4SysTickState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    QEMUTimer *timer;
    qemu_irq irq;

    uint32_t control;     /* SYST_CSR value */
    uint32_t reload;      /* SYST_RVR value */
    uint32_t current;     /* SYST_CVR value */
    uint32_t calibration; /* SYST_CALIB value */
    
    uint64_t tick_start_time;
    uint32_t freq_hz;     /* 4MHz system clock */
} STM32L4SysTickState;

static void stm32l4_systick_timeout(void *opaque);

static void stm32l4_systick_update(STM32L4SysTickState *s)
{
    uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    bool enabled = s->control & SYSTICK_CSR_ENABLE;
    
    qemu_log_mask(LOG_SYSTICK, "Update - Control: 0x%x, Current: 0x%x, Reload: 0x%x\n", 
                 s->control, s->current, s->reload);
    
    if (!enabled) {
        timer_del(s->timer);
        qemu_log_mask(LOG_SYSTICK, "Timer disabled\n");
        return;
    }

    uint64_t next_tick;
    uint64_t tick_period = NANOSECONDS_PER_SECOND / s->freq_hz;

    if (s->current <= 1) {
        s->current = s->reload;
        next_tick = now + tick_period;
        
        s->control |= SYSTICK_CSR_COUNTFLAG;

        if (s->control & SYSTICK_CSR_TICKINT) {
            qemu_log_mask(LOG_SYSTICK, "Generating interrupt\n");
            qemu_irq_pulse(s->irq);
        }
    } else {
        uint64_t elapsed = now - s->tick_start_time;
        uint64_t ticks = elapsed / tick_period;
        if (ticks > 0) {
            if (ticks >= s->current) {
                s->current = s->reload;
                if (s->control & SYSTICK_CSR_TICKINT) {
                    qemu_log_mask(LOG_SYSTICK, "Generating interrupt on overflow\n");
                    qemu_irq_pulse(s->irq);
                }
                s->control |= SYSTICK_CSR_COUNTFLAG;
            } else {
                s->current -= ticks;
            }
        }
        next_tick = now + tick_period;
    }

    qemu_log_mask(LOG_SYSTICK, "Next tick in %"PRId64" ns\n", next_tick - now);
    s->tick_start_time = now;
    timer_mod(s->timer, next_tick);
}

static void stm32l4_systick_timeout(void *opaque)
{
    STM32L4SysTickState *s = STM32L4_SYSTICK(opaque);
    qemu_log_mask(LOG_SYSTICK, "Timer timeout\n");
    stm32l4_systick_update(s);
}

static uint64_t stm32l4_systick_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    STM32L4SysTickState *s = STM32L4_SYSTICK(opaque);
    uint32_t value = 0;

    switch (offset) {
    case SYSTICK_CSR:
        value = s->control;
        s->control &= ~SYSTICK_CSR_COUNTFLAG;
        qemu_log_mask(LOG_SYSTICK, "Read CSR: 0x%x\n", value);
        break;
    case SYSTICK_RVR:
        value = s->reload;
        qemu_log_mask(LOG_SYSTICK, "Read RVR: 0x%x\n", value);
        break;
    case SYSTICK_CVR:
        value = s->current;
        qemu_log_mask(LOG_SYSTICK, "Read CVR: 0x%x\n", value);
        break;
    case SYSTICK_CALIB:
        value = s->calibration;
        qemu_log_mask(LOG_SYSTICK, "Read CALIB: 0x%x\n", value);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                     "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, offset);
    }

    return value;
}

static void stm32l4_systick_write(void *opaque, hwaddr offset,
                                 uint64_t value, unsigned size)
{
    STM32L4SysTickState *s = STM32L4_SYSTICK(opaque);
    
    switch (offset) {
    case SYSTICK_CSR:
        s->control = value & 0x00010007;
        qemu_log_mask(LOG_SYSTICK, "Write CSR: 0x%x\n", (uint32_t)value);
        if (value & SYSTICK_CSR_ENABLE) {
            s->tick_start_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            timer_mod(s->timer, s->tick_start_time + 
                     (NANOSECONDS_PER_SECOND / s->freq_hz));
            qemu_log_mask(LOG_SYSTICK, "Timer enabled\n");
        }
        break;
    case SYSTICK_RVR:
        s->reload = value & 0x00FFFFFF;
        qemu_log_mask(LOG_SYSTICK, "Write RVR: 0x%x\n", (uint32_t)value);
        break;
    case SYSTICK_CVR:
        qemu_log_mask(LOG_SYSTICK, "Write CVR: 0x%x\n", (uint32_t)value);
        s->current = 0; // Any write clears to 0
        s->control &= ~SYSTICK_CSR_COUNTFLAG;
        s->tick_start_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        timer_mod(s->timer, s->tick_start_time +
                 (NANOSECONDS_PER_SECOND / s->freq_hz));
        break;
    case SYSTICK_CALIB:
        qemu_log_mask(LOG_SYSTICK, "Attempted write to read-only CALIB register\n");
        break;
    }
}

static const MemoryRegionOps stm32l4_systick_ops = {
    .read = stm32l4_systick_read,
    .write = stm32l4_systick_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void stm32l4_systick_init(Object *obj)
{
    STM32L4SysTickState *s = STM32L4_SYSTICK(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &stm32l4_systick_ops, s,
                         TYPE_STM32L4_SYSTICK, 0x10);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, stm32l4_systick_timeout, s);
    s->freq_hz = 4000000; /* 4MHz clock */
    
    /* Initialize calibration register for 4MHz clock */
    s->calibration = 0x00000FA0 |  /* 10ms at 4MHz = 40000 = 0xFA0 */
                    (0 << 30) |    /* No SKEW */
                    (0 << 31);     /* NOREF = 0 */
    
    qemu_log_mask(LOG_SYSTICK, "SysTick initialized with 4MHz clock\n");
}


static const VMStateDescription vmstate_stm32l4_systick = {
    .name = TYPE_STM32L4_SYSTICK,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(control, STM32L4SysTickState),
        VMSTATE_UINT32(reload, STM32L4SysTickState),
        VMSTATE_UINT32(current, STM32L4SysTickState),
        VMSTATE_UINT32(calibration, STM32L4SysTickState),
        VMSTATE_UINT64(tick_start_time, STM32L4SysTickState),
        VMSTATE_UINT32(freq_hz, STM32L4SysTickState),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32l4_systick_class_init(ObjectClass *klass, void *data)
{
    qemu_log_mask(LOG_SYSTICK, "==== SysTick device initialization start ====\n");
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->desc = "STM32L4 SysTick Timer";
    dc->vmsd = &vmstate_stm32l4_systick;
    qemu_log_mask(LOG_SYSTICK, "==== SysTick device initialization complete ====\n");
}

static const TypeInfo stm32l4_systick_info = {
    .name = TYPE_STM32L4_SYSTICK,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32L4SysTickState),
    .instance_init = stm32l4_systick_init,
    .class_init = stm32l4_systick_class_init,
};

static void stm32l4_systick_register_types(void)
{
    type_register_static(&stm32l4_systick_info);
}

type_init(stm32l4_systick_register_types)
