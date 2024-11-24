#ifndef HW_STM32L45_SYSCFG_H
#define HW_STM32L45_SYSCFG_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_STM32L45_SYSCFG "stm32l45-syscfg"
OBJECT_DECLARE_SIMPLE_TYPE(STM32L45SyscfgState, STM32L45_SYSCFG)

#define SYSCFG_NUM_EXTICR 4

/* Register offsets */
#define SYSCFG_MEMRMP     0x00
#define SYSCFG_PMC        0x04
#define SYSCFG_EXTICR1    0x08
#define SYSCFG_EXTICR2    0x0C
#define SYSCFG_EXTICR3    0x10
#define SYSCFG_EXTICR4    0x14
#define SYSCFG_SCSR       0x18
#define SYSCFG_CFGR2      0x1C
#define SYSCFG_SWPR       0x20
#define SYSCFG_SKR        0x24

struct STM32L45SyscfgState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    qemu_irq irq;
    qemu_irq gpio_out[16];
    Clock *clk; 

    /* Registers */
    uint32_t syscfg_memrmp;
    uint32_t syscfg_pmc;
    uint32_t syscfg_exticr[SYSCFG_NUM_EXTICR];
    uint32_t syscfg_scsr;
    uint32_t syscfg_cfgr2;
    uint32_t syscfg_swpr;
    uint32_t syscfg_skr;
};

#endif
