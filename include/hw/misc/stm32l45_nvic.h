#ifndef HW_STM32L45_NVIC_H
#define HW_STM32L45_NVIC_H

#include "hw/sysbus.h"
#include "qemu/log.h"

#define TYPE_STM32L45_NVIC "stm32l45-nvic"
OBJECT_DECLARE_SIMPLE_TYPE(STM32L45NVICState, STM32L45_NVIC)

// NVIC Register Offsets
#define NVIC_AIRCR_OFFSET     0xD0C
#define NVIC_VTOR_OFFSET      0xD08
#define NVIC_ICSR_OFFSET      0xD04
#define NVIC_SHPR1_OFFSET     0xD18
#define NVIC_SHPR2_OFFSET     0xD1C
#define NVIC_SHPR3_OFFSET     0xD20

// AIRCR bits
#define AIRCR_VECTKEY_MASK    0xFFFF0000
#define AIRCR_VECTKEY_WRITE   0x05FA0000
#define AIRCR_VECTKEY_READ    0xFA050000
#define AIRCR_PRIGROUP_MASK   0x00000700
#define AIRCR_PRIGROUP_SHIFT  8

typedef struct STM32L45NVICState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/
    
    MemoryRegion mmio;
    
    // Core NVIC registers
    uint32_t aircr;  // Application Interrupt and Reset Control Register
    uint32_t vtor;   // Vector Table Offset Register
    uint32_t icsr;   // Interrupt Control and State Register
    uint32_t shpr[3];// System Handler Priority Registers
    
    // Additional state if needed
    uint8_t irq_state[240];  // State for each possible interrupt
    qemu_irq irq[240];       // IRQ outputs
} STM32NVICState;


#endif /* STM32_NVIC_H */