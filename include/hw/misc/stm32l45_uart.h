#ifndef STM32L45_UART_H
#define STM32L45_UART_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/char/serial.h"
#include "hw/registerfields.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "migration/vmstate.h"
#include "chardev/char-fe.h"
#include "hw/qdev-clock.h"
#define UART_BRR_MIN 0x10
#define UART_BRR_MAX 0xFFFF

#define TYPE_STM32L45_UART "stm32l45-uart"
#define STM32L45_UART(obj) \
    OBJECT_CHECK(STM32L45UARTState, (obj), TYPE_STM32L45_UART)

/* Register offsets */

#define USART_CR1_OFFSET   0x00  /* Control register 1 */
#define USART_CR2_OFFSET   0x04  /* Control register 2 */
#define USART_CR3_OFFSET   0x08  /* Control register 3 */
#define USART_CFGR_OFFSET  0x0C  /* Configuration register */
#define USART_BRR_OFFSET   0x10  /* Baud rate register */
#define USART_GTPR_OFFSET  0x14  /* Guard time and prescaler register */
#define USART_RTOR_OFFSET  0x18  /* Receiver timeout register */
#define USART_RQR_OFFSET   0x1C  /* Request register */
#define USART_ISR_OFFSET   0x20  /* Interrupt and status register */
#define USART_ICR_OFFSET   0x24  /* Interrupt flag clear register */
#define USART_RDR_OFFSET   0x28  /* Receive data register */
#define USART_TDR_OFFSET   0x2C  /* Transmit data register */
#define USART_PRESC_OFFSET 0x30  /* Prescaler register */

/* Register field definitions */
REG32(USART_CR1, USART_CR1_OFFSET)
    FIELD(USART_CR1, UE, 0, 1)    /* UART enable */
    FIELD(USART_CR1, UESM, 1, 1)  /* UART enable in Stop mode */
    FIELD(USART_CR1, RE, 2, 1)    /* Receiver enable */
    FIELD(USART_CR1, TE, 3, 1)    /* Transmitter enable */
    FIELD(USART_CR1, M0, 12, 1)   /* Word length bit 0 */
    FIELD(USART_CR1, M1, 28, 1)   /* Word length bit 1 */
    FIELD(USART_CR1, OVER8, 15, 1) /* Oversampling mode */

REG32(USART_CR2, USART_CR2_OFFSET)
    FIELD(USART_CR2, STOP, 12, 2)  /* STOP bits */

REG32(USART_CR3, USART_CR3_OFFSET)
    FIELD(USART_CR3, ONEBIT, 11, 1) /* One sample bit method enable */
    FIELD(USART_CR3, OVRDIS, 12, 1) /* Overrun Disable */
    FIELD(USART_CR3, HDSEL, 3, 1)   /* Half-duplex selection */

REG32(USART_CFGR, USART_CFGR_OFFSET)
    FIELD(USART_CFGR, PRESCALER, 0, 4)   /* Prescaler configuration */
    FIELD(USART_CFGR, RESERVED1, 4, 4)   /* Reserved bits */
    FIELD(USART_CFGR, MSBLAST, 8, 1)     /* Most significant bit first */
    FIELD(USART_CFGR, STOP, 12, 2)       /* STOP bits configuration */
    FIELD(USART_CFGR, SWAP, 14, 1)       /* Swap TX/RX pins */
    FIELD(USART_CFGR, RXINV, 15, 1)      /* RX pin inverse */
    FIELD(USART_CFGR, TXINV, 16, 1)      /* TX pin inverse */
    FIELD(USART_CFGR, TAINV, 17, 1)      /* TxD/RxD pin active level inversion */
    FIELD(USART_CFGR, MSBFIRST, 18, 1)   /* Most significant bit first */
    FIELD(USART_CFGR, RESERVED2, 19, 13) /* Reserved bits */

REG32(USART_BRR, USART_BRR_OFFSET)

REG32(USART_ISR, USART_ISR_OFFSET)
    FIELD(USART_ISR, PE, 0, 1)      /* Parity error */
    FIELD(USART_ISR, FE, 1, 1)      /* Framing error */
    FIELD(USART_ISR, NF, 2, 1)      /* Noise detection flag */
    FIELD(USART_ISR, ORE, 3, 1)     /* Overrun error */
    FIELD(USART_ISR, IDLE, 4, 1)    /* Idle line detected */
    FIELD(USART_ISR, RXNE, 5, 1)    /* Read data register not empty */
    FIELD(USART_ISR, TC, 6, 1)      /* Transmission complete */
    FIELD(USART_ISR, TXE, 7, 1)     /* Transmit data register empty */

REG32(USART_ICR, USART_ICR_OFFSET)
REG32(USART_RDR, USART_RDR_OFFSET)
REG32(USART_TDR, USART_TDR_OFFSET)
REG32(USART_PRESC, USART_PRESC_OFFSET)
REG32(USART_GTPR, USART_GTPR_OFFSET)
REG32(USART_RTOR, USART_RTOR_OFFSET)
REG32(USART_RQR, USART_RQR_OFFSET)


typedef struct {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    CharBackend chr;
    qemu_irq irq;
    // Add clock inputs
    Clock *pclk1;     // PCLK1 clock input
    Clock *sysclk;    // SYSCLK clock input
    uint32_t cr1;
    uint32_t cfgr;
    uint32_t cr2;
    uint32_t cr3;
    uint32_t brr;
    uint32_t gtpr;
    uint32_t rtor;
    uint32_t rqr;
    uint32_t isr;
    uint32_t icr;
    uint32_t rdr;
    uint32_t tdr;
    uint32_t presc;
} STM32L45UARTState;

#endif /* STM32L45_UART_H */
