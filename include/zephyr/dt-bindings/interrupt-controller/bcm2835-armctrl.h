/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_INTERRUPT_CONTROLLER_BCM2835_ARMCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_INTERRUPT_CONTROLLER_BCM2835_ARMCTRL_H_

/*
 * BCM2835 ARMC peripheral interrupt numbering, exposed as a flat
 * 0..71 space:
 *   basic bank: 0..7
 *   GPU PEND1:  8..39  (bits 0..31)
 *   GPU PEND2:  40..71 (bits 0..31)
 *
 * Use BCM2835_BANK1(n) / BCM2835_BANK2(n) for any source not enumerated
 * here.
 */

#define BCM2835_BASIC_IRQ(n)        (n)               /* 0..7  */
#define BCM2835_BANK1(n)            (8 + (n))         /* PEND1 */
#define BCM2835_BANK2(n)            (40 + (n))        /* PEND2 */

/* basic bank shortcuts */
#define BCM2835_IRQ_ARM_TIMER       BCM2835_BASIC_IRQ(0)
#define BCM2835_IRQ_ARM_MAILBOX     BCM2835_BASIC_IRQ(1)
#define BCM2835_IRQ_ARM_DOORBELL0   BCM2835_BASIC_IRQ(2)
#define BCM2835_IRQ_ARM_DOORBELL1   BCM2835_BASIC_IRQ(3)
#define BCM2835_IRQ_GPU0_HALT       BCM2835_BASIC_IRQ(4)
#define BCM2835_IRQ_GPU1_HALT       BCM2835_BASIC_IRQ(5)
#define BCM2835_IRQ_ILLEGAL_TYPE0   BCM2835_BASIC_IRQ(6)
#define BCM2835_IRQ_ILLEGAL_TYPE1   BCM2835_BASIC_IRQ(7)

/* GPU PEND1 (peripheral lower bank) */
#define BCM2835_IRQ_TIMER0          BCM2835_BANK1(0)
#define BCM2835_IRQ_TIMER1          BCM2835_BANK1(1)
#define BCM2835_IRQ_TIMER2          BCM2835_BANK1(2)
#define BCM2835_IRQ_TIMER3          BCM2835_BANK1(3)
#define BCM2835_IRQ_AUX             BCM2835_BANK1(29) /* mini-UART, SPI1, SPI2 */

/* GPU PEND2 (peripheral upper bank) */
#define BCM2835_IRQ_I2C_SPI_SLV     BCM2835_BANK2(11)
#define BCM2835_IRQ_GPIO0           BCM2835_BANK2(17)
#define BCM2835_IRQ_GPIO1           BCM2835_BANK2(18)
#define BCM2835_IRQ_GPIO2           BCM2835_BANK2(19)
#define BCM2835_IRQ_GPIO3           BCM2835_BANK2(20)
#define BCM2835_IRQ_I2C             BCM2835_BANK2(21)
#define BCM2835_IRQ_SPI             BCM2835_BANK2(22)
#define BCM2835_IRQ_PCM             BCM2835_BANK2(23)
#define BCM2835_IRQ_UART            BCM2835_BANK2(25) /* PL011 UART0 */

#ifndef IRQ_TYPE_LEVEL
#define IRQ_TYPE_LEVEL  2
#endif
#ifndef IRQ_TYPE_EDGE
#define IRQ_TYPE_EDGE   4
#endif

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_INTERRUPT_CONTROLLER_BCM2835_ARMCTRL_H_ */
