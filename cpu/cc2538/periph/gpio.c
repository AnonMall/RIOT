/*
 * Copyright (C) 2014 Loci Controls Inc.
 *               2016 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_cc2538
 * @{
 *
 * @file
 * @brief       Low-level GPIO driver implementation
 *
 * @author      Ian Martin <ian@locicontrols.com>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <stdint.h>

#include "cpu.h"
#include "periph/gpio.h"


#define GPIO_MASK           (0xfffff000)
//#define GPIO_MASK           (0xffffff38)
#define PORTNUM_MASK        (0x00003000)
//#define PORTNUM_MASK        (0x000038)
#define PORTNUM_SHIFT       (12U)
//#define PORTNUM_SHIFT       (3U)
#define PIN_MASK            (0x00000007)
#define MODE_NOTSUP         (0xff)

static inline cc2538_gpio_t *gpio(gpio_t pin)
{
    if(((uint32_t)pin &GPIO_MASK) == 0){
        uint32_t port = (pin & 0x18) >> 3;
        return (cc2538_gpio_t*)(((uint32_t)GPIO_A)+(port << PORTNUM_SHIFT));
    }else{
        return (cc2538_gpio_t *)(pin & GPIO_MASK);
    }
        //return (cc2538_gpio_t *)(pin & GPIO_MASK);
}

static inline int port_num(gpio_t pin)
{
    //return (int)((pin & PORTNUM_MASK) >> PORTNUM_SHIFT) - 1;
    if(((uint32_t)pin &GPIO_MASK) == 0){
        return ((pin & 0x18) >> 3);
    }
    return (int)(((pin - (uint32_t)GPIO_A) & PORTNUM_MASK) >> PORTNUM_SHIFT);
}

static inline int pin_num(gpio_t pin)
{
    return (int)(pin & PIN_MASK);
}

static inline uint32_t pin_mask(gpio_t pin)
{
    return (1 << (pin & PIN_MASK));
}

static inline int pp_num(gpio_t pin)
{
    return (port_num(pin) * 8) + pin_num(pin);
}

static gpio_isr_ctx_t isr_ctx[4][8];

int gpio_init(gpio_t pin, gpio_mode_t mode)
{
    /* check if mode is valid */
    if (mode == MODE_NOTSUP) {
        return -1;
    }


    /* disable any alternate function and any eventual interrupts */
    gpio(pin)->IE &= ~pin_mask(pin);
    gpio(pin)->AFSEL &= ~pin_mask(pin);
    /* configure pull configuration */
    IOC->OVER[pp_num(pin)] = mode;

    /* set pin direction */
    if (mode == IOC_OVERRIDE_OE) {
        gpio(pin)->DIR |= pin_mask(pin);
    }
    else {
        gpio(pin)->DIR &= ~pin_mask(pin);
    }
    /* clear pin */
    gpio(pin)->DATA &= ~pin_mask(pin);

    return 0;
}

int gpio_init_int(gpio_t pin, gpio_mode_t mode, gpio_flank_t flank,
                  gpio_cb_t cb, void *arg)
{

    if (gpio_init(pin, mode) != 0) {
        return -1;
    }

    /* store the callback information for later: */
    isr_ctx[port_num(pin)][pin_num(pin)].cb  = cb;
    isr_ctx[port_num(pin)][pin_num(pin)].arg = arg;

    /* enable power-up interrupts for this GPIO port: */
    SYS_CTRL->IWE |= (1 << port_num(pin));
    //SYS_CTRL->IWE |= port_num(pin);

    /* configure the active flank(s) */
    gpio(pin)->IS &= ~pin_mask(pin);
    switch(flank) {
        case GPIO_FALLING:
            gpio(pin)->IBE &= ~pin_mask(pin);
            gpio(pin)->IEV &= ~pin_mask(pin);
            gpio(pin)->P_EDGE_CTRL |= (1 << pp_num(pin));
            break;
        case GPIO_RISING:
            gpio(pin)->IBE &= ~pin_mask(pin);
            gpio(pin)->IEV |=  pin_mask(pin);
            gpio(pin)->P_EDGE_CTRL &= ~(1 << pp_num(pin));
            break;
        case GPIO_BOTH:
            gpio(pin)->IBE |= pin_mask(pin);
            break;
        default:
            return -1;
    }
    /* reset interrupt status */
    gpio(pin)->IC = pin_mask(pin);
    gpio(pin)->PI_IEN |= (1 << pp_num(pin));
    /* enable global interrupt for the selected GPIO port */
    printf("gp/io_init_int port: 0x%x\n", (unsigned int)(port_num(pin)&0x1FUL));
    NVIC_EnableIRQ(GPIO_PORT_A_IRQn + port_num(pin));
    //core_panic(PANIC_DEBUG_MON, message);
    /* unmask pin interrupt */
    gpio(pin)->IE |= pin_mask(pin);

    return 0;
}

void gpio_irq_enable(gpio_t pin)
{
    gpio(pin)->IE |= pin_mask(pin);
}

void gpio_irq_disable(gpio_t pin)
{
    gpio(pin)->IE &= ~pin_mask(pin);
}

int gpio_read(gpio_t pin)
{
    return (int)(gpio(pin)->DATA & pin_mask(pin));
}

void gpio_set(gpio_t pin)
{
    gpio(pin)->DATA |= pin_mask(pin);
}

void gpio_clear(gpio_t pin)
{
    gpio(pin)->DATA &= ~pin_mask(pin);
}

void gpio_toggle(gpio_t pin)
{
    gpio(pin)->DATA ^= pin_mask(pin);
}

void gpio_write(gpio_t pin, int value)
{
    if (value) {
        //gpio(pin)->DATA |= pin_mask(pin);
        gpio_set(pin);
    }
    else {
        //gpio(pin)->DATA &= ~pin_mask(pin);
        gpio_clear(pin);
    }
}

static inline void handle_isr(cc2538_gpio_t *gpio, int port_num)
{
    uint32_t state       = gpio->MIS;
    gpio->IC             = 0x000000ff;
    gpio->IRQ_DETECT_ACK = 0x000000ff;

    for (int i = 0; i < GPIO_BITS_PER_PORT; i++) {
        if (state & (1 << i)) {
            isr_ctx[port_num][i].cb(isr_ctx[port_num][i].arg);
        }
    }

    cortexm_isr_end();
}

/** @brief Interrupt service routine for Port A */
void isr_gpioa(void)
{
    handle_isr(GPIO_A, 0);
}

/** @brief Interrupt service routine for Port B */
void isr_gpiob(void)
{
    handle_isr(GPIO_B, 1);
}

/** @brief Interrupt service routine for Port C */
void isr_gpioc(void)
{
    handle_isr(GPIO_C, 2);
}

/** @brief Interrupt service routine for Port D */
void isr_gpiod(void)
{
    handle_isr(GPIO_D, 3);
}
