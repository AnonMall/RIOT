/*
 * Copyright (C) 2015
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_cc2538rf
 * @{
 *
 * @file
 * @brief       Implementation of public functions for CC2538RF drivers
 *
 * @author      Anon Mall <anon.mall@dai-labor.de>
 *
 * @}
 */

#include "xtimer.h"
#include "periph/cpuid.h"
#include "byteorder.h"
#include "net/ieee802154.h"
#include "net/gnrc.h"
#include "cc2538rf.h"

#define ENABLE_DEBUG (0)
#include "debug.h"


#define RESET_DELAY             (1U)        /* must be > 625ns */

/*
static void _irq_handler(void *arg)
{
}

int cc2538rf_init(cc2538rf_t *dev, spi_t spi, spi_speed_t spi_speed,
                   gpio_t cs_pin, gpio_t int_pin,
                   gpio_t sleep_pin, gpio_t
*/
int cc2538rf_init(cc2538rf_t *dev, gpio_t cs_pin, gpio_t int_pin,
                   gpio_t sleep_pin, gpio_t reset_pin)
{

    return 0;
}

void cc2538rf_reset(cc2538rf_t *dev)
{

    DEBUG("cc2538rf_reset(): reset complete.\n");
}

bool cc2538rf_cca(cc2538rf_t *dev)
{
  return false;
}

size_t cc2538rf_send(cc2538rf_t *dev, uint8_t *data, size_t len)
{
  return 0;
}

void cc2538rf_tx_prepare(cc2538rf_t *dev)
{
}

size_t cc2538rf_tx_load(cc2538rf_t *dev, uint8_t *data,
                         size_t len, size_t offset)
{
  return 0;
}

void cc2538rf_tx_exec(cc2538rf_t *dev)
{
}

size_t cc2538rf_rx_len(cc2538rf_t *dev)
{
  return 0;
}

void cc2538rf_rx_read(cc2538rf_t *dev, uint8_t *data, size_t len,
                       size_t offset)
{
}
