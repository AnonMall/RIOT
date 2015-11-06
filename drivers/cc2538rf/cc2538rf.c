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

#define UDMA_ENABLED 0

/*Some Status Flags for the Radio */
static uint8_t rf_flags;

/* Local RF Flags */
#define RX_ACTIVE     0x80
#define RF_MUST_RESET 0x40
#define RF_ON         0x01

#define RESET_DELAY             (1U)        /* must be > 625ns */

static int _send(gnrc_netdev_t *netdev, gnrc_pktsnip_t *pkt){

return -1;
}

const gnrc_netdev_driver_t cc2538rf_driver = {
    .send_data = _send
    //.add_event_callback = _add_event_cb,
    //.rem_event_callback = _rem_event_cb,
    //.get = _get,
    //.set = _set,
    //.isr_event = _isr_event,
};


/*
static void _irq_handler(void *arg)
{
}

int cc2538rf_init(cc2538rf_t *dev, spi_t spi, spi_speed_t spi_speed,
                   gpio_t cs_pin, gpio_t int_pin,
                   gpio_t sleep_pin, gpio_t
*/
//TODO check with contiki and stuff
int cc2538rf_init(cc2538rf_t *dev)
{
  DEBUG("cc2538rf: Init\n");

  dev->driver = &cc2538rf_driver;

  /* Enable clock for the RF Core while Running, in Sleep and Deep Sleep */
  SYS_CTRL_RCGCRFC = 1;
  SYS_CTRL_SCGCRFC = 1;
  SYS_CTRL_DCGCRFC = 1;

  RFCORE_XREG_CCACTRL0 = CC2538RF_CCA_THRES;

  /*
   * Changes from default values
   * See User Guide, section "Register Settings Update"
   */
  RFCORE_XREG_TXFILTCFG = 0x09;    /** TX anti-aliasing filter bandwidth */
  RFCORE_XREG_AGCCTRL1 = 0x15;     /** AGC target value */
  ANA_REGS_IVCTRL = 0x0B;          /** Bias currents */

  /*
   * Defaults:
   * Auto CRC; Append RSSI, CRC-OK and Corr. Val.; CRC calculation;
   * RX and TX modes with FIFOs
   */
  RFCORE_XREG_FRMCTRL0 = RFCORE_XREG_FRMCTRL0_AUTOCRC;

#if CC2538_RF_AUTOACK
  RFCORE_XREG_FRMCTRL0 |= RFCORE_XREG_FRMCTRL0_AUTOACK;
#endif

  /* If we are a sniffer, turn off frame filtering */
#if CC2538_RF_CONF_SNIFFER
  RFCORE_XREG_FRMFILT0 &= ~RFCORE_XREG_FRMFILT0_FRAME_FILTER_EN;
#endif

  /* Disable source address matching and autopend */
  RFCORE_XREG_SRCMATCH = 0;

  /* MAX FIFOP threshold */
  RFCORE_XREG_FIFOPCTRL = CC2538RF_MAX_PACKET_LEN;

  /* Set TX Power */
  RFCORE_XREG_TXPOWER = CC2538RF_TX_POWER_RECOMMENDED;

  cc2538rf_set_chan(dev, CC2538RF_DEFAULT_CHANNEL);

  /* Acknowledge RF interrupts, FIFOP only */
  RFCORE_XREG_RFIRQM0 |= RFCORE_XREG_RFIRQM0_FIFOP;
  NVIC_EnableIRQ(NVIC_INT_RF_RXTX);
  /* Acknowledge all RF Error interrupts */
  RFCORE_XREG_RFERRM = RFCORE_XREG_RFERRM_RFERRM;
  NVIC_EnableIRQ(NVIC_INT_RF_ERR);

#if UDMA_ENABLED
  if(CC2538_RF_CONF_TX_USE_DMA) {
    /* Disable peripheral triggers for the channel */
    udma_channel_mask_set(CC2538_RF_CONF_TX_DMA_CHAN);


    /*
     * Set the channel's DST. SRC can not be set yet since it will change for
     * each transfer
     */
    udma_set_channel_dst(CC2538_RF_CONF_TX_DMA_CHAN, RFCORE_SFR_RFDATA);
  }

  if(CC2538_RF_CONF_RX_USE_DMA) {
    /* Disable peripheral triggers for the channel */
    udma_channel_mask_set(CC2538_RF_CONF_RX_DMA_CHAN);

    /*
     * Set the channel's SRC. DST can not be set yet since it will change for
     * each transfer
     */
    udma_set_channel_src(CC2538_RF_CONF_RX_DMA_CHAN, RFCORE_SFR_RFDATA);
  }

  #endif

//process_start(&cc2538_rf_process, NULL);

  rf_flags |= RF_ON;

  //ENERGEST_ON(ENERGEST_TYPE_LISTEN);

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


void cc2538rf_set_chan(cc2538rf_t *dev, uint8_t chan)
{

}
