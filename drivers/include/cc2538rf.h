/*
 * Copyright (C)
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    driver_cc2538rf CC2538 Transceiver based drivers
 * @ingroup     drivers_netdev
 *
 * This module contains drivers for radio devices in Texas Instruments CC2538 series.
 * The driver is aimed to work with all devices of this series.
 *
 * @{
 *
 * @file
 * @brief       Interface definition for CC2538 Transceiver based drivers
 *
 * @author      Anon Mall <anon.mall@dai-labor.de>
 */

#ifndef CC2538RF_H_
#define CC2538RF_H_

#include <stdint.h>

#include "board.h"
#include "periph/spi.h"
#include "periph/gpio.h"
#include "net/gnrc/netdev.h"
#include "cc2538rf.h"
#include "rfcore.h"
#include "nvic.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CC2538RF_CCA_THRES                    (0xF8) /** CCA Threshold */
#define RFCORE_XREG_FRMCTRL0_AUTOCRC          (0x40) /** AUTO CRC XREG FRMCTRL0 */
#define RFCORE_XREG_FRMCTRL0_AUTOACK          (0x20) /** AUTO ACK XREG FRMCTRL0 */
#define RFCORE_XREG_FRMFILT0_FRAME_FILTER_EN  (0x01) /** enables frame filtering */
#define CC2538RF_MAX_PACKET_LEN               (127) /** cc2538rf core max packet length */
#define CC2538RF_TX_POWER_RECOMMENDED         (0xD5) /* ToDo: Determine value */
#define CC2538RF_DEFAULT_CHANNEL              (18) /** CC2538rf default channel */
#define RFCORE_XREG_RFIRQM0_FIFOP             (0x04) /**< RX FIFO exceeded threshold */

typedef enum {
    CC2538RF_FREQ_915MHZ,       /**< frequency 915MHz enabled */
    CC2538RF_FREQ_868MHZ,       /**< frequency 868MHz enabled */
} cc2538rf_freq_t;

/**
 * @brief   Device descriptor for cc2538rf radio devices
 */
typedef struct {
    /* netdev fields */
    const gnrc_netdev_driver_t *driver; /**< pointer to the devices interface */
    gnrc_netdev_event_cb_t event_cb;    /**< netdev event callback */
    kernel_pid_t mac_pid;               /**< the driver's thread's PID */
    gnrc_nettype_t proto;               /**< protocol the radio expects */
    uint8_t state;                      /**< current state of the radio */
    uint8_t seq_nr;                     /**< sequence number to use next */
    uint8_t frame_len;                  /**< length of the current TX frame */
    uint16_t pan;                       /**< currently used PAN ID */
    uint8_t chan;                       /**< currently used channel */
    uint8_t addr_short[2];              /**< the radio's short address */
    uint8_t addr_long[8];               /**< the radio's long address */
    uint16_t options;                   /**< state of used options */
    uint8_t idle_state;                 /**< state to return to after sending */
} cc2538rf_t;

/**
 * @brief struct holding all params needed for device initialization
 */
typedef struct cc2538rf_params {
    //spi_t spi;              /**< SPI bus the device is connected to */
    //spi_speed_t spi_speed;  /**< SPI speed to use */
    gpio_t cs_pin;          /**< GPIO pin connected to chip select */
    gpio_t int_pin;         /**< GPIO pin connected to the interrupt pin */
    gpio_t sleep_pin;       /**< GPIO pin connected to the sleep pin */
    gpio_t reset_pin;       /**< GPIO pin connected to the reset pin */
} cc2538rf_params_t;

/**
 * @brief   Initialize a given cc2538rf device
 *
 * @param[out] dev          device descriptor
 * @param[in] spi           SPI bus the device is connected to
 * @param[in] spi_speed     SPI speed to use
 * @param[in] cs_pin        GPIO pin connected to chip select
 * @param[in] int_pin       GPIO pin connected to the interrupt pin
 * @param[in] sleep_pin     GPIO pin connected to the sleep pin
 * @param[in] reset_pin     GPIO pin connected to the reset pin
 *
 * @return                  0 on success
 * @return                  <0 on error
int cc2538rf_init(cc2538rf_t *dev, spi_t spi, spi_speed_t spi_speed,
                   gpio_t cs_pin, gpio_t int_pin,
                   gpio_t sleep_pin, gpio_t reset_pin);
 */
 int cc2538rf_init(cc2538rf_t *dev);

/**
 * @brief   Trigger a hardware reset and configure radio with default values
 *
 * @param[in] dev           device to reset
 */
void cc2538rf_reset(cc2538rf_t *dev);

/**
 * @brief   Trigger a clear channel assessment
 *
 * @param[in] dev           device to use
 *
 * @return                  true if channel is clear
 * @return                  false if channel is busy
 */
bool cc2538rf_cca(cc2538rf_t *dev);

/**
 * @brief   Get the short address of the given device
 *
 * @param[in] dev           device to read from
 *
 * @return                  the currently set (2-byte) short address
 */
uint16_t cc2538rf_get_addr_short(cc2538rf_t *dev);

/**
 * @brief   Set the short address of the given device
 *
 * @param[in] dev           device to write to
 * @param[in] addr          (2-byte) short address to set
 */
void cc2538rf_set_addr_short(cc2538rf_t *dev, uint16_t addr);

/**
 * @brief   Get the configured long address of the given device
 *
 * @param[in] dev           device to read from
 *
 * @return                  the currently set (8-byte) long address
 */
uint64_t cc2538rf_get_addr_long(cc2538rf_t *dev);

/**
 * @brief   Set the long address of the given device
 *
 * @param[in] dev           device to write to
 * @param[in] addr          (8-byte) long address to set
 */
void cc2538rf_set_addr_long(cc2538rf_t *dev, uint64_t addr);

/**
 * @brief   Get the configured channel of the given device
 *
 * @param[in] dev           device to read from
 *
 * @return                  the currently set channel
 */
uint8_t cc2538rf_get_chan(cc2538rf_t *dev);

/**
 * @brief   Set the channel of the given device
 *
 * @param[in] dev           device to write to
 * @param[in] chan          channel to set
 */
void cc2538rf_set_chan(cc2538rf_t *dev, uint8_t chan);

/**
 * @brief   Get the configured frequency of the given device
 *
 * @param[in] dev           device to read from
 *
 * @return                  the currently set frequency
 */
cc2538rf_freq_t cc2538rf_get_freq(cc2538rf_t *dev);

/**
 * @brief   Set the frequency of the given device
 *
 * @param[in] dev           device to write to
 * @param[in] chan          frequency to set
 */
void cc2538rf_set_freq(cc2538rf_t *dev, cc2538rf_freq_t freq);

/**
 * @brief   Get the configured PAN ID of the given device
 *
 * @param[in] dev           device to read from
 *
 * @return                  the currently set PAN ID
 */
uint16_t cc2538rf_get_pan(cc2538rf_t *dev);

/**
 * @brief   Set the PAN ID of the given device
 *
 * @param[in] dev           device to write to
 * @param[in] pan           PAN ID to set
 */
void cc2538rf_set_pan(cc2538rf_t *dev, uint16_t pan);

/**
 * @brief   Get the configured transmission power of the given device [in dBm]
 *
 * @param[in] dev           device to read from
 *
 * @return                  configured transmission power in dBm
 */
int16_t cc2538rf_get_txpower(cc2538rf_t *dev);

/**
 * @brief   Set the transmission power of the given device [in dBm]
 *
 * If the device does not support the exact dBm value given, it will set a value
 * as close as possible to the given value. If the given value is larger or
 * lower then the maximal or minimal possible value, the min or max value is
 * set, respectively.
 *
 * @param[in] dev           device to write to
 * @param[in] txpower       transmission power in dBm
 */
void cc2538rf_set_txpower(cc2538rf_t *dev, int16_t txpower);

/**
 * @brief   Get the maximum number of retransmissions
 *
 * @param[in] dev           device to read from
 *
 * @return                  configured number of retransmissions
 */
uint8_t cc2538rf_get_max_retries(cc2538rf_t *dev);

/**
 * @brief   Set the maximum number of retransmissions
 *
 * This setting specifies the number of attempts to retransmit a frame, when it
 * was not acknowledged by the recipient, before the transaction gets cancelled.
 * The maximum value is 7.
 *
 * @param[in] dev           device to write to
 * @param[in] max           the maximum number of retransmissions
 */
void cc2538rf_set_max_retries(cc2538rf_t *dev, uint8_t max);

/**
 * @brief   Get the maximum number of channel access attempts per frame (CSMA)
 *
 * @param[in] dev           device to read from
 *
 * @return                  configured number of retries
 */
uint8_t cc2538rf_get_csma_max_retries(cc2538rf_t *dev);

/**
 * @brief   Set the maximum number of channel access attempts per frame (CSMA)
 *
 * This setting specifies the number of attempts to access the channel to
 * transmit a frame. If the channel is busy @p retries times, then frame
 * transmission fails.
 * Valid values: 0 to 5, -1 means CSMA disabled
 *
 * @param[in] dev           device to write to
 * @param[in] max           the maximum number of retries
 */
void cc2538rf_set_csma_max_retries(cc2538rf_t *dev, int8_t retries);

/**
 * @brief   Set the min and max backoff exponent for CSMA/CA
 *
 * - Maximum BE: 0 - 8
 * - Minimum BE: 0 - [max]
 *
 * @param[in] dev           device to write to
 * @param[in] min           the minimum BE
 * @param[in] max           the maximum BE
 */
void cc2538rf_set_csma_backoff_exp(cc2538rf_t *dev, uint8_t min, uint8_t max);

/**
 * @brief   Set seed for CSMA random backoff
 *
 * @param[in] dev           device to write to
 * @param[in] entropy       11 bit of entropy as seed for random backoff
 */
void cc2538rf_set_csma_seed(cc2538rf_t *dev, uint8_t entropy[2]);

/**
 * @brief   Enable or disable driver specific options
 *
 * @param[in] dev           device to set/clear option flag for
 * @param[in] option        option to enable/disable
 * @param[in] state         true for enable, false for disable
 */
void cc2538rf_set_option(cc2538rf_t *dev, uint16_t option, bool state);

/**
 * @brief   Set the state of the given device (trigger a state change)
 *
 * @param[in] dev           device to change state of
 * @param[in] state         the targeted new state
 */
void cc2538rf_set_state(cc2538rf_t *dev, uint8_t state);

/**
 * @brief   Reset the internal state machine to TRX_OFF mode.
 *
 * This will force a transition to TRX_OFF regardless of whether the transceiver
 * is currently busy sending or receiving. This function is used to get back to
 * a known state during driver initialization.
 *
 * @param[in] dev           device to operate on
 */
void cc2538rf_reset_state_machine(cc2538rf_t *dev);

/**
 * @brief   Convenience function for simply sending data
 *
 * @note This function ignores the PRELOADING option
 *
 * @param[in] dev           device to use for sending
 * @param[in] data          data to send (must include IEEE802.15.4 header)
 * @param[in] len           length of @p data
 *
 * @return                  number of bytes that were actually send
 * @return                  0 on error
 */
size_t cc2538rf_send(cc2538rf_t *dev, uint8_t *data, size_t len);

/**
 * @brief   Prepare for sending of data
 *
 * This function puts the given device into the TX state, so no receiving of
 * data is possible after it was called.
 *
 * @param[in] dev            device to prepare for sending
 */
void cc2538rf_tx_prepare(cc2538rf_t *dev);

/**
 * @brief   Load chunks of data into the transmit buffer of the given device
 *
 * @param[in] dev           device to write data to
 * @param[in] data          buffer containing the data to load
 * @param[in] len           number of bytes in @p buffer
 * @param[in] offset        offset used when writing data to internal buffer
 *
 * @return                  offset + number of bytes written
 */
size_t cc2538rf_tx_load(cc2538rf_t *dev, uint8_t *data, size_t len,
                         size_t offset);

/**
 * @brief   Trigger sending of data previously loaded into transmit buffer
 *
 * @param[in] dev           device to trigger
 */
void cc2538rf_tx_exec(cc2538rf_t *dev);

/**
 * @brief   Read the length of a received packet
 *
 * @param dev               device to read from
 *
 * @return                  overall length of a received packet in byte
 */
size_t cc2538rf_rx_len(cc2538rf_t *dev);

/**
 * @brief   Read a chunk of data from the receive buffer of the given device
 *
 * @param[in]  dev          device to read from
 * @param[out] data         buffer to write data to
 * @param[in]  len          number of bytes to read from device
 * @param[in]  offset       offset in the receive buffer
 */
void cc2538rf_rx_read(cc2538rf_t *dev, uint8_t *data, size_t len,
                       size_t offset);

#ifdef __cplusplus
}
#endif

#endif /* cc2538rf_H_ */
/** @} */
