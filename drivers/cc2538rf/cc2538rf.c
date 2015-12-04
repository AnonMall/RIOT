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

#define ENABLE_DEBUG (1)
#include "debug.h"

#define UDMA_ENABLED 0

/*Some Status Flags for the Radio */
static uint8_t rf_flags;

/* Local RF Flags */
#define RX_ACTIVE     0x80
#define RF_MUST_RESET 0x40
#define RF_ON         0x01

//TODO check back with real value
#define _MAX_MHR_OVERHEAD   (25)

#define RESET_DELAY             (1U)        /* must be > 625ns */

static size_t _make_data_frame_hdr(cc2538rf_t *dev, uint8_t *buf,
                                   gnrc_netif_hdr_t *hdr)
{

  DEBUG("cc2538rf: making 802.15.4 header:\n");


    int pos = 0;

    /* we are building a data frame here */
    buf[0] = IEEE802154_FCF_TYPE_DATA;
    buf[1] = IEEE802154_FCF_VERS_V1;

    /* if AUTOACK is enabled, then we also expect ACKs for this packet */
    if (!(hdr->flags & GNRC_NETIF_HDR_FLAGS_BROADCAST) &&
        !(hdr->flags & GNRC_NETIF_HDR_FLAGS_MULTICAST) &&
        (dev->options & CC2538_RF_AUTOACK)) {
        buf[0] |= IEEE802154_FCF_ACK_REQ;
    }

    /* fill in destination PAN ID */
    pos = 3;
    buf[pos++] = (uint8_t)((dev->pan) & 0xff);
    buf[pos++] = (uint8_t)((dev->pan) >> 8);

    /* fill in destination address */
    if (hdr->flags &
        (GNRC_NETIF_HDR_FLAGS_BROADCAST | GNRC_NETIF_HDR_FLAGS_MULTICAST)) {
        buf[1] |= IEEE802154_FCF_DST_ADDR_SHORT;
        buf[pos++] = 0xff;
        buf[pos++] = 0xff;
    }
    else if (hdr->dst_l2addr_len == 2) {
        uint8_t *dst_addr = gnrc_netif_hdr_get_dst_addr(hdr);
        buf[1] |= IEEE802154_FCF_DST_ADDR_SHORT;
        buf[pos++] = dst_addr[1];
        buf[pos++] = dst_addr[0];
    }
    else if (hdr->dst_l2addr_len == 8) {
        buf[1] |= IEEE802154_FCF_DST_ADDR_LONG;
        uint8_t *dst_addr = gnrc_netif_hdr_get_dst_addr(hdr);
        for (int i = 7;  i >= 0; i--) {
            buf[pos++] = dst_addr[i];
        }
    }
    else {
        /* unsupported address length */
        return 0;
    }

    /* fill in source PAN ID (if applicable) */
    if (dev->options & CC2538RF_OPT_USE_SRC_PAN) {
        buf[pos++] = (uint8_t)((dev->pan) & 0xff);
        buf[pos++] = (uint8_t)((dev->pan) >> 8);
    } else {
        buf[0] |= IEEE802154_FCF_PAN_COMP;
    }
        buf[0] |= IEEE802154_FCF_PAN_COMP;

    /* fill in source address*/
    if (dev->options & CC2538RF_OPT_SRC_ADDR_LONG) {
        buf[1] |= IEEE802154_FCF_SRC_ADDR_LONG;
        memcpy(&(buf[pos]), dev->addr_long, 8);
        pos += 8;
    }
    else {
        buf[1] |= IEEE802154_FCF_SRC_ADDR_SHORT;
        buf[pos++] = dev->addr_short[0];
        buf[pos++] = dev->addr_short[1];
    }


        buf[1] |= IEEE802154_FCF_SRC_ADDR_LONG;
        memcpy(&(buf[pos]), dev->addr_long, 8);
        pos += 8;

    /* set sequence number */
    buf[2] = dev->seq_nr++;
    /* return actual header length */
    return pos;
}

#if 0
/* TODO: generalize and move to ieee802154 */
/* TODO: include security header implications */
static size_t _get_frame_hdr_len(uint8_t *mhr)
{
    uint8_t tmp;
    size_t len = 3;

    /* figure out address sizes */
    tmp = (mhr[1] & IEEE802154_FCF_DST_ADDR_MASK);
    if (tmp == IEEE802154_FCF_DST_ADDR_SHORT) {
        len += 4;
    }
    else if (tmp == IEEE802154_FCF_DST_ADDR_LONG) {
        len += 10;
    }
    else if (tmp != IEEE802154_FCF_DST_ADDR_VOID) {
        return 0;
    }
    tmp = (mhr[1] & IEEE802154_FCF_SRC_ADDR_MASK);
    if (tmp == IEEE802154_FCF_SRC_ADDR_VOID) {
        return len;
    }
    else {
        if (!(mhr[0] & IEEE802154_FCF_PAN_COMP)) {
            len += 2;
        }
        if (tmp == IEEE802154_FCF_SRC_ADDR_SHORT) {
            return (len + 2);
        }
        else if (tmp == IEEE802154_FCF_SRC_ADDR_LONG) {
            return (len + 8);
        }
    }
    return 0;
}



/* TODO: generalize and move to (gnrc_)ieee802154 */
static gnrc_pktsnip_t *_make_netif_hdr(uint8_t *mhr)
{
    uint8_t tmp;
    uint8_t *addr;
    uint8_t src_len, dst_len;
    gnrc_pktsnip_t *snip;
    gnrc_netif_hdr_t *hdr;

    /* figure out address sizes */
    tmp = mhr[1] & IEEE802154_FCF_SRC_ADDR_MASK;
    if (tmp == IEEE802154_FCF_SRC_ADDR_SHORT) {
        src_len = 2;
    }
    else if (tmp == IEEE802154_FCF_SRC_ADDR_LONG) {
        src_len = 8;
    }
    else if (tmp == IEEE802154_FCF_SRC_ADDR_VOID) {
        src_len = 0;
    }
    else {
        return NULL;
    }
    tmp = mhr[1] & IEEE802154_FCF_DST_ADDR_MASK;
    if (tmp == IEEE802154_FCF_DST_ADDR_SHORT) {
        dst_len = 2;
    }
    else if (tmp == IEEE802154_FCF_DST_ADDR_LONG) {
        dst_len = 8;
    }
    else if (tmp == IEEE802154_FCF_DST_ADDR_VOID) {
        dst_len = 0;
    }
    else {
        return NULL;
    }
    /* allocate space for header */
    snip = gnrc_pktbuf_add(NULL, NULL, sizeof(gnrc_netif_hdr_t) + src_len + dst_len,
                           GNRC_NETTYPE_NETIF);
    if (snip == NULL) {
        return NULL;
    }
    /* fill header */
    hdr = (gnrc_netif_hdr_t *)snip->data;
    gnrc_netif_hdr_init(hdr, src_len, dst_len);
    if (dst_len > 0) {
        tmp = 5 + dst_len;
        addr = gnrc_netif_hdr_get_dst_addr(hdr);
        for (int i = 0; i < dst_len; i++) {
            addr[i] = mhr[5 + (dst_len - i) - 1];
        }
    }
    else {
        tmp = 3;
    }
    if (!(mhr[0] & IEEE802154_FCF_PAN_COMP)) {
        tmp += 2;
    }
    if (src_len > 0) {
        addr = gnrc_netif_hdr_get_src_addr(hdr);
        for (int i = 0; i < src_len; i++) {
            addr[i] = mhr[tmp + (src_len - i) - 1];
        }
    }
    return snip;
}
#endif

//TODO: not implemented yet
static int _send(gnrc_netdev_t *netdev, gnrc_pktsnip_t *pkt){


  DEBUG("cc2538rf: trying to send stuff now package:\n");



    cc2538rf_t *dev = (cc2538rf_t *)netdev;
    gnrc_pktsnip_t *snip;
    uint8_t mhr[IEEE802154_MAX_HDR_LEN];
    size_t len;

    if (pkt == NULL) {
        return -ENOMSG;
      }


    len = _make_data_frame_hdr(dev, mhr, (gnrc_netif_hdr_t *)pkt->data);
    for(int i = 0; i<IEEE802154_MAX_HDR_LEN; i++){
      DEBUG("0x%x ", mhr[i]);
    }
        DEBUG("\n");

      uint8_t *data;
      gnrc_pktsnip_t *current = pkt;
      while(current){
        data = current->data;
        switch(current->type){
          case GNRC_NETTYPE_IOVEC:
            DEBUG("PKT Type is GNRC_NETTYPE_IOVEC  ");
            break;

          case GNRC_NETTYPE_NETIF:
            DEBUG("PKT Type is GNRC_NETTYPE_NETIF  ");
            break;

          case GNRC_NETTYPE_UNDEF:
            DEBUG("PKT Type is GNRC_NETTYPE_UNDEF  ");
            break;


          case GNRC_NETTYPE_SIXLOWPAN:
            DEBUG("PKT Type is GNRC_NETTYPE_SIXLOWPAN  ");
            break;

          case GNRC_NETTYPE_IPV6:
            DEBUG("PKT Type is GNRC_NETTYPE_IPV6  ");
            break;

          case GNRC_NETTYPE_ICMPV6:
            DEBUG("PKT Type is GNRC_NETTYPE_ICMPV6  ");
            break;

/*
          case GNRC_NETTYPE_TCP:
            DEBUG("PKT Type is GNRC_NETTYPE_TCP  ");
            break;
*/
          case GNRC_NETTYPE_UDP:
            DEBUG("PKT Type is GNRC_NETTYPE_UDP  ");
            break;

          case GNRC_NETTYPE_NUMOF:
            DEBUG("PKT Type is GNRC_NETTYPE_NUMOF  ");
            break;

          default:
            DEBUG("PKT Type is something else  ");
            break;

        }
        for(int i = 0; i < current->size; i++){
          DEBUG("0x%x ", *(data+i));
        }
        DEBUG("\n");
        current = current->next;
      }


    if (dev == NULL) {
        gnrc_pktbuf_release(pkt);
        return -ENODEV;
    }

    if (pkt->type != GNRC_NETTYPE_NETIF) {
        DEBUG("[cc2538rf]: First header was not generic netif header\n");
        gnrc_pktbuf_release(pkt);
        return -EBADMSG;
    }

    /* create 802.15.4 header */
    if (len == 0) {
        DEBUG("[cc2538rf] error: 802.15.4 header is empty\n");
        gnrc_pktbuf_release(pkt);
        return -ENOMSG;
    }



    /* check if packet (header + payload + FCS) fits into FIFO */
    snip = pkt->next;
    if ((gnrc_pkt_len(snip) + len + 2) > CC2538RF_MAX_PACKET_LEN) {
        printf("[cc2538rf] error: packet too large (%u byte) to be send\n",
               gnrc_pkt_len(snip) + len + 2);
        gnrc_pktbuf_release(pkt);
        DEBUG("cc2538rf: package overflow\n");
        return -EOVERFLOW;
    }

    //TODO: prepare package for sending and send over fifo of cc2538rf


    DEBUG("cc2538rf: trying to send over cc2538 RF DATA register\n");
    for(int i = 0; i<IEEE802154_MAX_HDR_LEN; i++){
      RFCORE_SFR_RFDATA = mhr[i];
    }
    while(snip){
      for(int i = 0; i< snip->size; i++){
        RFCORE_SFR_RFDATA = *(((uint8_t*)(snip->data))+i);
    }
      snip = snip->next;
    }

    //send transmit opcode to csp
    RFCORE_SFR_RFST = 0xe9;

#if ENABLE_DEBUG

    while(RFCORE_XREG_FSMSTAT1 & 0x2 )
      DEBUG("cc2538rf: Still Sending\n");

      DEBUG("cc2538rf: sending complete\n");

#endif



    gnrc_pktbuf_release(pkt);

return -1;
}


//TODO: not implemented yet
static int _add_event_cb(gnrc_netdev_t *dev, gnrc_netdev_event_cb_t cb)
{
  DEBUG("cc2538rf: adding event cb:\n");
    return -1;
}


//TODO: not implemented yet
static int _rem_event_cb(gnrc_netdev_t *dev, gnrc_netdev_event_cb_t cb)
{

  DEBUG("cc2538rf: removing event cb:\n");
    return -1;
}

//TODO: not implemented yet
static int _get(gnrc_netdev_t *device, netopt_t opt, void *val, size_t max_len)
{

  DEBUG("cc2538rf: getting options:\n");

    if(device == NULL) {
      return -ENODEV;
    }

    cc2538rf_t *dev = (cc2538rf_t *) device;

    switch (opt) {
        case NETOPT_CHANNEL:
            if (max_len < sizeof(uint16_t)) {
                return -EOVERFLOW;
            }
            ((uint8_t *)val)[1] = 0;
            ((uint8_t *)val)[0] = cc2538rf_get_chan(dev);
            return sizeof(uint16_t);

        default:
            break;
    }


    switch (opt) {

      case NETOPT_ADDRESS:
      DEBUG("NETOPT_ADDRESS  ");
            if (max_len < sizeof(uint16_t)) {
                return -EOVERFLOW;
            }
            *((uint16_t *)val) = cc2538rf_get_addr_short(dev);
            return sizeof(uint16_t);
      break;

      case NETOPT_ADDRESS_LONG:
      DEBUG("NETOPT_ADDRESS_LONG  ");
      break;

      case NETOPT_ADDR_LEN:
      DEBUG("NETOPT_ADDR_LEN  ");
      break;

      case NETOPT_SRC_LEN:
      DEBUG("NETOPT_SRC_LEN  ");
            if (max_len < sizeof(uint16_t)) {
                return -EOVERFLOW;
            }
            if (dev->options & CC2538RF_OPT_SRC_ADDR_LONG) {
                *((uint16_t *)val) = 8;
            }
            else {
                *((uint16_t *)val) = 2;
            }
            return sizeof(uint16_t);
      break;

      case NETOPT_NID:
      DEBUG("NETOPT_NID  ");
      break;

      case NETOPT_IPV6_IID:
      DEBUG("NETOPT_IPV6_IID  ");
            if (max_len < sizeof(eui64_t)) {
                return -EOVERFLOW;
            }
            if (dev->options & CC2538RF_OPT_SRC_ADDR_LONG) {
                uint64_t addr = cc2538rf_get_addr_long(dev);
                ieee802154_get_iid(val, (uint8_t *)&addr, 8);
            }
            else {
                uint16_t addr = cc2538rf_get_addr_short(dev);
                ieee802154_get_iid(val, (uint8_t *)&addr, 2);
            }
            return sizeof(eui64_t);
      break;

      case NETOPT_PROTO:
      DEBUG("NETOPT_PROTO  ");
            if (max_len < sizeof(gnrc_nettype_t)) {
                return -EOVERFLOW;
            }
            *((gnrc_nettype_t *)val) = dev->proto;
            return sizeof(gnrc_nettype_t);
      break;

      case NETOPT_CHANNEL:
      DEBUG("NETOPT_CHANNEL  ");
      break;

      case NETOPT_TX_POWER:
      DEBUG("NETOPT_TX_POWER  ");
      break;

      case NETOPT_MAX_PACKET_SIZE:
      DEBUG("NETOPT_MAX_PACKET_SIZE  ");
            if (max_len < sizeof(int16_t)) {
                return -EOVERFLOW;
            }
            *((uint16_t *)val) = CC2538RF_MAX_PACKET_LEN - _MAX_MHR_OVERHEAD;
            return sizeof(uint16_t);
      break;

      case NETOPT_STATE:
      DEBUG("NETOPT_STATE  ");
      break;

      case NETOPT_PRELOADING:
      DEBUG("NETOPT_PRELOADING  ");
      break;

      case NETOPT_AUTOACK:
      DEBUG("NETOPT_AUTOACK  ");
      break;

      case NETOPT_RETRANS:
      DEBUG("NETOPT_RETRANS  ");
      break;

      case NETOPT_PROMISCUOUSMODE:
      DEBUG("NETOPT_PROMISCUOUSMODE  ");
      break;

      case NETOPT_RAWMODE:
      DEBUG("NETOPT_RAWMODE  ");
      break;

      case NETOPT_IS_CHANNEL_CLR:
      DEBUG("NETOPT_IS_CHANNEL_CLR  ");
      break;

      case NETOPT_RX_START_IRQ:
      DEBUG("NETOPT_RX_START_IRQ  ");
      break;

      case NETOPT_RX_END_IRQ:
      DEBUG("NETOPT_RX_END_IRQ  ");
      break;

      case NETOPT_TX_START_IRQ:
      DEBUG("NETOPT_TX_START_IRQ  ");
      break;

      case NETOPT_TX_END_IRQ:
      DEBUG("NETOPT_TX_END_IRQ  ");
      break;

      case NETOPT_CSMA:
      DEBUG("NETOPT_CSMA  ");
      break;

      case NETOPT_CSMA_RETRIES:
      DEBUG("NETOPT_CSMA_RETRIES  ");
      break;

      default:
      DEBUG("default");
      return -ENOTSUP;

    }

  return -1;
}

//TODO: not implemented yet
static int _set(gnrc_netdev_t *device, netopt_t opt, void *val, size_t len)
{

  DEBUG("cc2538rf: setting options:\n");
  switch(opt){
    case NETOPT_ADDRESS_LONG:
      DEBUG("NETOPT_ADDRESS_LONG\n");
      break;
    case NETOPT_ADDRESS:
      DEBUG("NETOPT_ADDRESS\n");
      break;
    default:
      DEBUG("DEFAULT\n");
      break;
  }

  if(device == NULL) {
    return -ENODEV;
  }


  return -1;
}

//TODO: not implemented yet
static void _isr_event(gnrc_netdev_t *device, uint32_t event_type)
{
}


const gnrc_netdev_driver_t cc2538rf_driver = {
    .send_data = _send,
    .add_event_callback = _add_event_cb,
    .rem_event_callback = _rem_event_cb,
    .get = _get,
    .set = _set,
    .isr_event = _isr_event,
};


//TODO check with contiki and stuff also device not fully initialized
//in sense of riot yet
int cc2538rf_init(cc2538rf_t *dev)
{
  DEBUG("cc2538rf: Init\n");

  dev->driver = &cc2538rf_driver;

  //Setting addresses from flash
  //should be outscourced to other method later

  //Long address
  /*
  uint32_t highLongTmp = IEEE_ADDR_MSWORD;
  uint32_t lowLongTmp = IEEE_ADDR_LSWORD;
  uint8_t highLongArray[4];
  uint8_t lowLongArray[4];
  for(int i = 0; i<4; i++){
    lowLongArray[i] = lowLongTmp & 0xFF;
    highLongArray[i] = highLongTmp & 0xFF;
    dev->addr_long[7-i] = lowLongArray[i];
    dev->addr_long[3-i] = highLongArray[i];
    highLongTmp = highLongTmp >> 8;
    lowLongTmp = lowLongTmp >> 8;

  }

  RFCORE_FFSM_EXT_ADDR0 = lowLongArray[0];
  RFCORE_FFSM_EXT_ADDR1 = lowLongArray[1];
  RFCORE_FFSM_EXT_ADDR2 = lowLongArray[2];
  RFCORE_FFSM_EXT_ADDR3 = lowLongArray[3];
  RFCORE_FFSM_EXT_ADDR4 = highLongArray[0];
  RFCORE_FFSM_EXT_ADDR5 = highLongArray[1];
  RFCORE_FFSM_EXT_ADDR6 = highLongArray[2];
  RFCORE_FFSM_EXT_ADDR7 = highLongArray[3];
  */

  uint64_t mLongAddress = IEEE_ADDR_MSWORD;
  uint64_t lLongAddress = IEEE_ADDR_LSWORD;
  uint64_t longAddress = ( mLongAddress << 32) | lLongAddress;
  cc2538rf_set_addr_long(dev, longAddress);

  DEBUG("cc2538rf: current long address:\n");
  for(int i = 0; i<8; i++){
    DEBUG("%x:", dev->addr_long[7-i]);
  }
  DEBUG("\n");

  uint16_t addr_short = (dev->addr_long[1]<<8) | dev->addr_long[0];
  cc2538rf_set_addr_short(dev, addr_short);

  DEBUG("cc2538rf: short address:\n");
  for(int i = 0; i<2; i++){
    DEBUG("%x:", dev->addr_short[i]);
  }
  DEBUG("\n");

#ifdef MODULE_GNRC_SIXLOWPAN
  DEBUG("cc2538rf: 6lowpan enabled \n");
  dev->proto = GNRC_NETTYPE_SIXLOWPAN;
#else
  dev->proto = GNRC_NETTYPE_UNDEF;
#endif


/*
  Commented out the init block to make a dummy first



  * Enable clock for the RF Core while Running, in Sleep and Deep Sleep *
  SYS_CTRL_RCGCRFC = 1;
  SYS_CTRL_SCGCRFC = 1;
  SYS_CTRL_DCGCRFC = 1;

  RFCORE_XREG_CCACTRL0 = CC2538RF_CCA_THRES;

  *
   * Changes from default values
   * See User Guide, section "Register Settings Update"
   *
  RFCORE_XREG_TXFILTCFG = 0x09;    ** TX anti-aliasing filter bandwidth *
  RFCORE_XREG_AGCCTRL1 = 0x15;     ** AGC target value *
  ANA_REGS_IVCTRL = 0x0B;          ** Bias currents *

  *
   * Defaults:
   * Auto CRC; Append RSSI, CRC-OK and Corr. Val.; CRC calculation;
   * RX and TX modes with FIFOs
   *
  RFCORE_XREG_FRMCTRL0 = RFCORE_XREG_FRMCTRL0_AUTOCRC;

#if CC2538_RF_AUTOACK
  RFCORE_XREG_FRMCTRL0 |= RFCORE_XREG_FRMCTRL0_AUTOACK;
#endif

  * If we are a sniffer, turn off frame filtering *
#if CC2538_RF_CONF_SNIFFER
  RFCORE_XREG_FRMFILT0 &= ~RFCORE_XREG_FRMFILT0_FRAME_FILTER_EN;
#endif

  * Disable source address matching and autopend *
  RFCORE_XREG_SRCMATCH = 0;

  * MAX FIFOP threshold *
  RFCORE_XREG_FIFOPCTRL = CC2538RF_MAX_PACKET_LEN;

  * Set TX Power *
  RFCORE_XREG_TXPOWER = CC2538RF_TX_POWER_RECOMMENDED;

  cc2538rf_set_chan(dev, CC2538RF_DEFAULT_CHANNEL);

  * Acknowledge RF interrupts, FIFOP only *
  RFCORE_XREG_RFIRQM0 |= RFCORE_XREG_RFIRQM0_FIFOP;
  NVIC_EnableIRQ(NVIC_INT_RF_RXTX);
  * Acknowledge all RF Error interrupts *
  RFCORE_XREG_RFERRM = RFCORE_XREG_RFERRM_RFERRM;
  NVIC_EnableIRQ(NVIC_INT_RF_ERR);

#if UDMA_ENABLED
  if(CC2538_RF_CONF_TX_USE_DMA) {
    * Disable peripheral triggers for the channel *
    udma_channel_mask_set(CC2538_RF_CONF_TX_DMA_CHAN);


    *
     * Set the channel's DST. SRC can not be set yet since it will change for
     * each transfer
     *
    udma_set_channel_dst(CC2538_RF_CONF_TX_DMA_CHAN, RFCORE_SFR_RFDATA);
  }

  if(CC2538_RF_CONF_RX_USE_DMA) {
    * Disable peripheral triggers for the channel *
    udma_channel_mask_set(CC2538_RF_CONF_RX_DMA_CHAN);


     * Set the channel's SRC. DST can not be set yet since it will change for
     * each transfer
     *
    udma_set_channel_src(CC2538_RF_CONF_RX_DMA_CHAN, RFCORE_SFR_RFDATA);
  }

  #endif
*/
#ifdef MODULE_GNRC_SIXLOWPAN
    dev->proto = GNRC_NETTYPE_SIXLOWPAN;
#else
    dev->proto = GNRC_NETTYPE_UNDEF;
#endif

//process_start(&cc2538_rf_process, NULL);

  rf_flags |= RF_ON;
  dev->state = NETOPT_STATE_IDLE;

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
  return -1;
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


uint8_t cc2538rf_get_chan(cc2538rf_t *dev)
{
  return dev->chan;
}


void cc2538rf_set_chan(cc2538rf_t *dev, uint8_t chan)
{

}

void cc2538rf_set_addr_long(cc2538rf_t *dev, uint64_t addr)
{
  cc2538_rfcore_t* rfcore = RFCORE;

  for(int i = 0; i<8; i++){
    dev->addr_long[i] = (addr >> ((7-i)*8));
  }
  uint8_t *addr_cut = (uint8_t*) &addr;
  rfcore->FFSM_EXT_ADDR0 = addr_cut[7];
  rfcore->FFSM_EXT_ADDR1 = addr_cut[6];
  rfcore->FFSM_EXT_ADDR2 = addr_cut[5];
  rfcore->FFSM_EXT_ADDR3 = addr_cut[4];
  rfcore->FFSM_EXT_ADDR4 = addr_cut[3];
  rfcore->FFSM_EXT_ADDR5 = addr_cut[2];
  rfcore->FFSM_EXT_ADDR6 = addr_cut[1];
  rfcore->FFSM_EXT_ADDR7 = addr_cut[0];

}


uint64_t cc2538rf_get_addr_long(cc2538rf_t *dev)
{
  uint64_t longAddr = 0x0;
  for(int i=0; i<8; i++){
    longAddr = (longAddr << 8) | dev->addr_long[7-i];
  }

  return longAddr;
}


void cc2538rf_set_addr_short(cc2538rf_t *dev, uint16_t addr)
{
    uint8_t *addr_cut = (uint8_t*) &addr;
    dev->addr_short[0] = addr_cut[1];
    dev->addr_short[1] = addr_cut[0];
    cc2538_rfcore_t* rfcore = RFCORE;
    rfcore->FFSM_SHORT_ADDR0 = addr_cut[1];
    rfcore->FFSM_SHORT_ADDR1 = addr_cut[0];
}


uint16_t cc2538rf_get_addr_short(cc2538rf_t *dev)
{
  uint16_t shortAddr = (dev->addr_short[1] << 8) | dev->addr_short[0];
  return shortAddr;
}
