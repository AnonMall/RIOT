/*
 * Copyright (C) 2015
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cc2538
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
#include "net/netdev2.h"
#include "cc2538rf.h"
#include "nvic.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

#define UDMA_ENABLED 0
#define CHECKSUM_LEN 2

/*Some Status Flags for the Radio */
//static uint8_t rf_flags;

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
    buf[1] = IEEE802154_FCF_VERS_V0;

    /* if AUTOACK is enabled, then we also expect ACKs for this packet */
    if (!(hdr->flags & GNRC_NETIF_HDR_FLAGS_BROADCAST) &&
        !(hdr->flags & GNRC_NETIF_HDR_FLAGS_MULTICAST) &&
        (dev->options & CC2538_RF_AUTOACK)) {
        DEBUG("cc23538rf: autoack enabled\n");
        buf[0] |= IEEE802154_FCF_ACK_REQ;
    }

    /* fill in destination PAN ID */
    pos = 3;
    buf[pos++] = (uint8_t)((dev->pan) & 0xff);
    buf[pos++] = (uint8_t)((dev->pan) >> 8);

    /* fill in destination address */
    if (hdr->flags &
        (GNRC_NETIF_HDR_FLAGS_BROADCAST | GNRC_NETIF_HDR_FLAGS_MULTICAST)) {
        DEBUG("cc23538rf: its a broadcast/multicast\n");
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
        DEBUG("cc2538rf mac header: unsupported address length\n");
        /* unsupported address length */
        return 0;
    }

    /* fill in source PAN ID (if applicable) */
    if (dev->options & CC2538RF_OPT_USE_SRC_PAN) {
        DEBUG("cc2538rf: using src pan\n");
        buf[pos++] = (uint8_t)((dev->pan) & 0xff);
        buf[pos++] = (uint8_t)((dev->pan) >> 8);
    } else {
        buf[0] |= IEEE802154_FCF_PAN_COMP;
    }

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

//TODO: not finished yet, also need outsourcing into other methods
//otherwise it looks too full
static int _send(gnrc_netdev_t *netdev, gnrc_pktsnip_t *pkt){


    DEBUG("cc2538rf: trying to send stuff now package:\n");

    //wait until previous transmittion is finished
    while(RFCORE_XREG_FSMSTAT1 & RFCORE_XREG_FSMSTAT1_TX_ACTIVE);

    //clearing TX FIFO
    RFCORE_SFR_RFST = CC2538_RF_CSP_OP_ISFLUSHTX;
    RFCORE_SFR_RFST = CC2538_RF_CSP_OP_ISFLUSHTX;

    cc2538rf_t *dev = (cc2538rf_t *)netdev;
    gnrc_pktsnip_t *snip;
    uint8_t mhr[IEEE802154_MAX_HDR_LEN];
    memset(mhr, 0, IEEE802154_MAX_HDR_LEN);
    size_t len;


    if (pkt == NULL) {
        return -ENOMSG;
      }


    len = _make_data_frame_hdr(dev, mhr, (gnrc_netif_hdr_t *)pkt->data);
    for(int i = 0; i<len; i++){
      DEBUG("0x%x ", mhr[i]);
    }
        DEBUG("\n");

#if 0
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

#endif


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
    DEBUG("cc2538rf: mac header len= %x, payload_len= %x\n", len, gnrc_pkt_len(snip));


    //TODO: prepare package for sending and send over fifo of cc2538rf


    DEBUG("cc2538rf: putting mac_header + payload_length+2 into RFDATA register\n");
    RFCORE_SFR_RFDATA =  gnrc_pkt_len(snip) + len + 2;

    DEBUG("cc2538rf: putting stuff into the RFDATA register\n");

    for(int i = 0; i<len; i++){
      RFCORE_SFR_RFDATA = mhr[i];
    }

    while(snip){
        for(int i = 0; i< snip->size; i++){
          RFCORE_SFR_RFDATA = ((uint8_t*)(snip->data))[i];
      }
      snip = snip->next;
    }






    //read how much is in the TX FIFO
    DEBUG("cc2538rf: amount currently in the TX FIFO: %u\n", (uint8_t)RFCORE_XREG_TXFIFOCNT);

    //send transmit opcode to csp
    dev->state = NETOPT_STATE_TX;
    DEBUG("cc2538rf: trying to send over cc2538 RF DATA register\n");
    RFCORE_SFR_RFST = CC2538_RF_CSP_OP_ISTXON;

    //check current csp instruction
    uint8_t currentCSP = (uint8_t) RFCORE_SFR_RFST;
    DEBUG("cc2538rf: current CSP instruction OPCODE: %x\n", currentCSP);

    DEBUG("cc2538rf: amount currently in the TX FIFO after send: %u\n", (uint8_t)RFCORE_XREG_TXFIFOCNT);

    RFCORE_SFR_RFST = CC2538_RF_CSP_OP_ISFLUSHTX;
    RFCORE_SFR_RFST = CC2538_RF_CSP_OP_ISFLUSHTX;

    DEBUG("cc2538rf: amount currently in the TX FIFO after flush: %u\n", (uint8_t)RFCORE_XREG_TXFIFOCNT);

/*  not sure if needed, but stall program until everything has been sent
  int counter = 0;
  while(!((RFCORE_XREG_FSMSTAT1 & RFCORE_XREG_FSMSTAT1_TX_ACTIVE))
        && (counter++ < 3)) {
    DEBUG("cc2538rf: sleeping\n");
    xtimer_usleep(6);
  }
*/


    while(RFCORE_XREG_FSMSTAT1 & RFCORE_XREG_FSMSTAT1_TX_ACTIVE )
      DEBUG("cc2538rf: Still Sending\n");

      DEBUG("cc2538rf: sending complete\n");

      dev->event_cb(NETDEV_EVENT_TX_COMPLETE, NULL);



    gnrc_pktbuf_release(pkt);
    dev->state = NETOPT_STATE_IDLE;

return -1;
}


//TODO: not implemented yet
static int _add_event_cb(gnrc_netdev_t *dev, gnrc_netdev_event_cb_t cb)
{
  DEBUG("cc2538rf: adding event cb:\n");
    if (dev == NULL) {
        return -ENODEV;
    }
    if (dev->event_cb) {
        return -ENOBUFS;
    }

    dev->event_cb = cb;
    return 0;
}


//TODO: not implemented yet
static int _rem_event_cb(gnrc_netdev_t *dev, gnrc_netdev_event_cb_t cb)
{

  DEBUG("cc2538rf: removing event cb:\n");
  if (dev == NULL) {
      return -ENODEV;
  }
  if (dev->event_cb != cb) {
      return -ENOENT;
  }

  dev->event_cb = NULL;
  return 0;
}

//TODO: not implemented yet
static int _get(gnrc_netdev_t *device, netopt_t opt, void *val, size_t max_len)
{

  DEBUG("cc2538rf: getting options:\n");

    if(device == NULL) {
      return -ENODEV;
    }

    cc2538rf_t *dev = (cc2538rf_t *) device;

/*
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
*/

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
            if (max_len < sizeof(uint64_t)) {
                return -EOVERFLOW;
            }
            *((uint64_t *)val) = cc2538rf_get_addr_long(dev);
            return sizeof(uint64_t);

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

      case NETOPT_NID:
      DEBUG("NETOPT_NID  ");
            if (max_len < sizeof(uint16_t)) {
                return -EOVERFLOW;
            }
            *((uint16_t *)val) = dev->pan;
            return sizeof(uint16_t);

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
            if (max_len < sizeof(uint16_t)) {
                return -EOVERFLOW;
            }
            ((uint8_t *)val)[1] = 0;
            ((uint8_t *)val)[0] = cc2538rf_get_chan(dev);
            return sizeof(uint16_t);
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
  cc2538rf_t* dev = (cc2538rf_t*) device;

  DEBUG("cc2538rf: setting options:\n");
  switch(opt){
    case NETOPT_ADDRESS_LONG:
      DEBUG("NETOPT_ADDRESS_LONG\n");
            if (len > sizeof(uint16_t)) {
                return -EOVERFLOW;
            }
            cc2538rf_set_addr_long(dev, *((uint64_t*)val));
            return sizeof(uint16_t);


    case NETOPT_ADDRESS:
      DEBUG("NETOPT_ADDRESS\n");
            if (len > sizeof(uint16_t)) {
                return -EOVERFLOW;
            }
            cc2538rf_set_addr_short(dev, *((uint16_t*)val));
            return sizeof(uint16_t);


    case NETOPT_NID:
      DEBUG("NETOPT_NID\n");
            if (len > sizeof(uint16_t)) {
                return -EOVERFLOW;
            }
            cc2538rf_set_pan(dev, *((uint16_t *)val));
            return sizeof(uint16_t);


    case NETOPT_SRC_LEN:
      DEBUG("NETOPT_SRC_LEN\n");
            if (len > sizeof(uint16_t)) {
                return -EOVERFLOW;
            }
            if (*((uint16_t *)val) == 2) {
                dev->options &= ~(CC2538RF_OPT_SRC_ADDR_LONG);
            }
            else if (*((uint16_t *)val) == 8) {
                dev->options |= CC2538RF_OPT_SRC_ADDR_LONG;
            }
            else {
                return -ENOTSUP;
            }
            return sizeof(uint16_t);

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
  DEBUG("cc2538rf: isr event has been called\n");
}


const gnrc_netdev_driver_t cc2538rf_driver = {
    .send_data = _send,
    .add_event_callback = _add_event_cb,
    .rem_event_callback = _rem_event_cb,
    .get = _get,
    .set = _set,
    .isr_event = _isr_event,
};

/*
const netdev2_driver_t cc2538rf_driver_netdev2 = {
    .send = _send,
    .recv = _receive,
    .init = cc2538rf_init,
    .get = _get,
    .set = _set,
    .isr = _isr_event,
};
*/

//TODO check with contiki and stuff also device not fully initialized
//in sense of riot yet
int cc2538rf_init(cc2538rf_t *dev)
{
  DEBUG("cc2538rf: Init new\n");

  DEBUG("cc2538rf: checking if CSP is runngin or idle: \n");
  if((uint8_t)RFCORE_XREG_CSPSTAT & CC2538RF_CSP_RUNNING){
    DEBUG("cc2538rf: CSP is runngin\n");
  }else{
    DEBUG("cc2538rf: CSP is idle\n");
  }

  //setting clock for the rfcore
  SYS_CTRL_RCGCRFC = 1;
  SYS_CTRL_SCGCRFC = 1;
  SYS_CTRL_DCGCRFC = 1;

  RFCORE_XREG_CCACTRL0 = CC2538RF_CCA_THRES;

  dev->driver = &cc2538rf_driver;

  //setting up addresses

  uint64_t longAddress;
  cpuid_get((void*)&longAddress);
  uint8_t* longToShortAddress = (uint8_t*) &longAddress;
  cc2538rf_set_addr_long(dev, longAddress);

  DEBUG("cc2538rf: long address from flash: \n");
  for(int i = 0; i<8; i++){
    DEBUG("%x:", longToShortAddress[i]);
  }
  DEBUG("\n");


  DEBUG("cc2538rf: current long address:\n");
  for(int i = 0; i<8; i++){
    DEBUG("%x:", dev->addr_long[i]);
  }
  DEBUG("\n");

  uint16_t addr_short = (dev->addr_long[6]<<8) | dev->addr_long[7];
  cc2538rf_set_addr_short(dev, addr_short);

  DEBUG("cc2538rf: short address:\n");
  for(int i = 0; i<2; i++){
    DEBUG("%x:", dev->addr_short[i]);
  }
  DEBUG("\n");

  //setting pan id
  DEBUG("cc2538rf: setting pan id: %u\n", CC2538RF_DEFAULT_PANID);
  cc2538rf_set_pan(dev, CC2538RF_DEFAULT_PANID);

  //setting channel
  DEBUG("cc2538rf: setting channel: %u\n", CC2538RF_DEFAULT_CHANNEL);
  cc2538rf_set_chan(dev, CC2538RF_DEFAULT_CHANNEL);

#ifdef MODULE_GNRC_SIXLOWPAN
  DEBUG("cc2538rf: 6lowpan enabled \n");
  dev->proto = GNRC_NETTYPE_SIXLOWPAN;
#else
  dev->proto = GNRC_NETTYPE_UNDEF;
#endif

//set device options to use pan
dev->options |= CC2538RF_OPT_USE_SRC_PAN;
//set device options to use long address
dev->options |= CC2538RF_OPT_SRC_ADDR_LONG;


#ifdef MODULE_GNRC_SIXLOWPAN
    dev->proto = GNRC_NETTYPE_SIXLOWPAN;
#else
    dev->proto = GNRC_NETTYPE_UNDEF;
#endif


  //set seq number
  dev->seq_nr = 0;


  //enable fifop interrupts
  RFCORE_XREG_RFIRQM0 |= RFCORE_XREG_RFIRQM0_FIFOP;
  nvic_interrupt_enable(NVIC_INT_RF_RXTX);

  /* Acknowledge all RF Error interrupts */
  RFCORE_XREG_RFERRM = RFCORE_XREG_RFERRM_RFERRM;
  nvic_interrupt_enable(NVIC_INT_RF_ERR);

  //setting up calibration register
  RFCORE_XREG_TXFILTCFG = 0x09; /* TX anti-aliasing filter */
  RFCORE_XREG_AGCCTRL1 = 0x15;  /* AGC target value */
  RFCORE_XREG_FSCAL1 = 0x00;    /* Reduce the VCO leakage */

  //setting up AUTOCRC
  RFCORE_XREG_FRMCTRL0 |= RFCORE_XREG_FRMCTRL0_AUTOCRC;

  //disable source matching
  RFCORE_XREG_SRCMATCH = 0;

  //Setup max fifo threshold
  RFCORE_XREG_FIFOPCTRL = CC2538RF_MAX_PACKET_LEN;

  //rf_flags |= RF_ON;
  RFCORE_XREG_TXPOWER = CC2538RF_TX_POWER;
  dev->state = NETOPT_STATE_IDLE;

  //ENERGEST_ON(ENERGEST_TYPE_LISTEN);


  //put RF into RX mode
    RFCORE_SFR_RFST = CC2538_RF_CSP_OP_ISRXON;


    return 0;
}

void cc2538rf_reset(cc2538rf_t *dev)
{

    DEBUG("cc2538rf_reset(): reset not implemented yet.\n");
}

bool cc2538rf_cca(cc2538rf_t *dev)
{
  DEBUG("cc2538rf_cca(): reset not implemented yet.\n");
  return false;
}

size_t cc2538rf_send(cc2538rf_t *dev, uint8_t *data, size_t len)
{
  DEBUG("cc2538rf_send(): reset not implemented yet.\n");
  return -1;
}

void cc2538rf_tx_prepare(cc2538rf_t *dev)
{
  DEBUG("cc2538rf_tx_prepare(): reset not implemented yet.\n");
}

size_t cc2538rf_tx_load(cc2538rf_t *dev, uint8_t *data,
                         size_t len, size_t offset)
{
  DEBUG("cc2538rf_tx_load(): reset not implemented yet.\n");
  return 0;
}

void cc2538rf_tx_exec(cc2538rf_t *dev)
{
  DEBUG("cc2538rf_tx_exec(): reset not implemented yet.\n");
}

size_t cc2538rf_rx_len(cc2538rf_t *dev)
{
  DEBUG("cc2538rf_rx_len(): reset not implemented yet.\n");
  return 0;
}

void cc2538rf_rx_read(cc2538rf_t *dev, uint8_t *data, size_t len,
                       size_t offset)
{
  DEBUG("cc2538rf_rx_read(): reset not implemented yet.\n");
}


uint8_t cc2538rf_get_chan(cc2538rf_t *dev)
{
  return dev->chan;
}


void cc2538rf_set_chan(cc2538rf_t *dev, uint8_t chan)
{
  uint8_t regChannel = 11+(5*(chan-11));
  RFCORE_XREG_FREQCTRL = regChannel;
  dev->chan = chan;

}

void cc2538rf_set_addr_long(cc2538rf_t *dev, uint64_t addr)
{
  cc2538_rfcore_t* rfcore = RFCORE;

  uint8_t *addr_cut = (uint8_t*) &addr;
  for(int i = 0; i<8; i++){
    //dev->addr_long[i] = (addr >> ((7-i)*8));
    dev->addr_long[i] = addr_cut[i];
  }

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
    longAddr = (longAddr << 8) | dev->addr_long[i];
  }

  return longAddr;
}


void cc2538rf_set_addr_short(cc2538rf_t *dev, uint16_t addr)
{
    uint8_t *addr_cut = (uint8_t*) &addr;
    dev->addr_short[0] = addr_cut[0];
    dev->addr_short[1] = addr_cut[1];
    cc2538_rfcore_t* rfcore = RFCORE;
    rfcore->FFSM_SHORT_ADDR0 = addr_cut[1];
    rfcore->FFSM_SHORT_ADDR1 = addr_cut[0];
}


uint16_t cc2538rf_get_addr_short(cc2538rf_t *dev)
{
  uint16_t shortAddr = (dev->addr_short[1] << 8) | dev->addr_short[0];
  return shortAddr;
}


void cc2538rf_set_pan(cc2538rf_t *dev, uint16_t pan)
{
    RFCORE_FFSM_PAN_ID0 = (uint8_t) pan;
    RFCORE_FFSM_PAN_ID1 = (uint8_t) pan>>8;
    dev->pan = pan;

}


uint16_t cc2538rf_get_pan(cc2538rf_t *dev)
{
  return dev->pan;
}
