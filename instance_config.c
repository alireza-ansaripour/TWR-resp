/*
 * instance_config.c
 *
 *  Created on: Oct 14, 2019
 *      Author: milad
 */

#include "instance_config.h"
#include "identity.h"
#include "instance.h"
#include "port.h"
#define PAN_ID      0
//#define SHORT_ADDR  0x0200 /* "RX" */
#define SHORT_ADDR  0x0000 /* "RX" */
//#define SHORT_ADDR  0x003 /* "RX" */
instance_info_t instance_info;



//dwt_config_t radio_config = {
//    5,               /* Channel number. */
//    DWT_PLEN_2048,    /* Preamble length. Used in TX only. */
//    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
//    9,               /* TX preamble code. Used in TX only. */
//    9,               /* RX preamble code. Used in RX only. */
//    1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
//    DWT_BR_850K,      /* Data rate. */
//    DWT_PHRMODE_EXT, /* PHY header mode. */
//    DWT_PHRRATE_STD, /* PHY header rate. */
//    (8000 + 1 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
//    DWT_STS_MODE_OFF, /* STS disabled */
//    DWT_STS_LEN_64,/* STS length see allowed values in Enum dwt_sts_lengths_e */
//    DWT_PDOA_M0      /* PDOA mode off */
//};


dwt_config_t radio_config = {
    5,               /* Channel number. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard 8 symbol SFD,
                      *   1 to use non-standard 8 symbol,
                      *   2 for non-standard 16 symbol SFD and
                      *   3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,/* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0      /* PDOA mode off */
};
void identity_set_role(Role role);
dwt_txconfig_t txconfig =
{
    0x34,           /* PG delay. */
    0x10101010,      /* TX power. */
    0x0             /*PG count*/
};


packet_std_t ack_pkt;


void instance_config_identity_init() {
  uint32_t dev_id = NRF_FICR->DEVICEADDR[0];
  init_config();
  identity_set_role(ROLE_RESP); 
  switch(dev_id){      
    case 0xf965bd2e:
      identity_set_address(0x0000);
      instance_info.node.tx_delay_ms = 1;
    break;
    default:
      identity_set_address(0x0001);
      instance_info.node.tx_delay_ms = 4;
    break;

  }
  
  configure_node();  
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  //dwt_setpanid(PAN_ID);
}

void identity_set_role(Role role) {
  identity_set_operations(IDENTITY_OPERATIONS_NONE);
  switch (role) {
    case ROLE_INIT:
      identity_append_operations(IDENTITY_OPERATIONS_INIT_TWR);
      instance_info.node.tx_enable = 1;
      instance_info.node.rx_enable = 0;
      instance_info.node.tx_msg_type = MSG_POLL;
    break;
    case ROLE_RESP:
      identity_append_operations(IDENTITY_OPERATIONS_RESP_TWR);
      instance_info.node.rx_enable = 1;
      instance_info.node.tx_enable = 0;
    break;
  } 
}

void init_config() {
  instance_info.config.packet_size = 100;  
  instance_info.node.end_sequence_number = 5000;
  instance_info.config.tx_config = txconfig;
  instance_info.node.sequence_number = 0;
  instance_info.node.tx_enable = 0;
  instance_info.node.rx_enable = 0;
}

void configure_node() {
  instance_info.config.radio_config = radio_config;
  dwt_configure(&instance_info.config.radio_config);
  dwt_configuretxrf(&txconfig);
}

