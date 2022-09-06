#include "deca_device_api.h"
//#include "instance.h"
#include "util.h"

typedef struct
{
	struct configuration_t{
		uint16_t packet_size;
		dwt_config_t radio_config;
		dwt_txconfig_t tx_config;
		uint32_t IPI_wait;
                uint32_t cca_wait;
                uint16_t dst_addr;
	}config;
        struct node_t{
          uint8_t tx_enable;
          uint8_t rx_enable;
          uint8_t update_radio_config;
          uint64_t tx_timestape;
          uint32_t end_sequence_number;
          uint32_t sequence_number;
          uint64_t tx_delay_ms;
          uint8_t tx_msg_type;
          
        }node;
        struct {
          struct{
            uint64_t poll_tx_ts;
            uint64_t resp_rx_ts;
            uint64_t final_tx_ts;
          }initiator;
          struct{
            uint64_t poll_rx_ts;
            uint64_t resp_tx_ts;
          }responder;
        }twr;
} instance_info_t;

extern instance_info_t instance_info;
extern dwt_config_t radio_config;
extern dwt_config_t TS_config;
extern dwt_txconfig_t txconfig;


#define DELAY_TX DELAY_TX_MS(100)
#define DELAY_TX_IPI DELAY_TX_100US(45)
#define DELAY_TS     DELAY_TX_MS(1000);
#define TX_PKT_CNT      500
#define TX_DELAY_MS     1000




#define POLL_TX_TO_RESP_RX_DLY_UUS 700
/* This is the delay from Frame RX timestamp to TX reply timestamp used for 
 * calculating/setting the DW IC's delayed TX function. This includes the
 * frame length of approximately 190 us with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 700
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 300
//#define DELAY_TX DELAY_TX_100US(0)
//#define DELAY_TX_IPI DELAY_TX_100US(0)

#define RNG_DELAY_MS 1000


/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
