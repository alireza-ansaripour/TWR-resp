#include "instance.h"
#include <stdio.h>
#include <stdlib.h>
#include <sdk_config.h>
#include "boards.h"
#include "port.h"
#include "deca_spi.h"
#include <math.h>
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port.h"
#include "identity.h"
#include "util.h"
#include "instance_config.h"
#include "ring_buffer.h"
#include "shared_defines.h"
#include "mac_802_15_4.h"
#include "nrf_drv_uart.h"
#include "messages.h"
#define CIR_SAMPLE 32
#define CIR_SAMPLE_BUFFER CIR_SAMPLE * 6 + 1

int data_index = 0;
uint8_t UART_msg_payload[200];
uint8_t UART_rx_msg_payload[200];
uint8_t UART_byte, packet_len;
static uint64_t m_systick_cnt;
uint64_t get_tx_timestamp_u64(void);
uint64_t get_rx_timestamp_u64(void);
static nrf_drv_uart_t app_uart_inst = NRF_DRV_UART_INSTANCE(APP_UART_DRIVER_INSTANCE);
enum State_machine {
  SYNC_1, SYNC_2, P_LEN, RECV_DATA, DATA_DONE
};

void set_dwt_config(dwt_config_t *config){
  memcpy(&instance_info.config.radio_config, config, sizeof(dwt_config_t));
  instance_info.node.update_radio_config = 1;
}

enum State_machine state = SYNC_1;
void handle_uart_message(uint8_t *data, size_t data_len){
  uint8_t msg_type = data[0];
  uint8_t *payload = &data[1];
  uint32_t *seq;
  uint64_t *wait_time;
  switch(msg_type){
    case CONFIG:
      set_dwt_config((dwt_config_t *) payload);
    break;
    case START_TX:
      instance_info.node.tx_enable = 1;
      instance_info.node.tx_timestape = m_systick_cnt;
    break;
    case SEQ_NUM:
      seq = (uint32_t *) payload;
      instance_info.node.sequence_number = *seq;
    break;
    case TX_NUM:
      seq = (uint32_t *) payload;
      instance_info.node.end_sequence_number = instance_info.node.sequence_number + *seq;
    break;
    case WAIT_TIME:
      wait_time = (uint64_t *) payload;
      instance_info.node.tx_delay_ms = wait_time;
    break;

      //instance_info.config.sequence_number = 
  };
  //printf("MSG type: %X\n", msg_type);

}


static void uart_event_handler(nrf_drv_uart_event_t * p_event, void* p_context)
{
    if (p_event->type == NRF_DRV_UART_EVT_RX_DONE)
    {
        if (p_event->data.rxtx.bytes)
        {
          switch (state){
            case SYNC_1:
            if (UART_byte == UART_HEADER[0]){
              state = SYNC_2;
            }
            break;
            case SYNC_2:
              if (UART_byte == UART_HEADER[1])
                state = P_LEN;
              else
                state = SYNC_1;
              break;
            case P_LEN:
              packet_len = UART_byte;
              data_index = 0;
              state = RECV_DATA;
            break;
            case RECV_DATA:
              UART_rx_msg_payload[data_index] = UART_byte;
              data_index++;
            if (data_index == packet_len){
              data_index = 0;
              state = SYNC_1;
              uint8_t msg_type = UART_rx_msg_payload[0];
              UART_ACK[sizeof(UART_ACK) - 1] = msg_type;
              handle_uart_message(UART_rx_msg_payload, packet_len);
              send_UART_msg(UART_ACK, sizeof(UART_ACK));
            }
          }

          nrf_drv_uart_rx(&app_uart_inst, &UART_byte, 1);

            
            
        }
    }
    else if (p_event->type == NRF_DRV_UART_EVT_ERROR)
    {
      printf("ERR\n");
      nrf_drv_uart_rx(&app_uart_inst, &UART_byte, 1);
        // Event to notify that an error has occured in the UART peripheral
    }
    else if (p_event->type == NRF_DRV_UART_EVT_TX_DONE)
    {
       // Event to notify that the last byte from FIFO has been transmitted

    }
}

static void uart_init(){
    nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;
    uart_config.baudrate = UART_BAUDRATE_BAUDRATE_Baud115200; //User defined
    uart_config.hwfc = NRF_UART_HWFC_DISABLED; //User defined
    uart_config.interrupt_priority = APP_IRQ_PRIORITY_LOWEST; //User defined
    uart_config.parity = NRF_UART_PARITY_EXCLUDED; //User defined
    uart_config.pselcts = CTS_PIN_NUMBER; //User defined. Remove this line if flow control is disabled.
    uart_config.pselrts = RTS_PIN_NUMBER; //User defined. Remove this line if flow control is disabled.
    uart_config.pselrxd = RX_PIN_NUMBER; //User defined
    uart_config.pseltxd = TX_PIN_NUMBER; //User defined

    uint32_t err_code = nrf_drv_uart_init(&app_uart_inst, &uart_config, uart_event_handler);
    APP_ERROR_CHECK(err_code);
}


#define ADDRESS 0xFFFF
#define SET_OUPUT_GPIOs 0xFFFF & ~(GPIO_DIR_GDP1_BIT_MASK | GPIO_DIR_GDP2_BIT_MASK | GPIO_DIR_GDP3_BIT_MASK)
#define ENABLE_ALL_GPIOS_MASK 0x200000
#define SWITCH_CONF_INDEX 10


uint32_t sequence_numbers[10000];
uint8_t rx_buffer[100];
mac_frame_802_15_4_format_t mac_frame;

uint32_t seq_num = 0;
uint32_t tx_num = 0;

uint32_t rx_timestamp;

uint32_t swap_uint32( uint32_t num )
{
    return((num>>24)&0xff) | // move byte 3 to byte 0
          ((num<<8)&0xff0000) | // move byte 1 to byte 2
          ((num>>8)&0xff00) | // move byte 2 to byte 1
          ((num<<24)&0xff000000); // byte 0 to byte 3;
}

uint16_t swap_uint16( uint16_t num )
{
    return (num>>8) | (num<<8);
}


void init_LEDs(){
  dwt_enablegpioclocks();
  dwt_write32bitoffsetreg(GPIO_MODE_ID, 0, ENABLE_ALL_GPIOS_MASK);
  dwt_write16bitoffsetreg(GPIO_OUT_ID, 0, 0x0);
  dwt_write16bitoffsetreg(GPIO_DIR_ID, 0, SET_OUPUT_GPIOs);
}
int res;

void gpio_set(uint16_t port){
  dwt_or16bitoffsetreg(GPIO_OUT_ID, 0, (port));
}

void gpio_reset(uint16_t port){
  dwt_and16bitoffsetreg(GPIO_OUT_ID, 0, (uint16_t)(~(port)));
}


int flg = 0;


void SysTick_Handler(void) {
    m_systick_cnt++;
}

void init_NRF(){
  bsp_board_init(BSP_INIT_BUTTONS);
  /* Initialise the SPI for nRF52840-DK */
  nrf52840_dk_spi_init();
  /* Configuring interrupt*/
  dw_irq_init();
  /* Small pause before startup */
  nrf_delay_ms(2);
  SysTick_Config(64);
  NVIC_EnableIRQ(SysTick_IRQn);
}



packet_std_t rx_packet, tx_packet;
packet_ranging_t * ranging_data;
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;
static uint64_t final_rx_ts;
static double tof;
static double distance;


void tx_conf_cb(const dwt_cb_data_t *cb_data){
  //printf("tx done\n");
  gpio_reset(LED_TX);
  switch(instance_info.node.tx_msg_type){
    case MSG_RESP:
      dwt_readtxtimestamp(&resp_tx_ts);
  }
}

void rx_to_cb(const dwt_cb_data_t *cb_data){
  //gpio_set(LED_RX);
	//instance_info.diagnostics.uwb.rx.to_cb_count++;
}

void rx_err_cb(const dwt_cb_data_t *cb_data){
  instance_info.node.rx_enable = 1;
  //instance_info.diagnostics.uwb.rx.err_cb_count++;
}

void rx_ok_cb(const dwt_cb_data_t *cb_data){
  dwt_readrxdata(&rx_packet, cb_data->datalength, 0);
  switch(rx_packet.msg_type){
    case MSG_POLL:
      dwt_readrxtimestamp(&poll_rx_ts);
      instance_info.node.tx_msg_type = MSG_RESP;
      instance_info.node.tx_enable = 1;
      instance_info.node.tx_timestape = m_systick_cnt + instance_info.node.tx_delay_ms * 1000;
      //printf("POLL RX\n");
    break;
    case MSG_FINAL:
      dwt_readrxtimestamp(&final_rx_ts);
      ranging_data = (packet_ranging_t *) rx_packet.payload;
      uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
      uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
      double Ra, Rb, Da, Db;
      int64_t tof_dtu;
      poll_tx_ts = ranging_data->poll_tx_ts;
      resp_rx_ts = ranging_data->resp_rx_ts[identity_get_address()];
      final_tx_ts = ranging_data->final_tx_ts;
      poll_rx_ts_32 = (uint32_t)poll_rx_ts;
      resp_tx_ts_32 = (uint32_t)resp_tx_ts;
      final_rx_ts_32 = (uint32_t)final_rx_ts;
      Ra = (double)(resp_rx_ts - poll_tx_ts);
      Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
      Da = (double)(final_tx_ts - resp_rx_ts);
      Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
      tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));
      tof = tof_dtu * DWT_TIME_UNITS;
      distance = tof * SPEED_OF_LIGHT;
      int dist = (int)(100 * distance);
      printf("Distance %d\n", dist);
      instance_info.node.rx_enable = 1;
    break;
  
  }
  if(instance_info.node.tx_enable == 0){
    instance_info.node.rx_enable = 1;
  }
}


void create_tx_packet(){
  tx_packet.packet_id = 0x8841;
  tx_packet.sequence_number = 0;
  tx_packet.pan_id = 0xDECA;
  tx_packet.src = identity_get_address();
  tx_packet.dst = 0xffff;
}

void enable_intrupt_DW(){
  dwt_setcallbacks(&tx_conf_cb, &rx_ok_cb, NULL, &rx_err_cb, NULL, NULL);

  /* Enable wanted interrupts (TX confirmation, RX good frames,
   * RX timeouts and RX errors). */
  dwt_setinterrupt(SYS_ENABLE_LO_TXFRS_ENABLE_BIT_MASK |
                   SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK |
                   SYS_ENABLE_LO_RXFTO_ENABLE_BIT_MASK |
                   SYS_ENABLE_LO_RXPTO_ENABLE_BIT_MASK |
                   SYS_ENABLE_LO_RXPHE_ENABLE_BIT_MASK |
                   SYS_ENABLE_LO_RXFCE_ENABLE_BIT_MASK |
                   SYS_ENABLE_LO_RXFSL_ENABLE_BIT_MASK |
                   SYS_ENABLE_LO_RXSTO_ENABLE_BIT_MASK,
                   0,
                   DWT_ENABLE_INT);

  /* Clearing the SPI ready interrupt */
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RCINIT_BIT_MASK | SYS_STATUS_SPIRDY_BIT_MASK);

  /* Install DW IC IRQ handler. */
  port_set_dwic_isr(dwt_isr);
}

uint64_t start_time = 0;
void instance_init(){
  init_NRF();
  uart_init();
  //port_set_dwic_isr(ali_isr);
  port_set_dw_ic_spi_fastrate();

  /* Reset DW IC */
  reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

  Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

  while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */
  { };
  
  dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
      while (1)
      { };
  }
  instance_config_identity_init();
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RCINIT_BIT_MASK | SYS_STATUS_SPIRDY_BIT_MASK);
  init_LEDs();
  enable_intrupt_DW();
  nrf_drv_uart_rx(&app_uart_inst, &UART_byte, 1);
  create_tx_packet();
  instance_info.node.tx_timestape = m_systick_cnt;
}

void send_UART_msg(uint8_t *msg, uint16_t payload_len){
  uint8_t pointer = 0;
  UART_msg_payload[0] = 0x5C;
  UART_msg_payload[1] = 0x51;
  memcpy(&UART_msg_payload[2], (uint8_t *) &payload_len, 2);
  memcpy(UART_msg_payload + 3, msg, payload_len);
  send_uart(&UART_msg_payload[0], payload_len + 3);
  return;
}



uint64_t get_tx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int8_t i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

uint64_t get_rx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int8_t i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}



void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts)
{
    uint8_t i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        *ts += ((uint32_t)ts_field[i] << (i * 8));
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
void final_msg_set_ts(uint8_t *ts_field, uint64_t ts)
{
    uint8_t i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8_t)ts;
        ts >>= 8;
    }
}




uint32_t ts1, ts2;
int err_counter = 0;
static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xff, 0xff, 0xff, 0xff, 0x01, 0x02, 0x02, 0, 0, 0};
static uint8_t rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#define ALL_MSG_SN_IDX 2
#define ALL_MSG_COMMON_LEN 5
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define RX_BUF_LEN 128
#define POLL_RX_TO_RESP_TX_DLY_UUS 900
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
#define FINAL_RX_TIMEOUT_UUS 220
#define PRE_TIMEOUT 5

uint8_t frame_seq_nb = 0;
uint32_t final_tx_ts;

/* Hold copies of computed time of flight and distance here for reference
 * so that it can be examined at a debug breakpoint. */



void instance_loop(){
  uint64_t t = m_systick_cnt;
  if (instance_info.node.tx_enable && t > instance_info.node.tx_timestape){
    gpio_set(LED_TX);
    instance_info.node.tx_enable = 0;
    tx_packet.msg_type = MSG_RESP;
    uint32_t current_ts = dwt_readsystimestamphi32();
    uint32_t resp_tx_time = ((( ((uint64_t)(current_ts & 0xFFFFFFFEUL))<< 8) + 500 * UUS_TO_DWT_TIME) >> 8);
    dwt_setdelayedtrxtime(resp_tx_time);
    dwt_writetxdata(PACKET_STD_HDR_LEN + FCS_LEN, &tx_packet, 0);
    dwt_writetxfctrl(PACKET_STD_HDR_LEN + FCS_LEN, 0, 1); 
    int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED); 
    //int ret = dwt_starttx(DWT_START_TX_IMMEDIATE); 
    if (ret == DWT_ERROR) {
        printf("fuck\n");
        return;
    }
  }
  if (instance_info.node.rx_enable){
    instance_info.node.rx_enable = 0;
    int res = dwt_rxenable(DWT_START_RX_IMMEDIATE);
  } 

}


//uint32_t status_reg;
//void instance_loop(){
//  dwt_setpreambledetecttimeout(0);
//  /* Clear reception timeout to start next ranging process. */
//  dwt_setrxtimeout(0);

//  /* Activate reception immediately. */
//  int res = dwt_rxenable(DWT_START_RX_IMMEDIATE);
//  printf("RX res: %d\n", res);

//  /* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
//  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
//                                          (SYS_STATUS_RXFCG_BIT_MASK |
//                                           SYS_STATUS_ALL_RX_TO |
//                                           SYS_STATUS_ALL_RX_ERR)))
//  { /* spin */ };
//  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {

//    /* Clear good RX frame event in the DW IC status register. */
//    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

//    /* A frame has been received, read it into the local buffer. */
//    uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
//    if (frame_len <= RX_BUF_LEN) {
//        dwt_readrxdata(&rx_packet, frame_len, 0);
//    }
//     rx_buffer[ALL_MSG_SN_IDX] = 0;
//    if (rx_packet.dst && rx_packet.msg_type == MSG_POLL) {
//        uint32_t resp_tx_time;

//        /* Retrieve poll reception timestamp. */
//        //poll_rx_ts = get_rx_timestamp_u64();
//        tx_packet.msg_type = MSG_RESP;
//        dwt_readrxtimestamp(&poll_rx_ts);

//        resp_tx_time = (poll_rx_ts + ((2000)) * UUS_TO_DWT_TIME) >> 8;
//        dwt_setdelayedtrxtime(resp_tx_time);

//        /* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
//        dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
//        //dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
//        //dwt_setpreambledetecttimeout(PRE_TIMEOUT);

//        /* Write and send the response message. See NOTE 10 below.*/
//        tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        
                       
//        dwt_writetxdata(PACKET_STD_HDR_LEN + FCS_LEN, &tx_packet, 0);
//        dwt_writetxfctrl(PACKET_STD_HDR_LEN + FCS_LEN, 0, 1); 
//        int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED); 
//        if (ret == DWT_ERROR) {
//            printf("fuck\n");
//            return;
//        }

//        /* Poll for reception of expected "final" frame or error/timeout.
//         * See NOTE 8 below.
//         */
//        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
//                                               (SYS_STATUS_RXFCG_BIT_MASK |
//                                                SYS_STATUS_ALL_RX_TO |
//                                                SYS_STATUS_ALL_RX_ERR)))
//        { /* spin */ };
//        frame_seq_nb++;

//        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {

//            /* Clear good RX frame event and TX frame sent in the DW IC status register. */
//            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);

//            /* A frame has been received, read it into the local buffer. */
//            frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
//            if (frame_len <= RX_BUF_LEN) {
//                dwt_readrxdata(&rx_packet, frame_len, 0);
//            }
//            rx_buffer[ALL_MSG_SN_IDX] = 0;
//            if (rx_packet.dst  && rx_packet.msg_type == MSG_FINAL) {
//                ranging_data = (packet_ranging_t *) rx_packet.payload;
//                uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
//                uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
//                double Ra, Rb, Da, Db;
//                int64_t tof_dtu;
//                resp_tx_ts = get_tx_timestamp_u64();
//                final_rx_ts = get_rx_timestamp_u64();
//                poll_tx_ts = ranging_data->poll_tx_ts;
//                resp_rx_ts = ranging_data->resp_rx_ts[identity_get_address()];
//                final_tx_ts = ranging_data->final_tx_ts;
//                poll_rx_ts_32 = (uint32_t)poll_rx_ts;
//                resp_tx_ts_32 = (uint32_t)resp_tx_ts;
//                final_rx_ts_32 = (uint32_t)final_rx_ts;
//                Ra = (double)(resp_rx_ts - poll_tx_ts);
//                Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
//                Da = (double)(final_tx_ts - resp_rx_ts);
//                Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
//                tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

//                tof = tof_dtu * DWT_TIME_UNITS;
//                distance = tof * SPEED_OF_LIGHT;
//                int dist = (int)(100 * distance);
//                printf("Distance %d\n", dist);
//            }
//        }else{
//          printf("RX ERR!!!\n");
//        }

//    }
// }

//}
