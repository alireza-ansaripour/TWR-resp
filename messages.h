typedef enum Msg_type{
	DATA_OK,
	DATA_ERR,
	CONFIG,
	CONFIG_TX,
	ROLE,
	START_TX,
	PACKET_SIZE,
	SEQ_NUM,
	TX_NUM,
	DUMMY,
	WAIT_TIME,
	ID,
	ACK_EN,
	PLEN64_EN,
        TX_DONE,
}Msg_type;

typedef struct{
  Msg_type type;
  uint8_t dataLen;
  uint8_t payload;
}UART_data; 


uint8_t UART_HEADER[] = {0x5c, 0x51};
uint8_t UART_ACK[] = {0xC0, 0};
