#ifndef COMM_DRIVER_H_
#define COMM_DRIVER_H_


#define RX_UART2_SOFT_TIMEOUT   1     //  use software timer to find end of frame timeout

#define REG_READ    0
#define REG_WRITE   1

//  GHC-frame size
#define RX_BUFF_ARR_SIZE 20

#define RX_START_POS                0
#define RX_FRAME_TYPE_POS           2
#define RX_DEV_ID_ARR_POS           4
#define RX_DEV_GROUP_ARR_POS        5
#define RX_DEV_SERIAL_H_ARR_POS     6
#define RX_DEV_SERIAL_L_ARR_POS     7
#define RX_REMOTE_ADDR_ARR_POS      8   //  [8,9,10,11,12]

#define RX_DATA_LENGTH_ARR_POS      13

#define RX_COMMAND_ARR_POS          14

//  payload for 5bytes data length
#define RX_REG_H_BYTE_ARR_POS       15
#define RX_REG_L_BYTE_ARR_POS       16
#define RX_DATA_HIGH_BYTE_ARR_POS   17
#define RX_DATA_LOW_BYTE_ARR_POS    18
#define RX_CRC_ARR_POS              19

//  frame type
#define FRAME_TYPE_REG_RW       0x0A    //  register RW operation frame
#define FRAME_TYPE_REG_ACK      0x0B    //  ack frame in response to a remote request
#define FRAME_TYPE_REG_REPLY    0x0C    //  reply for READ request

#define FRAME_TYPE_DATA         0x12    //  data frame type
#define FRAME_TYPE_BROAD        0x31    //  request to remote devices network to send all known data registers type
//  and updating own registers
#define FRAME_TYPE_BROAD_ACK    0x32    //  ack frame in response to a received broadcast frame

//  command to execute
#define CMD_REG_increase    0x01
#define CMD_REG_decrease    0x02
#define CMD_REG_read        0x10
#define CMD_REG_write       0x12

//  register identifier
#define id_MM_STOP			0x0050
#define id_MM_FREE			0x0051
#define id_MM_RUN_ANGLE		0x0052
#define id_MM_RUN_DISTANCE	0x0053
#define id_MM_RUN_SPEED		0x0054
#define id_MM_RUN_START		0x0055
#define id_MM_RUN_DONE		0x0056
#define id_MM_ROTATE_ANGLE	0x0057
#define id_MM_ROTATE_DONE	0x0058
#define id_MM_TOTAL_DISTANCE 0x0059
#define id_MM_LOGIC_DISTANCE 0x005A


#define id_MM_OVF			0x0060
#define id_MM_CLR_QUE		0x0061

#define id_MM_CTRL_A		0x0062
#define id_MM_CTRL_B		0x0063

#define id_HAPPINESS_LEVEL   0x0200
#define id_HAPPINESS_ANGLE   0x0201

//  device network identifier
#define REMOTE_DEV_MOD_MAIN		0x00
#define REMOTE_DEV_MOD_MOTION	0x01

//  send data request flag array position
#define TX_ACK_ARR_SIZE 8

//  flag position in ACK array
#define TX_ACK_MM_STOP_ARR_POS 0
#define TX_ACK_MM_RUN_START_ARR_POS 1
#define TX_ACK_MM_ROTATE_ANGLE_ARR_POS 2
#define TX_ACK_MM_OVF_ARR_POS 3
#define TX_ACK_MM_CLR_QUE_ARR_POS 4

#define TX_ACK_CTRLA_ARR_POS 5
#define TX_ACK_CTRLB_ARR_POS 6

#define TX_BROAD_ARR_POS 7

volatile uint8_t RxBufferArray_UART2 [RX_BUFF_ARR_SIZE];

volatile uint32_t RxCnt_WD;     //  EOT WD timer, watchdog timer is used to check long time period of inactivity on UART

volatile uint8_t UART2_frame_buffer_flag;
volatile uint32_t GW_TMR_UART2_Rx_EOT_tmr;
volatile uint32_t GW_TMR_UART2_Rx_act;

uint8_t commEvent_UART2 ( void );

void RxFrame_UART2 ( void );

void Rx_dataFramePayloadDecoder ( void );
void Rx_dataFrameCtrlAdecoder(void);
void Rx_dataFrameCtrlBdecoder(void);

void Tx_broadcast (void);
void TxFrame_UART2 ( void );
void Tx_UART2 ( uint8_t frame_type, uint16_t register_code, uint16_t data_to_send) ;

void frameIdAckArrayReset_UART2 ( void );

uint16_t W_check_diff (uint16_t W_tmp, uint16_t *W_prev, uint8_t W_diff);
uint8_t humAssistRemoteStatus( void);
uint16_t sensorADCtoCelsius( uint16_t adcVal );

#endif	/* COMM_GW_H */





#endif /* COMM_DRIVER_H_ */