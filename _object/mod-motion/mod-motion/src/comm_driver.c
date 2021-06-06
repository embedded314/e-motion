/*
 * comm_driver.c
 *
 * Created: 16/05/2021 20:22:02
 *  Author: Krzysztof Siejka
 */ 
#include <comm_driver.h>

#include <stdlib.h>
#include <inttypes.h>
#include <math.h>

volatile uint8_t GW_UART2_frame_type;
volatile uint16_t GW_register_id;
volatile uint16_t GW_register_val;
volatile uint8_t GW_register_CRC;

#define COMM_UART2_TX_PERIOD 20    //  outgoing transmission frame period 20ms

volatile uint8_t TxAckArray_UART2 [TX_ACK_ARR_SIZE];    //  received frame ACK
volatile uint32_t WD_comm_tmr;      //  communication watchdog timer
uint8_t commEvent_UART2(void) {
    uint8_t frameReceivedFlag = 0;
    //  frame buffer full, all frame data received
    if (UART2_frame_buffer_flag == 1) {
        //  run data frame decoder, after it clear frame buffer busy flag
        RxFrame_UART2();
        UART2_frame_buffer_flag = 0;

        frameReceivedFlag = 1;
    }

    //  5ms timeout resets UART RxBuffer element pointer
    //  now is a set as a hardware solution based on TMR6 support
    //  TMR6 it's started after rising edge UART2 event are noticed, last bit means last rising edge 
    if ((millis() - GW_TMR_UART2_Rx_act) > 5) {
        GW_UART2_frame_byte_pos = 0;
    }

    //  send to remote device another frame from the queue 
    if ((millis() - WD_comm_tmr) > COMM_UART2_TX_PERIOD) {
        TxFrame_UART2();
        WD_comm_tmr = millis();
    }
    
    return frameReceivedFlag;
}

volatile uint16_t frameControlBlockA_UART2, frameControlBlockB_UART2;
volatile uint8_t GW_register_CRC;

void RxFrame_UART2(void) {
    if (RxCnt_WD == 0) RxCnt_WD = millis();
    //  receive frame type Id
    GW_UART2_frame_type = RxBufferArray_UART2 [RX_FRAME_TYPE_POS];
    
    if (GW_UART2_frame_type == FRAME_TYPE_BROAD_ACK) {
        //  frame type: received acknowledge in response to a broadcast frame
        //  if broadcast frame acknowledge is received from remote, host should 
        //  stop sending broadcast frame.
        TxAckArray_UART2 [TX_BROAD_ARR_POS] = 0;

        //  end of transmission watchdog reset
        RxCnt_WD = millis();
    } else if (GW_UART2_frame_type == FRAME_TYPE_REG_RW) {
        if (RxBufferArray_UART2 [RX_COMMAND_ARR_POS] == FRAME_TYPE_DATA) {
            GW_register_id =  ((RxBufferArray_UART2 [RX_REG_H_BYTE_ARR_POS] << 8) |
                               (RxBufferArray_UART2 [RX_REG_L_BYTE_ARR_POS] & 0xff));
            GW_register_val = ((RxBufferArray_UART2 [RX_DATA_HIGH_BYTE_ARR_POS] << 8) | 
                               (RxBufferArray_UART2 [RX_DATA_LOW_BYTE_ARR_POS] & 0xff));

            GW_register_CRC = RxBufferArray_UART2 [RX_REG_H_BYTE_ARR_POS] + 
                              RxBufferArray_UART2 [RX_REG_L_BYTE_ARR_POS] + 
                              RxBufferArray_UART2 [RX_DATA_HIGH_BYTE_ARR_POS] + 
                              RxBufferArray_UART2 [RX_DATA_LOW_BYTE_ARR_POS];

            //  frame payload decoder
            Rx_dataFramePayloadDecoder ();
        }
        
        //  end of transmission watchdog reset
        RxCnt_WD = millis();
    }
}

volatile uint8_t P_frame_cnt;
void Rx_dataFramePayloadDecoder(void) {
    //  reject not supported values for some type of frames
    if ((GW_register_CRC != RxBufferArray_UART2 [RX_CRC_ARR_POS])) {
        GW_register_id = 0x00;
    }
    
    uint8_t RW_flag;
    if (RxBufferArray_UART2 [RX_COMMAND_ARR_POS] == CMD_REG_write) {
        RW_flag = REG_WRITE;
    } else if (RxBufferArray_UART2 [RX_COMMAND_ARR_POS] == CMD_REG_read) {
        RW_flag = REG_READ;
    }
    
    switch (GW_register_id) {
        case id_MM_STOP:
	        if (RW_flag == REG_WRITE) {
		        //	mm_stop register = GW_register_val;
				TxAckArray_UART2 [TX_ACK_MM_STOP_ARR_POS] = 1;
	        } else {
				Tx_UART2(FRAME_TYPE_REG_REPLY, id_MM_STOP, 1);
			}
			break;
//  ----------------------------------------------------------------------------
        case id_MM_TOTAL_DISTANCE:
			if (RW_flag == REG_READ) {
			    Tx_UART2(FRAME_TYPE_REG_REPLY, id_MM_TOTAL_DISTANCE, mm_total_distance());
		    }
	        break;
//  ----------------------------------------------------------------------------
        case id_MM_CTRL_A:
            Rx_dataFrameCtrlAdecoder ();
            break;
        case id_MM_CTRL_B:
            Rx_dataFrameCtrlBdecoder ();
            break;
//  ----------------------------------------------------------------------------
    }
}

void Tx_broadcast (void) {
    TxAckArray_UART2 [TX_BROAD_ARR_POS] = 1;
}

void Rx_dataFrameCtrlAdecoder(void) {
    TxAckArray_UART2 [TX_ACK_CTRLA_ARR_POS] = 1;
    frameControlBlockA_UART2 = GW_register_val;

    //  VPD heater request flag
    if ((frameControlBlockA_UART2 & (1 << 0)) && (frameControlBlockA_UART2 & (1 << 1))) {
		//	bit 0 & 2 is set to 1
    } else {
		//	bit 0 & 2 is set to 0
    }
}

void Rx_dataFrameCtrlBdecoder(void) {
    TxAckArray_UART2 [TX_ACK_CTRLB_ARR_POS] = 1;
    frameControlBlockB_UART2 = GW_register_val;
}

volatile uint8_t frameQueCnt_UART2;
void TxFrame_UART2(void) {
    //  skip frame transmission if remote ACK flag is cleared
    while ((TxAckArray_UART2 [frameQueCnt_UART2] == 0) && (frameQueCnt_UART2 < 12)) {
        frameQueCnt_UART2++;
    }
    //  send frame receive confirmation to remote device and clear ACK flag
    if (TxAckArray_UART2 [frameQueCnt_UART2] == 1) {
        switch (frameQueCnt_UART2) {
            case TX_ACK_MM_STOP_ARR_POS:
	            Tx_UART2(FRAME_TYPE_REG_ACK, id_MM_STOP, 1);
				TxAckArray_UART2 [frameQueCnt_UART2] = 0;
				break;
            case TX_ACK_CTRLA_ARR_POS:
                Tx_UART2(FRAME_TYPE_REG_ACK, id_MM_CTRL_A, frameControlBlockA_UART2);
                TxAckArray_UART2 [frameQueCnt_UART2] = 0;
                break;
            case TX_ACK_CTRLB_ARR_POS:
                Tx_UART2(FRAME_TYPE_REG_ACK, id_MM_CTRL_B, frameControlBlockB_UART2);
                TxAckArray_UART2 [frameQueCnt_UART2] = 0;
                break;
            case TX_BROAD_ARR_POS:
                //  send to network broadcast command to activate any other connected devices
                Tx_UART2(FRAME_TYPE_BROAD, 0xAAAA, 0xAAAA);
                TxAckArray_UART2 [frameQueCnt_UART2] = 0;
                break;
        }
    }
    if (frameQueCnt_UART2 < TX_ACK_ARR_SIZE)
        frameQueCnt_UART2++;
    else {
        if (P_frame_cnt > 0) P_frame_cnt--;
        frameQueCnt_UART2 = 0;        
    }
}
void Tx_UART2(uint8_t frame_type, uint16_t register_code, uint16_t data_to_send) {
    if (UART2_is_tx_ready() == 1) {
        //  synchronize 0x00 0x00
        UART2_Write(0x00);  //  pos 0
        UART2_Write(0x00);  //  pos 1
        //  sender identifier
        UART2_Write(frame_type);    //  pos 2

        //  frame header: sender identifier
#define CONTRLOLLED_AREA 0b00000001  //  0x01 closead area env controllers
        UART2_Write(CONTRLOLLED_AREA);   //  pos 3

#ifdef  MONO_ECFC
        UART2_Write(REMOTE_DEV_MONO_ECFC);   //  pos 4
#endif
#ifdef  DUAL_ECFC
        UART2_Write(REMOTE_DEV_DUAL_ECFC);   //  pos 4
#endif
#ifdef  MONO_ACFC
        UART2_Write(REMOTE_DEV_MONO_ACFC);   //  pos 4
#endif
#ifdef  MULTIFAN
        UART2_Write(REMOTE_DEV_MULTIFAN);   //  pos 4
#endif

#define WORKING_TYPE 0b00001000  //  0x08 autonomous device with AEI support
        UART2_Write(WORKING_TYPE);   //  pos 5

        //  2 * 1B  serial number
        UART2_Write((DEV_config.serialNumber >> 0x08)); //  pos 6
        UART2_Write((DEV_config.serialNumber & 0xff));  //  pos 7

        //  frame header: destination address / identifier
#define SRC_ID1 0b00000001 //  0x01
#define SRC_ID2 0b00001000 //  0x08
#define SRC_ID3 0b00000010 //  0x02
        UART2_Write(SRC_ID1);   //  pos 8
        UART2_Write(SRC_ID2);   //  pos 9
        UART2_Write(SRC_ID3);   //  pos 10
        UART2_Write(0x00); //  0x00     //  pos 11
        UART2_Write(0x00); //  0x00     //  pos 12

        //  --------------------------------------------------------------------
        
        uint8_t frame_crc = 0;
        UART2_Write(0x05); //  0x05 frame lenght (5 bytes)
        UART2_Write(0x12); //  0x12 write command
        
        uint8_t dataH = (register_code >> 0x08);
        uint8_t dataL = register_code & 0xff;

        frame_crc += dataH;
        frame_crc += dataL;

        //  device control register address to send, 16b
        UART2_Write(dataH);
        UART2_Write(dataL);

        //  command to execute, 8b
        dataH = (data_to_send >> 0x08);
        dataL = data_to_send & 0xff;

        frame_crc += dataH;
        frame_crc += dataL;

        //  register data value, 16b
        UART2_Write(dataH);
        UART2_Write(dataL);

        //  register data value CRC
        UART2_Write(frame_crc);
    }
}

void frameIdAckArrayReset_UART2(void) {
    for (uint8_t i = 0; i<sizeof (TxAckArray_UART2); i++) 
        TxAckArray_UART2 [i] = 1;
}

int compare(const void * a, const void * b) {
    return ( *(int*) a - *(int*) b);
}

void transmissionIRQ () {
    //  data transmission activity
    //  check frame buffer empty flag
    if (UART2_frame_buffer_flag == 0) {
	    RxBufferArray_UART2 [GW_UART2_frame_byte_pos] = U2RXB;
	} else {
	    //  in case of false initial readings read all data to trash
		//	anyway after 5ms brake Rx buffer should be clear by EOT timer
	    uint8_t *false_read;
	    *(false_read) = U2RXB;
    }
	//	if it is a last byte in frame
    if (GW_UART2_frame_byte_pos == 19) {
	    //  reset frame byte counter
	    GW_UART2_frame_byte_pos = 0;

	    //  set frame buffer busy flag.
	    UART2_frame_buffer_flag = 1;
    } else GW_UART2_frame_byte_pos++;	//	increase frame byte counter
	
    #ifdef RX_UART2_SOFT_TIMEOUT
	//	reset EOT timer
    GW_TMR_UART2_Rx_EOT_tmr = millis();
    #endif
}
	