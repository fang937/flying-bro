#ifndef __CAN_RECEIVE_SEND_H
#define __CAN_RECEIVE_SEND_H

#include "main.h"

#define CAN_RX_ID  0x301U
#define CAN_TX_ID  0x305U

#define CMD_FIRE    0x01U
#define CMD_RELOAD  0x02U
#define CMD_STOP    0x03U
#define CMD_QUERY   0x04U

#define STATE_IDLE      0x00U
#define STATE_RUNNING   0x01U
#define STATE_FINISHED  0x02U
#define STATE_ERROR     0xE0U

#define ERR_NONE        0x00U
#define ERR_BUSY        0x01U
#define ERR_BAD_CMD     0x02U
#define ERR_SEQUENCE    0x03U

void CAN_App_Init(void);
void CAN_App_Process(void);

#endif
