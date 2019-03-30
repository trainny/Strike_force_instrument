
  /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NANOPB_USERCODE_H
#define __NANOPB_USERCODE_H

#ifdef __cplusplus
extern "C" {
#endif

	/* Includes ------------------------------------------------------------------*/
#include "pb.h"
#include "monitor.pb.h"
#include "pb_common.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "sys.h"
#include "usart.h"
#include "adc.h"

extern DeviceSend msg_got;
extern NodeSend  msg_sent;


/* Exported functions prototypes ---------------------------------------------*/
u8 Cmd_Cmd_Call(void);
bool PB_ENCODE_Rsp_CALL(u8 errorcode, u8 node_channle);
bool PB_ENCODE_Val_CALL(float Value, u8 node_channle);
bool PB_DECODE_CMD_CALL(void);

#ifdef __cplusplus
}
#endif

#endif
