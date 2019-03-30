/* Includes ------------------------------------------------------------------*/
#include "nanopb_usercode.h"

/* Exported types ------------------------------------------------------------*/
DeviceSend msg_got = DeviceSend_init_zero;
NodeSend  msg_sent = NodeSend_init_zero;

/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/

/*Encode message of node command respond*/
bool PB_ENCODE_Rsp_CALL(u8 errorcode,u8 node_channle)
{
	uint8_t buffer[NodeSend_size];
	//size_t message_length;
	bool status;

	/*NcrtlRsp*/
	msg_sent.payload.control.node.nodeId  = Node_ID;
	msg_sent.payload.control.node.address = Node_Address;
	msg_sent.payload.control.node.channel = node_channle;
	msg_sent.payload.control.errorcode    = errorcode;
	msg_sent.which_payload = NodeSend_control_tag;

	/* Encode our message */
	/* Allocate space on the stack to store the message data.
	 *
	 * Nanopb generates simple struct definitions for all the messages.
	 * - check out the contents of simple.pb.h!
	 * It is a good idea to always initialize your structures
	 * so that you do not have garbage data from RAM in there.
	 */
	/* Create a stream that will write to our buffer. */
	pb_ostream_t stream;

	memset(buffer, 0, NodeSend_size);

	stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

	status = pb_encode(&stream, NodeSend_fields, &msg_sent);

	//message_length = stream.bytes_written;

	if (!status)
	{
		printf("Encoding failed: %s\r\n", PB_GET_ERROR(&stream));
		return 1;
	}

	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, sizeof(buffer), 1000);	//发送数据
	while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) != SET);		//等待发送结束

	return 0;
}

/*Encode message of node value*/
bool PB_ENCODE_Val_CALL(float Value, u8 node_channle)
{
	uint8_t buffer[NodeSend_size];
	//size_t message_length;
	bool status;

	/*SensorValue*/
	msg_sent.payload.sensorValue.node.nodeId  = Node_ID;
	msg_sent.payload.sensorValue.node.address = Node_Address;
	msg_sent.payload.sensorValue.node.channel = node_channle;
	msg_sent.payload.sensorValue.value1       = Value;
	msg_sent.which_payload = NodeSend_sensorValue_tag;

	/* Encode our message */
	/* Allocate space on the stack to store the message data.
	 *
	 * Nanopb generates simple struct definitions for all the messages.
	 * - check out the contents of simple.pb.h!
	 * It is a good idea to always initialize your structures
	 * so that you do not have garbage data from RAM in there.
	 */
	 /* Create a stream that will write to our buffer. */
	pb_ostream_t stream;

	memset(buffer, 0, NodeSend_size);

	stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

	status = pb_encode(&stream, NodeSend_fields, &msg_sent);

	//message_length = stream.bytes_written;

	if (!status)
	{
		printf("Encoding failed: %s\r\n", PB_GET_ERROR(&stream));
		return 1;
	}

	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, sizeof(buffer), 1000);	//发送接收到的数据
	while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) != SET);		//等待发送结束

	return 0;
}
/*Decode message from server*/
bool PB_DECODE_CMD_CALL(void)
{
	//uint8_t buffer[DeviceSend_size];
	bool status;
	//uint8_t i;
	/* Allocate space for the decoded message. */
	//DeviceSend msg_got = DeviceSend_init_zero;

	/* Create a stream that reads from the buffer. */
	pb_istream_t stream;

	//memset(buffer, 0, DeviceSend_size);
	//for(i=0;i<sizeof(USART_RX_BUF);i++)	buffer[i] = USART_RX_BUF[i];

	stream = pb_istream_from_buffer(USART_RX_BUF, sizeof(USART_RX_BUF));
	/* Now we are ready to decode the message. */
	status = pb_decode(&stream, DeviceSend_fields, &msg_got);

	/* Check for errors... */
	if (!status)
	{
		printf("Decoding failed: %s\r\n", PB_GET_ERROR(&stream));
		return 1;
	}

	/* Print the data contained in the message. */
	printf("decode_value\r\n");
	printf("sid:     %d\r\n", msg_got.sid);
	printf("nodeId:  %d\r\n", msg_got.payload.control.node.nodeId);
	printf("address: %d\r\n", msg_got.payload.control.node.address);
	printf("channel: %d\r\n", msg_got.payload.control.node.channel);
	printf("cmd:     %d\r\n", msg_got.payload.control.cmd);
	/***	Test Code    ***/

	return 0;
}

/*Server command action*/
u8 Cmd_Cmd_Call(void)
{
	if (cmd_start_flag == 1)
	{
		if (HAL_ADC_Start_DMA(&hadc1, (u32*)&adc_value_buf, Length) != HAL_OK)
		{
			cmd_start_flag = 0;
			return 0x01;	//ADC_Start error
		}
		cmd_start_flag = 0;
		return 0;
	}

	if (cmd_stop_flag == 1)
	{
		if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK)
		{
			cmd_stop_flag = 0;
			return 0x02;	//ADC_Stop error
		}
		cmd_stop_flag = 0;
		return 0;
	}

	else
		return 0xff;	//Cmd error
}

