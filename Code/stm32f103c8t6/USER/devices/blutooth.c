#include "blutooth.h"

/******************************************************************************/
/*---------------------------------���ò���-----------------------------------*/
/******************************************************************************/
//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------
 
// <h>STDOUT USART Interface
 
//   <o>Connect to hardware via Driver_USART# <0-3>
//   <i>Select driver control block for USART interface
#define USART_DRV_NUM           3
 
//   <o>Baudrate
#define USART_BAUDRATE          115200
 
// </h>
// <<< end of configuration section >>>

#define _USART_Driver_(n)  Driver_USART##n
#define  USART_Driver_(n) _USART_Driver_(n)
 
extern ARM_DRIVER_USART  USART_Driver_(USART_DRV_NUM);
#define ptrUSART       (&USART_Driver_(USART_DRV_NUM))
 
/******************************************************************************/
/*----------------------------------��-����-----------------------------------*/
/******************************************************************************/
#define CHECK_STATUS(status) \
    do { \
        if ((status) != ARM_DRIVER_OK) return (-1); \
} while (0)

/******************************************************************************/
/*----------------------------------ȫ�ֱ���----------------------------------*/
/******************************************************************************/
#define FIFO_SIZE																64
static  uint8_t g_FIFO_GET_DATA[FIFO_SIZE];



/******************************************************************************/
/*---------------------------------��̬����-----------------------------------*/
/******************************************************************************/
/**
 * @brief  			����һλ����
 * @param				byte  ��Ҫ���͵��ֽ� 
 */
static inline void bluetooth_send_byte(uint8_t byte){
	USART_SendData(USART2,byte);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {
	}
}

/**
 * @brief  			�����ַ���
 * @param				byte  ��Ҫ���͵��ֽ� 
 */
static inline void bluetooth_send_string(uint8_t *str){
	while(*str != '\0'){
		bluetooth_send_byte(*str++);
	}
}


/******************************************************************************/
/*-----------------------------------�ӿ�ʵ��---------------------------------*/
/******************************************************************************/
void bluetooth_callback(uint32_t event){
	if(event & ARM_USART_EVENT_RX_TIMEOUT){
		
		// �ص�
		ptrUSART->Control(ARM_USART_ABORT_RECEIVE, 1);
		uint32_t length = ptrUSART->GetRxCount();
		ptrUSART->Send(g_FIFO_GET_DATA, length);
		ptrUSART->Receive(g_FIFO_GET_DATA, FIFO_SIZE);		// ��ջ�����
		
		// �¼�����
		/*
		// 1.�����ݰ�β
		if(length > 4
			&& g_FIFO_GET_DATA[length - 1] == 0xFB
			&& g_FIFO_GET_DATA[length - 2] == 0xFA){
				ptrUSART->Control(ARM_USART_ABORT_RECEIVE, 1);
				
				// 2.�����ݰ�ͷ
				uint8_t index = length;
				boolean find_pack_header = FALSE;
				while(--index >= 2){							
					if(g_FIFO_GET_DATA[index - 1] == 0xAB
						&& g_FIFO_GET_DATA[index - 2] == 0xAA){
						find_pack_header = TRUE;
						break;
					}
				}
				
				// 3.��������
				if(find_pack_header){
					ptrUSART->Send(g_FIFO_GET_DATA, length);
//					for(uint8_t i = index;i < length;++i){
//							parse_packet(g_FIFO_GET_DATA[i]);
//					}
				}
				ptrUSART->Receive(g_FIFO_GET_DATA, FIFO_SIZE);		// ��ջ�����
		}
*/
	}
}
		
		
/**
 * @brief  			Initialize stdout
 * @return          0 on success, or -1 on error.
 */
static int8_t bluetooth_init(void) {
  int32_t status;
 
  status = ptrUSART->Initialize(bluetooth_callback);	// ���ûص�����
  CHECK_STATUS(status);
 
  status = ptrUSART->PowerControl(ARM_POWER_FULL);
  CHECK_STATUS(status);
 
  status = ptrUSART->Control(ARM_USART_MODE_ASYNCHRONOUS |
                             ARM_USART_DATA_BITS_8       |
                             ARM_USART_PARITY_NONE       |
                             ARM_USART_STOP_BITS_1       |
                             ARM_USART_FLOW_CONTROL_NONE,
                             USART_BAUDRATE);
  CHECK_STATUS(status);

  status = ptrUSART->Control(ARM_USART_CONTROL_TX, 1);
  CHECK_STATUS(status);

	status = ptrUSART->Control(ARM_USART_CONTROL_RX, 1);
	CHECK_STATUS(status);
  
	ptrUSART->Receive(g_FIFO_GET_DATA, sizeof(g_FIFO_GET_DATA));
	return (0);
}


/**
 * @brief 		�������͸�ʽ���ַ����Ĵ�����
 * @param[in] fmt ��ʽ���ַ���
 * @return		None
 * @note			variable argument list string printf return num
 */
static void blutooth_send(const char *fmt,...) {
	// 1.���������ڴ�
	va_list args;
	va_start(args, fmt);
	int len = vsnprintf(NULL, 0, fmt, args);
	va_end(args);
	
	// 2.�����ڴ�
	char *buffer = (char *)malloc(len + 1);
	if (buffer == NULL) {
			debug_uart_printf_2(0,"malloc fail!!!\r\n");
			return;
	}
	
	// 3.��ʽ�����
	va_start(args, fmt);
	vsnprintf(buffer,len + 1 , fmt, args);
	va_end(args);
	
	// 4.����
	// ptrUSART->Send(buffer, len);
	bluetooth_send_string((uint8_t*)buffer);
	
	// 5.�ͷ�
	free(buffer);
}

/******************************************************************************/
/*-------------------------------------�ӿ�-----------------------------------*/
/******************************************************************************/
RRD_DEVICE_BLUETOOTH BLUETOOTH_DEVICE = {
	.init = bluetooth_init,
	.send = blutooth_send
};


