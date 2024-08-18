#include "debug_uart.h"
 
/******************************************************************************/
/*---------------------------------���ò���-----------------------------------*/
/******************************************************************************/
//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------
 
// <h>STDOUT USART Interface
 
//   <o>Connect to hardware via Driver_USART# <0-3>
//   <i>Select driver control block for USART interface
#define USART_DRV_NUM           2
 
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
/*---------------------------------��̬����-----------------------------------*/
/******************************************************************************/
/**
 * @brief  			����һλ����
 * @param				byte  ��Ҫ���͵��ֽ� 
 */
static void debug_uart_send_byte(uint8_t byte){
	USART_SendData(USART2,byte);
	// �ȴ��������
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {
	}
}

/**
 * @brief  			�����ַ���
 * @param				byte  ��Ҫ���͵��ֽ� 
 */
static void debug_uart_send_string(uint8_t *str){
	while(*str != '\0'){
		debug_uart_send_byte(*str++);
	}
}
/******************************************************************************/
/*---------------------------------�ع�����-----------------------------------*/
/******************************************************************************/
/**
 * @brief  			Put a character to the stdout
 * @param[in]   ch  Character to output
 * @return          The character written, or -1 on write error.
 */
int stdout_putchar(int ch) {
  uint8_t buf[1];
  buf[0] = ch;
  if (ptrUSART->Send(buf, 1) != ARM_DRIVER_OK) {
    return (-1);
  }
  while (ptrUSART->GetTxCount() != 1);
  return (ch);
}


/******************************************************************************/
/*---------------------------------ȫ�ֺ���-----------------------------------*/
/******************************************************************************/
/**
 * @brief  			Initialize stdout
 * @return      0 on success, or -1 on error.
 */
int8_t debug_uart_init(void) {
  int32_t status;
 
  status = ptrUSART->Initialize(NULL);
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

  return (0);
}

/**
 * @brief  			Initialize stdout
 * @param				uint8_t 
 * @return[in]  
 */
void debug_uart_print_handle(const uint8_t pass ,const char *fmt, const char *file, int line, ...) {
	if(pass){return;}
	
	va_list args;
	va_start(args, line);

	printf("[%s:%d] ", file, line);
	vprintf(fmt, args);

	va_end(args);
}

/**
 * @brief  			���͸�ʽ���ַ���
 * @param[in] 	fmt ��ʽ���ַ���
 * @return			None
 * @note				variable argument list string printf return num
 * @note				�������ж���ʹ��
 * @note				�������ж���ʹ��
 * @note				�������ж���ʹ��
 */
void debug_uart_send(const char *fmt, ...) {
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
	debug_uart_send_string((uint8_t*)buffer);
	// 5.�ͷ�
	free(buffer);
}

/**
 * @brief  			���͸�ʽ���ַ���
 * @param[in] 	pass �����Ƿ��ӡ�ı�־
 * @param[in] 	fmt ��ʽ���ַ���
 * @return			None
 * @note				variable argument list string printf return num
 * @note				�������ж���ʹ��
 * @note				�������ж���ʹ��
 * @note				�������ж���ʹ��
 */
void debug_uart_send_2(const uint8_t pass,const char *fmt, ...) {
	if(pass){return;}
	
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
	debug_uart_send_string((uint8_t*)buffer);
	// 5.�ͷ�
	free(buffer);
}

/**
 * @brief  			����buff
 * @param				vector  ��������
 * @param				len 		���ݳ���
 */
void debug_uart_send_buff(uint8_t *vector,uint16_t len){
	for(uint16_t index = 0;index < len;index++){
		debug_uart_send_byte(vector[index]);
	}
}
