#include "debug_uart.h"
 
/******************************************************************************/
/*---------------------------------配置参数-----------------------------------*/
/******************************************************************************/
//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------
 
// <h>STDOUT USART Interface
 
//   <o>Connect to hardware via Driver_USART# <0-255>
//   <i>Select driver control block for USART interface
#define USART_DRV_NUM           1
 
//   <o>Baudrate
#define USART_BAUDRATE          115200
 
// </h>
// <<< end of configuration section >>>


#define _USART_Driver_(n)  Driver_USART##n
#define  USART_Driver_(n) _USART_Driver_(n)
 
extern ARM_DRIVER_USART  USART_Driver_(USART_DRV_NUM);
#define ptrUSART       (&USART_Driver_(USART_DRV_NUM))
 
/******************************************************************************/
/*----------------------------------宏-函数-----------------------------------*/
/******************************************************************************/
#define CHECK_STATUS(status) \
    do { \
        if ((status) != ARM_DRIVER_OK) return (-1); \
    } while (0)


		
/******************************************************************************/
/*---------------------------------重构函数-----------------------------------*/
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
/*---------------------------------全局函数-----------------------------------*/
/******************************************************************************/
/**
 * @brief  			Initialize stdout
 * @return          0 on success, or -1 on error.
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
 * @return[in]  uint8_t        0 on success, or -1 on error.
 */
void debug_uart_print_handle(const uint8_t pass ,const char *fmt, const char *file, int line, ...) {
	if(pass){return;}
	
	va_list args;
	va_start(args, line);

	printf("[%s:%d] ", file, line);
	vprintf(fmt, args);

	va_end(args);
}
