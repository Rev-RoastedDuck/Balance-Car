#include "systick.h"


/******************************************************************************/
/*-----------------------------------�ӿ�ʵ��---------------------------------*/
/******************************************************************************/

/**
 * @brief  ��ʼ��systick ���������ȼ�
 * @param  period_ms: ��ʱʱ��(0 ~ 233)
 * @param  priority: �ж����ȼ�(0 ���ȼ����)
 * @param  open_interrupt:�Ƿ����ж�
 * @return None
 * @note	 SysTick�ļ�����Χ��24λ
 */
static void init(uint32_t period_ms,boolean open_interrupt,uint8_t priority) {
	uint32_t ticks = SystemCoreClock / 1000 * period_ms;
	if (SysTick_Config(ticks)) {
		debug_uart_printf_2(0,"init faild \r\n");
		while (1) {
		}
	}
	SYSTICK_DRIVER.open_interrupt = open_interrupt;
	if(open_interrupt){
		NVIC_SetPriority(SysTick_IRQn, priority);
		NVIC_EnableIRQ(SysTick_IRQn);
	}
}

static void set_interrupt_function(void (*interrupt_function) (void)){
	if(NULL == interrupt_function){
		SYSTICK_DRIVER.open_interrupt = FALSE;
		debug_uart_printf_2(0,"set interrupt function faild \r\n");
		return;
	}
	SYSTICK_DRIVER.interrupt_function = interrupt_function;
}

/*
 * @note �ж��ڲ�Ҫʹ��putchar() getchar()
*/
void SysTick_Handler(void) {
	if(NULL != SYSTICK_DRIVER.interrupt_function){
		SYSTICK_DRIVER.interrupt_function();
	}
}

/******************************************************************************/
/*-----------------------------------�ⲿ�ӿ�---------------------------------*/
/******************************************************************************/
RRD_DRIVER_SYSTICK SYSTICK_DRIVER = {
	.open_interrupt = FALSE,
	.interrupt_function = NULL,
	
	.init = init,
	.set_interrupt_function = set_interrupt_function
};

/******************************************************************************/
/*-----------------------------------DEBUG�ӿ�--------------------------------*/
/******************************************************************************/
#if OPEN_SYSTICK_TEST

#include "delay.h"

uint32_t count = 0;
void callback(void){
		count ++;
		debug_uart_send("count : %d \r\n",count);
}

void systick_test(void){
	debug_uart_init();
	debug_uart_printf_1("init now!\r\n");
	
	
	SYSTICK_DRIVER.init(100,TRUE,0);
	SYSTICK_DRIVER.set_interrupt_function(callback);
	
	while(1)
	{		
//		delay_ms_soft(100);
//		printf("count: %d \r\n",count);
	}
}
#endif



