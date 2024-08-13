#include "systick.h"


/******************************************************************************/
/*-----------------------------------接口实现---------------------------------*/
/******************************************************************************/

/**
 * @brief  初始化systick 并设置优先级
 * @param  period_ms: 定时时长(0 ~ 233)
 * @param  priority: 中断优先级(0 优先级最高)
 * @param  open_interrupt:是否开启中断
 * @return None
 * @note	 SysTick的计数范围是24位
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
 * @note 中断内不要使用putchar() getchar()
*/
void SysTick_Handler(void) {
	if(NULL != SYSTICK_DRIVER.interrupt_function){
		SYSTICK_DRIVER.interrupt_function();
	}
}

/******************************************************************************/
/*-----------------------------------外部接口---------------------------------*/
/******************************************************************************/
RRD_DRIVER_SYSTICK SYSTICK_DRIVER = {
	.open_interrupt = FALSE,
	.interrupt_function = NULL,
	
	.init = init,
	.set_interrupt_function = set_interrupt_function
};

/******************************************************************************/
/*-----------------------------------DEBUG接口--------------------------------*/
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



