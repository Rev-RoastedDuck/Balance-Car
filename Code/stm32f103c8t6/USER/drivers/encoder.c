#include "encoder.h"

/******************************************************************************/
/*-----------------------------------接口实现---------------------------------*/
/******************************************************************************/

/**
 * @brief  			判断时钟是否初始化
 * @param[in]   tim_x  TIM1 TIM2 TIM3 TIM4 TIM5 TIM8
 * @return      1 如果定时器已初始化，0 如果定时器未初始化
 */
static uint8_t is_tim_initialized(const TIM_TypeDef *tim_x) {
    return (tim_x->CR1 & TIM_CR1_CEN);
}

/**
 * @brief   		初始化编码器
 * @param[in]   driver     指向编码器驱动结构体的指针
 * @param[in]   ic1_gpio_x  指向第一个编码器输入引脚的 GPIO 端口
 * @param[in]   ic1_gpio_pin  第一个编码器输入引脚的 GPIO 引脚
 * @param[in]   ic2_gpio_x  指向第二个编码器输入引脚的 GPIO 端口
 * @param[in]   ic2_gpio_pin  第二个编码器输入引脚的 GPIO 引脚
 * @param[in]   tim_x       指向使用的定时器的指针，如 TIM、TIM2、TIM3、TIM4
 * @return  		无
 */
static void encoder_init(			RRD_DRIVER_ENCODER* driver,
															GPIO_TypeDef* 			ic1_gpio_x, 
												const uint16_t			 			ic1_gpio_pin, 
															GPIO_TypeDef* 			ic2_gpio_x, 
												const uint16_t			 			ic2_gpio_pin, 
															TIM_TypeDef* 				tim_x){
	if(is_tim_initialized(tim_x)){
		debug_uart_printf_2(0,"tim is initialized. address: %p \r\n",(void*)tim_x);
	}
	driver->tim_x = tim_x;
																							
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = ic1_gpio_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(ic1_gpio_x, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = ic2_gpio_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(ic2_gpio_x, &GPIO_InitStructure);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; 									// 预分频器 
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF; 									// 设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 		// 选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 边沿计数模式 
	TIM_TimeBaseInit(tim_x, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(tim_x, 
														TIM_EncoderMode_TI12, 
														TIM_ICPolarity_Rising, 
														TIM_ICPolarity_Rising); 					// 使用编码器模式3
	
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICStructInit(&TIM_ICInitStructure); 										// TIM_ICInitStruct初始化
	TIM_ICInitStructure.TIM_ICFilter = 8; 											// 设置滤波器长度
	TIM_ICInit(tim_x, &TIM_ICInitStructure); 										// 根据 TIM_ICInitStruct 的参数初始化外设 TIMx

	TIM_SetCounter(tim_x, 0);
	TIM_Cmd(tim_x, ENABLE);
}

/**
 * @brief   		获取编码器的计数值
 * @param[in]   driver  指向编码器驱动结构体的指针
 * @return  		编码器的计数值
 */
static int32_t encoder_get_count(RRD_DRIVER_ENCODER* driver){
    driver->count = (short)TIM_GetCounter(driver->tim_x);
    TIM_SetCounter(driver->tim_x, 0);
    return driver->count;
}

/**
 * @brief   		清除编码器的计数值
 * @param[in]   driver  指向编码器驱动结构体的指针
 * @return  		无
 */
static void encoder_clear_count(RRD_DRIVER_ENCODER* driver){
    TIM_SetCounter(driver->tim_x, 0);
}

/**
 * @brief   		删除编码器驱动
 * @param[in]   self  指向编码器驱动结构体的指针
 * @return  		无
 */
static void encoder_driver_del(RRD_DRIVER_ENCODER* self){
	free(self);
}

/******************************************************************************/
/*----------------------------------接口初始化--------------------------------*/
/******************************************************************************/
/**
 * @brief   		创建新的编码器驱动
 * @param[in]   无
 * @return  		指向新创建的编码器驱动结构体的指针，如果内存分配失败，则返回 NULL
 */
RRD_DRIVER_ENCODER* encoder_driver_new(void){
	RRD_DRIVER_ENCODER* self;
	self = malloc(sizeof(RRD_DRIVER_ENCODER));
	if (self == NULL) {
		debug_uart_printf_2(0,"Memory allocation failed");
		return self;
	}
	
	self->encoder_driver_del = encoder_driver_del;
	
	self->encoder_clear_count = encoder_clear_count;
  self->encoder_get_count = encoder_get_count;
  self->encoder_config = encoder_init;
	
	return self;
}


/******************************************************************************/
/*-----------------------------------测试工具---------------------------------*/
/******************************************************************************/
#if OPEN_ENCODER_TEST
#include "delay.h"

void encoder_test_start(void){
	debug_uart_init();
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4|RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	
	RRD_DRIVER_ENCODER* encoder_driver = encoder_driver_new();
//	encoder_driver->encoder_config(encoder_driver,GPIOA,GPIO_Pin_0,GPIOB,GPIO_Pin_1,TIM2);
//	encoder_driver->encoder_config(encoder_driver,GPIOA,GPIO_Pin_6,GPIOA,GPIO_Pin_7,TIM3);
//	encoder_driver->encoder_config(encoder_driver,GPIOB,GPIO_Pin_6,GPIOB,GPIO_Pin_7,TIM4);
	encoder_driver->encoder_config(encoder_driver,GPIOA,GPIO_Pin_0,GPIOB,GPIO_Pin_1,TIM5);
	
	short ret = 0;
	while(1){
		ret = encoder_driver->encoder_get_count(encoder_driver);
		printf("编码器: %d\r\n",ret);
		delay_ms_soft(100);
	}
}

#endif
