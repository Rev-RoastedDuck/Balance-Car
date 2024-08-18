#include "pwm.h"


/******************************************************************************/
/*-----------------------------------�ӿ�ʵ��---------------------------------*/
/******************************************************************************/
/**
 * @brief  			�ж�ʱ���Ƿ��ʼ��
 * @param[in]   tim_x  TIM1 TIM2 TIM3 TIM4 TIM5 TIM8
 * @return      1 �����ʱ���ѳ�ʼ����0 �����ʱ��δ��ʼ��
 */
static uint8_t is_tim_initialized(const TIM_TypeDef *tim_x) {
    return (tim_x->CR1 & TIM_CR1_CEN);
}

/**
 * @brief       ��ʼ����ʱ����GPIO������PWM���
 * @param[in]   gpio_x         GPIO�˿� (���� GPIOA, GPIOB, ...)
 * @param[in]   gpio_pin       GPIO���� (���� GPIO_Pin_6)
 * @param[in]   tim_x          ��ʱ��ʵ�� (���� TIM1, TIM2, ...)
 * @param[in]   tim_channel    ��ʱ��ͨ�� (CH1, CH2, CH3, CH4)
 * @param[in]   tim_period     ��ʱ������ֵ
 * @param[in]   tim_prescaler  ��ʱ��Ԥ��Ƶ��ֵ
 * @param[in]   tim_clock_div  ��ʱ��ʱ�ӷ�Ƶֵ
 * @return      0 on success, or -1 on error.
 */
static int8_t init(	     GPIO_TypeDef* 	gpio_x, 
									 const uint16_t			 	gpio_pin, 
												 TIM_TypeDef* 	tim_x,
									 const TIM_CHANNEL 		tim_channel,
									 const uint16_t				tim_period,
									 const uint16_t				tim_prescaler,
									 const uint16_t				tim_clock_div){
	if(is_tim_initialized(tim_x)){
		debug_uart_printf_2(0,"tim is initialized. address: %p \r\n",(void*)tim_x);
	}

	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInitStructure.TIM_Period = tim_period;
	TIM_TimeBaseInitStructure.TIM_Prescaler = tim_prescaler;	
	TIM_TimeBaseInitStructure.TIM_ClockDivision = tim_clock_div;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(tim_x,&TIM_TimeBaseInitStructure);
	
	TIM_ARRPreloadConfig(tim_x,ENABLE);
	TIM_Cmd(tim_x, ENABLE);				
	if(TIM1 == tim_x){
		TIM_CtrlPWMOutputs(tim_x, ENABLE); // �߼���ʱ��
	}
									 
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = gpio_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(gpio_x, &GPIO_InitStructure);

	TIM_OCInitTypeDef TIM_OCInitStructure;		
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
								 
	switch(tim_channel){
		case CH1:
			TIM_OC1Init(tim_x,&TIM_OCInitStructure);
			TIM_OC1PreloadConfig(tim_x,TIM_OCPreload_Enable);
			break;
		case CH2:
			TIM_OC2Init(tim_x,&TIM_OCInitStructure);
			TIM_OC2PreloadConfig(tim_x,TIM_OCPreload_Enable);
			break;
		case CH3:
			TIM_OC3Init(tim_x,&TIM_OCInitStructure);
			TIM_OC4PreloadConfig(tim_x,TIM_OCPreload_Enable);
			break;
		case CH4:
			TIM_OC4Init(tim_x,&TIM_OCInitStructure);
			TIM_OC4PreloadConfig(tim_x,TIM_OCPreload_Enable);
			break;			
	}
		
	return 0;				 
}
									 
/**
 * @brief       ���ö�ʱ���Ƚ�ֵ
 * @param[in]   tim_x          ��ʱ��ʵ�� (���� TIM1, TIM2, ...)
 * @param[in]   tim_channel    ��ʱ��ͨ�� (CH1, CH2, CH3, CH4)
 * @param[in]   compare        �Ƚ�ֵ
 */			 
static void set_compare(			TIM_TypeDef* tim_x,
												const TIM_CHANNEL tim_channel,
												const uint16_t compare){
	switch(tim_channel){
		case CH1:
			TIM_SetCompare1(tim_x,compare);
			break;
		case CH2:
			TIM_SetCompare2(tim_x,compare);
			break;
		case CH3:
			TIM_SetCompare3(tim_x,compare);
			break;
		case CH4:
			TIM_SetCompare4(tim_x,compare);
			break;
	}

}
												
/**
 * @brief       ��ȡ��ʱ���Ƚ�ֵ
 * @param[in]   tim_x          ��ʱ��ʵ�� (���� TIM1, TIM2, ...)
 * @param[in]   tim_channel    ��ʱ��ͨ�� (CH1, CH2, CH3, CH4)
 * @return      �Ƚ�ֵ
 */
static uint16_t get_compare(const TIM_TypeDef* tim_x,
														const TIM_CHANNEL tim_channel){
	switch(tim_channel){
		case CH1:
			return tim_x->CCR1;
		case CH2:
			return tim_x->CCR2;
		case CH3:
			return tim_x->CCR3;
		case CH4:
			return tim_x->CCR4;
	}
	
	return 0;
}

/**
 * @brief       ��ȡ��ʱ������ֵ
 * @param[in]   tim_x          ��ʱ��ʵ�� (���� TIM1, TIM2, ...)
 * @return      ��ʱ������ֵ
 */
static uint16_t get_tim_period(const TIM_TypeDef* tim_x){
	return tim_x->ARR;
}

/******************************************************************************/
/*----------------------------------�ӿڳ�ʼ��--------------------------------*/
/******************************************************************************/
//RRD_DRIVER_PWM* pwm_driver_new(void){
//	RRD_DRIVER_PWM* self;
//	self = malloc(sizeof(RRD_DRIVER_PWM));
//	if (self == NULL) {
//		debug_uart_printf_2(0,"Memory allocation failed");
//		return self;
//	}
//	
//	self->init = init;
//	self->set_compare = set_compare;
//	self->get_compare = get_compare;
//	self->get_tim_period = get_tim_period;
//	
//	return self;
//}


/******************************************************************************/
/*-----------------------------------�ⲿ�ӿ�---------------------------------*/
/******************************************************************************/
RRD_DRIVER_PWM PWM_DRIVER = {
	.init = init,
	.set_compare = set_compare,
	.get_compare = get_compare,
	.get_tim_period = get_tim_period
};

/******************************************************************************/
/*-----------------------------------���Թ���---------------------------------*/
/******************************************************************************/
#if OPEN_PWM_TEST

#include "tool.h"
typedef struct {
    GPIO_TypeDef* gpio_port;
    uint16_t 			gpio_pin;
    TIM_TypeDef* 	timx;
    TIM_CHANNEL 	channel;
} PWM_Channel_Config;

void pwm_test_start(void){
	debug_uart_init();		// �ȳ�ʼ��CMSIS��
	debug_uart_printf_1("init now. \r\n");
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4|RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	
	// TIM3 ��ӳ��
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);		// ����JTAG

	// TIM2 ��ӳ�� 1
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);
	
	PWM_Channel_Config pwm_channels[] = {
		{GPIOA, GPIO_Pin_8,  TIM1, CH1},
		{GPIOA, GPIO_Pin_9,  TIM1, CH2},
		{GPIOA, GPIO_Pin_10, TIM1, CH3},
		{GPIOA, GPIO_Pin_11, TIM1, CH4},
		
		{GPIOA, GPIO_Pin_0,  TIM2, CH1},
		{GPIOA, GPIO_Pin_1,  TIM2, CH2},
		{GPIOA, GPIO_Pin_2,  TIM2, CH3},
		{GPIOA, GPIO_Pin_3,  TIM2, CH4},

		// TIM2 ��ӳ�� 1
//		{GPIOA, GPIO_Pin_15,  TIM2, CH1},
//		{GPIOB, GPIO_Pin_3,  TIM2, CH2},
//		{GPIOA, GPIO_Pin_2,  TIM2, CH3},
//		{GPIOA, GPIO_Pin_3,  TIM2, CH4},
		
		{GPIOA, GPIO_Pin_6,  TIM3, CH1},
		{GPIOA, GPIO_Pin_7,  TIM3, CH2},
		{GPIOB, GPIO_Pin_0,  TIM3, CH3},
		{GPIOB, GPIO_Pin_1,  TIM3, CH4},
		
		// TIM3 ��ӳ��
//		{GPIOB, GPIO_Pin_4,  TIM3, CH1},
//		{GPIOB, GPIO_Pin_5,  TIM3, CH2},
//		{GPIOB, GPIO_Pin_0,  TIM3, CH3},
//		{GPIOB, GPIO_Pin_1,  TIM3, CH4},
		
		{GPIOB, GPIO_Pin_6,  TIM4, CH1},
		{GPIOB, GPIO_Pin_7,  TIM4, CH2},
		{GPIOB, GPIO_Pin_8,  TIM4, CH3},
		{GPIOB, GPIO_Pin_9,  TIM4, CH4},
	};
	
	uint16_t PWM_CHANNEL_COUNT = ARRAY_LENGTH(pwm_channels);
	
	for (int i = 0; i < PWM_CHANNEL_COUNT; i++) {
			PWM_DRIVER.init(pwm_channels[i].gpio_port, pwm_channels[i].gpio_pin, pwm_channels[i].timx, pwm_channels[i].channel, 20000 - 1, 72 - 1, TIM_CKD_DIV1);
	}
	
	debug_uart_printf_1("init finished. \r\n");

	while(1){
		for (uint16_t value = 0; value < 20000 - 1; ++value) {
				for (int i = 0; i < PWM_CHANNEL_COUNT; i++) {
						PWM_DRIVER.set_compare(pwm_channels[i].timx, pwm_channels[i].channel, value);
				}
		}
	
	}
}

#endif

