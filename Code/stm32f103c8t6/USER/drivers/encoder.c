#include "encoder.h"

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
 * @brief   		��ʼ��������
 * @param[in]   driver     ָ������������ṹ���ָ��
 * @param[in]   ic1_gpio_x  ָ���һ���������������ŵ� GPIO �˿�
 * @param[in]   ic1_gpio_pin  ��һ���������������ŵ� GPIO ����
 * @param[in]   ic2_gpio_x  ָ��ڶ����������������ŵ� GPIO �˿�
 * @param[in]   ic2_gpio_pin  �ڶ����������������ŵ� GPIO ����
 * @param[in]   tim_x       ָ��ʹ�õĶ�ʱ����ָ�룬�� TIM��TIM2��TIM3��TIM4
 * @return  		��
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
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; 									// Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF; 									// �趨�������Զ���װֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 		// ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // ���ؼ���ģʽ 
	TIM_TimeBaseInit(tim_x, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(tim_x, 
														TIM_EncoderMode_TI12, 
														TIM_ICPolarity_Rising, 
														TIM_ICPolarity_Rising); 					// ʹ�ñ�����ģʽ3
	
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICStructInit(&TIM_ICInitStructure); 										// TIM_ICInitStruct��ʼ��
	TIM_ICInitStructure.TIM_ICFilter = 8; 											// �����˲�������
	TIM_ICInit(tim_x, &TIM_ICInitStructure); 										// ���� TIM_ICInitStruct �Ĳ�����ʼ������ TIMx

	TIM_SetCounter(tim_x, 0);
	TIM_Cmd(tim_x, ENABLE);
}

/**
 * @brief   		��ȡ�������ļ���ֵ
 * @param[in]   driver  ָ������������ṹ���ָ��
 * @return  		�������ļ���ֵ
 */
static int32_t encoder_get_count(RRD_DRIVER_ENCODER* driver){
    driver->count = (short)TIM_GetCounter(driver->tim_x);
    TIM_SetCounter(driver->tim_x, 0);
    return driver->count;
}

/**
 * @brief   		����������ļ���ֵ
 * @param[in]   driver  ָ������������ṹ���ָ��
 * @return  		��
 */
static void encoder_clear_count(RRD_DRIVER_ENCODER* driver){
    TIM_SetCounter(driver->tim_x, 0);
}

/**
 * @brief   		ɾ������������
 * @param[in]   self  ָ������������ṹ���ָ��
 * @return  		��
 */
static void encoder_driver_del(RRD_DRIVER_ENCODER* self){
	free(self);
}

/******************************************************************************/
/*----------------------------------�ӿڳ�ʼ��--------------------------------*/
/******************************************************************************/
/**
 * @brief   		�����µı���������
 * @param[in]   ��
 * @return  		ָ���´����ı����������ṹ���ָ�룬����ڴ����ʧ�ܣ��򷵻� NULL
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
/*-----------------------------------���Թ���---------------------------------*/
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
		printf("������: %d\r\n",ret);
		delay_ms_soft(100);
	}
}

#endif
