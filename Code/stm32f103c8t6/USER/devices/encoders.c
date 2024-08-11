#include "encoders.h"


/******************************************************************************/
/*-----------------------------------接口实现---------------------------------*/
/******************************************************************************/
static void encoders_init(void){
	ENCODERS_DEVICE.encoder_driver_left = encoder_driver_new();
	ENCODERS_DEVICE.encoder_driver_right = encoder_driver_new();

	ENCODERS_DEVICE.encoder_driver_left->encoder_config(ENCODERS_DEVICE.encoder_driver_left,GPIOA,GPIO_Pin_0,GPIOA,GPIO_Pin_1,TIM2);
	ENCODERS_DEVICE.encoder_driver_right->encoder_config(ENCODERS_DEVICE.encoder_driver_right,GPIOA,GPIO_Pin_6,GPIOA,GPIO_Pin_7,TIM3);
}

static void encoders_get_counts(int16_t *encoder_left,int16_t *encoder_right){
	*encoder_left = (int16_t)ENCODERS_DEVICE.encoder_driver_left->encoder_get_count(ENCODERS_DEVICE.encoder_driver_left);
	*encoder_right = (-1) * (int16_t)ENCODERS_DEVICE.encoder_driver_right->encoder_get_count(ENCODERS_DEVICE.encoder_driver_right);
}

/******************************************************************************/
/*-----------------------------------外部接口---------------------------------*/
/******************************************************************************/
RRD_DEVICE_ENCODERS ENCODERS_DEVICE = {
	.init = encoders_init,
	.get_counts = encoders_get_counts
};
