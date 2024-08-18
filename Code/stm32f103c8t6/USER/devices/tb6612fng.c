#include "tb6612fng.h"

/******************************************************************************/
/*-----------------------------------接口实现---------------------------------*/
/******************************************************************************/

static int8_t init(RRD_DEVICE_TB6612FNG* driver,
									 GPIO_TypeDef* in1_gpio_x, const uint16_t in1_gpio_pin,
									 GPIO_TypeDef* in2_gpio_x, const uint16_t in2_gpio_pin)
{
	driver->in1_gpio_x = in1_gpio_x;
	driver->in1_gpio_pin = in1_gpio_pin;
	
	driver->in2_gpio_x = in2_gpio_x;
	driver->in2_gpio_pin = in2_gpio_pin;
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = in1_gpio_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(in1_gpio_x, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = in2_gpio_pin;
	GPIO_Init(in2_gpio_x, &GPIO_InitStructure);
	
	GPIO_WriteBit(in1_gpio_x, in1_gpio_pin,Bit_RESET);
	GPIO_WriteBit(in2_gpio_x, in2_gpio_pin,Bit_RESET);
	
	return 0;
}


static void forward(RRD_DEVICE_TB6612FNG* driver){
	GPIO_WriteBit(driver->in2_gpio_x,driver->in2_gpio_pin,Bit_SET);
	GPIO_WriteBit(driver->in1_gpio_x,driver->in1_gpio_pin,Bit_RESET);
}

static void backward(RRD_DEVICE_TB6612FNG* driver){
	GPIO_WriteBit(driver->in1_gpio_x,driver->in1_gpio_pin,Bit_SET);
	GPIO_WriteBit(driver->in2_gpio_x,driver->in2_gpio_pin,Bit_RESET);
}

static void stop(RRD_DEVICE_TB6612FNG* driver){
	GPIO_WriteBit(driver->in2_gpio_x,driver->in2_gpio_pin,Bit_SET);
	GPIO_WriteBit(driver->in1_gpio_x,driver->in1_gpio_pin,Bit_SET);
}

/******************************************************************************/
/*----------------------------------接口初始化--------------------------------*/
/******************************************************************************/
RRD_DEVICE_TB6612FNG* tb6612fng_device_new(void){
	RRD_DEVICE_TB6612FNG* self;
	self = malloc(sizeof(RRD_DEVICE_TB6612FNG));
	if (self == NULL) {
		debug_uart_printf_2(0,"Memory allocation failed");
		return self;
	}
	
	self->init = init;
	self->forward = forward;
	self->backward = backward;
	self->stop = stop;
	
	return self;
}


