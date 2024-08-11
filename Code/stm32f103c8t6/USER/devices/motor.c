#include "motor.h"


/******************************************************************************/
/*-----------------------------------接口实现---------------------------------*/
/******************************************************************************/

static int8_t init(RRD_DEVICE_MOTOR* motor_driver,
						GPIO_TypeDef* gpio_x,
						const uint16_t gpio_pin,
						TIM_TypeDef* tim_x,
						const TIM_CHANNEL tim_channel,
						const uint16_t	tim_period,
						const uint16_t	tim_prescaler,
						const uint16_t	tim_clock_div){
							
		motor_driver->tim_x = tim_x;
		motor_driver->tim_channel = tim_channel;
		
		int8_t ret;
		ret = PWM_DRIVER.init(gpio_x,gpio_pin,tim_x,tim_channel,tim_period,tim_prescaler,tim_clock_div);
		if(ret){
			debug_uart_printf_2(0,"init false.");
			return -1;
		}
		return 0;
}

void set_compare_range(RRD_DEVICE_MOTOR* motor_driver,const uint16_t min,const uint16_t max){
	if(min > max){
		debug_uart_printf_2(0,"Set compare range failed");
		return;
	}
	motor_driver->compare_range[0] = min;
	motor_driver->compare_range[1] = max;
}


static inline void set_compare(RRD_DEVICE_MOTOR* motor_driver,const uint16_t compare){
	if(compare < motor_driver->compare_range[0]
		|| compare > motor_driver->compare_range[1]){
			debug_uart_printf_2(0,"Set compare range failed");
		return;
	}
	PWM_DRIVER.set_compare(motor_driver->tim_x,motor_driver->tim_channel,compare);
}

static inline uint16_t get_compare(RRD_DEVICE_MOTOR* motor_driver){
		return PWM_DRIVER.get_compare(motor_driver->tim_x,motor_driver->tim_channel);
}

static inline uint16_t get_tim_period(RRD_DEVICE_MOTOR* motor_driver){
		return PWM_DRIVER.get_tim_period(motor_driver->tim_x);
}

/******************************************************************************/
/*----------------------------------接口初始化--------------------------------*/
/******************************************************************************/
RRD_DEVICE_MOTOR* motor_driver_new(void){
		RRD_DEVICE_MOTOR* self;
		self = malloc(sizeof(RRD_DEVICE_MOTOR));
		if(self == NULL){
			debug_uart_printf_2(0,"Memory allocation failed");
			return self;
		}
		
		self->init = init;
		self->set_compare = set_compare;
		self->get_compare = get_compare;
		self->set_compare_range = set_compare_range;
		self->get_tim_period = get_tim_period;
		return self;	
}
