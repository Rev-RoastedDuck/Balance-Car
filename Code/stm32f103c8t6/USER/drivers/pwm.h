/*
 * pwm.h
 *	
 *  Note: ֻ���������ö�ʱ����ͨͨ����PWM
 *  Created on: 2024��7��29��
 *      Author: Rev_RoastDuck
 */

#ifndef DRIVERS_PWM_H_
#define DRIVERS_PWM_H_
/******************************************************************************/
/*---------------------------------ͷ�ļ�����---------------------------------*/
/******************************************************************************/
#include "stm32f10x.h"
#include "debug_uart.h"
#include <stdlib.h>

/******************************************************************************/
/*------------------------------------ö��------------------------------------*/
/******************************************************************************/
typedef enum{
	CH1 = 1,
	CH2,
	CH3,
	CH4
}TIM_CHANNEL;

/******************************************************************************/
/*------------------------------------�ṹ��----------------------------------*/
/******************************************************************************/
typedef struct _RRD_DRIVER_PWM {
	int8_t		(*init) 					(GPIO_TypeDef* gpio_x, const uint16_t gpio_pin, TIM_TypeDef* tim_x,const TIM_CHANNEL tim_channel,const uint16_t	tim_period,const uint16_t	tim_prescaler,const uint16_t	tim_clock_div);
	void 			(*set_compare) 		(TIM_TypeDef* tim_x,const TIM_CHANNEL tim_channel, const uint16_t compare);
	uint16_t 	(*get_compare)		(const TIM_TypeDef* tim_x,const TIM_CHANNEL tim_channel);
	uint16_t 	(*get_tim_period)	(const TIM_TypeDef* tim_x);
} RRD_DRIVER_PWM;

/******************************************************************************/
/*-----------------------------------DEBUG�ӿ�--------------------------------*/
/******************************************************************************/
#define OPEN_PWM_TEST		0

#if OPEN_PWM_TEST
	void pwm_test_start(void);
#endif


/******************************************************************************/
/*-----------------------------------�ⲿ�ӿ�---------------------------------*/
/******************************************************************************/
extern RRD_DRIVER_PWM PWM_DRIVER;
//RRD_DRIVER_PWM* pwm_driver_new(void);


#endif

