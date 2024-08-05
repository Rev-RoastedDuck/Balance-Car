#include "stm32f10x.h"
#include "delay.h"
#include "pwm.h"
#include "mpu6050.h"
#include "encoder.h"
#include "mpu6050_dmp.h"
#include "oled.h"
#include "mpu6050.h"

/*
 * PWMA GPIOA8 
 * GPIOA12 AIN1 GPIOA15 AIN2
 * PWMB GPIOA9
 * GPIOAB3 BIN1 GPIOB4 BIN2
 * 
 * AIN1 AIN2 BIN1 BIN2 PWMA PWMB AO1/AO2
 *	1		 	0		1			0		1			1			正转
 *	0			1		0			1		1			1			反转
 *	1			1		1			1		1			1			刹车
 *	0			0		0			0		1			1			自由停车
 *	x			x		x			x		0			0			刹车
*/

int main(void){
	mpu6050_test();
}
