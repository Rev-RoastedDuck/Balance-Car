/*
 * globals.h
 *
 *  Created on: 2024��8��11��
 *      Author: intl4419
 */

#ifndef APPLICATION_GLOBALS_H_
#define APPLICATION_GLOBALS_H_

/******************************************************************************/
/*---------------------------------ͷ�ļ�����---------------------------------*/
/******************************************************************************/
#include "typedef_app.h"
#include "motor.h"
#include "tb6612fng.h"

extern Transmit_Data	g_TRANSMIT_DATA;
extern BalanceCarInfo g_BALANCE_CAR_INFO;

extern RRD_DEVICE_MOTOR* g_MOTOR_DEVICE_LEFT;
extern RRD_DEVICE_MOTOR* g_MOTOR_DEVICE_RIGHT;

extern RRD_DEVICE_TB6612FNG* g_TB6612FNG_DEVICE_LEFT;
extern RRD_DEVICE_TB6612FNG* g_TB6612FNG_DEVICE_RIGHT;

extern RRD_DRIVER_ENCODER* g_ENCODER_DEVICE_LEFT;
extern RRD_DRIVER_ENCODER* g_ENCODER_DEVICE_RIGHT;
#endif 


