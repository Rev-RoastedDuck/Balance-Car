/*
 * app.h
 *	
 *  Created on: 2024年8月9日
 *      Author: Rev_RoastDuck
 */

#ifndef APPLICATION_APP_H_
#define APPLICATION_APP_H_

/******************************************************************************/
/*---------------------------------头文件引入---------------------------------*/
/******************************************************************************/
#include "oled.h"
#include "typedef_app.h"
#include "tool.h"
#include "encoders.h"
#include "filter_rrd.h"
#include "motor.h"
#include "tb6612fng.h"

#include "mpu6050.h"
#include "mpu6050_dmp.h"
#include "blutooth.h"
#include "globals.h"
#include "systick.h"
#include "flash.h"


void oled_show_data(BalanceCarInfo *info);
void app_params_init(void);
void app_device_init(void);

void app_start_task(void);
void app_update_params(void);
void flash_save_data(BalanceCarInfo *info);
void flash_read_data(BalanceCarInfo *info);

#endif




