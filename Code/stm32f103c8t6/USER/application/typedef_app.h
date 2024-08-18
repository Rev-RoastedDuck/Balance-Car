/*
 * typedef.h
 *
 *  Created on: 2024��7��31��
 *      Author: Rev_RoastDuck
 */

#ifndef APPLICATION_TYPEDEF_APP_H_
#define APPLICATION_TYPEDEF_APP_H_

/******************************************************************************/
/*---------------------------------ͷ�ļ�����---------------------------------*/
/******************************************************************************/
#include "stm32f10x.h"
#include "pid_rrd.h"
#include "filter_rrd.h"
#include "encoder.h"
#include "typedef.h"

/******************************************************************************/
/*----------------------------------���Ͷ���----------------------------------*/
/******************************************************************************/
typedef struct _Mpu6050DataInfo{
	
		struct _AccData{
			float x;
			float y;
			float z;
		}mpu6050_acc_data;


		struct _GyroData{
			float x;
			float y;
			float z;
		}mpu6050_gyro_data;


		struct _EulerAngles{
			float pitch;
			float roll;
			float yaw;
		}mpu6050_euler_angles;
		
}Mpu6050DataInfo;

typedef struct _EncoderDataInfo{
		int16_t encoder_left;
		int16_t encoder_right;
		First_Order_Filter_Info encoder_left_filter;
		First_Order_Filter_Info encoder_right_filter;
}EncoderDataInfo;

typedef struct _PidOpenLoopDataInfo{
		int16_t	motor_left_pwm_reload_value;	// ����Ԥװ��ֵ����ֵС��0 ��ת
		int16_t	motor_right_pwm_reload_value;	// �ҵ��Ԥװ��ֵ����ֵС��0 ��ת
}PidOpenLoopDataInfo;

typedef struct _PidDataInfo{
		PID_Loc_Info balance_pid;
		PID_Loc_Info speed_pid;
		PID_Loc_Info turn_pid;
}PidDataInfo;

typedef struct _PidControl{
		boolean open_loop;
}PidControl;

typedef struct _HandleDataInfo{
		uint8_t command;
}HandleInfo;

typedef struct _FunctionOption{
		uint8_t need_dmp_reset;
		uint8_t need_oled_show;
		boolean need_oled_clear;
}FunctionOption;

typedef struct _BalanceCarInfo{
		PidDataInfo					pid_data;
		PidOpenLoopDataInfo pid_open_loop_data;
		PidControl 					pid_control;
	
		HandleInfo					handle_data;
		EncoderDataInfo 		encoder_date;
		Mpu6050DataInfo 		mpu6050_data;
		FunctionOption			function_option;
		
}BalanceCarInfo;


#endif
