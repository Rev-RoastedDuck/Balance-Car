/*
 * logic.c
 *
 *  Created on: 2024年2月4日
 *      Author: intl4419
 */
#include "logic.h"

// 直立环  
// 		震荡值 160 9.2
//		稳定值 160 8.5
// 		取	值	96 5.1

// 速度环
// 		稳定值 110 0.58

/**
 * @brief   		计算pid位置环
 * @param[in]   pid  位置式pid
 * @param[in]   current_angle  当前角度
 * @param[in]   gyro_y  			 y轴速度
 * @return  		pid计算结果
 */
float logic_calc_pid_balance(PID_Loc_Info *pid,
														float current_angle,
														float gyro_y)
{
	pid->current_value = current_angle;
	
	pid->En_0 = pid->current_value - pid->desired_value;
															
	float t1 =  pid->Kp * pid->En_0;
	float t2 = pid->Td * gyro_y;	// Kd
	pid->calc_result = t1 + t2;	
	
	return pid->calc_result;
}

/**
 * @brief   		计算pid速度环
 * @param[in]   pid  位置式pid
 * @param[in]   encoder_value_left   左编码器数值
 * @param[in]   encoder_value_right  右编码器数值
 * @return  		pid计算结果
 */
float logic_calc_pid_speed(PID_Loc_Info *pid,
													int32_t encoder_value_left,
													int32_t encoder_value_right)
{
    float t1,t2;
		
		// pid->desired_value 期望速度
    pid->En_0 = (encoder_value_left + encoder_value_right) - 0;
    pid->En_0 = pid->En_0 * 0.5 + pid->En_1 * 0.5;								// 滤波
		pid->SEk += pid->En_0;
//		pid->SEk += pid->En_0 + pid->desired_value;
		
		if		 (pid->SEk > 10000 ) {pid->SEk = 10000;}								// 积分限幅
		else if(pid->SEk < -10000) {pid->SEk = -10000;}
    
		t1 = pid->Kp * pid->En_0;
    t2 = pid->Ti * pid->SEk;	// Ki
											
    pid->calc_result = t1 + t2;
    pid->En_1 = pid->En_0;
		return pid->calc_result;
}


///**
// * @brief   		计算pid转向环
// * @param[in]   pid  位置式pid
// * @param[in]   gyro_z  z轴速度
// * @return  		pid计算结果
// * @note				用于走直线
// */
//float logic_calc_pid_turn(PID_Loc_Info *pid,float gyro_z){
//	pid->calc_result = pid->Kp * gyro_z;
//	return pid->calc_result;
//}

/**
 * @brief   		计算pid转向环(yaw)
 * @param[in]   pid  位置式pid
 * @return  		pid计算结果
 */
float logic_calc_pid_turn(PID_Loc_Info *pid){
	pid->En_0 = pid->desired_value - pid->current_value;
	pid->calc_result =  pid->Kp * pid->En_0 + (pid->Td / pid->Tsam) * pid->Kp * (pid->En_0 - pid->En_1);
	pid->En_1 = pid->En_0;
	return pid->calc_result;
}
