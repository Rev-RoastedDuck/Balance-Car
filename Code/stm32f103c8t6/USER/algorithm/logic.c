/*
 * logic.c
 *
 *  Created on: 2024年2月4日
 *      Author: intl4419
 */
#include "logic.h"


/**
 * @brief   		计算pid位置环
 * @param[in]   pid  位置式pid
 * @param[in]   current_angle  当前角度
 * @param[in]   gyro_y  			 y轴速度
 * @return  		pid计算结果
 */
float logic_calc_pid_balance(PID_Loc_Info *pid,
														float current_angle,
														float gyro_y){
	pid->current_value = current_angle;
	
	pid->En_0 = pid->current_value - pid->desired_value;	
	pid->calc_result = pid->Kp * pid->En_0 + pid->Td / pid->Tsam * pid->Kp * gyro_y;
	
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
													int32_t encoder_value_right){
    float t1,t2;
		
		// pid->desired_value 期望速度
		// (encoder_value_left + encoder_value_right) / 2 - pid->desired_value
    pid->En_0 = (encoder_value_left + encoder_value_right) - 0;
    
		// pid->desired_value 期望增加的位移												
		pid->SEk += pid->En_0 + pid->desired_value;
		
		if		 (pid->SEk > 10000 ) {pid->SEk = 10000;}															// 积分限幅
		else if(pid->SEk < -10000) {pid->SEk = -10000;}
    
		t1 = pid->Kp * pid->En_0;
    t2 = ((pid->Tsam/pid->Ti)*pid->Kp)*pid->SEk*pid->Kp;
											
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
