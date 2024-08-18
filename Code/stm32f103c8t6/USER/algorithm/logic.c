/*
 * logic.c
 *
 *  Created on: 2024��2��4��
 *      Author: intl4419
 */
#include "logic.h"

// ֱ����  
// 		��ֵ 160 9.2
//		�ȶ�ֵ 160 8.5
// 		ȡ	ֵ	96 5.1

// �ٶȻ�
// 		�ȶ�ֵ 110 0.58

/**
 * @brief   		����pidλ�û�
 * @param[in]   pid  λ��ʽpid
 * @param[in]   current_angle  ��ǰ�Ƕ�
 * @param[in]   gyro_y  			 y���ٶ�
 * @return  		pid������
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
 * @brief   		����pid�ٶȻ�
 * @param[in]   pid  λ��ʽpid
 * @param[in]   encoder_value_left   ���������ֵ
 * @param[in]   encoder_value_right  �ұ�������ֵ
 * @return  		pid������
 */
float logic_calc_pid_speed(PID_Loc_Info *pid,
													int32_t encoder_value_left,
													int32_t encoder_value_right)
{
    float t1,t2;
		
		// pid->desired_value �����ٶ�
    pid->En_0 = (encoder_value_left + encoder_value_right) - 0;
    pid->En_0 = pid->En_0 * 0.5 + pid->En_1 * 0.5;								// �˲�
		pid->SEk += pid->En_0;
//		pid->SEk += pid->En_0 + pid->desired_value;
		
		if		 (pid->SEk > 10000 ) {pid->SEk = 10000;}								// �����޷�
		else if(pid->SEk < -10000) {pid->SEk = -10000;}
    
		t1 = pid->Kp * pid->En_0;
    t2 = pid->Ti * pid->SEk;	// Ki
											
    pid->calc_result = t1 + t2;
    pid->En_1 = pid->En_0;
		return pid->calc_result;
}


///**
// * @brief   		����pidת��
// * @param[in]   pid  λ��ʽpid
// * @param[in]   gyro_z  z���ٶ�
// * @return  		pid������
// * @note				������ֱ��
// */
//float logic_calc_pid_turn(PID_Loc_Info *pid,float gyro_z){
//	pid->calc_result = pid->Kp * gyro_z;
//	return pid->calc_result;
//}

/**
 * @brief   		����pidת��(yaw)
 * @param[in]   pid  λ��ʽpid
 * @return  		pid������
 */
float logic_calc_pid_turn(PID_Loc_Info *pid){
	pid->En_0 = pid->desired_value - pid->current_value;
	pid->calc_result =  pid->Kp * pid->En_0 + (pid->Td / pid->Tsam) * pid->Kp * (pid->En_0 - pid->En_1);
	pid->En_1 = pid->En_0;
	return pid->calc_result;
}
