/*
 * logic.h
 *
 *  Created on: 2024��2��4��
 *      Author: intl4419
 */

#ifndef ALGORITHM_LOGIC_H_
#define ALGORITHM_LOGIC_H_

#include "debug_uart.h"
#include "pid_rrd.h"

/**
 * @brief   		����pidλ�û�
 * @param[in]   pid  λ��ʽpid
 * @param[in]   current_angle  ��ǰ�Ƕ�
 * @param[in]   gyro_y  			 y���ٶ�
 * @return  		pid������
 */
float logic_calc_pid_balance(PID_Loc_Info *pid,float current_angle,float gyro_y);

/**
 * @brief   		����pid�ٶȻ�
 * @param[in]   pid  λ��ʽpid
 * @param[in]   encoder_value_left   ���������ֵ
 * @param[in]   encoder_value_right  �ұ�������ֵ
 * @return  		pid������
 */
float logic_calc_pid_speed(PID_Loc_Info *pid,int32_t encoder_value_left,int32_t encoder_value_right);

/**
 * @brief   		����pidת��(yaw)
 * @param[in]   pid  λ��ʽpid
 * @return  		pid������
 */
float logic_calc_pid_turn(PID_Loc_Info *pid);

#endif /* CODE_LOGIC_H_ */
