/*
 * pid_rrd.c
 *
 *  Created on: 2024��1��28��
 *      Author: intl4419
 */

#include "pid_rrd.h"

/** \addtogroup ������
 * \{ */
/******************************************************************************/
/*----------------------------------��̬����----------------------------------*/
/******************************************************************************/

/**
 * @brief  Ԥ���������
 * @param  pid:    PID
 * @return None
 */
static boolean __preprocessing(void *pid,boolean is_pid_int)
{
    boolean will_continue = TRUE;

    // ��������

    // ��ֵ�˲�

    return will_continue;
}

/**
 * @brief  ���������
 * @param  pid:    PID
 * @return None
 */
static void __postprocessing(void *pid,boolean is_pid_inc)
{

}
/** \} */



/** \addtogroup ������PID
 * \{ */
/******************************************************************************/
/*----------------------------------��̬����-----------------------------------*/
/******************************************************************************/
/**
 * @brief  ����ʽpid����
 * @param  pid:    ����ʽPID
 * @return None
 */
static void __pid_inc_calc_rrd(PID_Inc_Info *pid)
{
   float dk1;
   float dk2;
   float t1,t2,t3;

    pid->En_0 = pid->desired_value - pid->current_value;                     // ���㱾�����
    dk1 = pid->En_0 - pid->En_1;                                             // ���㱾��ƫ�����ϴ�ƫ��֮��
    dk2 = pid->En_0 - 2 * pid->En_1 + pid->En_2;

    t1 = pid->Kp * dk1;
    t2 = pid->Ti == 0? 0 : ((pid->Kp * pid->Tsam * pid->En_0) / pid->Ti);    // �ж��Ƿ���Ҫ����I���
    t3 = pid->Td == 0? 0 : ((pid->Td * pid->Kp * dk2) / pid->Tsam);

    pid->calc_result = t1 + t2 + t3;                                         // ����PWMӦ�����������ֵ
	
    pid->En_2 = pid->En_1;                                                   // �������ֵ
    pid->En_1 = pid->En_0;

}

/**
 * @brief  λ��ʽ pid����
 * @param  pid:    PID
 * @return None
 */
static void __pid_loc_calc_rrd(PID_Loc_Info *pid)
{
    float td;
    float t1,t2,t3;

    pid->En_0 = pid->desired_value - pid->current_value;                         // �õ���ǰ��ƫ��ֵ
    pid->SEk += pid->En_0;                                                       // ��ʷƫ���ܺ�

    t1 = pid->Kp * pid->En_0;                                                    // ������� P
    t2 = pid->Ti == 0? 0:((pid->Tsam/pid->Ti)*pid->Kp)*pid->SEk*pid->Kp;         // ������� I

    td = pid->Td / pid->Tsam;                                                    // ΢����� D
    t3 = td * pid->Kp * (pid->En_0 - pid->En_1);                                 // (�������ƫ��֮��)

    pid->calc_result = t1 + t2 + t3;
    pid->En_1 = pid->En_0;                                                       //����ƫ��
}

/******************************************************************************/
/*----------------------------------ȫ�ֺ���-----------------------------------*/
/******************************************************************************/
/**
 * @brief  pid��ʼ��
 * @param  pid:    PID
 * @return None
 */
void pid_parament_init_rrd(PID_Inc_Info *pid)
{
    pid->Kp=0;
    pid->Td=0;
    pid->Ti=0;

    pid->En_0 = 0;
    pid->En_1 = 0;
    pid->En_2 = 0;

    pid->desired_value = 0;
    pid->current_value = 0;

    pid->desired_value =0;
    pid->pwm_data->pwm_curr_duty=0;
    pid->pwm_data->pwm_count_period=0;
}


void pid_Inc_calc_rrd(PID_Inc_Info *pid)
{
    __preprocessing(pid,FALSE);
    __pid_inc_calc_rrd(pid);
    __postprocessing(pid,TRUE);
}

void pid_loc_calc_rrd(PID_Loc_Info *pid)
{
    __preprocessing(pid,FALSE);
    __pid_loc_calc_rrd(pid);
    __postprocessing(pid,FALSE);
}

/** \} */
