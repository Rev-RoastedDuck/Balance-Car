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
/*----------------------------------��̬����-----------------------------------*/
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
//    2024/3/22
//    ����Ԫ����Ҫ�������������ⲿ����ƫ��ֵ���������ԣ�
//    �ȵȣ����񳵿⿪�����ƾͿ�����

    // 2024/06/20
//    pid->pwm_data->pwm_curr_duty += (uint32)(pid->calc_result);                        // ���㱾��PWMӦ�����������ֵ

    pid->En_2 = pid->En_1;                                                   // �������ֵ
    pid->En_1 = pid->En_0;

//    printf("t1:%f \r\n",t1);
//    printf("t2:%f \r\n",t2);
//    printf("t3:%f \r\n",t3);
//    printf("calc_result:%f\r\n",pid->calc_result);
////    printf("pwm_curr_duty:%ld\r\n",pid->pwm_data->pwm_curr_duty);
//    printf("=========\r\n");


    /**\NOTE ������PID
     *  ���ǵ�motor.h�У�����ռ�ձȿ��ƺ���
     *   void motor_set_speed_full_rrd(Motor_Channel motor_channel ,uint16 speed,Motor_Direction direction);
     *   ���ԣ����������ֵ����
     */
//    pid->calc_result = t1 + t2 + t3;                                         // ����PWMӦ�����������ֵ
//    pid->pwm_data->pwm_curr_duty += pid->calc_result;                        // ����PWMӦ�����������ֵ
//
//    pid->En_2 = pid->En_1;                                                   // �������ֵ
//    pid->En_1 = pid->En_0;
//    if(pid->pwm_data->pwm_curr_duty<0)
//    {
//        pid->pwm_data->pwm_curr_duty=0;
//        return;
//    }
//
//    if(pid->pwm_data->pwm_curr_duty>pid->pwm_data->pwm_count_period)
//    {
//        pid->pwm_data->pwm_curr_duty=pid->pwm_data->pwm_count_period;
//        return;
//    }

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

//    pid->En_0 = pid->desired_value - pid->current_value;                         // �õ���ǰ��ƫ��ֵ
    pid->SEk += pid->En_0;                                                       // ��ʷƫ���ܺ�

    t1 = pid->Kp * pid->En_0;                                                    // ������� P
    t2 = pid->Ti == 0? 0:((pid->Tsam/pid->Ti)*pid->Kp)*pid->SEk*pid->Kp;         // ������� I

    td = pid->Td / pid->Tsam;                                                    // ΢����� D
    t3 = td * pid->Kp * (pid->En_0 - pid->En_1);                                 // (�������ƫ��֮��)

    pid->calc_result = t1 + t2 + t3;
//    pid->pwm_data->pwm_curr_duty = pid->calc_result;                             // pid������
    pid->En_1 = pid->En_0;                                                       //����ƫ��

//    printf("t1:%f\r\n",t1);
//    printf("t2:%f\r\n",t2);
//    printf("t3:%f\r\n",t3);
//    printf("pid->calc_result:%f\r\n",pid->calc_result);
//    printf("pid->pwm_data->pwm_curr_duty:%ld\r\n",pid->pwm_data->pwm_curr_duty);

//    ���pwm���ô����з�Χ����
//    // ��Χ����
//    if(pid->pwm_data->pwm_curr_duty<0)
//    {
//        pid->pwm_data->pwm_curr_duty=0;
//        return;
//    }
//
//    if(pid->pwm_data->pwm_curr_duty>pid->pwm_data->pwm_count_period)
//    {
//        pid->pwm_data->pwm_curr_duty=pid->pwm_data->pwm_count_period;
//        return;
//    }
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

# if 0
/** \addtogroup ����PID
 * \{ */
/******************************************************************************/
/*----------------------------------��̬����-----------------------------------*/
/******************************************************************************/

void __pid_int_calc_rrd(PID_Int *pid)
{
    sint32 dk1;
    sint32 dk2;
    sint32 t1, t2, t3;

    pid->En_0 = (pid->desired_value - pid->current_value);                   // ���㱾�����
    dk1 = pid->En_0 - pid->En_1;                                             // ���㱾��ƫ�����ϴ�ƫ��֮��
    dk2 = pid->En_0 - 2 * pid->En_1 + pid->En_2;

    t1 = pid->Kp * dk1;
    t2 = ((pid->Kp * pid->Tsam * pid->En_0) / pid->Ti);
    t3 = ((pid->Kp * pid->Td * dk2) / pid->Tsam);

    pid->calc_result = t1 + t2 + t3;                                         // ����PWMӦ�����������ֵ
    pid->pwm_data->pwm_curr_duty += (uint16)(pid->calc_result / pid->multiplier);      // ����PWMӦ�����������ֵ

    pid->En_2 = pid->En_1;                                                   // �������ֵ
    pid->En_1 = pid->En_0;

    // ��Χ����
    if (pid->pwm_data->pwm_curr_duty < 0) {
        pid->pwm_data->pwm_curr_duty = 0;
        return;
    }

    if (pid->pwm_data->pwm_curr_duty > pid->pwm_data->pwm_count_period) {
        pid->pwm_data->pwm_curr_duty = pid->pwm_data->pwm_count_period;
        return;
    }
}

/******************************************************************************/
/*----------------------------------ȫ�ֺ���-----------------------------------*/
/******************************************************************************/
void pid_int_parament_init_rrd(PID_Int *pid)
{
    pid->Kp=0;
    pid->Td=0;
    pid->Ti=0;

    pid->En_0 = 0;
    pid->En_1 = 0;
    pid->En_2 = 0;

    pid->multiplier = 1000;

    pid->desired_value = 0;
    pid->current_value = 0;

    pid->desired_value =0;
    pid->pwm_data->pwm_curr_duty=0;
    pid->pwm_data->pwm_count_period=0;
}

void pid_int_calc_rrd(PID_Int *pid)
{
    __preprocessing(pid,TRUE);
    __pid_int_calc_rrd(pid);
    __postprocessing(pid,TRUE);
}
/** \} */
#endif
