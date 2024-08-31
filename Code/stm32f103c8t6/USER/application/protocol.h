/*
 * protocol.h
 *
 *  Created on: 2024��2��14��
 *      Author: intl4419
 */

#ifndef CODE_DEVICE_COMMUNICATION_MODULE_PROTOCOL_H_
#define CODE_DEVICE_COMMUNICATION_MODULE_PROTOCOL_H_


// Note: ʹ��˵��
//   ����:
//       ��check_packet_tail()װ�ص������жϺ����У�����м�⵽���������ݰ�β��
//       �Ͱѻ�����������ȫ����ȡ������ͨ��ѭ��ÿ���ֽڰ����ݽ���parse_packet()����
//   ����:
//       �Ѷ�Ӧ�����ݷ��ͺ���д��transmit_interface()�ӿ�,
//       ͨ��transmit_data()ѡ��Ҫ���͵����ݡ�


/******************************************************************************/
/*---------------------------------ͷ�ļ�����----------------------------------*/
/******************************************************************************/
#include "stm32f10x.h"
#include "typedef.h"
#include "globals.h"
#include "debug_uart.h"


#define Only_Receive_Data		(1)

/******************************************************************************/
/*--------------------------------���ݽṹ����---------------------------------*/
/******************************************************************************/
typedef enum{
    none_server = 0 ,                     // �޶�Ӧ����
	
		set_config_params = 1,							  // ���ò���
		set_movement_params = 2,							// �����˶���Ϣ
		parse_handle_params = 3,							// �����ֱ�����
		
		send_balance_data = 11									// ����ƽ�⳵����
}Server_Type;                             // �����������

// ����
// Note: ʹ��sizeof()��buff�����У���ȡ��Ӧ���ݵĸ���
typedef union{
		struct{
//				float 		balance_pid[2];							// ֱ���� P D
//				float 		speed_pid[2];								// �ٶȻ� P I
//				float 		turn_pid[2];								// ת�� P D 
				float 		pid_info[2];								// PID  1: ֱ���� P D 2: �Ȼ� P I 3: �� P D 
				float 		desired_pitch_angle;				// ��е��ֵ
				uint8_t		function_options[4];				// pid���ú� oled��ʾ	 ����״̬��Ϣ need_open_loop_control
		}config_params;
		
		struct{
				float turn_angle;											// ת��Ƕ�
				float move_speed[2];									// ��������ʱ��[0]:���� pwm 
																							//						 [1]:����pwm;
																							// �ջ�����ʱ, [0]:speed_pid(����ʽpid) Ŀ���ٶ� 
																							// 						 [1]:speed_pid(λ��ʽpid) Ŀ���ƶ�����
		}movement_params;
		
		struct{
				uint32_t command;
		}handle_params;
		
    uint8_t rx_data_buff[60];
}Receive_Data;

#if !Only_Receive_Data
// ����
typedef union{
	
		struct{
				float pitch;
				float groy_y;
		}balance_data;
		
		uint8_t tx_data_buff[50];
}Transmit_Data;
#endif


/******************************************************************************/
/*----------------------------------ȫ�ֺ���----------------------------------*/
/******************************************************************************/
uint8_t buile_packet(const uint8_t *buff, uint8_t len,Server_Type server,uint8_t *packet);

void parse_packet_simple(uint8_t* data,uint8_t start_index,uint8_t end_index);

#endif /* CODE_DEVICE_COMMUNICATION_MODULE_PROTOCOL_H_ */
