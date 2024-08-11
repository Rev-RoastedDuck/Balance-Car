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


#define Only_Receive_Data		(0)

/******************************************************************************/
/*--------------------------------���ݽṹ����---------------------------------*/
/******************************************************************************/
typedef enum{
    none_server = 0 ,                     // �޶�Ӧ����
		set_config_params,							  	  // ���ò���
		set_movement_params,									// �����˶���Ϣ
		parse_handle_params,									// �����ֱ�����
}Server_Type;                             // �����������

// ����
// Note: ʹ��sizeof()��buff�����У���ȡ��Ӧ���ݵĸ���
typedef union{
		struct{
				float balance_pid[2];							// ֱ���� P D
				float speed_pid[2];								// �ٶȻ� P I
				float turn_pid[1];								// ת�� P 
				float desired_pitch_angle;				// ��е��ֵ
				float	encoder_l_filter[1];				// ��������˲������� һ���˲� K
				float	encoder_r_filter[1];				// �ұ������˲������� һ���˲� K
		}set_params_info;
		
		struct{
				float turn_angle;
				float move_speed;
				float move_distance;
		}set_movement_info;
		
		struct{
				uint8_t command;
		}parse_handle_info;
		
    uint8_t rx_data_buff[40];
}Receive_Data;

#if Only_Receive_Data
// ����
typedef union{
		uint8_t block;
    // ע���ֽڶ��������
}Transmit_Data;
#endif


/******************************************************************************/
/*----------------------------------ȫ�ֺ���-----------------------------------*/
/******************************************************************************/
void transmit_data(Server_Type server);
void parse_packet(uint8_t data);
boolean check_packet_tail(uint8_t data);


#endif /* CODE_DEVICE_COMMUNICATION_MODULE_PROTOCOL_H_ */
