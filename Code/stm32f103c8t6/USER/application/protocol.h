/*
 * protocol.h
 *
 *  Created on: 2024年2月14日
 *      Author: intl4419
 */

#ifndef CODE_DEVICE_COMMUNICATION_MODULE_PROTOCOL_H_
#define CODE_DEVICE_COMMUNICATION_MODULE_PROTOCOL_H_


// Note: 使用说明
//   接收:
//       把check_packet_tail()装载到串口中断函数中，如果有检测到完整的数据包尾，
//       就把缓存区的数据全部读取出来，通过循环每个字节把数据交给parse_packet()处理。
//   发送:
//       把对应的数据发送函数写入transmit_interface()接口,
//       通过transmit_data()选择要发送的数据。


/******************************************************************************/
/*---------------------------------头文件引入----------------------------------*/
/******************************************************************************/
#include "stm32f10x.h"
#include "typedef.h"
#include "globals.h"
#include "debug_uart.h"


#define Only_Receive_Data		(0)

/******************************************************************************/
/*--------------------------------数据结构定义---------------------------------*/
/******************************************************************************/
typedef enum{
    none_server = 0 ,                     // 无对应服务
	
		set_config_params = 1,							  // 设置参数
		set_movement_params = 2,							// 设置运动信息
		parse_handle_params = 3,							// 解析手柄数据
	
}Server_Type;                             // 定义服务类型

// 接收
// Note: 使用sizeof()从buff数组中，获取对应数据的个数
typedef union{
		struct{
				uint32_t 	need_open_loop_control;				// 开启开环控制  1:开环  0:闭环
				float 		balance_pid[2];							// 直立环 P D
				float 		speed_pid[2];								// 速度环 P I
				float 		turn_pid[2];								// 转向环 P D 
				float 		desired_pitch_angle;				// 机械中值
				float			encoder_l_filter[1];				// 左编码器滤波器参数 一阶滤波 K
				float			encoder_r_filter[1];				// 右编码器滤波器参数 一阶滤波 K
				uint8_t		function_options[4];				// 重置dmp 显示pid 显示状态
		}config_params;
		
		struct{
				float turn_angle;											// 转向角度
				float move_speed[2];									// 开环控制时，[0]:左电机 pwm 
																							//						 [1]:左电机pwm;
																							// 闭环控制时, [0]:speed_pid(增量式pid) 目标速度 
																							// 						 [1]:speed_pid(位置式pid) 目标移动距离
		}movement_params;
		
		struct{
				uint32_t command;
		}handle_params;
		
    uint8_t rx_data_buff[50];
}Receive_Data;

#if Only_Receive_Data
// 发送
typedef union{
		uint8_t block;
    // 注意字节对齐的问题
}Transmit_Data;
#endif


/******************************************************************************/
/*----------------------------------全局函数-----------------------------------*/
/******************************************************************************/
void transmit_data(Server_Type server);
void parse_packet(uint8_t data);
boolean check_packet_tail(uint8_t data);

void parse_packet_simple(uint8_t* data,uint8_t start_index,uint8_t end_index);

#endif /* CODE_DEVICE_COMMUNICATION_MODULE_PROTOCOL_H_ */
