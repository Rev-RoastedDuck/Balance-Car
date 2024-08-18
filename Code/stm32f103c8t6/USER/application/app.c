#include "app.h"

/** \addtogroup mpu6050
 *  \{ */
 
#define Has_Picked_Up_Threshold_ACC_Z 				6 		// 判定为摔倒的阈值  
#define Has_Picked_Up_Threshold_Pitch 				2			// 判定为摔倒的阈值 
#define Has_Picked_Up_Threshold_Roll 					5			// 判定为摔倒的阈值 
#define Has_Picked_Up_Check_Count_Limit 			25 	  // 摔倒检测二次的次数
#define Has_Picked_Up_Threshold_Encoder 			3200 	// 摔倒检测二次确认为摔倒的次数
boolean mpu6050_has_picked_up(BalanceCarInfo *info){
		static uint8_t  flag_status;
		static uint16_t keep_flag_count;
		
		// 1.前倾角和横滚角很小
		if(0 == flag_status){
			if(info->mpu6050_data.mpu6050_acc_data.z > Has_Picked_Up_Threshold_ACC_Z
				&& ABS(info->mpu6050_data.mpu6050_euler_angles.pitch) < Has_Picked_Up_Threshold_Pitch
				&& ABS(info->mpu6050_data.mpu6050_euler_angles.roll) < Has_Picked_Up_Threshold_Roll){
				flag_status = 1;
			}
		}
		
		// 2.因为正反馈 转速达到最大
		if(1 == flag_status){
			if(++keep_flag_count > Has_Picked_Up_Check_Count_Limit){
				flag_status = 0;
				keep_flag_count = 0;
			}
			
			if(ABS(info->encoder_date.encoder_left + info->encoder_date.encoder_right) > Has_Picked_Up_Threshold_Encoder){
				flag_status = 0;
				keep_flag_count = 0;
				return TRUE;
			}
		}
		return FALSE;
}

#define Has_Fell_Down_Threshold_PITCH 				65		// 判定为摔倒的阈值  
#define Has_Fell_Down_Check_Count_Limit 			100 	// 摔倒检测二次的次数
#define Has_Fell_Down_Threshold_Fall_Count 		30  	// 摔倒检测二次确认为摔倒的次数
boolean mpu6050_has_fell_down(BalanceCarInfo *info){
		static boolean   is_falling;	
		static uint16_t  checked_count,keep_fell_down_count;
		
		// 俯仰角很大
	
		if(!is_falling){
			if(ABS(info->mpu6050_data.mpu6050_euler_angles.pitch) > Has_Fell_Down_Threshold_PITCH){
				is_falling = TRUE;
			}
		}
		
		if(is_falling){
			// 统计摔倒状态的次数
			if(ABS(info->mpu6050_data.mpu6050_euler_angles.pitch) > Has_Fell_Down_Threshold_PITCH){
				++ keep_fell_down_count;
			}
			// 重置检测
			else{
				is_falling = FALSE;
				checked_count = 0;
				keep_fell_down_count = 0;
			}
			
			if(keep_fell_down_count > Has_Fell_Down_Threshold_Fall_Count){
				is_falling = FALSE;
				checked_count = 0;
				keep_fell_down_count = 0;
				return TRUE;
			}
			
			// 统计检测的次数
			if(++ checked_count > Has_Fell_Down_Check_Count_Limit){
				is_falling = FALSE;
				checked_count = 0;
				keep_fell_down_count = 0;
			}
			
		}

		return FALSE;
}


#define Has_Put_Dowm_Threshold_Pitch 					5			// 判定为摔倒的阈值 
#define Has_Put_Dowm_Threshold_Roll 					5			// 判定为摔倒的阈值 
#define Has_Put_Dowm_Check_Count_Limit 			100 		// 摔倒检测二次的次数
#define Has_Put_Dowm_Threshold_Encoder_Min 	10  		// 摔倒检测二次确认为摔倒的次数
#define Has_Put_Dowm_Threshold_Encoder_Max 	50  		// 摔倒检测二次确认为摔倒的次数
boolean mpu6050_has_put_down(BalanceCarInfo *info){
		static uint8_t  flag_status;
		static uint16_t keep_flag_count;
		
		// 1.横滚角和俯仰角很小
		if(0 == flag_status){
			if(ABS(info->mpu6050_data.mpu6050_euler_angles.pitch) < Has_Picked_Up_Threshold_Pitch
				&& ABS(info->mpu6050_data.mpu6050_euler_angles.roll) < Has_Picked_Up_Threshold_Roll){
				flag_status = 1;
			}
		}
		
		// 2.轮子被人为转动
		if(1 == flag_status){
			if(++keep_flag_count > Has_Put_Dowm_Check_Count_Limit){
				flag_status = 0;
				keep_flag_count = 0;
			}
			
			if((info->encoder_date.encoder_left > Has_Put_Dowm_Threshold_Encoder_Min 
					&& info->encoder_date.encoder_left < Has_Put_Dowm_Threshold_Encoder_Max)
				&& (info->encoder_date.encoder_right > Has_Put_Dowm_Threshold_Encoder_Min 
						&& info->encoder_date.encoder_right < Has_Put_Dowm_Threshold_Encoder_Max)){
				flag_status = 0;
				keep_flag_count = 0;
				return TRUE;
			}
			
		}
		
		return FALSE;
}

#define Has_Rolled_Over_Threshold					15			// 判定为侧翻的阈值 
boolean mpu6050_has_rolled_over(BalanceCarInfo *info){
	if(ABS(info->mpu6050_data.mpu6050_euler_angles.roll) > Has_Rolled_Over_Threshold){
		return TRUE;
	}
	return FALSE;
}

void mpu6050_trun_way(int16_t angle){
	// yaw角控制


}


/** \} */

/** \addtogroup encoder
 *  \{ */
void encoder_get_filter_data(BalanceCarInfo *info,
														RRD_DRIVER_ENCODER *left_encoder_driver,
														RRD_DRIVER_ENCODER *right_encoder_driver)
{
		int16_t data;
		data = left_encoder_driver->encoder_get_count(left_encoder_driver);
		info->encoder_date.encoder_left = (int16_t)first_order_filter(&info->encoder_date.encoder_left_filter,(float)data);
															
		data = (-1) * right_encoder_driver->encoder_get_count(right_encoder_driver);
		info->encoder_date.encoder_right = (int16_t)first_order_filter(&info->encoder_date.encoder_right_filter,(float)data);	
}

void encoders_get_counts(BalanceCarInfo *info,
													RRD_DRIVER_ENCODER *left_encoder_driver,
													RRD_DRIVER_ENCODER *right_encoder_driver){
	info->encoder_date.encoder_left = (int16_t)left_encoder_driver->encoder_get_count(left_encoder_driver);
	info->encoder_date.encoder_right = (-1) * (int16_t)right_encoder_driver->encoder_get_count(right_encoder_driver);
}
/** \} */


/** \addtogroup motor
 *  \{ */
void motor_set_speed(RRD_DEVICE_MOTOR* motor_device,
											RRD_DEVICE_TB6612FNG* tb6612fng_device,
											int16_t motor_speed)
{		
		if(motor_speed > 0){
			tb6612fng_device->forward(tb6612fng_device);
		}
		else if(motor_speed < 0){
			tb6612fng_device->backward(tb6612fng_device);
		}	
		else{
			tb6612fng_device->stop(tb6612fng_device);
		}
		motor_device->set_compare(motor_device,(uint16_t)ABS(motor_speed));
}

#define Has_Stall_Encoder_Count 							5					// 判定为堵转的编码器数值
#define Has_Stall_Pwm_Compare 								2500			// 开启堵转保护的占空比值
boolean motor_has_stall(RRD_DEVICE_MOTOR* motor_device,
											RRD_DRIVER_ENCODER *encoder_driver)
{
	if(ABS(encoder_driver->count) < Has_Stall_Encoder_Count
			&& motor_device->get_compare(motor_device) > Has_Stall_Pwm_Compare){
		return TRUE;
	}
	return FALSE;
}
/** \} */

/** \addtogroup oled
 *  \{ */
void oled_show_data(BalanceCarInfo *info,
										RRD_DEVICE_MOTOR *motor_left,
										RRD_DEVICE_MOTOR *motor_right)
{
	OLED_DRIVER.oled_show_string(1,1,"pitch: ");	
	OLED_DRIVER.oled_show_float(1,12,info->mpu6050_data.mpu6050_euler_angles.pitch,2);
	OLED_DRIVER.oled_show_string(2,1,"roll: ");	
	OLED_DRIVER.oled_show_float(2,12,info->mpu6050_data.mpu6050_euler_angles.roll,2);
	OLED_DRIVER.oled_show_string(3,1,"yaw: ");	
	OLED_DRIVER.oled_show_float(3,12,info->mpu6050_data.mpu6050_euler_angles.yaw,2);

//	OLED_DRIVER.oled_show_string(4,1,"encoder_l:");	
//	OLED_DRIVER.oled_show_float(4,12,info->encoder_date.encoder_left,2);
//	
//	OLED_DRIVER.oled_show_string(5,1,"encoder_r:");	
//	OLED_DRIVER.oled_show_float(5,12,info->encoder_date.encoder_right,2);
	OLED_DRIVER.oled_show_string(4,1,"sek:");									
	OLED_DRIVER.oled_show_float(4,12,info->pid_data.speed_pid.SEk,2);			
	
	OLED_DRIVER.oled_show_string(5,1,"I:");									
	OLED_DRIVER.oled_show_float(5,12,info->pid_data.speed_pid.Ti,2);								
	
	OLED_DRIVER.oled_show_string(6,1,"pwm_l:");	
	OLED_DRIVER.oled_show_signed_num(6,12,motor_left->get_compare(motor_left),4);
	
	OLED_DRIVER.oled_show_string(7,1,"pwm_r:");	
	OLED_DRIVER.oled_show_signed_num(7,12,motor_right->get_compare(motor_right),4);
}

void oled_show_pid_data(BalanceCarInfo *info){
	OLED_DRIVER.oled_show_string(1,1,"balance");	
	OLED_DRIVER.oled_show_string(2,3,"P:");		OLED_DRIVER.oled_show_float(2,5,info->pid_data.balance_pid.Kp,2);
	OLED_DRIVER.oled_show_string(2,14,"D:");	OLED_DRIVER.oled_show_float(2,16,info->pid_data.balance_pid.Td,2);
	
	OLED_DRIVER.oled_show_string(3,1,"speed");	
	OLED_DRIVER.oled_show_string(4,3,"P:");		OLED_DRIVER.oled_show_float(4,5,info->pid_data.speed_pid.Kp,2);
	OLED_DRIVER.oled_show_string(4,14,"I:");	OLED_DRIVER.oled_show_float(4,16,info->pid_data.speed_pid.Td,2);
	OLED_DRIVER.oled_show_string(5,3,"SEk:");	OLED_DRIVER.oled_show_float(5,9,info->pid_data.speed_pid.SEk,2);
	
	OLED_DRIVER.oled_show_string(6,1,"turn");	
	OLED_DRIVER.oled_show_string(7,3,"P:");	 OLED_DRIVER.oled_show_float(7,5,info->pid_data.turn_pid.Kp,2);
	OLED_DRIVER.oled_show_string(7,14,"D:");	OLED_DRIVER.oled_show_float(7,16,info->pid_data.turn_pid.Td,2);
}
/** \} */


/** \addtogroup bluetooth
 *  \{ */

 
/** \} */


/** \addtogroup flash
 *  \{ */
#define ADDR   ADDR_FLASH_SECTOR(120)

void flash_save_data(BalanceCarInfo *info){
	FLASH_DRIVER.flash_erase_page(ADDR);
	uint32_t add = FLASH_DRIVER.flash_write_vector_data(ADDR,&info->pid_data,sizeof(info->pid_data));
//	printf("add %x \r\n",add);
}

void flash_read_data(BalanceCarInfo *info){
	PidDataInfo pid_data;
	FLASH_DRIVER.flash_read_vector_data(ADDR,&pid_data,sizeof(PidDataInfo));
	info->pid_data = pid_data;
}
 
/** \} */


/** \addtogroup systick
 *  \{ */
void task_callback_function(void){
	app_update_params();

	// 电机控制
	// 开环控制
	if(g_BALANCE_CAR_INFO.pid_control.open_loop){
		motor_set_speed(g_MOTOR_DEVICE_LEFT,
										g_TB6612FNG_DEVICE_LEFT,
										g_BALANCE_CAR_INFO.pid_open_loop_data.motor_left_pwm_reload_value);
		
		motor_set_speed(g_MOTOR_DEVICE_RIGHT,
										g_TB6612FNG_DEVICE_RIGHT,
										g_BALANCE_CAR_INFO.pid_open_loop_data.motor_right_pwm_reload_value);
	}
	else{
		float calc_value = 0;
		calc_value = logic_calc_pid_balance(&g_BALANCE_CAR_INFO.pid_data.balance_pid,
																					g_BALANCE_CAR_INFO.mpu6050_data.mpu6050_euler_angles.pitch,
																					g_BALANCE_CAR_INFO.mpu6050_data.mpu6050_gyro_data.y);
		g_BALANCE_CAR_INFO.pid_data.speed_pid.desired_value = 0;
		calc_value += logic_calc_pid_speed(&g_BALANCE_CAR_INFO.pid_data.speed_pid,
																				g_BALANCE_CAR_INFO.encoder_date.encoder_left,
																				g_BALANCE_CAR_INFO.encoder_date.encoder_right);
//		g_BALANCE_CAR_INFO.pid_data.turn_pid.current_value = 0;
//		g_BALANCE_CAR_INFO.pid_data.turn_pid.desired_value = 0;
//		calc_value += logic_calc_pid_turn(&g_BALANCE_CAR_INFO.pid_data.turn_pid);
		
		motor_set_speed(g_MOTOR_DEVICE_LEFT,
										g_TB6612FNG_DEVICE_LEFT,
										(int16_t)calc_value);
		
		motor_set_speed(g_MOTOR_DEVICE_RIGHT,
										g_TB6612FNG_DEVICE_RIGHT,
										(int16_t)calc_value);
		debug_uart_send_2(1,"calc_value: %d\r\n",(int16_t)calc_value);
	}
	
}

inline void systick_start_task(void){
	SYSTICK_DRIVER.init(10,TRUE,0);
	SYSTICK_DRIVER.set_interrupt_function(task_callback_function);
}
 
/** \} */

/** \addtogroup all
 *  \{ */
inline void app_start_task(void){
	systick_start_task();
}


void app_run(void){
	if(mpu6050_has_rolled_over(&g_BALANCE_CAR_INFO)										// 侧翻检测
		|| mpu6050_has_picked_up(&g_BALANCE_CAR_INFO)										// 拿起检测
		|| motor_has_stall(g_MOTOR_DEVICE_LEFT,g_ENCODER_DEVICE_LEFT)		// 堵转保护
		|| motor_has_stall(g_MOTOR_DEVICE_RIGHT,g_ENCODER_DEVICE_RIGHT)){

		g_BALANCE_CAR_INFO.pid_control.open_loop = TRUE;
		motor_set_speed(g_MOTOR_DEVICE_LEFT,g_TB6612FNG_DEVICE_LEFT,0);
		motor_set_speed(g_MOTOR_DEVICE_RIGHT,g_TB6612FNG_DEVICE_RIGHT,0);
			
		pic_loc_clear_error(&g_BALANCE_CAR_INFO.pid_data.balance_pid);
		pic_loc_clear_error(&g_BALANCE_CAR_INFO.pid_data.speed_pid);
		pic_loc_clear_error(&g_BALANCE_CAR_INFO.pid_data.turn_pid);
	}
	
	// 重置dmp
	if(g_BALANCE_CAR_INFO.function_option.need_dmp_reset){
		g_BALANCE_CAR_INFO.function_option.need_dmp_reset = 0;
//		g_BALANCE_CAR_INFO.pid_control.open_loop = TRUE;
//		motor_set_speed(g_MOTOR_DEVICE_LEFT,g_TB6612FNG_DEVICE_LEFT,0);
//		motor_set_speed(g_MOTOR_DEVICE_RIGHT,g_TB6612FNG_DEVICE_RIGHT,0);
//		while(mpu_dmp_init()){
//			delay_ms_soft(20);
//		}
	}
	
	// oled显示
	if(g_BALANCE_CAR_INFO.function_option.need_oled_show >= 0){
		if(g_BALANCE_CAR_INFO.function_option.need_oled_clear){
			g_BALANCE_CAR_INFO.function_option.need_oled_clear = FALSE;
			OLED_DRIVER.oled_clear();
			delay_ms_soft(10);
		}

		switch(g_BALANCE_CAR_INFO.function_option.need_oled_show){
			case 1:
				oled_show_data(&g_BALANCE_CAR_INFO,g_MOTOR_DEVICE_LEFT,g_MOTOR_DEVICE_RIGHT);
				break;
			case 2:
				oled_show_pid_data(&g_BALANCE_CAR_INFO);
				break;
			default:
				break;
		}
	}

}

void app_device_init(void){
	// debug 初始化
	debug_uart_init();
	debug_uart_printf_1("init now!\r\n");
	
	// 开启时钟线
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4|RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);

	// oled初始化
	OLED_DRIVER.oled_init();
	debug_uart_printf_1("oled初始化完成\r\n");
	
	// 蓝牙初始化
	BLUETOOTH_DEVICE.init();
	BLUETOOTH_DEVICE.send("    bluetooth init finished\r\n");
	debug_uart_printf_1("蓝牙初始化完成\r\n");
	
	// 编码器
	g_ENCODER_DEVICE_LEFT = encoder_driver_new();
	g_ENCODER_DEVICE_LEFT->encoder_config(g_ENCODER_DEVICE_LEFT,GPIOA,GPIO_Pin_6,GPIOA,GPIO_Pin_7,TIM3);
	
	g_ENCODER_DEVICE_RIGHT = encoder_driver_new();
	g_ENCODER_DEVICE_RIGHT->encoder_config(g_ENCODER_DEVICE_RIGHT,GPIOA,GPIO_Pin_0,GPIOA,GPIO_Pin_1,TIM2);
	debug_uart_printf_1("编码器初始化完成\r\n");
	
	// 电机
	g_MOTOR_DEVICE_LEFT = motor_driver_new();
	g_MOTOR_DEVICE_LEFT->set_compare_range(g_MOTOR_DEVICE_LEFT,0,3500);
	g_MOTOR_DEVICE_LEFT->init(g_MOTOR_DEVICE_LEFT,GPIOB,GPIO_Pin_6,TIM4,CH1,4000 - 1, 72 - 1, TIM_CKD_DIV1);
	
	g_MOTOR_DEVICE_RIGHT = motor_driver_new();
	g_MOTOR_DEVICE_RIGHT->set_compare_range(g_MOTOR_DEVICE_RIGHT,0,3500);
	g_MOTOR_DEVICE_RIGHT->init(g_MOTOR_DEVICE_RIGHT,GPIOB,GPIO_Pin_7,TIM4,CH2,4000 - 1, 72 - 1, TIM_CKD_DIV1);
	debug_uart_printf_1("电机初始化完成\r\n");
	
	// 电机驱动
	// 禁用JTAG
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	g_TB6612FNG_DEVICE_LEFT = tb6612fng_device_new();
	g_TB6612FNG_DEVICE_LEFT->init(g_TB6612FNG_DEVICE_LEFT,GPIOA,GPIO_Pin_12,GPIOA,GPIO_Pin_15);
	g_TB6612FNG_DEVICE_LEFT->stop(g_TB6612FNG_DEVICE_LEFT);
	
	g_TB6612FNG_DEVICE_RIGHT = tb6612fng_device_new();
	g_TB6612FNG_DEVICE_RIGHT->init(g_TB6612FNG_DEVICE_RIGHT,GPIOB,GPIO_Pin_3,GPIOB,GPIO_Pin_4);
	g_TB6612FNG_DEVICE_RIGHT->stop(g_TB6612FNG_DEVICE_RIGHT);

	debug_uart_printf_1("电机驱动初始化完成\r\n");
	
	
	// mpu6050初始化
	MPU6050_DEVICE.mpu6050_init();
	
	if (!MPU6050_DEVICE.mpu6050_get_id()){
    debug_uart_printf_1("没有检测到MPU6050传感器！\r\n");
		while(1);
	}
	
	while(mpu_dmp_init()){
		delay_ms_soft(20);
	}
	debug_uart_printf_1("mpu6050初始化完成\r\n");
}

void app_params_init(void){
	// 开环模式
	g_BALANCE_CAR_INFO.pid_control.open_loop = TRUE;
	
	// 功能选项
	g_BALANCE_CAR_INFO.function_option.need_oled_clear = FALSE;
	g_BALANCE_CAR_INFO.function_option.need_oled_show = 1;
	
	// balance_pid
	g_BALANCE_CAR_INFO.pid_data.balance_pid.Kp = 0;
	g_BALANCE_CAR_INFO.pid_data.balance_pid.Td = 0;
	g_BALANCE_CAR_INFO.pid_data.balance_pid.Tsam = 10;
	
	g_BALANCE_CAR_INFO.pid_data.balance_pid.Ti = 0;
	g_BALANCE_CAR_INFO.pid_data.balance_pid.SEk = 0;
	g_BALANCE_CAR_INFO.pid_data.balance_pid.En_0 = 0;
	g_BALANCE_CAR_INFO.pid_data.balance_pid.En_1 = 0;

	g_BALANCE_CAR_INFO.pid_data.balance_pid.calc_result = 0;
	g_BALANCE_CAR_INFO.pid_data.balance_pid.current_value = 0;
	g_BALANCE_CAR_INFO.pid_data.balance_pid.desired_value = 0;
	
	// speed_pid
	g_BALANCE_CAR_INFO.pid_data.speed_pid.Kp = 0;
	g_BALANCE_CAR_INFO.pid_data.speed_pid.Ti = 0;
	g_BALANCE_CAR_INFO.pid_data.speed_pid.Tsam = 10;
	
	g_BALANCE_CAR_INFO.pid_data.speed_pid.Td = 0;
	g_BALANCE_CAR_INFO.pid_data.speed_pid.SEk = 0;
	g_BALANCE_CAR_INFO.pid_data.speed_pid.En_0 = 0;
	g_BALANCE_CAR_INFO.pid_data.speed_pid.En_1 = 0;

	g_BALANCE_CAR_INFO.pid_data.speed_pid.calc_result = 0;
	g_BALANCE_CAR_INFO.pid_data.speed_pid.current_value = 0;
	g_BALANCE_CAR_INFO.pid_data.speed_pid.desired_value = 0;
	
	// turn_pid
	g_BALANCE_CAR_INFO.pid_data.turn_pid.Kp = 0;
	g_BALANCE_CAR_INFO.pid_data.turn_pid.Td = 0;
	g_BALANCE_CAR_INFO.pid_data.turn_pid.Tsam = 10;
	
	g_BALANCE_CAR_INFO.pid_data.turn_pid.Ti = 0;
	g_BALANCE_CAR_INFO.pid_data.turn_pid.SEk = 0;
	g_BALANCE_CAR_INFO.pid_data.turn_pid.En_0 = 0;
	g_BALANCE_CAR_INFO.pid_data.turn_pid.En_1 = 0;

	g_BALANCE_CAR_INFO.pid_data.turn_pid.calc_result = 0;
	g_BALANCE_CAR_INFO.pid_data.turn_pid.current_value = 0;
	g_BALANCE_CAR_INFO.pid_data.turn_pid.desired_value = 0;

	// encoder_date
	g_BALANCE_CAR_INFO.encoder_date.encoder_left = 0;
	g_BALANCE_CAR_INFO.encoder_date.encoder_right = 0;
	g_BALANCE_CAR_INFO.encoder_date.encoder_left_filter.k = 0.6;
	g_BALANCE_CAR_INFO.encoder_date.encoder_right_filter.k = 0.6;
	
	// mpu6050_data
	g_BALANCE_CAR_INFO.mpu6050_data.mpu6050_acc_data.x = 0.0f;
	g_BALANCE_CAR_INFO.mpu6050_data.mpu6050_acc_data.y = 0.0f;
	g_BALANCE_CAR_INFO.mpu6050_data.mpu6050_acc_data.z = 0.0f;
	
	g_BALANCE_CAR_INFO.mpu6050_data.mpu6050_gyro_data.x = 0.0f;
	g_BALANCE_CAR_INFO.mpu6050_data.mpu6050_gyro_data.y = 0.0f;
	g_BALANCE_CAR_INFO.mpu6050_data.mpu6050_gyro_data.z = 0.0f;
	
	g_BALANCE_CAR_INFO.mpu6050_data.mpu6050_euler_angles.pitch = 0.0f;
	g_BALANCE_CAR_INFO.mpu6050_data.mpu6050_euler_angles.roll = 0.0f;
	g_BALANCE_CAR_INFO.mpu6050_data.mpu6050_euler_angles.yaw = 0.0f;
	
	// flash data
//	flash_read_data(&g_BALANCE_CAR_INFO);
//	
	debug_uart_send("P: %0.2f D:%0.2f T: %0.2f \r\n",g_BALANCE_CAR_INFO.pid_data.balance_pid.Kp,
																									g_BALANCE_CAR_INFO.pid_data.balance_pid.Td,
																									g_BALANCE_CAR_INFO.pid_data.balance_pid.Tsam);
	
	debug_uart_send("P: %0.2f D: %0.2f T: %0.2f \r\n",g_BALANCE_CAR_INFO.pid_data.speed_pid.Kp,
																										g_BALANCE_CAR_INFO.pid_data.speed_pid.Td,
																										g_BALANCE_CAR_INFO.pid_data.speed_pid.Tsam);

	debug_uart_send("P: %0.2f D: %0.2f T: %0.2f \r\n",g_BALANCE_CAR_INFO.pid_data.turn_pid.Kp,
																										g_BALANCE_CAR_INFO.pid_data.turn_pid.Td,
																										g_BALANCE_CAR_INFO.pid_data.turn_pid.Tsam);
	delay_ms_soft(10);
}

void app_update_params(void){
	// mpu6050 dmp
	mpu_dmp_get_data(&g_BALANCE_CAR_INFO.mpu6050_data.mpu6050_euler_angles.pitch,
									&g_BALANCE_CAR_INFO.mpu6050_data.mpu6050_euler_angles.roll,
									&g_BALANCE_CAR_INFO.mpu6050_data.mpu6050_euler_angles.yaw);
	// mpu6050
	int16_t Gyro[3];
	int16_t Accel[3];
	
	MPU6050_DEVICE.mpu6050_get_acc(Accel);
	MPU6050_DEVICE.mpu6050_get_gyro(Gyro);
	
	g_BALANCE_CAR_INFO.mpu6050_data.mpu6050_acc_data.x = MPU6050_DEVICE.mpu6050_acc_transition(Accel[0]);
	g_BALANCE_CAR_INFO.mpu6050_data.mpu6050_acc_data.y = MPU6050_DEVICE.mpu6050_acc_transition(Accel[1]);
	g_BALANCE_CAR_INFO.mpu6050_data.mpu6050_acc_data.z = MPU6050_DEVICE.mpu6050_acc_transition(Accel[2]);

	g_BALANCE_CAR_INFO.mpu6050_data.mpu6050_gyro_data.x = MPU6050_DEVICE.mpu6050_gyro_transition(Gyro[0]);
	g_BALANCE_CAR_INFO.mpu6050_data.mpu6050_gyro_data.y = MPU6050_DEVICE.mpu6050_gyro_transition(Gyro[1]);
	g_BALANCE_CAR_INFO.mpu6050_data.mpu6050_gyro_data.z = MPU6050_DEVICE.mpu6050_gyro_transition(Gyro[2]);

	// encoders
//	encoders_get_counts(&g_BALANCE_CAR_INFO,
//													g_ENCODER_DEVICE_LEFT,
//													g_ENCODER_DEVICE_RIGHT);
	 encoder_get_filter_data(&g_BALANCE_CAR_INFO,
	  												g_ENCODER_DEVICE_LEFT,
														g_ENCODER_DEVICE_RIGHT);
	
}
/** \} */
