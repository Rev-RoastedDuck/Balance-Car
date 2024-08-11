#include "app.h"

/** \addtogroup mpu6050
 *  \{ */
 
#define Has_Picked_Up_Threshold_ACC_Z 				7.9		// 判定为摔倒的阈值  
#define Has_Picked_Up_Threshold_Pitch 				5			// 判定为摔倒的阈值 
#define Has_Picked_Up_Threshold_Roll 					5			// 判定为摔倒的阈值 
#define Has_Picked_Up_Check_Count_Limit 			100 	// 摔倒检测二次的次数
#define Has_Picked_Up_Threshold_Encoder 			200  	// 摔倒检测二次确认为摔倒的次数
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
 
/** \} */


/** \addtogroup motor
 *  \{ */
void motor_set_speed(RRD_DEVICE_MOTOR* motor_device,
											RRD_DEVICE_TB6612FNG* tb6612fng_device,
											int16_t motor_speed)
{
		if(motor_speed > 0){
			tb6612fng_device->forward(tb6612fng_device);
		}else{
			tb6612fng_device->backward(tb6612fng_device);
		}
		motor_device->set_compare(motor_device,(uint16_t)ABS(motor_speed));
}


/** \} */

/** \addtogroup oled
 *  \{ */
void oled_show_data(BalanceCarInfo *info){
	OLED_DRIVER.oled_show_string(1,1,"pitch: ");	
	OLED_DRIVER.oled_show_float(1,12,info->mpu6050_data.mpu6050_euler_angles.pitch,2);
	OLED_DRIVER.oled_show_string(2,1,"roll: ");	
	OLED_DRIVER.oled_show_float(2,12,info->mpu6050_data.mpu6050_euler_angles.roll,2);
	OLED_DRIVER.oled_show_string(3,1,"yaw: ");	
	OLED_DRIVER.oled_show_float(3,12,info->mpu6050_data.mpu6050_euler_angles.yaw,2);

	OLED_DRIVER.oled_show_string(4,1,"encoder_l:");	
	OLED_DRIVER.oled_show_float(4,12,info->encoder_date.encoder_left,2);
	
	OLED_DRIVER.oled_show_string(5,1,"encoder_r:");	
	OLED_DRIVER.oled_show_float(5,12,info->encoder_date.encoder_right,2);
}
 
/** \} */


/** \addtogroup bluetooth
 *  \{ */

 
/** \} */

/** \addtogroup all
 *  \{ */
void app_device_init(void){
	// debug 初始化
	debug_uart_init();
	
	debug_uart_printf_1("init now!\r\n");
	
	// 开启时钟线
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4|RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);

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
	
	// oled初始化
	OLED_DRIVER.oled_init();
	debug_uart_printf_1("oled初始化完成\r\n");
	
	// 蓝牙初始化
	BLUETOOTH_DEVICE.init();
	BLUETOOTH_DEVICE.send("    bluetooth init finished\r\n");
	debug_uart_printf_1("蓝牙初始化完成\r\n");
	
	// 编码器
	ENCODERS_DEVICE.init();
}

void app_params_init(void){
	// balance_pid
	g_BALANCE_CAR_INFO.pid_data.balance_pid.Kp = 0;
	g_BALANCE_CAR_INFO.pid_data.balance_pid.Td = 0;
	g_BALANCE_CAR_INFO.pid_data.balance_pid.Tsam = 0;
	
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
	g_BALANCE_CAR_INFO.pid_data.speed_pid.Tsam = 0;
	
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
	g_BALANCE_CAR_INFO.pid_data.turn_pid.Tsam = 0;
	
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
	g_BALANCE_CAR_INFO.encoder_date.encoder_left_filter.k = 0.7;
	g_BALANCE_CAR_INFO.encoder_date.encoder_right_filter.k = 0.7;
	
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
//	ENCODERS_DEVICE.get_counts(&g_BALANCE_CAR_INFO.encoder_date.encoder_left,
//															&g_BALANCE_CAR_INFO.encoder_date.encoder_right);
	encoder_get_filter_data(&g_BALANCE_CAR_INFO,
													ENCODERS_DEVICE.encoder_driver_left,
													ENCODERS_DEVICE.encoder_driver_right);
	
}
/** \} */
