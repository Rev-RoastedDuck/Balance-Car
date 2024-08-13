#include "stm32f10x.h"
#include "delay.h"
#include "app.h"
#include "debug_uart.h"

int main(void){
	app_device_init();
	app_params_init();
	
	flash_read_data(&g_BALANCE_CAR_INFO);
	
	printf("%0.2f %0.2f %0.2f %0.2f \r\n",g_BALANCE_CAR_INFO.pid_data.balance_pid.Kp,
																				g_BALANCE_CAR_INFO.pid_data.balance_pid.Td,
																				g_BALANCE_CAR_INFO.pid_data.balance_pid.Ti,
																				g_BALANCE_CAR_INFO.pid_data.balance_pid.Tsam);
	
	printf("%0.2f %0.2f %0.2f %0.2f \r\n",g_BALANCE_CAR_INFO.pid_data.speed_pid.Kp,
																				g_BALANCE_CAR_INFO.pid_data.speed_pid.Td,
																				g_BALANCE_CAR_INFO.pid_data.speed_pid.Ti,
																				g_BALANCE_CAR_INFO.pid_data.speed_pid.Tsam);

	printf("%0.2f %0.2f %0.2f %0.2f \r\n",g_BALANCE_CAR_INFO.pid_data.turn_pid.Kp,
																				g_BALANCE_CAR_INFO.pid_data.turn_pid.Td,
																				g_BALANCE_CAR_INFO.pid_data.turn_pid.Ti,
																				g_BALANCE_CAR_INFO.pid_data.turn_pid.Tsam);
	
	while(1){
		delay_ms_soft(10);
		app_update_params();
		oled_show_data(&g_BALANCE_CAR_INFO);
	}
	
	
}
