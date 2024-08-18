#include "stm32f10x.h"
#include "delay.h"
#include "app.h"
#include "debug_uart.h"

// TODO 电源开关 pwm信号去耦电容 动态flash 防反接 电机续流 软启动 电流限制 低电压检测 底层正电间隔
// 减压优化

int main(void){
	app_device_init();
	app_params_init();
	
	delay_ms_soft(100);
	app_start_task();
	
	while(1){
		app_run();
	}
	
	
}
