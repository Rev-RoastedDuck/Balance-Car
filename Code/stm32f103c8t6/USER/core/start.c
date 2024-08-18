#include "stm32f10x.h"
#include "delay.h"
#include "app.h"
#include "debug_uart.h"

// TODO ��Դ���� pwm�ź�ȥ����� ��̬flash ������ ������� ������ �������� �͵�ѹ��� �ײ�������
// ��ѹ�Ż�

int main(void){
	app_device_init();
	app_params_init();
	
	delay_ms_soft(100);
	app_start_task();
	
	while(1){
		app_run();
	}
	
	
}
