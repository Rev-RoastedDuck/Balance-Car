#include "stm32f10x.h"
#include "delay.h"
#include "app.h"

int main(void){
	app_device_init();
	app_params_init();
	
  while(1)
  {		
		delay_ms_soft(10);
		app_update_params();
		oled_show_data(&g_BALANCE_CAR_INFO);
	}
}
