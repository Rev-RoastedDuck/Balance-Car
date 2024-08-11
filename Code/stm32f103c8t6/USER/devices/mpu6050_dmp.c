#include "mpu6050_dmp.h"

#if OPEN_MPU6050_DMP_TEST
void mpu6050_dmp_test(void){
	float pitch,roll,yaw;
	int16_t Gyro[3];	float Gyro_F[3];
	int16_t Accel[3];	float Accel_F[3];
	
	debug_uart_init();
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4|RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	
	MPU6050_DEVICE.mpu6050_init();
	
	if (!MPU6050_DEVICE.mpu6050_get_id()){
    printf("\r\n没有检测到MPU6050传感器！\r\n");
		while(1);
	}
	
	while(mpu_dmp_init()){
		delay_ms_soft(20);
	}
	
	uint8_t div;
	mpu_get_smplrt_div(&div);
	printf("采样频率分频值: %d \r\n",div);
	
	unsigned short gyro_fsr;
	mpu_get_gyro_fsr(&gyro_fsr);
	printf("量程: %d \r\n",gyro_fsr);
	
  while(1)
  {		
		delay_ms_soft(2);
		mpu_dmp_get_data(&pitch,&roll,&yaw);
//		printf("欧拉角： %8.2f%8.2f%8.2f    ",pitch,roll,yaw);
		
		MPU6050_DEVICE.mpu6050_get_gyro(Gyro);
////		mpu_get_gyro_reg(Gyro,NULL);
		Gyro_F[0] = MPU6050_DEVICE.mpu6050_gyro_transition(Gyro[0]);
		Gyro_F[1] = MPU6050_DEVICE.mpu6050_gyro_transition(Gyro[1]);
		Gyro_F[2] = MPU6050_DEVICE.mpu6050_gyro_transition(Gyro[2]);
//		printf("陀螺仪： %8.2f%8.2f%8.2f    \r\n",    Gyro_F[0], Gyro_F[1], Gyro_F[2]);

		MPU6050_DEVICE.mpu6050_get_acc(Accel);
		Accel_F[0] = MPU6050_DEVICE.mpu6050_acc_transition(Accel[0]);
		Accel_F[1] = MPU6050_DEVICE.mpu6050_acc_transition(Accel[1]);
		Accel_F[2] = MPU6050_DEVICE.mpu6050_acc_transition(Accel[2]);
//		printf("加速度： %8.2f%8.2f%8.2f    ",Accel_F[0],Accel_F[1],Accel_F[2]);
		
		printf(" %8.2f,%8.2f,%8.2f    ,%8.2f,%8.2f,%8.2f    ,%8.2f,%8.2f,%8.2f \r\n",pitch,roll,yaw,Gyro_F[0], Gyro_F[1], Gyro_F[2],Accel_F[0],Accel_F[1],Accel_F[2]);
	}
}
#endif   /* #if OPEN_MPU6050_DMP_TEST */
