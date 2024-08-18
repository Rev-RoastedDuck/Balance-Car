#include "iwdg.h"
/**
 * @brief  			�������Ź���ʼ��
 * @param[in]   prescaler 		��Ƶϵ��  IWDG_Prescaler_4(8,16,32,64,128,256)
 * @param[in]		reload_value	��װ��ֵ	0~4096(2^12)
 * @return 			None
 * @note				���Ź�ʱ��Ƶ����30~60KHZ��һ��ȡ40KHZ 
 * @example			iwdg_init(IWDG_Prescaler_64,3124)  ��ʱ5s
 */
static void iwdg_init(uint8_t prescaler,uint16_t reload_value) {
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(prescaler);
	IWDG_SetReload(reload_value);
	IWDG_ReloadCounter();
	IWDG_Enable();
}

/**
 * @brief  			ι��(������װ��ֵ)
 * @param[in]   None
 * @return 			None
 */
static inline void iwdg_feed(void) {
    IWDG_ReloadCounter();
}

/**
 * @brief  			�ж��Ƿ��Ź����¸�λ
 * @param[in]   None
 * @return 			None
 */
static inline boolean iwdg_is_iwdg_reset(void){
	return (boolean)(RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET);
}

/******************************************************************************/
/*-----------------------------------�ⲿ�ӿ�---------------------------------*/
/******************************************************************************/
RRD_DRIVER_IWDG IWDG_DRIVER = {
	.iwdg_init = iwdg_init,
	.iwdg_feed = iwdg_feed,
	.iwdg_is_iwdg_reset = iwdg_is_iwdg_reset
};




