#include "iwdg.h"
/**
 * @brief  			独立看门狗初始化
 * @param[in]   prescaler 		分频系数  IWDG_Prescaler_4(8,16,32,64,128,256)
 * @param[in]		reload_value	重装载值	0~4096(2^12)
 * @return 			None
 * @note				看门狗时钟频率是30~60KHZ，一般取40KHZ 
 * @example			iwdg_init(IWDG_Prescaler_64,3124)  定时5s
 */
static void iwdg_init(uint8_t prescaler,uint16_t reload_value) {
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(prescaler);
	IWDG_SetReload(reload_value);
	IWDG_ReloadCounter();
	IWDG_Enable();
}

/**
 * @brief  			喂狗(重置重装载值)
 * @param[in]   None
 * @return 			None
 */
static inline void iwdg_feed(void) {
    IWDG_ReloadCounter();
}

/**
 * @brief  			判断是否看门狗导致复位
 * @param[in]   None
 * @return 			None
 */
static inline boolean iwdg_is_iwdg_reset(void){
	return (boolean)(RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET);
}

/******************************************************************************/
/*-----------------------------------外部接口---------------------------------*/
/******************************************************************************/
RRD_DRIVER_IWDG IWDG_DRIVER = {
	.iwdg_init = iwdg_init,
	.iwdg_feed = iwdg_feed,
	.iwdg_is_iwdg_reset = iwdg_is_iwdg_reset
};




