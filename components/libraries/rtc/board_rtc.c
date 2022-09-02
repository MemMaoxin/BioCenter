/*********************************************************************
 * INCLUDES
 */
#include <time.h>
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "nrfx_rtc.h"

#include "board_rtc.h"
#include "sdk_common.h"

static void rtcCallbackFunc(nrf_drv_rtc_int_type_t interruptType);

/*********************************************************************
 * GLOBAL VARIABLES
 */
		volatile uint32_t g_timestamp = 0;								// 时间戳

/*********************************************************************
 * LOCAL VARIABLES
 */
static const nrf_drv_rtc_t s_rtcHandle = NRF_DRV_RTC_INSTANCE(2);		// Declaring an instance of nrf_drv_rtc for RTC2.
static uint8_t s_timeCount1second = 0;

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
/**
 @brief RTC时间的初始化函数
 @param 无
 @return 无
*/
void RTC_Init(void)
{
    ret_code_t errCode;
        
    nrf_drv_rtc_config_t rtcConfig = NRF_DRV_RTC_DEFAULT_CONFIG;		//Initialize RTC instance
    rtcConfig.prescaler = 4095; 										// 如实现8HZ的频率，则PRESCALER寄存器应该设为32768/8-1 = 4095
    
		errCode = nrf_drv_rtc_init(&s_rtcHandle, &rtcConfig, rtcCallbackFunc);
    APP_ERROR_CHECK(errCode);

}
/**
 @brief 初始化时间戳并使能RTC和tick
 @param 无
 @return 无
*/
void RTC_Enable(uint32_t timestampNow)
{
		g_timestamp = timestampNow;
		nrf_drv_rtc_tick_enable(&s_rtcHandle, true);						// Enable tick event & interrupt 
    nrf_drv_rtc_enable(&s_rtcHandle);									// Power on RTC instance
}

/**
 @brief 停止RTC和tick的使用
 @param 无
 @return 无
*/
void RTC_Disable(void)
{
		nrf_drv_rtc_tick_disable(&s_rtcHandle);						// Enable tick event & interrupt 
		nrf_drv_rtc_disable(&s_rtcHandle);									// Power on RTC instance
}


/**
 @brief 设置RTC时间
 @param timestampNow -[in] 当前时间戳
 @return 无
*/
void RTC_SetTime(uint32_t timestampNow)
{
	g_timestamp = timestampNow;
}

/**
 @brief 获取RTC时间
 @param 无
 @return 当前时间戳
*/
uint32_t RTC_GetTime(void)
{
    return g_timestamp;
}


/*********************************************************************
 * LOCAL FUNCTIONS
 */
/**
 @brief RTC计数回调函数
 @param interruptType - [in] 中断类型
 @return 无
*/

static void rtcCallbackFunc(nrf_drv_rtc_int_type_t interruptType)
{
    if(interruptType == NRF_DRV_RTC_INT_COMPARE0)						// 中断类型：比较中断
    {
    }
    else if(interruptType == NRF_DRV_RTC_INT_TICK)						// 中断类型：滴答中断
    {
				g_timestamp++;
    }
}

/****************************************************END OF FILE****************************************************/
