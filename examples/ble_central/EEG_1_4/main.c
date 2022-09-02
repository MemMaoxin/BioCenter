/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/**
 * @brief BLE LED Button Service central and client application main file.
 *
 * This example can be a central for up to 8 peripherals.
 * The peripheral is called ble_app_blinky and can be found in the ble_peripheral
 * folder.
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "app_uart.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_lbs_c.h"
#include "nrf_delay.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "SEGGER_RTT.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"

#include "board_rtc.h"

#define APP_BLE_CONN_CFG_TAG      1                                     /**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. Default tag is @ref APP_BLE_CONN_CFG_TAG. */
#define APP_BLE_OBSERVER_PRIO     3                                     /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define CENTRAL_SCANNING_LED      BSP_BOARD_LED_0
#define CENTRAL_CONNECTED_LED     BSP_BOARD_LED_1
#define LEDBUTTON_LED             BSP_BOARD_LED_2                       /**< LED to indicate a change of state of the the Button characteristic on the peer. */

#define LEDBUTTON_BUTTON          BSP_BUTTON_0                          /**< Button that will write to the LED characteristic of the peer. */
#define BUTTON_DETECTION_DELAY    APP_TIMER_TICKS(50)                   /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SCAN_INTERVAL             0x00A0//0x00A0                                /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW               0x0050//0x0050                                /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_DURATION             0x0000                                /**< Duration of the scanning in units of 10 milliseconds. If set to 0x0000, scanning will continue until it is explicitly disabled. */

#define MIN_CONNECTION_INTERVAL   MSEC_TO_UNITS(8, UNIT_1_25_MS)      /**7.5< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL   MSEC_TO_UNITS(8, UNIT_1_25_MS)       /**30< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY             90                                     /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT       MSEC_TO_UNITS(6000, UNIT_10_MS)       /**< Determines supervision time-out in units of 10 milliseconds. */

#define UART_TX_BUF_SIZE        256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                                     /**< UART RX buffer size. */

#define Enable1 4
#define Enable2 5
#define Enable3 6
#define Enable4 7

NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_LBS_C_ARRAY_DEF(m_lbs_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);           /**< LED Button client instances. */
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc, NRF_SDH_BLE_CENTRAL_LINK_COUNT);  /**< Database discovery module instances. */

static char const *m_target_periph_name = "ECG13"; //"ECG26";             /**< Name of the device we try to connect to. This name is searched for in the scan report data*/
static char const *m_target_periph_name1 = "PPG_100Hz";     //"RPPG11";             /**< Name of the device we try to connect to. This name is searched for in the scan report data*/
static char const *m_target_periph_name2 = "Dual_EEG7";    //"Dual_EEG5";             /**< Name of the device we try to connect to. This name is searched for in the scan report data*/
static char const *m_target_periph_name3 = "ImpedanceJYY12"; //"ImpedanceJYY10";

static uint8_t m_scan_buffer_data[BLE_GAP_SCAN_BUFFER_MIN]; /**< buffer where advertising reports will be stored by the SoftDevice. */

/**@brief Pointer to the buffer where advertising reports will be stored by the SoftDevice. */
static ble_data_t m_scan_buffer =
{
    m_scan_buffer_data,
    BLE_GAP_SCAN_BUFFER_MIN
};

/**@brief Scan parameters requested for scanning and connection. */
static ble_gap_scan_params_t const m_scan_params =
{
    .active   = 0,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,

    .timeout           = SCAN_DURATION,
    .scan_phys         = BLE_GAP_PHY_1MBPS,
    .filter_policy     = BLE_GAP_SCAN_FP_ACCEPT_ALL,

};

/**@brief Connection parameters requested for connection. */
static ble_gap_conn_params_t const m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,
    (uint16_t)MAX_CONNECTION_INTERVAL,
    (uint16_t)SLAVE_LATENCY,
    (uint16_t)SUPERVISION_TIMEOUT
};


/**@brief Function to handle asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}


/**@brief Function to start scanning. */
static void scan_start(void)
{
    ret_code_t ret;

    (void) sd_ble_gap_scan_stop();

    NRF_LOG_INFO("Start scanning for device name %s.", (uint32_t)m_target_periph_name);
    ret = sd_ble_gap_scan_start(&m_scan_params, &m_scan_buffer);
    //APP_ERROR_CHECK(ret);
    // Turn on the LED to signal scanning.
    bsp_board_led_on(CENTRAL_SCANNING_LED);
}


/**@brief Handles events coming from the LED Button central module.
 *
 * @param[in] p_lbs_c     The instance of LBS_C that triggered the event.
 * @param[in] p_lbs_c_evt The LBS_C event.
 */
static void lbs_c_evt_handler(ble_lbs_c_t * p_lbs_c, ble_lbs_c_evt_t * p_lbs_c_evt)
{

}



/**@brief Handles events coming from the LED Button central module.
 *
 * @param[in] p_lbs_c     The instance of LBS_C that triggered the event.
 * @param[in] p_lbs_c_evt The LBS_C event.
 */
static void event_of_b(ble_lbs_c_t * p_lbs_c, ble_lbs_c_evt_t * p_lbs_c_evt)
{
	    switch (p_lbs_c_evt->evt_type)
    {
        case BLE_LBS_C_EVT_DISCOVERY_COMPLETE://主服务发现完成
        {
            ret_code_t err_code;
            err_code = app_button_enable();//主机按键初始化
            APP_ERROR_CHECK(err_code);

            // LED Button service discovered. Enable notification of Button.
            err_code = ble_lbs_c_button_notif_enable(p_lbs_c);//主服务按键通知使能从机
            APP_ERROR_CHECK(err_code);
        } break; // BLE_LBS_C_EVT_DISCOVERY_COMPLETE

        case BLE_LBS_C_EVT_BUTTON_NOTIFICATION://按键通知使能后
        {
					
					uint32_t now_timeStamp = RTC_GetTime();
					char buffer[6] = "000000";
					sprintf(buffer, "%06d", now_timeStamp);
					for (int i = 0; i < 6; i++) {
						app_uart_put(buffer[i]);
					}
					
					
					app_uart_put('A');
					app_uart_put('0');
					for (int i=0;i<p_lbs_c_evt->length;i++)
						app_uart_put(p_lbs_c_evt->p_data[i]);
					app_uart_put('\n');
        } break; // BLE_LBS_C_EVT_BUTTON_NOTIFICATION

        default:
            // No implementation needed.
            break;
    }
}
static void event_of_b1(ble_lbs_c_t * p_lbs_c, ble_lbs_c_evt_t * p_lbs_c_evt)
{
    switch (p_lbs_c_evt->evt_type)
    {
        case BLE_LBS_C_EVT_DISCOVERY_COMPLETE://主服务发现完成
        {
            ret_code_t err_code;
            err_code = app_button_enable();//主机按键初始化
            APP_ERROR_CHECK(err_code);

            // LED Button service discovered. Enable notification of Button.
            err_code = ble_lbs_c_button_notif_enable(p_lbs_c);//主服务按键通知使能从机
            APP_ERROR_CHECK(err_code);
        } break; // BLE_LBS_C_EVT_DISCOVERY_COMPLETE

        case BLE_LBS_C_EVT_BUTTON_NOTIFICATION://按键通知使能后
        {
					
					uint32_t now_timeStamp = RTC_GetTime();
					char buffer[6] = "000000";
					sprintf(buffer, "%06d", now_timeStamp);
					for (int i = 0; i < 6; i++) {
						app_uart_put(buffer[i]);
					}
					
					
					app_uart_put('A');
					app_uart_put('1');
					for (int i=0;i<p_lbs_c_evt->length;i++)
						app_uart_put(p_lbs_c_evt->p_data[i]);
					app_uart_put('\n');
        } break; // BLE_LBS_C_EVT_BUTTON_NOTIFICATION

        default:
            // No implementation needed.
            break;
    }
}
static void event_of_b2(ble_lbs_c_t * p_lbs_c, ble_lbs_c_evt_t * p_lbs_c_evt)
{
    switch (p_lbs_c_evt->evt_type)
    {
        case BLE_LBS_C_EVT_DISCOVERY_COMPLETE://主服务发现完成
        {
            ret_code_t err_code;
            err_code = app_button_enable();//主机按键初始化
            APP_ERROR_CHECK(err_code);

            // LED Button service discovered. Enable notification of Button.
            err_code = ble_lbs_c_button_notif_enable(p_lbs_c);//主服务按键通知使能从机
            APP_ERROR_CHECK(err_code);
        } break; // BLE_LBS_C_EVT_DISCOVERY_COMPLETE

        case BLE_LBS_C_EVT_BUTTON_NOTIFICATION://按键通知使能后
        {
					
					uint32_t now_timeStamp = RTC_GetTime();
					char buffer[6] = "000000";
					sprintf(buffer, "%06d", now_timeStamp);
					for (int i = 0; i < 6; i++) {
						app_uart_put(buffer[i]);
					}
					
					
					app_uart_put('A');
					app_uart_put('2');
					for (int i=0;i<p_lbs_c_evt->length;i++)
						app_uart_put(p_lbs_c_evt->p_data[i]);
					app_uart_put('\n');
        } break; // BLE_LBS_C_EVT_BUTTON_NOTIFICATION

        default:
            // No implementation needed.
            break;
    }
}

static void event_of_b3(ble_lbs_c_t * p_lbs_c, ble_lbs_c_evt_t * p_lbs_c_evt)
{
    switch (p_lbs_c_evt->evt_type)
    {
        case BLE_LBS_C_EVT_DISCOVERY_COMPLETE://主服务发现完成
        {
            ret_code_t err_code;
            err_code = app_button_enable();//主机按键初始化
            APP_ERROR_CHECK(err_code);

            // LED Button service discovered. Enable notification of Button.
            err_code = ble_lbs_c_button_notif_enable(p_lbs_c);//主服务按键通知使能从机
            APP_ERROR_CHECK(err_code);
        } break; // BLE_LBS_C_EVT_DISCOVERY_COMPLETE

        case BLE_LBS_C_EVT_BUTTON_NOTIFICATION://按键通知使能后
        {
					
					uint32_t now_timeStamp = RTC_GetTime();
					char buffer[6] = "000000";
					sprintf(buffer, "%06d", now_timeStamp);
					for (int i = 0; i < 6; i++) {
						app_uart_put(buffer[i]);
					}
					
					
					app_uart_put('A');
					app_uart_put('3');
					for (int i=0;i<p_lbs_c_evt->length;i++)
						app_uart_put(p_lbs_c_evt->p_data[i]);
					app_uart_put('\n');
        } break; // BLE_LBS_C_EVT_BUTTON_NOTIFICATION

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling the advertising report BLE event.
 *
 * @param[in] p_adv_report  Advertising report from the SoftDevice.
 */
int adv_count = 5;
bool connected_state[4] = {0};
static void on_adv_report(ble_gap_evt_adv_report_t const * p_adv_report)//广播报告
{
    ret_code_t err_code;
		if (ble_advdata_name_find(p_adv_report->data.p_data,
                              p_adv_report->data.len,
                              m_target_periph_name)
				&& connected_state[0] == false)//发现指定名称的设备
    {
        // Name is a match, initiate connection.对指定参数进行连接
		  	NRF_LOG_INFO("%x datalen %d Adv: %s", p_adv_report->data.p_data,p_adv_report->data.len, m_target_periph_name);
   	            NRF_LOG_RAW_INFO("adv_data: ");
            for (uint16_t i = 0; i<p_adv_report->data.len; i++)
            {
                NRF_LOG_RAW_INFO("%02x:", p_adv_report->data.p_data[i]);
            }
            NRF_LOG_RAW_INFO("\r\n");		
			
			adv_count = 0;
        err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
                                      &m_scan_params,
                                      &m_connection_param,
                                      APP_BLE_CONN_CFG_TAG);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("Connection Request Failed, reason %d", err_code);
        }
    }
		else if (ble_advdata_name_find(p_adv_report->data.p_data,
                              p_adv_report->data.len,
                              m_target_periph_name1)
							&& connected_state[1] == false)//发现指定名称的设备
    {
        // Name is a match, initiate connection.对指定参数进行连接
			NRF_LOG_INFO("%x Adv: %s", p_adv_report->data.p_data, m_target_periph_name1);
				adv_count = 1;
        err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
                                      &m_scan_params,
                                      &m_connection_param,
                                      APP_BLE_CONN_CFG_TAG);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("Connection Request Failed, reason %d", err_code);
        }
    }else if (ble_advdata_name_find(p_adv_report->data.p_data,
                              p_adv_report->data.len,
                              m_target_periph_name2)
							&& connected_state[2] == false)//发现指定名称的设备
    {
        // Name is a match, initiate connection.对指定参数进行连接
				NRF_LOG_INFO("%x Adv: %s", p_adv_report->data.p_data, m_target_periph_name2);
				adv_count = 2;
        err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
                                      &m_scan_params,
                                      &m_connection_param,
                                      APP_BLE_CONN_CFG_TAG);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("Connection Request Failed, reason %d", err_code);
        }
    }
		else if (ble_advdata_name_find(p_adv_report->data.p_data,
                              p_adv_report->data.len,
                              m_target_periph_name3)
							&& connected_state[3] == false)//发现指定名称的设备
    {
        // Name is a match, initiate connection.对指定参数进行连接
				NRF_LOG_INFO("%x Adv: %s", p_adv_report->data.p_data, m_target_periph_name3);
				adv_count = 3;
        err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
                                      &m_scan_params,
                                      &m_connection_param,
                                      APP_BLE_CONN_CFG_TAG);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("Connection Request Failed, reason %d", err_code);
        }
    }
    else
    {
        err_code = sd_ble_gap_scan_start(NULL, &m_scan_buffer);//如果没发现继续扫描
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
uint8_t con_handle[4]={0,0,0,0};
uint8_t valid_acupoint_handle;
static void disable_connect_led(int index)
{
		if (con_handle[index] == 1) {
			nrf_gpio_pin_set(Enable1);
			connected_state[0] = false;
		} else if (con_handle[index] == 2){
			nrf_gpio_pin_set(Enable2);
			connected_state[1] = false;
		} else if (con_handle[index] == 3){
			nrf_gpio_pin_set(Enable3);
			connected_state[2] = false;
		} else if (con_handle[index] == 4){
			nrf_gpio_pin_set(Enable4);
			connected_state[3] = false;
		}
}
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    // For readability.
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral has connected, initiate DB
        // discovery, update LEDs status and resume scanning if necessary.
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("Connection 0x%x established, starting DB discovery.",
                         p_gap_evt->conn_handle);

						ble_gap_evt_adv_report_t const * p_adv_report=&(p_gap_evt->params.adv_report);
						APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < NRF_SDH_BLE_CENTRAL_LINK_COUNT);//连接句柄如果没有分配完
            err_code = ble_lbs_c_handles_assign(&m_lbs_c[p_gap_evt->conn_handle],
                                                p_gap_evt->conn_handle,
                                                NULL);//分频连接句柄
					  //NRF_LOG_INFO("%x Connect: %s", p_adv_report->data.p_data, m_target_periph_name);
						if (adv_count==0)
						{
							m_lbs_c[p_gap_evt->conn_handle].evt_handler = event_of_b;
							valid_acupoint_handle = p_gap_evt->conn_handle;
							
							con_handle[p_gap_evt->conn_handle]=1;//记录各个conn_handle的状态
							nrf_gpio_pin_clear(Enable1);
							connected_state[0] = true;
						}
						else if (adv_count==1)
						{
							m_lbs_c[p_gap_evt->conn_handle].evt_handler = event_of_b1;
							
							con_handle[p_gap_evt->conn_handle]=2;//记录各个conn_handle的状态
							nrf_gpio_pin_clear(Enable2);
							connected_state[1] = true;
						}
						else if (adv_count==2)
						{
						  m_lbs_c[p_gap_evt->conn_handle].evt_handler = event_of_b2;
							
							con_handle[p_gap_evt->conn_handle]=3;//记录各个conn_handle的状态
							nrf_gpio_pin_clear(Enable3);
							connected_state[2] = true;
						}
						else if (adv_count==3)
						{
						  m_lbs_c[p_gap_evt->conn_handle].evt_handler = event_of_b3;
							
							con_handle[p_gap_evt->conn_handle]=4;//记录各个conn_handle的状态
							nrf_gpio_pin_clear(Enable4);
							connected_state[3] = true;
						}


            APP_ERROR_CHECK(err_code);

            err_code = ble_db_discovery_start(&m_db_disc[p_gap_evt->conn_handle],
                                              p_gap_evt->conn_handle);//开始发现对应句柄的蓝牙服务
						if (err_code != NRF_ERROR_BUSY)
            {
                APP_ERROR_CHECK(err_code);
            }
				
						//NRF_LOG_INFO("LED Button service discovered on p_gap_evt conn_handle 0x%x of p_lbs_c conn_handle 0x%x ",
						//			 p_gap_evt->conn_handle, m_lbs_c[2].conn_handle);

            // Update LEDs status, and check if we should be looking for more
            // peripherals to connect to.
            bsp_board_led_on(CENTRAL_CONNECTED_LED);//更新LED灯
            if (ble_conn_state_central_conn_count() == NRF_SDH_BLE_CENTRAL_LINK_COUNT)//如果达到最大的连接数量
            {
                bsp_board_led_off(CENTRAL_SCANNING_LED);//关掉扫描LED
            }
            else
            {
                // Resume scanning.
                bsp_board_led_on(CENTRAL_SCANNING_LED);//否则继续扫描
                scan_start();
            }
        } break; // BLE_GAP_EVT_CONNECTED

        // Upon disconnection, reset the connection handle of the peer which disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("LBS central link 0x%x disconnected (reason: 0x%x)",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);
						disable_connect_led(p_gap_evt->conn_handle);
						con_handle[p_gap_evt->conn_handle]=0;
            if (ble_conn_state_central_conn_count() == 0)//如果连接设备为0
            {
                err_code = app_button_disable();
                APP_ERROR_CHECK(err_code);

                // Turn off connection indication LED
                bsp_board_led_off(CENTRAL_CONNECTED_LED);//关掉连接指示灯
            }

            // Start scanning
            scan_start();//开始扫描

            // Turn on LED for indicating scanning
            bsp_board_led_on(CENTRAL_SCANNING_LED);

        } break;

        case BLE_GAP_EVT_ADV_REPORT:
            on_adv_report(&p_gap_evt->params.adv_report);//报告扫描设备
            break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("Connection request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST.");
            // Accept parameters requested by peer.解释安全配对要求的参数
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
        {
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
					  disable_connect_led(p_gap_evt->conn_handle);
						con_handle[p_gap_evt->conn_handle]=0;
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
					  disable_connect_led(p_gap_evt->conn_handle);
						con_handle[p_gap_evt->conn_handle]=0;
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief LED Button collector initialization. */
static void lbs_c_init(void)//不同链路主服务初始化
{
    ret_code_t       err_code;
    ble_lbs_c_init_t lbs_c_init_obj;

    lbs_c_init_obj.evt_handler = lbs_c_evt_handler;

    for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        err_code = ble_lbs_c_init(&m_lbs_c[i], &lbs_c_init_obj);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for writing to the LED characteristic of all connected clients.
 *
 * @details Based on if the button is pressed or released, this function writes a high or low
 *          LED status to the server.
 *
 * @param[in] button_action The button action (press/release).
 *            Determines if the LEDs of the servers will be ON or OFF.
 *
 * @return If successful NRF_SUCCESS is returned. Otherwise, the error code from @ref ble_lbs_led_status_send.
 */
static ret_code_t led_status_send_to_all(uint8_t button_action)//按键状态发送
{
    ret_code_t err_code;

    for (uint32_t i = 0; i< NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        err_code = ble_lbs_led_status_send(&m_lbs_c[i], button_action);//按键状态发送给主机，主机写从机
        if (err_code != NRF_SUCCESS &&
            err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
            err_code != NRF_ERROR_INVALID_STATE)
        {
            return err_code;
        }
    }               
      return NRF_SUCCESS;
}


static ret_code_t led_status_send_to_two(uint8_t button_action)//按键状态发送
{
    ret_code_t err_code;
		err_code = ble_lbs_led_status_send(&m_lbs_c[0], button_action);//按键状态发送给主机，主机写从机
		if (err_code != NRF_SUCCESS &&
				err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
				err_code != NRF_ERROR_INVALID_STATE)
		{
				return err_code;
		}

		return NRF_SUCCESS;
}

/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    switch (pin_no)
    {
        case LEDBUTTON_BUTTON:
            err_code = led_status_send_to_two(button_action);//主机按下后，点亮所有从机
            {
                NRF_LOG_INFO("LBS write LED state %d", button_action);
            }
            break;
						
				//增加一个一对一的控制
       	 case BSP_BUTTON_1 :
            err_code = led_status_send_to_two(button_action);//主机按下后，点亮所有从机
            {
                NRF_LOG_INFO("LBS write LED2 state %d", button_action);
            }
            break;					

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    ret_code_t err_code;

   // The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {LEDBUTTON_BUTTON, false, BUTTON_PULL, button_event_handler},//按键中断
				{BSP_BUTTON_1, false, BUTTON_PULL, button_event_handler}//按键中断
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons), BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}
char string[20] = "Acupoint99";
char string1[20] = "LML_EEG";
char string2[20] = "LML_EEG";
char string3[20] = "LML_EEG";
void uart_event_handle(app_uart_evt_t * p_event)
{
		static uint8_t data_array[244];
    static uint16_t index = 0;
    uint32_t ret_val;

    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index = 0;						
            break;

        /**@snippet [Handling data from UART] */
        case APP_UART_COMMUNICATION_ERROR:
            NRF_LOG_INFO("Communication error occurred while handling UART.");
            //APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            NRF_LOG_INFO("Error occurred in FIFO module used by UART.");
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    NRF_LOG_DEBUG("call to ble_lbs_on_db_disc_evt for instance %d and link 0x%x!",
                  p_evt->conn_handle,
                  p_evt->conn_handle);

    ble_lbs_on_db_disc_evt(&m_lbs_c[p_evt->conn_handle], p_evt);
}


/** @brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handle any pending log operation(s), then sleep until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/** @brief Function for initializing the log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/** @brief Function for initializing the timer.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}
static void uart_init(void)
{
    ret_code_t err_code;

    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);
}


int main(void)
{
    // Initialize.
    log_init();
    timer_init();
		//定时器初始化
		RTC_Init();
	  uart_init();
    leds_init();
    buttons_init();
	  nrf_gpio_cfg_output(Enable1);
	  nrf_gpio_pin_set(Enable1);
		nrf_gpio_cfg_output(Enable2);
	  nrf_gpio_pin_set(Enable2);
		nrf_gpio_cfg_output(Enable3);
	  nrf_gpio_pin_set(Enable3);
		nrf_gpio_cfg_output(Enable4);
	  nrf_gpio_pin_set(Enable4);
	
    power_management_init();
    ble_stack_init();
    gatt_init();
    db_discovery_init();
    lbs_c_init();
    ble_conn_state_init();

	  RTC_Enable(0);

    // Start execution.
		printf("xxxxxxxxxxxxxxxxxxxxx");
    //NRF_LOG_INFO("Multilink example started.");
    scan_start();

    for (;;)
    {
        idle_state_handle();
    }
}
