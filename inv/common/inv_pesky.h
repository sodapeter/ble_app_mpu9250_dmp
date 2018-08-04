#ifndef _INV_PESKY_
#define _INV_PESKY_

#include "string.h"
#include "timestamping.h"
#include "nrf_delay.h"

#ifdef MPU_LOG_RTTT
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define log_i NRF_LOG_INFO
#define log_e NRF_LOG_ERROR
#else
#include "log.h"

#define log_i MPL_LOGI
#define log_e MPL_LOGE
#endif

inline void get_ms(long unsigned int *timestamp)
{
    *timestamp = timestamp_func();
}

#define delay_ms nrf_delay_ms

#define min(a,b) ((a<b)?a:b)

//static inline int reg_int_cb(struct int_param_s *int_param)
//{
//    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true); // true - high accurracy
//    //config.pull = NRF_GPIO_PIN_PULLUP;

//    APP_ERROR_CHECK(nrf_drv_gpiote_in_init(int_param->pin, &config, int_param->cb));
//    nrf_drv_gpiote_in_event_enable(int_param->pin, true);
//    
//    return 0;
//}

#define __no_operation __NOP

#endif // _INV_PESKY_
