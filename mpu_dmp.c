#include "mpu_dmp.h"

#include <math.h>
//#include "nrf_log.h"
//#include "nrf_log_ctrl.h"

#include "timestamping.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "data_builder.h"
#include "results_holder.h"
#include "invensense.h"

#include "nrf_drv_twi.h"
//#include "nrf_drv_clock.h"
//#include "nrf_drv_rtc.h"
#include "nrf_drv_gpiote.h"

//#include "boards.h"
#include "nrf.h"
//#include "bsp.h"

#define SDA_PIN 28
#define SCL_PIN 2

//static nrf_drv_rtc_t const m_rtc = NRF_DRV_RTC_INSTANCE(0);

/* TWI instance. */
nrf_drv_twi_t m_app_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
//static bool m_xfer_done = false;
//#define MPU_INT_PIN 10

volatile bool mpu_data_ready = false;

const signed char _orientation[9] = {
    1,  0,  0,
    0,  1,  0,
    0,  0,  1
}; 


// RTC tick events generation.
//static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
//{
//}

//static void rtc_config(void)
//{
//    uint32_t err_code;

//    // Initialize RTC instance with default configuration.
//    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
//    err_code = nrf_drv_rtc_init(&m_rtc, &config, rtc_handler);
//    APP_ERROR_CHECK(err_code);

//    // Enable tick event and interrupt.
//    //nrf_drv_rtc_tick_enable(&m_rtc, true);

//    // Power on RTC instance.
//    nrf_drv_rtc_enable(&m_rtc);
//}

//static void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
//{
//    switch (p_event->type)
//    {
//        case NRF_DRV_TWI_EVT_DONE:
////            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
////            {
////                data_handler(m_sample);
////            }
//            m_xfer_done = true;
//            break;
//        default:
//            break;
//    }
//}

// TWI (with transaction manager) initialization.
static void twi_config(void)
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = SCL_PIN,
       .sda                = SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };

    err_code = nrf_drv_twi_init(&m_app_twi, &config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    nrf_drv_twi_enable(&m_app_twi);
}

/**
 * @brief Function that configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_config(void)
{
    APP_ERROR_CHECK(nrf_drv_gpiote_init());
}

void dataReadyCallback(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    mpu_data_ready = true;
}

static int mpu_config(void)
{
    int ret;

    struct int_param_s int_param;
    int_param.cb = &dataReadyCallback;
    //int_param.pin = MPU_INT_PIN;

    //NRF_LOG_INFO("Init MPU\r\n");

    if (0 != (ret = mpu_init(&int_param)))
    {
        //NRF_LOG_ERROR("Failed to init mpu: %d\r\n", ret);
        return ret;
    }

    if (0 != (ret = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS)))
    {
        //NRF_LOG_ERROR("Failed to set sensors: %d\r\n", ret);
        return ret;
    }

    if (0 != (ret = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS)))
    {
        //NRF_LOG_ERROR("Failed to set fifo: %d\r\n", ret);
        return ret;
    }

    if (0 != (ret = mpu_set_compass_sample_rate(COMPASS_HZ))) // Sampling rate must be between xxHz and 100Hz.
    {
        //NRF_LOG_ERROR("Failed to set compass sample rate: %d\r\n", ret);
        return ret;
    }
    if (0 != (ret = mpu_set_sample_rate(MPU_HZ))) // Sampling rate must be between 4Hz and 1kHz. LPF will be set to 1/2 of smapling rate.
    {
        //NRF_LOG_ERROR("Failed to set sample rate: %d\r\n", ret);
        return ret;
    }
    // if (0 != (ret = mpu_set_lpf(10))) // The following Low-Pass Filter settings are supported: 188, 98, 42, 20, 10, 5.
    // {
    //     NRF_LOG_ERROR("Failed to set LPF: %d\r\n", ret);
    //     return ret;
    // }

    if (0 != (ret = mpu_set_gyro_fsr(GYRO_FSR))) // The following Gyro Full-Scale Ranges are supported: 250, 500, 1000, 2000.
    {
        //NRF_LOG_ERROR("Failed to set Gyro FSR: %d\r\n", ret);
        return ret;
    }
    if (0 != (ret = mpu_set_accel_fsr(ACCEL_FSR))) // The following Accel Full-Scale Ranges are supported: 2, 4, 8, 16.
    {
        //NRF_LOG_ERROR("Failed to set Accel FSR: %d\r\n", ret);
        return ret;
    }
    if (0 != (ret = dmp_load_motion_driver_firmware()))
    {
        //NRF_LOG_ERROR("Failed to load dmp firmware: %d\r\n", ret);
        return ret;
    }

    if (0 != (ret = dmp_set_orientation(inv_orientation_matrix_to_scalar(_orientation))))
    {
        //NRF_LOG_ERROR("Failed to set orientation: %d\r\n", ret);
        return ret;
    }

    if (0 != (ret = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL)))
    {
        //NRF_LOG_ERROR("Failed to enable feature: %d\r\n", ret);
        return ret;
    }

    if (0 != (ret = dmp_set_fifo_rate(MPU_HZ)))
    {
        //NRF_LOG_ERROR("Failed to set fifo rate: %d\r\n", ret);
        return ret;
    }

    if (0 != (ret = dmp_set_interrupt_mode(DMP_INT_CONTINUOUS)))
    {
        //NRF_LOG_ERROR("Failed to set DMP interrupt: %d\r\n", ret);
        return ret;
    }

    if (0 != (ret = mpu_set_dmp_state(USE_DMP)))
    {
        //NRF_LOG_ERROR("Failed to set DMP state: %d\r\n", ret);
        return ret;
    }

    return 0;
}

static inline void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    //NRF_LOG_INFO("Starting self-test\n");

#if defined (MPU6500) || defined (MPU9250)
    result = mpu_run_6500_self_test(gyro, accel, 1);
#elif defined (MPU6050) || defined (MPU9150)
    result = mpu_run_self_test(gyro, accel);
#endif
    if (result == 0x7) {
	    //NRF_LOG_INFO("Passed: \r\n");
      //  NRF_LOG_RAW_INFO("\taccel: "NRF_LOG_FLOAT_MARKER" "NRF_LOG_FLOAT_MARKER" "NRF_LOG_FLOAT_MARKER"\r\n",
      //              NRF_LOG_FLOAT(accel[0]/65536.f),
      //              NRF_LOG_FLOAT(accel[1]/65536.f),
      //              NRF_LOG_FLOAT(accel[2]/65536.f));
      //  NRF_LOG_RAW_INFO("\tgyro: "NRF_LOG_FLOAT_MARKER" "NRF_LOG_FLOAT_MARKER" "NRF_LOG_FLOAT_MARKER"\r\n",
      //              NRF_LOG_FLOAT(gyro[0]/65536.f),
      //              NRF_LOG_FLOAT(gyro[1]/65536.f),
      //              NRF_LOG_FLOAT(gyro[2]/65536.f));
        /* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

#ifdef USE_CAL_HW_REGISTERS
        /*
         * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
         * instead of pushing the cal data to the MPL software library
         */
        unsigned char i = 0;

        for(i = 0; i<3; i++) {
        	gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
        	accel[i] *= 2048.f; //convert to +-16G
        	accel[i] = accel[i] >> 16;
        	gyro[i] = (long)(gyro[i] >> 16);
        }

        mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
        mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
        mpu_set_accel_bias_6050_reg(accel);
#endif
#endif
    } else {
        if (!(result & 0x1)){
            //NRF_LOG_ERROR("Gyro failed.\r\n");
				}
        if (!(result & 0x2)){
            //NRF_LOG_ERROR("Accel failed.\r\n");
				}
        if (!(result & 0x4)){
            //NRF_LOG_ERROR("Compass failed.\r\n");
				}
    }
}

void run_dmp(float* value)
{
    //nrf_gpio_range_cfg_output(22, 24);

    unsigned long timestamp;
    unsigned long now;
    unsigned long compass_next_read_timestamp = 0;
    unsigned long temp_next_read_timestamp = 0;
    //printf("1\r\n");
	
    //while (true)
    //{
        //while (NRF_LOG_PROCESS()) ;

        //nrf_gpio_pin_clear(22); // Pin low when CPU is sleeping
        while(mpu_data_ready != true)
        {
            // Make sure any pending events are cleared
            //__SEV();
            //__WFE();
            // Enter System ON sleep mode
            //__WFE();
        }
        
				//nrf_gpio_pin_set(22); // Pin high when CPU is working

        short gyro[3];
        short accel[3];
        short compass[3];
        long temperature;

        unsigned char more;
        int ret = 0;

        now = timestamp_func();

        if (now > compass_next_read_timestamp) {
            compass_next_read_timestamp = now + 1000 / COMPASS_HZ;
            
            if (!mpu_get_compass_reg(compass, &timestamp)) {
                //NRF_LOG_INFO("%d,C,%d,%d,%d\r\n", timestamp, compass[VEC_X], compass[VEC_Y], compass[VEC_Z]);
            } else {
                //NRF_LOG_ERROR("C: failed to read\r\n");
            }
        }

        if (now > temp_next_read_timestamp) {
            temp_next_read_timestamp = now + 1000 / TEMP_HZ;
            
            if (!mpu_get_temperature(&temperature, &timestamp)) {
                //NRF_LOG_INFO("%d,T,%d,%d,%d\r\n", timestamp, temperature, 0, 0);
            } else {
                //NRF_LOG_ERROR("T: failed to read\r\n")
            }
        }

        if (USE_DMP == 1) {
            long quat[4];
            float quaternion[4];
            short sensors;

            do
            {
							ret = dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);
                if (ret == 0) {
                    if (INV_WXYZ_QUAT == (sensors & INV_WXYZ_QUAT)) {
                        quaternion[QUAT_W] = (float)quat[QUAT_W];
                        quaternion[QUAT_X] = (float)quat[QUAT_X];
                        quaternion[QUAT_Y] = (float)quat[QUAT_Y];
                        quaternion[QUAT_Z] = (float)quat[QUAT_Z];
                        inv_q_norm4(quaternion);

                        //NRF_LOG_INFO("Q: " NRF_LOG_FLOAT_MARKER " " NRF_LOG_FLOAT_MARKER " " NRF_LOG_FLOAT_MARKER " ",
                        //    NRF_LOG_FLOAT(quaternion[QUAT_W]),
                        //    NRF_LOG_FLOAT(quaternion[QUAT_X]),
                        //    NRF_LOG_FLOAT(quaternion[QUAT_Y]));
                        
                        //NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER "\r\n",
                        //    NRF_LOG_FLOAT(quaternion[QUAT_Z]));
                    }

                    if (INV_XYZ_GYRO == (sensors & INV_XYZ_GYRO)){
                        //NRF_LOG_INFO("%d,G,%d,%d,%d\r\n", timestamp, gyro[VEC_X], gyro[VEC_Y], gyro[VEC_Z]);
										}
                    if (INV_XYZ_ACCEL == (sensors & INV_XYZ_ACCEL)){
                        //NRF_LOG_INFO("%d,A,%d,%d,%d\r\n", timestamp, accel[VEC_X], accel[VEC_Y], accel[VEC_Z]);
										}
                }
                else{
                    //NRF_LOG_ERROR("DMP: failed to read %d\r\n", ret); // NOTE: frequently called when there is not enough data in fifo yet
								}
            }
            while (more > 0);
						
						memcpy(value, quaternion, sizeof(quaternion));
						
        } else {
            unsigned char sensors;

            do
            {
                if (0 == (ret = mpu_read_fifo(gyro, accel, &timestamp, &sensors, &more))) {
                    //NRF_LOG_INFO("%d,G,%d,%d,%d\r\n", timestamp, gyro[VEC_X], gyro[VEC_Y], gyro[VEC_Z]);
                    //NRF_LOG_INFO("%d,A,%d,%d,%d\r\n", timestamp, accel[VEC_X], accel[VEC_Y], accel[VEC_Z]);
                } else {
                    //NRF_LOG_ERROR("FIFO: failed to read %d\r\n", ret);
								}
            }
            while (more > 0);
        }

        mpu_data_ready = false;

        // NOTE: used for magnetometer calibration with MotionCal:
        // https://learn.adafruit.com/ahrs-for-adafruits-9-dof-10-dof-breakout/magnetometer-calibration
        // http://www.pjrc.com/store/prop_shield.html
        //printf("Raw:%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], compass[0], compass[1], compass[2]);
    //}
}

//int main(void)
void mpu9250_init(void)
{

    // Start internal LFCLK XTAL oscillator - it is needed for "read" ticks generation (by RTC).
    //lfclk_config();
    //rtc_config();

    gpio_config();
    twi_config();
    //m_app_twi = m_app_twi2;

    //uart_config();
    APP_ERROR_CHECK(mpu_config());

    run_self_test();
}


/** @} */
