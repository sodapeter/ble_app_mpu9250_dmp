#include "mpu_dmp.h"

#include "timestamping.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "data_builder.h"
#include "results_holder.h"
#include "invensense.h"

#include "twi_master.h"

#include "app_util_platform.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_gpiote.h"
#include "nrf.h"

#define QUAT_W 0
#define QUAT_X 1
#define QUAT_Y 2
#define QUAT_Z 3

#define VEC_X 0
#define VEC_Y 1
#define VEC_Z 2

#define MPU_HZ 100
#define COMPASS_HZ 50
#define TEMP_HZ 50

#define ACCEL_ON  0x01
#define GYRO_ON   0x02
#define PRINT_QUAT 0x04

const signed char _orientation[9] = {
    1,  0,  0,
    0,  1,  0,
    0,  0,  1
}; 

/* Invensence data */
struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
    struct rx_s rx;
};

static struct hal_s hal = {0};

/**
 * @brief Function that configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_config(void)
{
    APP_ERROR_CHECK(nrf_drv_gpiote_init());
}

static void mpu_config(void)
{
  uint32_t      err_code;
	
	twi_master_init();
	
	err_code = mpu_init();
	APP_ERROR_CHECK(err_code);
	
	err_code = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	APP_ERROR_CHECK(err_code);

	err_code = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	APP_ERROR_CHECK(err_code);

	err_code = mpu_set_sample_rate(MPU_HZ);
	APP_ERROR_CHECK(err_code);
	
//  mpu_set_lpf(10);
// if (0 != (ret = mpu_set_lpf(10))) // The following Low-Pass Filter settings are supported: 188, 98, 42, 20, 10, 5.
// {
//     NRF_LOG_ERROR("Failed to set LPF: %d\r\n", ret);
//     return ret;
// }

	/* Initialize HAL state variables. */
	memset(&hal, 0, sizeof(hal));
	hal.sensors = ACCEL_ON | GYRO_ON;
	hal.report = PRINT_QUAT;

	err_code = mpu_set_gyro_fsr(2000);
	APP_ERROR_CHECK(err_code);

	err_code = mpu_set_accel_fsr(16);
	APP_ERROR_CHECK(err_code);

  dmp_load_motion_driver_firmware();
  dmp_set_orientation(inv_orientation_matrix_to_scalar(_orientation));
   
	hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT
										| DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO;
	dmp_enable_feature(hal.dmp_features);
  dmp_set_fifo_rate(MPU_HZ);

  mpu_set_dmp_state(1);
	
}

void run_dmp(float* value)
{

  short gyro[3];
  short accel[3];

  unsigned char more;
  int ret = -1;

  long quat[4];
  float quaternion[4];
  short sensors;

  do {
    ret = dmp_read_fifo(gyro, accel, quat, &sensors, &more);

    if(ret == 0){
      if(more < 1){
        break;
      }
    }
  } while (1);

  quaternion[QUAT_W] = (float)quat[QUAT_W];
  quaternion[QUAT_X] = (float)quat[QUAT_X];
  quaternion[QUAT_Y] = (float)quat[QUAT_Y];
  quaternion[QUAT_Z] = (float)quat[QUAT_Z];
  inv_q_norm4(quaternion);
						
  memcpy(value, quaternion, sizeof(quaternion));

}

void mpu9250_init(void)
{
  gpio_config();
  mpu_config();
}


/** @} */
