#include "mpu_dmp.h"

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

enum{
    ACCEL_2G = 2,
    ACCEL_4G = 4,
    ACCEL_8G = 8,
    ACCEL_16G = 16,
};

enum{
  GYRO_250DPS	= 250,
  GYRO_500DPS	= 500,
  GYRO_1000DPS = 1000,
  GYRO_2000DPS = 2000,
};



static void mpu_config(void)
{
  uint32_t      err_code;
	
	err_code = mpu_init();
	APP_ERROR_CHECK(err_code);
	
	err_code = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS); //(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	APP_ERROR_CHECK(err_code);

	err_code = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS); //(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	APP_ERROR_CHECK(err_code);

  err_code = mpu_set_compass_sample_rate(COMPASS_HZ);

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

  #ifdef USE_DMP
    mpu_set_dmp_state(1);
  #else
    mpu_set_dmp_state(0);
	#endif

}

static void getAccel( float data[3], short temp[3] ) {

    uint8_t currentAccelRange = 0xFF;
    mpu_get_accel_fsr(&currentAccelRange);
	
    if (currentAccelRange == ACCEL_2G) {
        data[0]=(float)temp[0] / 16384.0f * 9.81f;
        data[1]=(float)temp[1] / 16384.0f * 9.81f;
        data[2]=(float)temp[2] / 16384.0f * 9.81f;
    } else if (currentAccelRange == ACCEL_4G){
        data[0]=(float)temp[0] / 8192.0f * 9.81f;
        data[1]=(float)temp[1] / 8192.0f * 9.81f;
        data[2]=(float)temp[2] / 8192.0f * 9.81f;
    } else if (currentAccelRange == ACCEL_8G){
        data[0]=(float)temp[0] / 4096.0f * 9.81f;
        data[1]=(float)temp[1] / 4096.0f * 9.81f;
        data[2]=(float)temp[2] / 4096.0f * 9.81f;
    } else if(currentAccelRange == ACCEL_16G){
        data[0]=(float)temp[0] / 2048.0f * 9.81f;
        data[1]=(float)temp[1] / 2048.0f * 9.81f;
        data[2]=(float)temp[2] / 2048.0f * 9.81f;
    }
    
//#ifdef DOUBLE_ACCELERO
//    data[0]*=2;
//    data[1]*=2;
//    data[2]*=2;
//#endif
    //return false;
}

/********************************************************
 * getGyro
 *******************************************************/
static void getGyro( float data[3], short temp[3]) {

    uint16_t currentGyroRange = 0xFFFF;
    mpu_get_gyro_fsr(&currentGyroRange);
	
	if (currentGyroRange == GYRO_250DPS) {
        data[0]=(float)temp[0] / 7505.7f;
        data[1]=(float)temp[1] / 7505.7f;
        data[2]=(float)temp[2] / 7505.7f;
    } 
  	if (currentGyroRange == GYRO_500DPS){
        data[0]=(float)temp[0] / 3752.9f;
        data[1]=(float)temp[1] / 3752.9f;
        data[2]=(float)temp[2] / 3752.9f;
    }
		if (currentGyroRange == GYRO_1000DPS){
        data[0]=(float)temp[0] / 1879.3f;
        data[1]=(float)temp[1] / 1879.3f;
        data[2]=(float)temp[2] / 1879.3f;
    } 
		if(currentGyroRange == GYRO_2000DPS){
        data[0]=(float)temp[0] / 939.7f;
        data[1]=(float)temp[1] / 939.7f;
        data[2]=(float)temp[2] / 939.7f;
    }
}

/********************************************************
 * getMag
 *******************************************************/
static void getMag(float data[3], short temp[3])
{
  for(int i=0; i<3; i++){
    data[i] = (float)temp[i] / 32768.0f * 4800.0f;
  }
}

void run_9dof(float* value){
  short gyro[3];
  short accel[3];
  short compass[3];
  long temperature;

  unsigned long timestamp;

  mpu_get_compass_reg(compass, &timestamp);
  mpu_get_temperature(&temperature, &timestamp);

  mpu_get_accel_reg(accel, &timestamp);
  mpu_get_gyro_reg(gyro, &timestamp);

  getAccel(&value[0], accel);
  getGyro(&value[3], gyro);
  value[6] = (float)(temperature / 340.00f) + 21.00f;
  getMag(&value[7], compass);

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

  mpu_config();
}


/** @} */
