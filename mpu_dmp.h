//#define NRF_LOG_MODULE_NAME "Pesky"
#define MPL_LOG_NDEBUG 0

#include "app_twi.h"
//#include "app_uart.h"
//#include "app_error.h"



#define QUAT_W 0
#define QUAT_X 1
#define QUAT_Y 2
#define QUAT_Z 3

#define VEC_X 0
#define VEC_Y 1
#define VEC_Z 2


#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

#define MPU_HZ 100
#define COMPASS_HZ 50
#define TEMP_HZ 50
#define USE_DMP 1
#define GYRO_FSR 250
#define ACCEL_FSR 2
//#define PRINT_TIMESTAMP_SENSORS
#define INT_ENABLE 0

#define TWI_INSTANCE_ID 0
#define MAX_PENDING_TRANSACTIONS 5




void run_dmp(float* value);
void mpu9250_init(void);

