#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "timestamping.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "data_builder.h"
#include "results_holder.h"
#include "invensense.h"

//void set_dmp_state(bool state);
void run_9dof(float* value);
void run_dmp(float* value);
void mpu9250_init(void);

