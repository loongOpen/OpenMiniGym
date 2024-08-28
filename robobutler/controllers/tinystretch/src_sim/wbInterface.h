#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/inertial_unit.h>
#include <webots/touch_sensor.h>
#include <webots/keyboard.h>
#include <webots/gps.h>
#include <webots/joystick.h>
#include <webots/camera.h>
#include <webots/lidar.h>
#include <webots/range_finder.h>
#include <webots/display.h>
#include <webots/accelerometer.h>
extern WbDeviceTag motor_wt[4], motor_armt[6], motor_headt[2], motor_capt[2];
extern WbDeviceTag posensor_wt[4], posensor_armt[6], posensor_headt[2], posensor_capt[2];
extern WbDeviceTag hand[2];
extern WbDeviceTag IMU;
extern WbDeviceTag ACC;
extern WbDeviceTag GPS;
extern WbDeviceTag RGB_F,RGB_H0;
extern WbDeviceTag LIDAR_F;
extern WbDeviceTag RANGE_F;
#define TIME_STEP   5//ms

void webots_device_init(void);
void update_lidar(void);
