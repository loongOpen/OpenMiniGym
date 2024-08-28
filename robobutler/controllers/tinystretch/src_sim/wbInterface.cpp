#include "wbInterface.h"
#include "robotmath.h"
#include "include.h"
WbDeviceTag motor_wt[4], motor_armt[6], motor_headt[2], motor_capt[2];
WbDeviceTag posensor_wt[4], posensor_armt[6], posensor_headt[2], posensor_capt[2];
WbDeviceTag IMU;
WbDeviceTag ACC;
WbDeviceTag GPS;
WbDeviceTag CAMERA_FRONT;
WbDeviceTag RGB_F,RGB_H0;
WbDeviceTag LIDAR_F;
WbDeviceTag RANGE_F;
WbDeviceTag display_lidar;
WbDeviceTag display_human1;
WbImageRef background, background_human1;

const char* motor_w[4]
{
"w0_joint","w1_joint","w2_joint","w3_joint"
 };

const char* motor_arm[6]
{
"arm_base_joint","arm0_joint","arm1_joint","arm2_joint","arm3_joint","arm4_joint"//
};

const char* motor_head[2]
{
"head0_joint","head1_joint"//
};

const char* motor_cap[2]
{
"cap0_joint","cap1_joint"//
};


#define DISPLAY_RECTANGLE_Y_OFFSET 10
#define DISPLAY_WIDTH 200
#define DISPLAY_HEIGHT 110
#define DISPLAY_MAX_RANGE 80
#define DISPLAY_CIRCLE_SIZE 10

#define WHITE 0x00FFFFFF
#define BLACK 0x00000000
#define RED 0x00994444
#define BLUE 0x00444499

int lidar_width;// = wb_lidar_get_horizontal_resolution(lms291);
int max_range;// = wb_lidar_get_max_range(lms291);

// init the background of the display, store it and return its pointer
WbImageRef init_display(WbDeviceTag display) {
    wb_display_fill_rectangle(display, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
    wb_display_set_color(display, BLACK);
    wb_display_draw_rectangle(display, (DISPLAY_WIDTH - 2 * DISPLAY_MAX_RANGE) / 2 - 1,
        (DISPLAY_HEIGHT - DISPLAY_MAX_RANGE) / 2 + DISPLAY_RECTANGLE_Y_OFFSET - 1,
        2 * DISPLAY_MAX_RANGE + 2, DISPLAY_MAX_RANGE + 2);
    wb_display_draw_text(display, "Brain", DISPLAY_RECTANGLE_Y_OFFSET, DISPLAY_RECTANGLE_Y_OFFSET);
    wb_display_set_color(display, BLUE);
    wb_display_fill_oval(display, DISPLAY_WIDTH / 2, (DISPLAY_HEIGHT - DISPLAY_MAX_RANGE) / 2 + DISPLAY_RECTANGLE_Y_OFFSET,
        DISPLAY_CIRCLE_SIZE, DISPLAY_CIRCLE_SIZE);
    WbImageRef background = wb_display_image_copy(display, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
    wb_display_set_color(display, RED);
    return background;
}

WbImageRef init_display_human(WbDeviceTag display) {
    wb_display_fill_rectangle(display, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
    wb_display_set_color(display, BLACK);
    wb_display_draw_text(display, "BITHumanoid", DISPLAY_RECTANGLE_Y_OFFSET, DISPLAY_RECTANGLE_Y_OFFSET);
    wb_display_set_color(display, BLUE);

    WbImageRef background = wb_display_image_copy(display, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
    wb_display_set_color(display, RED);
    return background;
}

// display the background and the points of the lidar
void update_display(WbDeviceTag display, WbImageRef background, const float *lidar_values, int lms291_width) {
    wb_display_image_paste(display, background, 0, 0, false);
    int i;
    double angle = -3.141596/2;

    for (i = 0; i < lms291_width; i++) {
        float x = lidar_values[i] * 40 * sin(angle);
        float y = lidar_values[i] * 40 * cos(angle);
        wb_display_draw_pixel(display, DISPLAY_WIDTH / 2 + x,
            (DISPLAY_HEIGHT - DISPLAY_MAX_RANGE) / 2 + DISPLAY_RECTANGLE_Y_OFFSET + y);
        angle += 3.141596 / lms291_width;
    }
}

// gaussian function
double gaussian(double x, double mu, double sigma) {
    return (1.0 / (sigma * sqrt(2 * 3.141596))) * exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma));
}

void update_lidar(void){
const float *lidar_values = wb_lidar_get_range_image(LIDAR_F);
update_display(display_lidar, background, lidar_values, lidar_width);
}

void update_human(void) {

}
//放在主函数循环之前 初始化设备
void webots_device_init(void)
{
    /* 陀螺仪设备初始化 */
    IMU = wb_robot_get_device("dogInertialUnit");
    wb_inertial_unit_enable(IMU, TIME_STEP);

    ACC = wb_robot_get_device("dogAccelerometer");
    wb_accelerometer_enable(ACC, TIME_STEP);
    /* GPS设备初始化 */
    //GPS = wb_robot_get_device("gps");
    //wb_gps_enable(GPS, TIME_STEP);
#if !BACK_INVERT||USE_PANDA5||USE_ANYMAL
    RGB_F = wb_robot_get_device("RGB_F");
    wb_camera_enable(RGB_F, TIME_STEP );

    RGB_H0 = wb_robot_get_device("RGB_H0");
    wb_camera_enable(RGB_H0, TIME_STEP );

    LIDAR_F = wb_robot_get_device("lidar");
    wb_lidar_enable(LIDAR_F, TIME_STEP );
    wb_lidar_enable_point_cloud(LIDAR_F);

    RANGE_F = wb_robot_get_device("range-finder");
    wb_range_finder_enable(RANGE_F, TIME_STEP);
    // init lms291
    display_lidar = wb_robot_get_device("lidar_display");
    display_human1 = wb_robot_get_device("human_display1");

    lidar_width = wb_lidar_get_horizontal_resolution(LIDAR_F);
    max_range = wb_lidar_get_max_range(LIDAR_F);

    // init the display
    background = init_display(display_lidar);
    background_human1 = init_display_human(display_human1);
#endif
    /* 电机及位置传感器设备初始化 */
    for (uint8_t i = 0; i < 4; i++) {
        motor_wt[i] = wb_robot_get_device(motor_w[i]);
        posensor_wt[i] = wb_motor_get_position_sensor(motor_wt[i]);
        wb_position_sensor_enable(posensor_wt[i], TIME_STEP);
        //wb_motor_enable_torque_feedback(motor1[i], TIME_STEP);
    }
    for (uint8_t i = 0; i < 6; i++) {
        motor_armt[i] = wb_robot_get_device(motor_arm[i]);
        posensor_armt[i] = wb_motor_get_position_sensor(motor_armt[i]);
        wb_position_sensor_enable(posensor_armt[i], TIME_STEP);
        //wb_motor_enable_torque_feedback(motor1[i], TIME_STEP);
    }
    for (uint8_t i = 0; i < 2; i++) {
        motor_headt[i] = wb_robot_get_device(motor_head[i]);
        posensor_headt[i] = wb_motor_get_position_sensor(motor_headt[i]);
        wb_position_sensor_enable(posensor_headt[i], TIME_STEP);
        //wb_motor_enable_torque_feedback(motor1[i], TIME_STEP);
    }
    for (uint8_t i = 0; i < 2; i++) {
        motor_capt[i] = wb_robot_get_device(motor_cap[i]);
        posensor_capt[i] = wb_motor_get_position_sensor(motor_capt[i]);
        wb_position_sensor_enable(posensor_capt[i], TIME_STEP);
        //wb_motor_enable_torque_feedback(motor1[i], TIME_STEP);
    }

    /* 使能键盘读取 */
    wb_keyboard_enable(TIME_STEP);

    wb_joystick_enable(TIME_STEP);
}
