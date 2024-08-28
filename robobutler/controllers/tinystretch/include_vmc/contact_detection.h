#ifndef _CONTACT_DETETION_HPP_
#define _CONTACT_DETETION_HPP_
#include "cppTypes.h"
#include <Eigen/Dense>

class contact_detection_s
{
public:
    
    void configuration();
    void periodic_task();
    void get_necessary_data();
    void calculate_probability();
    void kalman_fusion();
    void calculate_result();
    void print();
    void record(int i);
    int get_contact(int leg_id){return contact[leg_id];}
    
private:
    
    double force[4]={0};
    double phase[4]={0};
    double height[4]={0};
    
    double p_force[4]={0};
    double p_phase[4]={0};
    double p_height[4]={0};
    
    double mean_force=0;
    double probability_threshold=0;
    
    double variance_force=0;
    double variance_phase=0;
    double variance_height=0;
    
    double noise_force=0;
    double noise_phase=0;
    double noise_height=0;
    
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd K;
    Eigen::MatrixXd H;
    Eigen::MatrixXd x;
    Eigen::MatrixXd x_;
    Eigen::MatrixXd u;
    Eigen::MatrixXd z1;
    Eigen::MatrixXd z2;
    Eigen::MatrixXd z;
    Eigen::MatrixXd sigma;
    Eigen::MatrixXd sigma_;
    Eigen::MatrixXd sigma_w;
    Eigen::MatrixXd sigma_v1;
    Eigen::MatrixXd sigma_v2;
    Eigen::MatrixXd sigma_v;
    Eigen::MatrixXd I;
    FILE *fp=NULL;
        
    int contact[4]={0};
    
    
    int contact_touchsensor[4]={0};
    #ifdef simulation_mode
    webots::TouchSensor *touchsensor[4];
    const char* touchsensor_name[4]={"touch sensor rf","touch sensor lf","touch sensor lr","touch sensor rr"};
    #endif
    
};


extern contact_detection_s contact_detection;

#endif

