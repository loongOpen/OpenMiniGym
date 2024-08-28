

void vmc_param_set(void);
void convert_vmc_webot_data(float dt);
void publish_vmc_out(void);
void main_robot_sm(float dt);
void pos_force_control(float dt);
void pos_controlw(float dt);
void pos_control_pd(float dt);
void force_control(float dt);
void force_dis_n(void); 

void force_control_and_dis(float dt);//位力混控制
void force_control_and_dis_stand(float dt);//底层==>位力混控制  站立步态！！
void force_control_and_dis_trot(float dt);//底层==>位力混控制  TORT步态！！