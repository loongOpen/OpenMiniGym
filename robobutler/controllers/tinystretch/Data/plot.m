function plot
clc;
clear all;
close all;
data=importdata('./file1.txt'); 
exp_p=data(:,1);
LL=length(exp_p);
ST=0.1;
ADD=0.7;
ADDS=460/LL;
ED=ST+ADD;
if ED>1
    ED=1;
end
data=importdata('./file1.txt'); 
exp_p=data(:,1);
now_p=data(:,2);

exp_r=data(:,3);
now_r=data(:,4);

exp_y=data(:,5);
now_y=data(:,6);

now_pr=data(:,7);
now_rr=data(:,8);
now_yr=data(:,9);

now_cog_xr=limit(data(:,10),-0.5,0.5);
now_cog_x=limit(data(:,11),-0.5,0.5);
now_cog_dx=limit(data(:,12),-0.5,0.5);

now_cog_zr=limit(data(:,13),-0.5,0.5);
now_cog_z=limit(data(:,14),-0.5,0.5);
now_cog_dz=limit(data(:,15),-0.5,0.5);

g_pit=limit(data(:,16),-0.5,0.5);
g_rol=limit(data(:,17),-0.5,0.5);

tar_spdx=limit(data(:,18),-0.5,0.5);
tar_spdz=limit(data(:,19),-0.5,0.5);

%   
data=importdata('./file2.txt'); %足底力
cnt=1;
exp_fx0=data(:,cnt);cnt=cnt+1;
now_fx0=data(:,cnt);cnt=cnt+1;
exp_fy0=data(:,cnt);cnt=cnt+1;
now_fy0=data(:,cnt);cnt=cnt+1;
exp_fz0=data(:,cnt);cnt=cnt+1;
now_fz0=data(:,cnt);cnt=cnt+1;

exp_ty0=data(:,cnt);cnt=cnt+1;
now_ty0=data(:,cnt);cnt=cnt+1;
exp_tz0=data(:,cnt);cnt=cnt+1;
now_tz0=data(:,cnt);cnt=cnt+1;

g_flag0=data(:,cnt);cnt=cnt+1;
touch_flag0=data(:,cnt);cnt=cnt+1;
trig_state0=data(:,cnt);cnt=cnt+1;
 
leg0_q0_exp=    data(:,cnt);cnt=cnt+1;
leg0_q1_exp=    data(:,cnt);cnt=cnt+1;
leg0_q2_exp=    data(:,cnt);cnt=cnt+1;
leg0_q3_exp=    data(:,cnt);cnt=cnt+1;
leg0_q4_exp=    data(:,cnt);cnt=cnt+1;

leg0_q0_now=    data(:,cnt);cnt=cnt+1;
leg0_q1_now=    data(:,cnt);cnt=cnt+1;
leg0_q2_now=    data(:,cnt);cnt=cnt+1;
leg0_q3_now=    data(:,cnt);cnt=cnt+1;
leg0_q4_now=    data(:,cnt);cnt=cnt+1;

leg0_posx_h_exp=data(:,cnt);cnt=cnt+1;
leg0_posy_h_exp=data(:,cnt);cnt=cnt+1;
leg0_posz_h_exp=data(:,cnt);cnt=cnt+1;
leg0_posx_h_now=data(:,cnt);cnt=cnt+1;
leg0_posy_h_now=data(:,cnt);cnt=cnt+1;
leg0_posz_h_now=data(:,cnt);cnt=cnt+1;

leg0_t0_tar=    data(:,cnt);cnt=cnt+1;
leg0_t1_tar=    data(:,cnt);cnt=cnt+1;
leg0_t2_tar=    data(:,cnt);cnt=cnt+1;
leg0_t3_tar=    data(:,cnt);cnt=cnt+1;
leg0_t4_tar=    data(:,cnt);cnt=cnt+1;
leg0_t0_now=    data(:,cnt);cnt=cnt+1;
leg0_t1_now=    data(:,cnt);cnt=cnt+1;
leg0_t2_now=    data(:,cnt);cnt=cnt+1;
leg0_t3_now=    data(:,cnt);cnt=cnt+1;
leg0_t4_now=    data(:,cnt);cnt=cnt+1;
 
exp_fx1=data(:,cnt);cnt=cnt+1;
now_fx1=data(:,cnt);cnt=cnt+1;
exp_fy1=data(:,cnt);cnt=cnt+1;
now_fy1=data(:,cnt);cnt=cnt+1;
exp_fz1=data(:,cnt);cnt=cnt+1;
now_fz1=data(:,cnt);cnt=cnt+1;

exp_ty1=data(:,cnt);cnt=cnt+1;
now_ty1=data(:,cnt);cnt=cnt+1;
exp_tz1=data(:,cnt);cnt=cnt+1;
now_tz1=data(:,cnt);cnt=cnt+1;

g_flag1=data(:,cnt);cnt=cnt+1;
touch_flag1=data(:,cnt);cnt=cnt+1;
trig_state1=data(:,cnt);cnt=cnt+1;
 
leg1_q0_exp=    data(:,cnt);cnt=cnt+1;
leg1_q1_exp=    data(:,cnt);cnt=cnt+1;
leg1_q2_exp=    data(:,cnt);cnt=cnt+1;
leg1_q3_exp=    data(:,cnt);cnt=cnt+1;
leg1_q4_exp=    data(:,cnt);cnt=cnt+1;

leg1_q0_now=    data(:,cnt);cnt=cnt+1;
leg1_q1_now=    data(:,cnt);cnt=cnt+1;
leg1_q2_now=    data(:,cnt);cnt=cnt+1;
leg1_q3_now=    data(:,cnt);cnt=cnt+1;
leg1_q4_now=    data(:,cnt);cnt=cnt+1;

leg1_posx_h_exp=data(:,cnt);cnt=cnt+1;
leg1_posy_h_exp=data(:,cnt);cnt=cnt+1;
leg1_posz_h_exp=data(:,cnt);cnt=cnt+1;
leg1_posx_h_now=data(:,cnt);cnt=cnt+1;
leg1_posy_h_now=data(:,cnt);cnt=cnt+1;
leg1_posz_h_now=data(:,cnt);cnt=cnt+1;

leg1_t0_tar=    data(:,cnt);cnt=cnt+1;
leg1_t1_tar=    data(:,cnt);cnt=cnt+1;
leg1_t2_tar=    data(:,cnt);cnt=cnt+1;
leg1_t3_tar=    data(:,cnt);cnt=cnt+1;
leg1_t4_tar=    data(:,cnt);cnt=cnt+1;
leg1_t0_now=    data(:,cnt);cnt=cnt+1;
leg1_t1_now=    data(:,cnt);cnt=cnt+1;
leg1_t2_now=    data(:,cnt);cnt=cnt+1;
leg1_t3_now=    data(:,cnt);cnt=cnt+1;
leg1_t4_now=    data(:,cnt);cnt=cnt+1;

data=importdata('./file3.txt'); %零食记录
temp0=data(:,1);
temp1=data(:,2);
temp2=data(:,3);
temp3=data(:,4);
temp4=data(:,5);
temp5=data(:,6);
temp6=data(:,7);
temp7=data(:,8);
temp8=data(:,9);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
L=length(now_cog_dz);
P_ST=int16(ST*L);%选择显示区域
P_END=int16(ED*L);

figure('NumberTitle', 'off', 'Name', '姿态')
subplot(3,2,1);
plot(exp_p(P_ST:P_END),'-.r','LineWidth',2);
hold on;
plot(now_p(P_ST:P_END),'-k');
grid on;
ylabel('Pitch');
subplot(3,2,3);
plot(exp_r(P_ST:P_END),'-.r','LineWidth',2);
hold on;
plot(now_r(P_ST:P_END),'-k');
grid on;
ylabel('Roll');
subplot(3,2,5);
plot(exp_y(P_ST:P_END),'-.r');
hold on;
plot(now_y(P_ST:P_END));
grid on;
ylabel('Yaw');
subplot(3,2,2);
plot(now_pr(P_ST:P_END));
hold on;
grid on;
ylabel('Dpitch');
subplot(3,2,4);
plot(now_rr(P_ST:P_END));
hold on;
grid on;
ylabel('Droll');
subplot(3,2,6);
plot(now_yr(P_ST:P_END));grid on;
hold on;
ylabel('Dyaw');
grid on;

figure('NumberTitle', 'off', 'Name', '质心')
subplot(2,2,1);
plot(now_cog_xr(P_ST:P_END),'-.r','LineWidth',2);
hold on;
plot(now_cog_x(P_ST:P_END),'-k');
grid on;
ylabel('X方向Cog');
subplot(2,2,2);
plot(trig_state0(P_ST:P_END)/200+now_cog_z(P_ST),'LineWidth',1);
hold on;
plot(trig_state1(P_ST:P_END)/200+now_cog_z(P_ST),'LineWidth',1);
hold on;
plot(now_cog_zr(P_ST:P_END),'-.r','LineWidth',1.5);
hold on;
plot(now_cog_z(P_ST:P_END),'-k','LineWidth',1.5);
grid on;
ylabel('Z方向Cog');
subplot(2,2,3);
plot(tar_spdx(P_ST:P_END),'-.r','LineWidth',2);
hold on;
plot(now_cog_dx(P_ST:P_END),'-k');
ylim([-0.5,0.5]);
hold on;
grid on;
ylabel('X方向速度');
subplot(2,2,4);
plot(now_cog_dz(P_ST:P_END),'-k');
hold on;
plot(trig_state0(P_ST:P_END)/10+now_cog_dz(P_ST),'LineWidth',2);
hold on;
plot(trig_state1(P_ST:P_END)/10+now_cog_dz(P_ST),'LineWidth',2);
ylim([-0.5,0.5]);
hold on;
ylabel('Z方向速度');
grid on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1   2  3  4  5
% 6   7  8  9 10
% 11 12 13 14 15
%
%
figure('NumberTitle', 'off', 'Name', '右前0腿足底力')
% subplot(3,5,1);
% plot(g_flag0(P_ST:P_END),'-k','LineWidth',2);
% hold on;
% plot(touch_flag0(P_ST:P_END),'-.r');
% grid on;
% ylabel('着地状态');
subplot(3,5,1);
plot(now_fx0(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(exp_fx0(P_ST:P_END),'-.r','LineWidth',2);
grid on;
ylabel('X方向力');
subplot(3,5,6);
plot(now_fy0(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(exp_fy0(P_ST:P_END),'-.r','LineWidth',2);
grid on;
ylabel('Y方向力');
%ylim([-50,50]);
subplot(3,5,11);
plot(now_fz0(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(exp_fz0(P_ST:P_END),'-.r','LineWidth',2);
hold on;
plot(trig_state0(P_ST:P_END)*20,'LineWidth',1);
grid on;
ylabel('Z方向力');
ylim([-50,200]);

% subplot(3,5,7);
% plot(trig_state0(P_ST:P_END),'LineWidth',2);
% ylim([0,4]);
% ylabel('摆动相序');
% grid on;
subplot(3,5,3);
plot(leg0_q0_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg0_q0_exp(P_ST:P_END),'-.r','LineWidth',2);
ylabel('Q0大腿');
grid on;
subplot(3,5,8);
plot(leg0_q1_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg0_q1_exp(P_ST:P_END),'-.r','LineWidth',2);
ylabel('Q1小腿');
grid on;
subplot(3,5,13);
plot(leg0_q2_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg0_q2_exp(P_ST:P_END),'-.r','LineWidth',2);
ylabel('Q2侧展');
grid on;

subplot(3,5,4);
plot(leg0_q3_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg0_q3_exp(P_ST:P_END),'-.r','LineWidth',2);
ylabel('Q3偏转');
grid on;
subplot(3,5,9);
plot(leg0_q4_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg0_q4_exp(P_ST:P_END),'-.r','LineWidth',2);
ylabel('Q4足端');
grid on;



subplot(3,5,10);
plot(leg0_t3_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg0_t3_tar(P_ST:P_END),'-.r','LineWidth',2);
ylabel('T3偏转');
grid on;
subplot(3,5,15);
plot(leg0_t4_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg0_t4_tar(P_ST:P_END),'-.r','LineWidth',2);
ylabel('T4足端');
grid on;

subplot(3,5,2);
plot(leg0_posx_h_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg0_posx_h_exp(P_ST:P_END),'-.r','LineWidth',2);
ylabel('Hip X');
grid on;
subplot(3,5,7);
plot(leg0_posy_h_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg0_posy_h_exp(P_ST:P_END),'-.r','LineWidth',2);
ylabel('Hip Y');
grid on;
subplot(3,5,12);
plot(leg0_posz_h_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg0_posz_h_exp(P_ST:P_END),'-.r','LineWidth',2);
hold on;
plot(trig_state0(P_ST:P_END)/100+leg0_posz_h_now(P_ST),'LineWidth',2);
ylabel('Hip Z');
%ylim([-0.2,-0.1]);
grid on;
%---------------------------------------------------------------
figure('NumberTitle', 'off', 'Name', '左前1腿足底力')
subplot(4,2,1);
plot(g_flag1(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(touch_flag1(P_ST:P_END),'-.r');
grid on;
ylabel('着地状态');
subplot(4,2,3);
plot(now_fx1(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(exp_fx1(P_ST:P_END),'-.r','LineWidth',2);
grid on;
ylabel('X方向力');
%ylim([-50,50]);
subplot(4,2,5);
plot(now_fz1(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(exp_fz1(P_ST:P_END),'-.r','LineWidth',2);
hold on;
plot(trig_state1(P_ST:P_END)*20,'LineWidth',1);
grid on;
ylabel('Z方向力');
ylim([-50,250]);
subplot(4,2,7);
plot(trig_state1(P_ST:P_END),'LineWidth',2);
ylim([0,4]);
ylabel('摆动相序');
grid on;
subplot(4,2,2);
plot(leg1_q0_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg1_q0_exp(P_ST:P_END),'-.r','LineWidth',2);
ylabel('Q0');
grid on;
subplot(4,2,4);
plot(leg1_q1_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg1_q1_exp(P_ST:P_END),'-.r','LineWidth',2);
ylabel('Q1');
grid on;
subplot(4,2,6);
plot(leg1_posx_h_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg1_posx_h_exp(P_ST:P_END),'-.r','LineWidth',2);
ylabel('Hip X');
grid on;
subplot(4,2,8);
plot(leg1_posz_h_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg1_posz_h_exp(P_ST:P_END),'-.r','LineWidth',2);
hold on;
plot(trig_state1(P_ST:P_END)/100+leg1_posz_h_now(P_ST),'LineWidth',2);
ylabel('Hip Z');
%ylim([-0.2,-0.1]);
grid on;
 
end

function out=limit(in,min,max)
out=in;
if in<min
    out=min;
end
if in>max
    out=max;
end
end
