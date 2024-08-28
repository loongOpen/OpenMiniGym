function vision_location_1%视觉落足测试
clc;
clear all;
close all;
MAP_W=40;
MAP_H=40;
MAP_SIZE=0.02;
MAP=zeros(MAP_H,MAP_W);
MAP_H_N=60;
MAP_W_N=50;
MAP_SIZE=0.02;
%
cog_n=[0,MAP_W_N*MAP_SIZE/2];
spd_tar=0.15;
T_gait=0.3;
dt_draw=0.3;

b_H=0.34;
b_W=0.26;
r_L=0.15;
robot_param=[b_H,b_W,r_L];
map_param=[MAP_H,MAP_W,MAP_H_N,MAP_W_N,MAP_SIZE];
check_param=[1,0.05,0.15];%地图选择区域

%%
if(0)%壕沟
    Trend_Mid=MAP_H/2;
    Trend_Wide=0.05;
    for  i = Trend_Mid-int16(Trend_Wide/MAP_SIZE)/2:Trend_Mid+int16(Trend_Wide/MAP_SIZE)/2% X
        for j = 1:MAP_W%Y
            MAP(i,j)=-0.1;
        end
    end
end

MAP_W=40;
MAP_H=40;
MAP_SIZE=0.02;
if(0)%台阶
    Trend_Mid=MAP_H/2-10;
    Trend_Wide=0.12;
    for  i = Trend_Mid-int16(Trend_Wide/MAP_SIZE)/2:Trend_Mid+int16(Trend_Wide/MAP_SIZE)/2% X
        for j = 20:MAP_W%Y
            MAP(i,j)=-0.025;
        end
    end
    Trend_Mid=MAP_H/2-10;
    Trend_Wide=0.12;
    for  i = Trend_Mid-int16(Trend_Wide/MAP_SIZE)/2:Trend_Mid+int16(Trend_Wide/MAP_SIZE)/2% X
        for j = 1:MAP_W-20%Y
            MAP(i,j)=-0.1;
        end
    end
    
    Trend_Mid=MAP_H/2;
    Trend_Wide=0.12;
    for  i = Trend_Mid-int16(Trend_Wide/MAP_SIZE)/2:Trend_Mid+int16(Trend_Wide/MAP_SIZE)/2% X
        for j = 1:MAP_W%Y
            MAP(i,j)=-0.1;
        end
    end
    Trend_Mid=MAP_H/2+7;
    Trend_Wide=0.12;
     for  i = Trend_Mid-int16(Trend_Wide/MAP_SIZE)/2:Trend_Mid+int16(Trend_Wide/MAP_SIZE)/2% X
        for j = 1:MAP_W%Y
            MAP(i,j)=-0.2;
        end
     end
    
      Trend_Mid=MAP_H/2+14;
     Trend_Wide=0.12;
     for  i = Trend_Mid-int16(Trend_Wide/MAP_SIZE)/2:Trend_Mid+int16(Trend_Wide/MAP_SIZE)/2% X
        for j = 25:MAP_W%Y
            MAP(i,j)=-0.13;
        end
    end
end


cube_Size=0.1;
cube_start_x=0.05;
cube_dis = 0.25;
cube_cog=[ cube_start_x,0.3,0;
           cube_start_x,0.6,0;
           
           cube_dis*1+cube_start_x,0.3,0;
           cube_dis*1+cube_start_x,0.6,0;
    
           cube_dis*2+cube_start_x,0.3,0;
           cube_dis*2+cube_start_x,0.6,0;
           
           cube_dis*3+cube_start_x,0.3,0;
           cube_dis*3+cube_start_x,0.6,0;
    ];%全局地图质心位置
cube_num=2*3;
if(1)%梅花桩
    Trend_Mid=MAP_H/2;
    Trend_Wide=0.05;
    size_temp=cube_Size/MAP_SIZE/2;
    for n=1:cube_num
        [Gx_n,Gy_n]=find_pos_on_map_n(cube_cog(n,:),map_param);
        for  i = Gx_n-int16(size_temp):Gx_n+int16(size_temp)% X
            for j= Gy_n-int16(size_temp):Gy_n+int16(size_temp)% Y
                if i>0 && i<MAP_H_N && j>0 && i<MAP_W_N
                    MAP(i,j)=0.1;%这里是局部地图只有40*40
                end
            end
        end
    end
end


%% 全局地图------------------------------------------------------------
MAP_N=zeros(MAP_H_N,MAP_W_N);%XY
MAP_FAKE_O=[MAP_H_N/2-MAP_H/2,MAP_W_N/2-MAP_W/2];%放置于全局地图中间
for  i = 1:MAP_H% X
    for j = 1:MAP_W%Y
        MAP_N(i+MAP_FAKE_O(1),j+MAP_FAKE_O(2))=MAP(i,j);
    end
end
if(0)
    figure(2)%全局地图
    X = linspace(0, MAP_H_N*MAP_SIZE, MAP_H_N);
    Y = linspace(0, MAP_W_N*MAP_SIZE, MAP_W_N);
    mesh(Y,X,MAP_N)
    hold on;
    plot3(0,0,0,'or')
    xlabel('Y轴');
    ylabel('X轴');
    axis equal;
end


if (0)%动画模式
    for i=1:10
        clf; hold on;    
        cog_n(1)=cog_n(1)+T_gait*spd_tar;
        X = linspace(0, MAP_H_N*MAP_SIZE, MAP_H_N);
        Y = linspace(0, MAP_W_N*MAP_SIZE, MAP_W_N);
        mesh(Y,X,MAP_N)
        hold on;
        plot3(0,0,0,'or')
        hold on;
        move_robot(cog_n,b_H,b_W,r_L,'-k',1.2);
        hold on;
        move_local_map(cog_n,MAP_N,[MAP_H,MAP_W,MAP_H_N,MAP_W_N,MAP_SIZE],'-k',1.2);%
        hold on;
        xlabel('Y轴');
        ylabel('X轴');
        axis equal;
        grid on;
        drawnow; 
        pause(dt_draw); 
    end
else

     cog_his(1,:)=cog_n;
     while 1
         cog_n=[0,MAP_W_N*MAP_SIZE/2];
         for i=1:30%局部地图的落足 但是局部地图采集自全局地图
             %预测机器人位置
            figure(1)
            clf; hold on;%绘图清除   
            cog_n(1)=cog_n(1)+T_gait*spd_tar;
            draw_global_map(MAP_N,map_param);
            b_H=robot_param(1);
            b_W=robot_param(2);
            r_L=robot_param(3);
            move_robot(cog_n,b_H,b_W,r_L,'-k',1.2);%移动机器人质心 绘制机器人模型
            hold on;
            map_local=move_local_map(cog_n,MAP_N,map_param,'-k',2);%移动地图扣取局部地图
            hold on;
            map_location(cog_n,map_local,MAP_N,map_param,check_param,robot_param);%处理局部地图

            %cog_his(i,:)=cog_n;%绘图用
            xlabel('Y轴');
            ylabel('X轴');
            view(76,32);
            axis equal;
            grid on;
            drawnow; 
            %waitforbuttonpress;
            pause(dt_draw); 
         end
     end
    
end

end




function [leg_fr,leg_hr,leg_fl,leg_hl]=move_robot(cog_n,b_W,b_H,r_L,color,line)%必须反着绘制
z_robot=0.2;
leg_fr=[b_H/2,b_W/2]+[cog_n(2),cog_n(1)];
leg_hr=[-b_H/2,b_W/2]+[cog_n(2),cog_n(1)];
leg_fl=[b_H/2,-b_W/2]+[cog_n(2),cog_n(1)];
leg_hl=[-b_H/2,-b_W/2]+[cog_n(2),cog_n(1)];

plot3([leg_fr(1),leg_hr(1)],[leg_fr(2),leg_hr(2)],[z_robot,z_robot],color,'LineWidth',line);
hold on;
plot3([leg_hr(1),leg_hl(1)],[leg_hr(2),leg_hl(2)],[z_robot,z_robot],color,'LineWidth',line);
hold on;
plot3([leg_hl(1),leg_fl(1)],[leg_hl(2),leg_fl(2)],[z_robot,z_robot],color,'LineWidth',line);
hold on;
plot3([leg_fl(1),leg_fr(1)],[leg_fl(2),leg_fr(2)],[z_robot,z_robot],color,'LineWidth',line);
hold on;
%绘制工作空间
aplha=0:pi/40:2*pi;
r=r_L;
z_r=0.05;
x=r*cos(aplha)+leg_fr(1);
y=r*sin(aplha)+leg_fr(2);
z=z_r*ones(size(x));
plot3(x,y,z,'-w','LineWidth',line);
hold on;
x=r*cos(aplha)+leg_fl(1);
y=r*sin(aplha)+leg_fl(2);
z=z_r*ones(size(x));
plot3(x,y,z,'-w','LineWidth',line);
hold on;
x=r*cos(aplha)+leg_hr(1);
y=r*sin(aplha)+leg_hr(2);
z=z_r*ones(size(x));
plot3(x,y,z,'-w','LineWidth',line);
hold on;
x=r*cos(aplha)+leg_hl(1);
y=r*sin(aplha)+leg_hl(2);
z=z_r*ones(size(x));
plot3(x,y,z,'-w','LineWidth',line);
hold on;

plot3(cog_n(2),cog_n(1),z_robot,'ok')
end

function draw_global_map(map_n,map_param)
MAP_W_N=map_param(4);
MAP_H_N=map_param(3);
MAP_SIZE=map_param(5);
%绘制全局地图
X = linspace(0, MAP_H_N*MAP_SIZE, MAP_H_N);
Y = linspace(0, MAP_W_N*MAP_SIZE, MAP_W_N);
mesh(Y,X,map_n)
hold on;
plot3(0,0,0,'or')
hold on;
end


