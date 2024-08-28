function [gfoot,is_found]=foot_good_foot_found(leg_sel,spd_fb,now_foot,slip_foot,centroids,cluser,map_local_foot,map_param,check_param,robot_param)%输入局部高程图和名义落足点 优化安全落足点 导出函数
gfoot=[0,0,0];
MAP_H=40;%map_param(1);
MAP_W=40;%map_param(2);
MAP_SIZE=map_param(5);
if leg_sel==0
    Hw_id_2=int16(robot_param(1)/2/MAP_SIZE)+MAP_H/2;
    Ww_id_2=int16(robot_param(2)/2/MAP_SIZE)+MAP_W/2;
    r_L_id_2=int16(robot_param(3)/MAP_SIZE/2);
elseif leg_sel==1
    Hw_id_2=-int16(robot_param(1)/2/MAP_SIZE)+MAP_H/2;
    Ww_id_2=int16(robot_param(2)/2/MAP_SIZE)+MAP_W/2;
    r_L_id_2=int16(robot_param(3)/MAP_SIZE/2);
elseif leg_sel==2
    Hw_id_2=int16(robot_param(1)/2/MAP_SIZE)+MAP_H/2;
    Ww_id_2=-int16(robot_param(2)/2/MAP_SIZE)+MAP_W/2;
    r_L_id_2=int16(robot_param(3)/MAP_SIZE/2);
else%if leg_sel==3
    Hw_id_2=-int16(robot_param(1)/2/MAP_SIZE)+MAP_H/2;
    Ww_id_2=-int16(robot_param(2)/2/MAP_SIZE)+MAP_W/2;
    r_L_id_2=int16(robot_param(3)/MAP_SIZE/2);
end

is_found=0;
%绘制矢量方向线
Gx_foot_now=0;
Gy_foot_now=0;
Gx_slip_now=0;
Gy_slip_now=0;
[Gx_foot_now,Gy_foot_now]=find_pos_on_map_local(now_foot,map_param);
[Gx_slip_now,Gy_slip_now]=find_pos_on_map_local(slip_foot,map_param);

good_foot_id=[0];
good_foot_cnt1=0;
for i=1:MAP_H%查可落足X方向ID
    if map_local_foot(i,Gy_foot_now)>=check_param(2)&& map_local_foot(i,Gy_foot_now)<check_param(3)
        good_foot_cnt1=good_foot_cnt1+1;
        good_foot_id(good_foot_cnt1)=i;
    end
end

good_foot_id_f=[0];
good_foot_cnt2=0;
good_foot_id_b=[0];
good_foot_cnt3=0;
for i=1:good_foot_cnt1%查当前点速度前方ID
    if spd_fb>=0
        if good_foot_id(i)>=Gx_foot_now && good_foot_id(i)<=Hw_id_2+r_L_id_2
            good_foot_cnt2=good_foot_cnt2+1;
            good_foot_id_f(good_foot_cnt2)=good_foot_id(i);
        end

        if good_foot_id(i)<=Gx_foot_now && good_foot_id(i)>=Hw_id_2-r_L_id_2%查当前点后方ID
            good_foot_cnt3=good_foot_cnt3+1;
            good_foot_id_b(good_foot_cnt3)=good_foot_id(i);
        end
    else
        if good_foot_id(i)>=Gx_foot_now && good_foot_id(i)<=Hw_id_2+r_L_id_2
            good_foot_cnt3=good_foot_cnt3+1;
            good_foot_id_b(good_foot_cnt3)=good_foot_id(i);
        end

        if good_foot_id(i)<=Gx_foot_now && good_foot_id(i)>=Hw_id_2-r_L_id_2%查当前点后方ID
            good_foot_cnt2=good_foot_cnt2+1;
            good_foot_id_f(good_foot_cnt2)=good_foot_id(i);
        end        
    end
end

dis_min=int16(99);
dis_min_id=99;
if(good_foot_cnt2)%在移动方向前方找离SLIP最近的点
    for i=1:good_foot_cnt2
        dis=abs(good_foot_id_f(i)-Gx_slip_now);%计算SLIP落足
        if(dis<=dis_min)
            dis_min=dis;
            dis_min_id=good_foot_id_f(i);
        end
    end
    
     if dis_min_id<99%如果SLIP位置不在危险区域仍然采用
        if map_local_foot(Gx_slip_now,Gy_slip_now)>check_param(2) && map_local_foot(Gx_slip_now,Gy_slip_now)<check_param(3) && 1
           gfoot=slip_foot;
        else
           is_found=1;
           if leg_sel==0 ||leg_sel==2
             gfoot(1)=double(dis_min_id-MAP_H/2)*MAP_SIZE-MAP_SIZE/2;
           else
             gfoot(1)=double(dis_min_id-MAP_H/2)*MAP_SIZE+MAP_SIZE/2;
           end
           gfoot(2)=double(Gy_foot_now-MAP_W/2)*MAP_SIZE-MAP_SIZE/2;
        end
     end     
else%后方找离SLIP最近的点
    dis_min=int16(99);
    dis_min_id=99;    
     for i=1:good_foot_cnt3
        dis=abs(good_foot_id_b(i)-Gx_slip_now);
        if(dis<=dis_min)
            dis_min=dis;
            dis_min_id=good_foot_id_b(i);
        end
    end
    
    if dis_min_id<99%如果SLIP位置不在危险区域仍然采用
        if map_local_foot(Gx_slip_now,Gy_slip_now)>check_param(2) && map_local_foot(Gx_slip_now,Gy_slip_now)<check_param(3) &&1
           gfoot=slip_foot;
        else
           is_found=1;
           if leg_sel==0 ||leg_sel==2
             gfoot(1)=double(dis_min_id-MAP_H/2)*MAP_SIZE-MAP_SIZE/2;
           else
             gfoot(1)=double(dis_min_id-MAP_H/2)*MAP_SIZE+MAP_SIZE/2;
           end
               
           gfoot(2)=double(Gy_foot_now-MAP_W/2)*MAP_SIZE-MAP_SIZE/2;
        end
    else
        gfoot=slip_foot;
    end  
end

     if cluser>0 &&1 %梅花桩任务
        cube_center_dis=99;
        cube_closet_id=1;
        if 1
            for i=1:cluser
                x=double(centroids(i,2)-MAP_H/2)*MAP_SIZE-MAP_SIZE/2;
                y=double(centroids(i,1)-MAP_W/2)*MAP_SIZE-MAP_SIZE/2;
                temp=sqrt((slip_foot(2)-x)^2+(slip_foot(1)-y)^2);%slip坐标点
                if temp<cube_center_dis
                    cube_closet_id=i;
                    cube_center_dis=temp;
                end
            end      
            xg=int16(centroids(cube_closet_id,1));
            yg=int16(centroids(cube_closet_id,2));
            x=double(xg-MAP_H/2)*MAP_SIZE-MAP_SIZE/2;
            y=double(yg-MAP_W/2)*MAP_SIZE-MAP_SIZE/2;
            gfoott(1)=x;
            gfoott(2)=y;
            if spd_fb>=0%运动学校验
                if (xg>=Gx_foot_now && xg<=Hw_id_2+r_L_id_2)||(xg<=Gx_foot_now && xg>=Hw_id_2-r_L_id_2)
                    gfoot(1)=gfoott(1);
                    gfoot(2)=gfoott(2); 
                    gfoot(3)=0.12;
                    is_found=2;
                end     
            end
        end
     end 
end