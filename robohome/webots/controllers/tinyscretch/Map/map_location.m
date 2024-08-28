function [gfoot]=map_location(cog_n,map_local,map_n,map_param,check_param,robot_param)%落足点选择

        [map_local_foot,map_bit,map_mid,sobel_Img,good_Img,erosion_img,map_edge]=draw_local_map_good_location(cog_n,map_local,map_param,check_param);%绘制可落足地图 在本函数下部
        
        [centroids,cluser]=draw_local_map_good_location_cube(map_local_foot,map_param,check_param);%梅花桩聚类
        %落足点选择
        spd_tar=0.15;
        T_gait=0.3;
        spd_yaw=0;
        x_spd=sind(spd_yaw)*spd_tar;
        y_spd=cosd(spd_yaw)*spd_tar;

        spd_fb=1;%速度方向局部坐标系 前后
        plot3([cog_n(2),cog_n(2)+x_spd],[cog_n(1),cog_n(1)+y_spd],[0.2,0.2],'-y','LineWidth',3);
        hold on;
        
        MAP_H=40;%map_param(1);
        MAP_W=40;%map_param(2);
        MAP_SIZE=map_param(5);
        for i=1:cluser%绘制梅花桩
            x=double(centroids(i,2)-MAP_H/2)*MAP_SIZE-MAP_SIZE/2;
            y=double(centroids(i,1)-MAP_W/2)*MAP_SIZE-MAP_SIZE/2;
            plot3([x+cog_n(2)],[y+cog_n(1)],[0.2],'+k','LineWidth',1); 
        end
        %绘制SLIP落足
        for i=0:3
            if i==0
            anlke_pos_local=[robot_param(1)/2,robot_param(2)/2,0];
            elseif i==1
            anlke_pos_local=[-robot_param(1)/2,robot_param(2)/2,0];
            elseif i==2
            anlke_pos_local=[robot_param(1)/2,-robot_param(2)/2,0];
            elseif i==3
            anlke_pos_local=[-robot_param(1)/2,-robot_param(2)/2,0];
            end          
            slip_foot=[T_gait/2*x_spd,T_gait/2*y_spd,0.05]+anlke_pos_local;%倒立摆
            now_foot=slip_foot;
            [Gx_n,Gy_n]=find_pos_on_map_n([slip_foot(1)+cog_n(1),slip_foot(2)+cog_n(2)],map_param);
            slip_z=map_n(Gx_n,Gy_n)+0.005;
            plot3([slip_foot(2)+cog_n(2)],[slip_foot(1)+cog_n(1)],[slip_z],'*m','LineWidth',3);
            hold on;
            [gfoot,is_found]=foot_good_foot_found(i,spd_fb,now_foot,slip_foot,centroids,cluser,map_local_foot,map_param,check_param,robot_param);
            hold on;
            if(1)%绘制视觉落足
               [Gx_n,Gy_n]=find_pos_on_map_n([gfoot(1)+cog_n(1),gfoot(2)+cog_n(2)],map_param);
               vision_z=map_n(Gx_n,Gy_n)+0.005;
               if(is_found==1)%绘制视觉落足
                    plot3([gfoot(2)+cog_n(2)],[gfoot(1)+cog_n(1)],[vision_z],'*b','LineWidth',3); 
               elseif(is_found==2)%绘制Cube
                    plot3([gfoot(2)+cog_n(2)],[gfoot(1)+cog_n(1)],[vision_z],'+y','LineWidth',1.5);      
               else
                    plot3([gfoot(2)+cog_n(2)],[gfoot(1)+cog_n(1)],[vision_z],'*r','LineWidth',3); 
               end
            end  
        end
        
        if 0%绘制各阶段处理结果
            figure(4)
            clf; 
            subplot(2,3,1); 
            hold on;
            imshow(map_bit);
            xlabel('map_bit');
            subplot(2,3,2);
            hold on;
            imshow(map_mid);
            xlabel('map_mid');
            subplot(2,3,3);
            hold on;
            imshow(sobel_Img);
            xlabel('sobel_Img');
            subplot(2,3,4);
            hold on;
            imshow(good_Img);
            xlabel('good_Img');
            subplot(2,3,5);
            hold on;
            imshow(erosion_img);
            xlabel('erosion_img');
            subplot(2,3,6);
            hold on;
%             imshow(map_local_foot_cube);
%             xlabel('map_local_foot_cube');
        end
        
end


function [map_local_foot,map_bit,map_mid,sobel_Img,good_Img,erosion_img,map_edge]=draw_local_map_good_location(cog_n,map_local,map_param,check_param)%提取安全落足区域
GRID_INF=-0.1;
MAP_W=map_param(2);
MAP_H=map_param(1);
MAP_SIZE=map_param(5);
MAP=ones(MAP_H,MAP_W)*-0.1;
MAP_BIT=ones(MAP_H,MAP_W)*0;
safe_bind=check_param(1);%安全落足区域
h_safe_min=check_param(2);
h_safe_max=check_param(3);
for i=1:MAP_H%获取全局地图
   for j=1:MAP_W
      if(map_local(i,j)<h_safe_max &&map_local(i,j)>h_safe_min)
        MAP(i,j)=map_local(i,j)+0.002;
        MAP_BIT(i,j)=1;%构建二值图
      end
   end
end

%中值滤波
[ROW,COL, ~] = size(MAP_BIT); 
map_mid = MAP_BIT;
for r = 2:ROW-1
   for c = 2:COL-1
         median3x3 =[MAP_BIT(r-1,c-1)      MAP_BIT(r-1,c) MAP_BIT(r-1,c+1)
                     MAP_BIT(r,c-1)        MAP_BIT(r,c)   MAP_BIT(r,c+1)
                     MAP_BIT(r+1,c-1)      MAP_BIT(r+1,c) MAP_BIT(r+1,c+1)];
         sort1 = sort(median3x3, 2, 'descend');
         sort2 = sort([sort1(1), sort1(4), sort1(7)], 'descend');
         sort3 = sort([sort1(2), sort1(5), sort1(8)], 'descend');
         sort4 = sort([sort1(3), sort1(6), sort1(9)], 'descend');
         mid_num = sort([sort2(3), sort3(2), sort4(1)], 'descend');
         map_mid(r,c) = mid_num(2);
     end
end

%边沿检测 不需要满足高度要求
%Median_Img = double(map_mid);
Median_Img =map_local;
Sobel_Threshold = 0.35;
sobel_Img = zeros(ROW,COL);
for r = 2:ROW-1
   for c = 2:COL-1
        Sobel_x =  Median_Img(r-1,c+1) + 2*Median_Img(r,c+1) + Median_Img(r+1,c+1) - Median_Img(r-1,c-1) - 2*Median_Img(r,c-1) - Median_Img(r+1,c-1);
        Sobel_y = Median_Img(r-1,c-1) + 2*Median_Img(r-1,c) + Median_Img(r-1,c+1) - Median_Img(r+1,c-1) - 2*Median_Img(r+1,c) - Median_Img(r+1,c+1);
        %Sobel_Num = abs(Sobel_x) + abs(Sobel_y);
        Sobel_Num = sqrt(Sobel_x^2 + Sobel_y^2);
       if(Sobel_Num > Sobel_Threshold)
            sobel_Img(r,c)=1;
       else
            sobel_Img(r,c)=0;
       end
  end
end

%去除边沿
good_Img= zeros(ROW,COL);%sobel_Img*map_mid;
for r = 1:ROW
   for c = 1:COL
       if(sobel_Img(r,c) <0.5 && MAP_BIT(r,c)>0.5)
            good_Img(r,c)=1;
       else
            good_Img(r,c)=0;
       end
  end
end


%膨胀安全区域
if (safe_bind)
    erosion_img = zeros(ROW,COL);
    for r = 2:ROW-1
        for c = 2:COL-1
            and1 = bitand(good_Img(r-1, c-1), bitand(good_Img(r-1, c), good_Img(r-1, c+1)));
            and2 = bitand(good_Img(r, c-1),  bitand(good_Img(r, c), good_Img(r, c+1)));
            and3 = bitand(good_Img(r+1, c-1), bitand(good_Img(r+1, c), good_Img(r+1, c+1)));
            erosion_img(r, c) = bitand(and1, bitand(and2, and3));
        end
    end
else
    erosion_img=good_Img;
end

Dilation_img = zeros(ROW,COL);
for r = 2:ROW-1
    for c = 2:COL-1
        or1 = bitor(erosion_img(r-1, c-1), bitor(erosion_img(r-1, c), erosion_img(r-1, c+1)));
        or2 = bitor(erosion_img(r, c-1), bitor(erosion_img(r, c), erosion_img(r, c+1)));
        or3 = bitor(erosion_img(r+1, c-1), bitor(erosion_img(r+1, c), erosion_img(r+1, c+1)));
        Dilation_img(r, c) = bitor(or1, bitor(or2, or3));
    end
 end
%erosion_img = ones(ROW,COL);
%输出最终的高程图
for r = 1:ROW
   for c = 1:COL
       if(Dilation_img(r,c) <0.5)
            MAP(r,c)=GRID_INF;
       end
  end
end
%输出边沿图
MAP_E=ones(MAP_H,MAP_W)*GRID_INF;
for r = 1:ROW
   for c = 1:COL
       if(sobel_Img(r,c) >0.5)
            MAP_E(r,c)=find_3x3_height_on_grid([r,c],map_local,map_param);
       end
  end
end
X = linspace(0, MAP_H*MAP_SIZE, MAP_H)+cog_n(1)-MAP_H*MAP_SIZE/2;
Y = linspace(0, MAP_W*MAP_SIZE, MAP_W)+cog_n(2)-MAP_W*MAP_SIZE/2;

s = surf(Y,X,MAP_E);
s.EdgeColor = 'interp';
s.FaceColor = 'r';
s.FaceAlpha = 1;
hold on;
s = surf(Y,X,MAP);
s.EdgeColor = 'interp';
s.FaceColor = 'b';
s.FaceAlpha = 0.5;
%s.EdgeColor = 'none';

map_local_foot=MAP;
map_edge=MAP_E;
map_bit=MAP_BIT;
hold on;
end


function [grid_height]=find_3x3_height_on_grid(grid,map_local,map_param)%查找一个坐标在全局地图上的位置
MAP_H=map_param(1);
MAP_W=map_param(2);
MAP_SIZE=map_param(5);
height=zeros(5);
height(1)=map_local(limit(grid(1),1,MAP_H),limit(grid(2),1,MAP_W));
height(2)=map_local(limit(grid(1)+1,1,MAP_H),limit(grid(2),1,MAP_W));
height(3)=map_local(limit(grid(1)-1,1,MAP_H),limit(grid(2),1,MAP_W));
height(4)=map_local(limit(grid(1),1,MAP_H),limit(grid(2)+1,1,MAP_W));
height(5)=map_local(limit(grid(1),1,MAP_H),limit(grid(2)-1,1,MAP_W));
grid_height=height(1);
for i=1:5
    if height(i)>grid_height
        grid_height=height(i);
    end
end
end

