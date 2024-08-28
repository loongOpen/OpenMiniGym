function [location_map,map_edge_out]=foot_sel_on_map(map_local,map_param,check_param)%输入高程图 输出安全落足区域 和 梅花桩位置  导出库用
GRID_INF=-1;
MAP_H=80;
MAP_W=80;

ROW=map_param(1);
COL=map_param(2);

MAP=ones(MAP_H,MAP_W)*GRID_INF;
MAP_BIT=zeros(MAP_H,MAP_W);

location_map=zeros(MAP_H,MAP_W);
map_edge_out=zeros(MAP_H,MAP_W);

safe_bind=check_param(1);%安全落足区域
h_safe_min=check_param(2);
h_safe_max=check_param(3);
for i=1:MAP_H%获取全局地图
   for j=1:MAP_W
      if(map_local(i,j)<h_safe_max &&map_local(i,j)>h_safe_min)
        MAP(i,j)=map_local(i,j);
        MAP_BIT(i,j)=1;%构建二值图
      end
   end
end

% %中值滤波
% map_mid = MAP_BIT;
% for r = 2:ROW-1
%    for c = 2:COL-1
%          median3x3 =[MAP_BIT(r-1,c-1)      MAP_BIT(r-1,c) MAP_BIT(r-1,c+1)
%                      MAP_BIT(r,c-1)        MAP_BIT(r,c)   MAP_BIT(r,c+1)
%                      MAP_BIT(r+1,c-1)      MAP_BIT(r+1,c) MAP_BIT(r+1,c+1)];
%          sort1 = sort(median3x3, 2, 'descend');
%          sort2 = sort([sort1(1), sort1(4), sort1(7)], 'descend');
%          sort3 = sort([sort1(2), sort1(5), sort1(8)], 'descend');
%          sort4 = sort([sort1(3), sort1(6), sort1(9)], 'descend');
%          mid_num = sort([sort2(3), sort3(2), sort4(1)], 'descend');
%          map_mid(r,c) = mid_num(2);
%      end
% end

%边沿检测 不需要满足高度要求
%Median_Img = double(map_mid);
Median_Img =map_local;
Sobel_Threshold = 0.01;
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
if (safe_bind>=1)
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

if (safe_bind>=2)
    erosion_img1 = zeros(ROW,COL);
    for r = 2:ROW-1
        for c = 2:COL-1
            and1 = bitand(erosion_img(r-1, c-1), bitand(erosion_img(r-1, c), erosion_img(r-1, c+1)));
            and2 = bitand(erosion_img(r, c-1),  bitand(erosion_img(r, c), erosion_img(r, c+1)));
            and3 = bitand(erosion_img(r+1, c-1), bitand(erosion_img(r+1, c), erosion_img(r+1, c+1)));
            erosion_img1(r, c) = bitand(and1, bitand(and2, and3));
        end
    end
    erosion_img = erosion_img1;
end

if (safe_bind>=3)
    erosion_img2 = zeros(ROW,COL);
    for r = 2:ROW-1
        for c = 2:COL-1
            and1 = bitand(erosion_img(r-1, c-1), bitand(erosion_img(r-1, c), erosion_img(r-1, c+1)));
            and2 = bitand(erosion_img(r, c-1),  bitand(erosion_img(r, c), erosion_img(r, c+1)));
            and3 = bitand(erosion_img(r+1, c-1), bitand(erosion_img(r+1, c), erosion_img(r+1, c+1)));
            erosion_img2(r, c) = bitand(and1, bitand(and2, and3));
        end
    end
    erosion_img = erosion_img2;
end

if (safe_bind>=4)
    erosion_img3 = zeros(ROW,COL);
    for r = 2:ROW-1
        for c = 2:COL-1
            and1 = bitand(erosion_img(r-1, c-1), bitand(erosion_img(r-1, c), erosion_img(r-1, c+1)));
            and2 = bitand(erosion_img(r, c-1),  bitand(erosion_img(r, c), erosion_img(r, c+1)));
            and3 = bitand(erosion_img(r+1, c-1), bitand(erosion_img(r+1, c), erosion_img(r+1, c+1)));
            erosion_img3(r, c) = bitand(and1, bitand(and2, and3));
        end
    end
    erosion_img = erosion_img3;
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
 Dilation_img=erosion_img;
%输出高程图
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

for r = 1:ROW
   for c = 1:COL
       location_map(r,c)=MAP(r,c);
       map_edge_out(r,c)=MAP_E(r,c)+0.005;
  end
end

end


function [grid_height]=find_3x3_height_on_grid(grid,map_local,map_param)%查找一个坐标在全局地图上的位置
MAP_H=map_param(1);
MAP_W=map_param(2);
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

