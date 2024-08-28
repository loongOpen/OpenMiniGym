function fake_map_anal%输入一个局部高程图 采用GIS分析地图信息
clc;
clear all;
close all;

MAP_H=40;
MAP_W=40;
MAP_SIZE=0.02;
MAP=ones(MAP_H,MAP_W)*0.001;

%fileID = fopen('fake_map0.txt','w');
if(1)%台阶
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
    
    Trend_Mid=MAP_H/2+14;
    Trend_Wide=0.12;
    for  j = Trend_Mid-int16(Trend_Wide/MAP_SIZE)/2:Trend_Mid+int16(Trend_Wide/MAP_SIZE)/2% X
        for i = 20:MAP_W%Y
            MAP(i,j)=-0.025;
        end
    end
    
    Trend_Mid=MAP_H/2-14;
    Trend_Wide=0.12;
    for  j = Trend_Mid-int16(Trend_Wide/MAP_SIZE)/2:Trend_Mid+int16(Trend_Wide/MAP_SIZE)/2% X
        for i = 20:MAP_W%Y
            MAP(i,j)=-0.025;
        end
    end
end
% for i = 1:size(MAP,1)
%     for j = 1:size(MAP,2)
%         fprintf(fileID,'%.2f\t',MAP(i,j));
%     end
%     fprintf(fileID,'\n');
% end
% fclose(fileID);

figure(1)
X = linspace(0, MAP_H*MAP_SIZE, MAP_H);
Y = linspace(0, MAP_W*MAP_SIZE, MAP_W);
mesh(Y,X,MAP)
hold on;
plot3(0,0,0,'or')
xlabel('Y轴');
ylabel('X轴');
axis equal;

%GIS地图分析
 GIS(MAP,[MAP_H,MAP_W,MAP_SIZE]);
end

function [MAP_RP,MAP_D,R]=GIS(MAP,Param)
MAP_H=Param(1);
MAP_W=Param(2);
MAP_SIZE=Param(3);
ROW=MAP_H;
COL=MAP_W;
%地形起伏度RF======================================
mask_size=3;
MAP_RP=MAP-0.001;
for r = mask_size*2:ROW-mask_size
    for c = mask_size*2:COL-mask_size
         median3x3 =[MAP(r-mask_size,c-mask_size)      MAP(r-mask_size,c) MAP(r-mask_size,c+mask_size)
                     MAP(r,c-mask_size)                MAP(r,c)           MAP(r,c+mask_size)
                     MAP(r+mask_size,c-mask_size)      MAP(r+1,c)         MAP(r+mask_size,c+mask_size)];
         sort1 = sort([median3x3(1,1),median3x3(1,2),median3x3(1,3),median3x3(2,1),median3x3(2,2),median3x3(2,3),median3x3(3,1),median3x3(3,2),median3x3(3,3)],  'descend');
         MAP_RP(r,c)=MAP(r,c)-(sort1(9)-sort1(1)); 
    end
   
end
if 0
hold on;
X = linspace(0, MAP_H*MAP_SIZE, MAP_H);
Y = linspace(0, MAP_W*MAP_SIZE, MAP_W);
s = surf(Y,X,MAP_RP);
s.EdgeColor = 'none';
s.FaceColor = 'flat';
s.FaceAlpha = 0.4;
hold on;
end


%地表割裂深度D====================================
mask_size=3;
MAP_D=MAP-0.001;
for r = mask_size*2:ROW-mask_size
    for c = mask_size*2:COL-mask_size
         median3x3 =[MAP(r-mask_size,c-mask_size)      MAP(r-mask_size,c) MAP(r-mask_size,c+mask_size)
                     MAP(r,c-mask_size)                MAP(r,c)           MAP(r,c+mask_size)
                     MAP(r+mask_size,c-mask_size)      MAP(r+1,c)         MAP(r+mask_size,c+mask_size)];
         sort1 = sort([median3x3(1,1),median3x3(1,2),median3x3(1,3),median3x3(2,1),median3x3(2,2),median3x3(2,3),median3x3(3,1),median3x3(3,2),median3x3(3,3)],  'descend');
         mean=(median3x3(1,1)+median3x3(1,2)+median3x3(1,3)+median3x3(2,1)+median3x3(2,2)+median3x3(2,3)+median3x3(3,1)+median3x3(3,2)+median3x3(3,3))/9;
         MAP_D(r,c)=MAP(r,c)-(mean-sort1(9)); 
    end
   
end
if 1
hold on;
X = linspace(0, MAP_H*MAP_SIZE, MAP_H);
Y = linspace(0, MAP_W*MAP_SIZE, MAP_W);
s = surf(Y,X,MAP_D);
s.EdgeColor = 'none';
s.FaceColor = 'flat';
s.FaceAlpha = 0.4;
hold on;
end

%崎岖度R========================================
MAP_R=MAP-0.001;
%计算地表面积
[Fx,Fy]=gradient(MAP,0.004);%斜率
S=sqrt(1+Fx.^2+Fy.^2)*0.0004.*( ~isnan(MAP) ) ;
S_map=sum(S(~isnan(S)));
%计算投影面积
S_norm=MAP_H*MAP_W*MAP_SIZE*MAP_SIZE;
R=S_map/S_norm;
a=get(gca);
x=a.XLim;%获取横坐标上下限
y=a.YLim;%获取纵坐标上下限
k=[0.5 1.7];%给定text相对位置
x0=x(1)+k(1)*(x(2)-x(1));%获取text横坐标
y0=y(1)+k(2)*(y(2)-y(1));%获取text纵坐标
mean_h=sum(MAP(~isnan(MAP)))/MAP_H/MAP_W;
if 1
str=sprintf('R= %4.2f',R);   % 2位小数的浮点型text
text(x0,y0,str,'FontSize',16);
str1=sprintf('Mean= %4.4f',mean_h);   % 2位小数的浮点型text
text(x0-0.08,y0-0.08,str1,'FontSize',16);
end


%高程异变系数V====================================
mask_size=1;
MAP_V=MAP-0.001;
for r = mask_size*2:ROW-mask_size
    for c = mask_size*2:COL-mask_size
         median3x3 =[MAP(r-mask_size,c-mask_size)      MAP(r-mask_size,c) MAP(r-mask_size,c+mask_size)
                     MAP(r,c-mask_size)                MAP(r,c)           MAP(r,c+mask_size)
                     MAP(r+mask_size,c-mask_size)      MAP(r+1,c)         MAP(r+mask_size,c+mask_size)];
         z_mean=(median3x3(1,1)+median3x3(1,2)+median3x3(1,3)+median3x3(2,1)+median3x3(2,2)+median3x3(2,3)+median3x3(3,1)+median3x3(3,2)+median3x3(3,3))/9;
         sums=0;
         for i=1:9
            sums=(median3x3(i)-z_mean)^2+sums;
         end
         s=(1/9*(sums))^0.5;
         V=s/z_mean;
         MAP_V(r,c)=MAP(r,c)-V/8; 
    end
   
end
if 0
hold on;
X = linspace(0, MAP_H*MAP_SIZE, MAP_H);
Y = linspace(0, MAP_W*MAP_SIZE, MAP_W);
s = surf(Y,X,MAP_V);
s.EdgeColor = 'none';
s.FaceColor = 'r';
s.FaceAlpha = 0.2;
hold on;
end


%坡形P=======================================
MAP_P=MAP-0.001;
%计算地表面积
[Fx,Fy]=gradient(MAP,0.004);%斜率
S=sqrt(1+Fx.^2+Fy.^2)*0.004.*( ~isnan(MAP) ) ;
%计算投影面积
S_norm=MAP_H*MAP_W*MAP_SIZE*MAP_SIZE;
R=S_map/S_norm;
a=get(gca);
x=a.XLim;%获取横坐标上下限
y=a.YLim;%获取纵坐标上下限
k=[0.1 1.3];%给定text相对位置
x0=x(1)+k(1)*(x(2)-x(1));%获取text横坐标
y0=y(1)+k(2)*(y(2)-y(1));%获取text纵坐标
str=sprintf('R= %4.2f',R);   % 2位小数的浮点型text

if 0
hold on;
X = linspace(0, MAP_H*MAP_SIZE, MAP_H);
Y = linspace(0, MAP_W*MAP_SIZE, MAP_W);
s = surf(Y,X,Fx*0.004);
s.EdgeColor = 'none';
s.FaceColor = 'b';
s.FaceAlpha = 0.45;
hold on;
hold on;
X = linspace(0, MAP_H*MAP_SIZE, MAP_H);
Y = linspace(0, MAP_W*MAP_SIZE, MAP_W);
s = surf(Y,X,Fy*0.004);
s.EdgeColor = 'none';
s.FaceColor = 'r';
s.FaceAlpha = 0.45;
hold on;
end
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
