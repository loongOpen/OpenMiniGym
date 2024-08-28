function fake_map_gen_guass%生成力觉感知后的猜测地图
clc;
clear all;
close all;

MAP_H=100;
MAP_W=100;
MAP_SIZE=0.02;
%------------------------------------------------
MAP=ones(MAP_H,MAP_W)*0.001;
fileID = fopen('g_map_t_15cm.txt','w');
if 1
    Trend_Mid=MAP_H/2;
    for  i = Trend_Mid:MAP_H% W->Y
        for j = 1:MAP_W%H->X
            MAP(i,j)=0.2;
        end
    end
end

for i = 1:size(MAP,1)
    for j = 1:size(MAP,2)
        fprintf(fileID,'%.2f\t',MAP(i,j));
    end
    fprintf(fileID,'\n');
end
fclose(fileID);

figure(1)
X = linspace(0, MAP_H*MAP_SIZE, MAP_H);
Y = linspace(0, MAP_W*MAP_SIZE, MAP_W);
mesh(Y,X,MAP)
hold on;
plot3(0,0,0,'or')
xlabel('Y轴');
ylabel('X轴');
axis equal;
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
