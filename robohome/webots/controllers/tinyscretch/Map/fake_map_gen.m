function fake_map_gen%生成离线地图
clc;
clear all;
close all;

MAP_H=40;
MAP_W=40;
MAP_SIZE=0.02;

%----------------------
MAP=ones(MAP_H,MAP_W)*0.001;
fileID = fopen('fake_map0.txt','w');
if 1
    Trend_Mid=MAP_H/2;
    Trend_Wide=0.03;
    for  i = Trend_Mid-int16(Trend_Wide/MAP_SIZE)/2:Trend_Mid+int16(Trend_Wide/MAP_SIZE)/2% W->Y
        for j = 1:size(MAP,1)%H->X
            MAP(i,j)=-0.2;
        end
    end

    Trend_Mid=MAP_H/2-15;
    Trend_Wide=0.03;
    for  i = Trend_Mid-int16(Trend_Wide/MAP_SIZE)/2:Trend_Mid+int16(Trend_Wide/MAP_SIZE)/2% W->Y
        for j = 1:size(MAP,1)%H->X
            MAP(i,j)=-0.2;
        end
    end


    Trend_Mid=MAP_H/2+10;
    Trend_Wide=0.03;
    for  i = Trend_Mid-int16(Trend_Wide/MAP_SIZE)/2:Trend_Mid+int16(Trend_Wide/MAP_SIZE)/2% W->Y
        for j = 1:size(MAP,1)%H->X
            MAP(i,j)=-0.2;
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
%------------------------------------------------
MAP=ones(MAP_H,MAP_W)*0.001;
fileID = fopen('fake_map1.txt','w');
if 1
    Trend_Mid=MAP_H/2;
    Trend_Wide=0.03;
    for  i = Trend_Mid-15:MAP_H% W->Y
        for j = 1:MAP_W%H->X
            MAP(i,j)=0.06;
        end
    end
    
    for  i = Trend_Mid+10-15:MAP_H% W->Y
        for j = 1:MAP_W%H->X
            MAP(i,j)=0.125;
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

figure(2)
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
