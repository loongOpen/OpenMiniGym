function [Gx,Gy]=find_pos_on_map_local(pos,map_param)%查找一个坐标在全局地图上的位置
MAP_W=40;%map_param(1);
MAP_H=40;%map_param(2);
MAP_SIZE=map_param(5);
Gx=int16(pos(1)/MAP_SIZE)+int16(mod(pos(1),MAP_SIZE))+ MAP_H/2;
Gy=int16(pos(2)/MAP_SIZE)+int16(mod(pos(2),MAP_SIZE))+ MAP_W/2 ;
Gx=limit_i(Gx,1,MAP_H);
Gy=limit_i(Gy,1,MAP_W);
Gx=(Gx);
Gy=(Gy);
end
