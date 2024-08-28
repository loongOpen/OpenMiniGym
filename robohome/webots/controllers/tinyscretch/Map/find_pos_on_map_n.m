function [Gx,Gy]=find_pos_on_map_n(pos,map_param)%查找一个坐标在全局地图上的位置
MAP_W_N=map_param(4);
MAP_H_N=map_param(3);
MAP_SIZE=map_param(5);
Gx=int16(pos(1)/MAP_SIZE)+int16(mod(pos(1),MAP_SIZE)) ;
Gy=int16(pos(2)/MAP_SIZE)+int16(mod(pos(2),MAP_SIZE)) ;
Gx=limit(Gx,1,MAP_H_N);
Gy=limit(Gy,1,MAP_W_N);
Gx=(Gx);
Gy=(Gy);
end