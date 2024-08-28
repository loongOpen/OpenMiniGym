
function [centroids,num]=draw_local_map_good_location_cube(map_local,map_param,check_param)%梅花桩下提取块中心
    MAP_H=40;
    MAP_W=40;
    MAP_BIT=ones(MAP_H,MAP_W)*0;
    h_safe_min=check_param(2);
    h_safe_max=check_param(3);
    for i=1:MAP_H%获取全局地图 高度限制
       for j=1:MAP_W
          if(map_local(i,j)<h_safe_max &&map_local(i,j)>h_safe_min)
            MAP_BIT(i,j)=1;%构建二值图
          end
       end
    end
    %扣取包络
    openbw=MAP_BIT;
    [L,num] = bwlabel(openbw,4);%Cluster

    centroids=zeros(20,2);
    for i=1:num
        X_c=0;
        Y_c=0;
        Cnt=0;
        for xs=1:MAP_H
            for ys=1:MAP_W
                if(L(xs,ys)==i)
                    X_c=X_c+xs;
                    Y_c=Y_c+ys;
                    Cnt=Cnt+1;
                end
            end   
        end
        centroids(i,:)=[X_c/Cnt,Y_c/Cnt];
    end
end
