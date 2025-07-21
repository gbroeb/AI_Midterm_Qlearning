function [map,dis]= PlotLine(robot,sensor,map,MODE)

N=size(map,1);
dis=100*ones(5,1);  % 第dtheta條雷射偵測到第一個障礙物時的距離（單位是格數）, 如果一直走都沒遇到障礙物，就維持初始的 100

% plot a line
 % add angle resolution?
for dtheta=1:sensor.scan
    theta=-0.0174*36+(0.0174*sensor.phi/sensor.scan)*dtheta;

    r=sensor.r;
    resolution=sensor.r;
    dx=r*cos(robot.theta+theta)/resolution;
    dy=r*sin(robot.theta+theta)/resolution;
    for i=1:resolution  % i 代表「沿著那個方向移動了幾格」
    di=fix(robot.x+dx*i);
    dj=fix(robot.y+dy*i);
        if ((di>0) && (di<N)) && ((dj>0) && (dj<N)) && (map(di,dj)==2) % obstacles inside, 如果遇到障礙物 (map(di,dj) == 2)：
        dis(dtheta)=i;                                                 % 一旦遇到障礙物 (map(di,dj)==2)，就把當時的 i（掃到的距離格數）存到 dis(dtheta)。
        break;
        end
        if ((di>0) && (di<N)) && ((dj>0) && (dj<N))
            if map(di,dj)==3
                if MODE==1
                    if MODE==1
                    break;
                    elseif MODE==3
                    end
                elseif MODE==2
                map(di,dj)=4; % overlapping cells
                end
            elseif map(di,dj)==0
            map(di,dj)=1; % scanned cell
            end
        end
    end
end

for i=1:N
    for j=1:N
       if map(i,j)==1
       map(i,j)=3;  
       end
    end
end

%dis
