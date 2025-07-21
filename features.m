% function [f]= features(robot,goal,laser,a)
% % laser: 5x1, 每個角度掃到的障礙物距離,如果沒有就是100
% % f1(S): constant.
% % f2(S): d: the distance between the robot and goal
% % f3(S): ϕ: the angle between the robot and goal
% % f4(S, a): the laser data (dis) from direction a1 to a5. for example, if a=a1, the robot uses the
% % laser data from −30◦ (方向a角度跟障礙物的距離)
% % if dis ≤ 70, f4 = f3
% % otherwise, f4 = −f3
% 
% f2 = sqrt((robot.x - goal.x)^2 + (robot.y - goal.y)^2);
% f3 = atan2(goal.y - robot.y, goal.x - robot.x) -robot.t; 
% f3 = wrapToPi(f3);
% if (laser(a) <= 70)
%     f4 = f3;
% else 
%     f4 = -f3;
% end
% f = [1; f2/100; f3; f4]; % 因為距離跟其他feature比起來值大太多了都它在主導所以把他除100
% 
% % 都是radian
% function angle = wrapToPi(angle)
%     % 將角度用到 [-pi, pi] 跟之前的Generalize_z一樣
%     angle = mod(angle + pi, 2*pi) - pi;
% end
% 
% end
%--------------------------------------------------------------------------

function [f] = features(robot, goal, laser, a)
    % 計算到目標的距離
    dist_to_goal = sqrt((robot.x - goal.x)^2 + (robot.y - goal.y)^2);

    % 計算到目標的角度差
    angle_to_goal = atan2(goal.y - robot.y, goal.x - robot.x) - robot.t;
    angle_to_goal = wrapToPi(angle_to_goal);

    % 獲取當前動作的轉向角度
    degrees = [-30, -15, 0, 15, 30];
    action_angle = degrees(a) * (pi / 180);

    % 計算動作角度與目標角度的匹配度
    angle_match = cos(angle_to_goal - action_angle);
    if (laser(a) <= 70)
        f4 = -angle_to_goal;
    else 
        f4 = angle_to_goal;
    end

    % 障礙物接近度
    obstacle_proximity = min(laser)/100;

    % 動作特定特徵 - 根據不同動作角度與目標方向的匹配度
    action_value = 0;
    if abs(angle_to_goal) < 0.5  % 如果目標基本在前方
        % 對於接近直線行駛的動作(a=3)给更高權重
        if a == 3
            action_value = 1;
        else
            action_value = 0.5;
        end
    elseif angle_to_goal > 0  % 目標在左侧
        % 對左轉動作(a=4,5)给予更高權重
        if a >= 4
            action_value = 0.8;
        end
    else  % 目標在右側
        % 對右轉動作(a=1,2)给更高權重
        if a <= 2
            action_value = 0.8;
        end
    end
    % 正前方障礙物的強懲罰項
    front_laser = laser(a);  % 正前方雷射
    f_obstacle = -1.7 / (front_laser / 100 + 0.1);  % 雷射越短 → 懲罰越大


    % 返回特徵向量
    f = [
        1;  % 常數特徵
        1/(dist_to_goal/100 + 0.1);  % 距離特徵（使用倒數）
        angle_to_goal;  % 動作方向與目標方向的匹配度 angle_match
        f4; % 
        -1/(obstacle_proximity + 0.1);  % 障礙物接近度 %-0.1*obstacle_proximity;
        action_value;  % 動作特定價值
        f_obstacle;
    ];
end

%---------------------------------------------------------------------------

