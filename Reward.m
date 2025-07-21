% function [R,Terminal]= Reward(robot,a,goal,obs)
% 
% %% WRITE YOUR CODE HERE
% R=-0.05;
% Terminal=0;
% %----------------------------------------------------------------------------
% % 撞到牆壁 {0~300,0~300} D_RB<5就撞到
% if (robot.x<=5) || (robot.x>=295) || (robot.y<=5) || (robot.y>=295)
%     Terminal = 1;
%     R = -1*10;    
%     return;
% end
% % 撞到障礙物{150,150} D_RO<10就撞到
% if (sqrt((robot.x - obs.x)^2 + (robot.y - obs.y)^2) <= 10)
%     Terminal = 1;
%     R = -1*10;
%     return;
% end
% % 到達goal{150,250} D_RG<10就達到
% if (sqrt((robot.x - goal.x)^2 + (robot.y - goal.y)^2) <= 10)
%     Terminal = 1;
%     R = 1*10;
%     return;
% end
% 

function [R, Terminal] = Reward(robot, a, goal, obs)
    % 基本步驟懲罰
    R = -0.1;
    Terminal = 0;
    
    % 邊界檢查
    if (robot.x<=5) || (robot.x>=295) || (robot.y<=5) || (robot.y>=295)
        Terminal = 1;
        R = -10;
        return;
    end
    
    % 障礙物檢查
    if (sqrt((robot.x - obs.x)^2 + (robot.y - obs.y)^2) <= 10)
        Terminal = 1;
        R = -10;
        return;
    end
    
    % 目標檢查
    if (sqrt((robot.x - goal.x)^2 + (robot.y - goal.y)^2) <= 10)
        Terminal = 1;
        R = 10;
        return;
    end
    % 接近障礙但尚未撞到 → 提前給懲罰
    laser = GetLaser(robot.x, robot.y, robot.t, 1, 1, 2, 0);  % 取得雷射資訊（關鍵）
    min_dis = min(laser);
    
    if min_dis < 15
        R = R - 2;  % 非常接近障礙物
    elseif min_dis < 30
        R = R - 1;  % 有點近
    end

    
    % 額外獎勵 - 朝向目標的進展
    angle_to_goal = atan2(goal.y - robot.y, goal.x - robot.x) - robot.t;
    angle_to_goal = wrapToPi(angle_to_goal);
    
    % 獲取動做的轉向角度
    degrees = [-30, -15, 0, 15, 30];
    action_angle = degrees(a) * (pi / 180);
    
    % 如果動作角度接近目標方向，给予獎勵
    angle_diff = abs(angle_to_goal - action_angle);
    angle_diff = min(angle_diff, 2*pi - angle_diff);
    if angle_diff < 0.3
        R = R + 0.2; 
    end
    
    % 距離改善獎勵 
    persistent prev_dist;
    persistent last_robot_x;
    persistent last_robot_y;
    
    curr_dist = sqrt((robot.x - goal.x)^2 + (robot.y - goal.y)^2);
    
    % 檢查是否是新的回合開始或首次調用
    if isempty(prev_dist) || isempty(last_robot_x) || ...
       abs(robot.x - last_robot_x) > 20 || abs(robot.y - last_robot_y) > 20
        % 重置記憶值（新回合或首次调用）
        prev_dist = curr_dist;
    else
        % 限制獎勵大小，防止無限增長
        dist_improvement = prev_dist - curr_dist;
        if dist_improvement > 0
            R = R + min(0.1 * dist_improvement, 0.5); % 設置上限為0.5
        end
    end
    
    % 更新記憶值
    prev_dist = curr_dist;
    last_robot_x = robot.x;
    last_robot_y = robot.y;
end