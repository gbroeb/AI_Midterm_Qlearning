function [opt_a, Wt, J] = Q_learning(a, W, robot,robot_, goal, laser, R, terminal, alpha)

% Q-learning Hyperparameters
% alpha = 0.27   
gamma = 0.9;    
epsilon = 0.1; % epsilon-greedy

dt = 0.1;       
CoverMODE = 2;  
M = 1; m = 1; Eff_robot = 1; % GetLaser參數
num_actions = 5;

% 計算目前狀態的Feature和Q值
f = features(robot, goal, laser, a); %wait to check
Q_current = W' * f;

% 計算 Q_target 

%if ~terminal
Q_values = zeros(num_actions, 1);
for i = 1:num_actions
    next_a = i;

%     % 模擬如果做next_a，robot會到哪
%     temp_robot = motion_model(robot_, next_a, dt);
% 
%     % 在新位置打laser
%     temp_robot_data = [temp_robot.x, temp_robot.y, temp_robot.t];
    temp_laser = GetLaser(robot_.x, robot_.y, robot_.t, Eff_robot, 1, CoverMODE, 0);

    % 新位置的特徵
    next_f = features(robot_, goal, temp_laser, next_a);

    % 新位置的Q
    Q_values(i) = W' * next_f;
end
max_Q_next = max(Q_values);

% Q_target
Q_target = R + gamma * max_Q_next;
% else
%     % 如果terminal，只有 R
%     Q_target = R;
% end

% 更新權重Weighting
delta = Q_target - Q_current;
Wt = W + alpha * delta * f;
% 如果有一堆一樣大的 max Q，隨機選一個
random_explore = rand;
fprintf('random_explore: %f\n', random_explore);
if random_explore < epsilon
    opt_a = randi(num_actions);  % 隨機探索
    disp('隨機選取')
else
    max_val = max(Q_values);
    max_indices = find(Q_values == max_val);
    opt_a = max_indices(randi(length(max_indices)));
    if (length(max_indices) > 1)
        disp('隨機選取')
    else
        disp('選最大的')
    end
end

% Cost function
J = 0.5 * (Q_target - Q_current)^2;

end
