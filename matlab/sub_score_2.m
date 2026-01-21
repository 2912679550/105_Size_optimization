%% 基于score1的方案, 将惩罚改为直线MN与圆的交点到线段MN中点的距离
function [optimal_theta, min_loss] = sub_score_2(X , R_ , figID)
    % 优化参数配置 (已注释，改为使用 PSO)
    options = optimoptions('fmincon', ...
        'Algorithm', 'interior-point', ...       % 序列二次规划算法 可选参数:  sqp interior-point
        'Display', 'off', ... % 显示详细迭代信息 iter, iter-detailed, notify, off
        'MaxIterations', 6, ... % 最大迭代次数
        'StepTolerance', 1e-3, ... % 步长容限
        'ConstraintTolerance', 1e-3);
    
    
    % 初始值
    initial_theta = [deg2rad(30), deg2rad(153)]; % [theta1, theta2]
    % 变量边界
    lb= [deg2rad(-10) , deg2rad(50)];
    ub = [deg2rad(90), deg2rad(180)];
    R = R_; % pipe radius
    
    [theta_temp, ~] = fmincon(@(theta) lossFunction2(theta, X, R , figID , 0), ...
        initial_theta, [], [], [], [], lb, ub, @(theta) nonlinConstraints(theta, X, R), options);
        
    % % 第一步：使用粒子群算法进行全局搜索 (不带非线性约束，仅边界)
    % % 粒子群优化 (PSO) 参数配置 - 用于全局搜索初步寻优
    % pso_options = optimoptions('particleswarm', ...
    %     'SwarmSize', 20, ...
    %     'MaxIterations', 10, ...
    %     'Display', 'none'); 
    % [theta_temp, ~] = particleswarm(@(theta) lossFunction2(theta, X, R , figID , 0), 2, lb, ub, pso_options);
    
    options = optimoptions('fmincon', ...
        'Algorithm', 'sqp', ...       % 序列二次规划算法 可选参数:  sqp interior-point
        'Display', 'off', ... % 显示详细迭代信息 iter, iter-detailed, notify, off
        'MaxIterations', 15, ... % 最大迭代次数
        'StepTolerance', 1e-3, ... % 步长容限
        'ConstraintTolerance', 1e-3);
    [optimal_theta, min_loss] = fmincon(@(theta) lossFunction2(theta, X, R , figID , 0), ...
        theta_temp, [], [], [], [], lb, ub, @(theta) nonlinConstraints(theta, X, R), options);

%     fprintf('Optimal theta: [%.4f, %.4f] degrees\n', rad2deg(optimal_theta(1)), rad2deg(optimal_theta(2)));
%     fprintf('Minimum loss: %.4f\n', min_loss);

    function [c, ceq] = nonlinConstraints(theta , X , R)
        % 非线性不等式约束 c(theta) <= 0
        % 非线性等式约束 ceq(theta) = 0
        % points = [P ; A ; B ; C ; D ; E ; Q ; N ; M];
        points = forward(X , theta , R , figID);
        A = points(2 , :);
        B = points(3 , :);
        E = points(6 , :);
        O = [0 , 0]; % 圆心
        M = points(9 , :);
        N = points(8 , :);

        safeEvaluation = 2; % 安全距离
        R = R + safeEvaluation; % 增加安全距离后的有效半径

        c1 = R - pointToLineDistance(O , A , B); % 圆心到AB线段距离 >= R
        c2 = R - pointToLineDistance(O , B , E); % 圆心到BE线段距离 >= R
        % 保证MN线段与圆相交. 使用算法: 圆心到线段距离 <= R
        % c3 = pointToLineDistance(O , M , N) - R; % 圆心到MN线段距离 <= R
        c = [c1 , c2 ];
        % c = [c1 , c2 , c3];
        ceq = [];
    end
end