X_origin = [59.8448  175.3350   23.9903  184.8515  188.7162   45.6815    1.5670   47.7451];
lb = [20, 150, 20, 100, 100, 30, deg2rad(20), 40];
ub = [60, 250, 50, 200, 200, 50, deg2rad(120), 80];

%% 第一段采用PSO进行全局优化

options_1 = optimoptions('particleswarm', ...
        'SwarmSize', 1000, ...
        'MaxIterations', 50, ...
        'Display', 'iter', ...
        'UseParallel', true);
[optimal_X_ps, min_total_loss_ps] = particleswarm(@all_loss, length(X_origin), lb, ub, options_1);
fprintf('Optimal X from PSO: \n');
fprintf('Minimum Total Loss from PSO: %.4f\n', min_total_loss_ps);
disp(optimal_X_ps);

% %% 第二段采用GA优化
% % 基于PSO调整上下界, 边长±10 , 角度±10度
% new_lb = optimal_X_ps - 10;
% new_ub = optimal_X_ps + 10;
% % 保留原始的第7个下界和第8个上界
% new_lb(7) = lb(7);
% new_ub(8) = ub(8);
% lb = new_lb;
% ub = new_ub;

% options_ga = optimoptions('ga', ...
%         'PopulationSize', 200, ...
%         'MaxGenerations', 100, ...
%         'CrossoverFraction',0.8, ...
%         'Display', 'iter', ...
%         'UseParallel', true);

% 再次进行遗传算法优化
% [optimal_X_ga, min_total_loss_ga] = ga(@all_loss, length(X_origin), [], [], [], [], lb, ub, [], options_ga);
% fprintf('Optimal X from 2nd GA: \n');
% disp(optimal_X_ga);
% fprintf('Minimum Total Loss from 2nd GA: %.4f\n', min_total_loss_ga);

options = optimoptions('fmincon', ...
    'Algorithm', 'sqp', ...
    'Display', 'iter-detailed', ...
    'MaxIterations', 200, ... % 增加迭代次数
    'MaxFunctionEvaluations', 5000, ...
    'StepTolerance', 1e-10, ... % 更严格的收敛条件
    'OptimalityTolerance', 1e-6, ...
    'ConstraintTolerance', 1e-4, ...
    'FiniteDifferenceType', 'central', ... % 更精确的梯度估计
    'FiniteDifferenceStepSize', 1e-4, ...
    'ScaleProblem', 'obj-and-constr', ... % 自动缩放问题
    'HonorBounds', true, ...
    'CheckGradients', false, ...
    'UseParallel', true);

[optimal_X, min_total_loss] = fmincon(@all_loss, optimal_X_ps, [], [], [], [], lb, ub, [], options);
fprintf('Optimal X: \n');
disp(optimal_X);
fprintf('Minimum Total Loss: %.4f\n', min_total_loss);



function loss = all_loss(X)
    loss = 0;
    for R = 110 : 5 : 200
        [~, min_loss] = sub_score_1(X , R , -1);
        % fprintf('R = %d, Min loss: %.4f\n', R, min_loss);
        loss = loss + min_loss;
    end
end

function [c, ceq] = fourBarConstraints(X)
    % 提取四连杆尺寸参数
    l_BC = X(3);  % BC
    l_BE = X(4);  % BE
    l_CD = X(5);  % CD
    l_DE = X(6);  % DE
    
    % 四杆长度向量
    lengths = [l_BC, l_BE, l_CD, l_DE];
    l_max = max(lengths);
    l_min = min(lengths);
    
    % 计算其余两杆长度之和
    remaining_lengths = sum(lengths) - l_max - l_min;
    
    % Grashof条件：双摇杆机构要求最长杆+最短杆 > 其余两杆之和
    grashof_condition = (l_max + l_min) > remaining_lengths;
    
    % % 双摇杆机构的附加条件：最短杆为连杆（CD或DE）
    % % 对于以BC为机架的双摇杆机构，要求最短杆是CD或DE（连杆）
    % is_shortest_link = (l_min == l_CD) || (l_min == l_DE);
    
    % 不等式约束：要求满足双摇杆机构条件
    % 如果满足条件，c <= 0；否则c > 0（违反约束）
    c = [];
    % if ~(grashof_condition && is_shortest_link)
    if grashof_condition
        c(1) = -1;  % 满足约束
    else
        c(1) = 1; % 不满足约束
    end
    
    % 等式约束（无）
    ceq = [];
end


