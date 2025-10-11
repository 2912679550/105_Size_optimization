clear;
clc;
close all;

%% forward kinematics test
% l1 = PA = 37.95
% l2 = AB = 184.12
% l3 = BC = 26.88
% l4 = BE = 162
% l5 = CD = 151.50
% l6 = DE = 37
% α = ∠EDQ = 62°
% l7 =DQ = 58.50 
X_test = [37.95 , 184.12 , 26.88 , 162 , 151.50 , 37 , deg2rad(62) , 58.50];
R = 180; % pipe radius
% theta1 = deg2rad(31.85); % angle (+x)AB
% theta2 = deg2rad(180 - 47.51); % angle ABE
% figID = 2;
% forward(X_test , [theta1 , theta2] , R , figID)

%% 线段与圆的交点测试
% P0 = [0 , 2];
% P1 = [2 , 0];
% center = [0 , 0];
% radius = 2;
% intersections = lineCircleIntersection(P0, P1, center, radius);
% disp('Intersections:');
% disp(intersections);

% pointToLineDistance([0.5,0.5], [0,0], [1,1])

%% 优化测试
X = [37.95 , 184.12 , 26.88 , 162 , 151.50 , 37 , deg2rad(77) , 58.50];
R = 180; % pipe radius
ticStart = tic;                       % <-- 计时开始
[optimal_theta, min_loss] = sub_score_1(X , R , -1);
opt_time = toc(ticStart);             % <-- 计时结束
fprintf('Optimization elapsed time: %.3f ms\n', opt_time * 1000);
% 评估最终损失与相关数据
lossFunction1(optimal_theta , X , R , 2 , 1);

