%% 本函数用于运动学正解, 将根据输入的所有几何信息求解辅助轮位置,并评价相应分数
%% This function is used for forward kinematics, 
%  which will solve the auxiliary wheel position based on all input geometric information and evaluate the corresponding score.
function points = forward(X , theta_  , R_ , figID)
    R = R_; % pipe radius
    R_m = 50.0; % main wheel radius
    R_a = 30.0; % assist wheel radius
    OP = R + 26.0; % point P pos
    MN = 25.0;
    %% * Parse dimension variables
    l1 = X(1); % AP
    l2 = X(2); % AB
    l3 = X(3); % BC
    l4 = X(4); % BE
    l5 = X(5); % CD
    l6 = X(6); % DE
    alpha = X(7); % angle EDQ
    l7 = X(8); % DQ
    theta1 = theta_(1); % angle (+x)AB
    theta2 = theta_(2); % angle ABE
    %% * Calculate point coordinates
    P = [0 , OP];
    A = [l1  , OP];
    B = [l1 + l2 * cos(theta1) , OP - l2 * sin(theta1)];
    C = [l1 + (l2 + l3) * cos(theta1) , OP - (l2 + l3) * sin(theta1)];
    E = [l1 + l2*cos(theta1) - l4 * cos(theta2 - theta1) , OP - l2 * sin(theta1) - l4 * sin(theta2 - theta1)];
    % solve D
    CE = sqrt( (E(1) - C(1))^2 + (E(2) - C(2))^2 );
    angle_BCE = real(acos( (l3^2 + CE^2 - l4^2) / (2 * l3 * CE) ));
    angle_ECD = real(acos( (CE^2 + l5^2 - l6^2) / (2 * CE * l5) ));
    beta1 = pi - (angle_BCE + angle_ECD - theta1);  % angle (+x)CD
    D = [C(1) + l5 * cos(beta1) , C(2) - l5 * sin(beta1)];
    % solve Q
    beta2 = atan2( (E(2) - D(2)) , (E(1) - D(1)) ); % angle (+x)DE
    if beta2 < 0
        beta2 = beta2 + 2 * pi;
    end
    beta3 = 2 * pi - (beta2 + alpha); % angle (+x)DQ
    Q = [D(1) + l7 * cos(beta3) , D(2) - l7 * sin(beta3)];
    % solve M , N
    beta4 = pi - beta3 + 0.5 * pi; % angle (+x)QN
    N = [Q(1) + 2 * R_a * cos(beta4) , Q(2) + 2 * R_a * sin(beta4)];
    M = [N(1) + MN * cos(pi - beta3) , N(2) + MN * sin(pi - beta3)];
    %% * plot
    if figID > 0
        figure(figID);
        hold on;
        axis equal;
        % plot line : PA , AB , BC , CD , BE , DE , DQ , QN , MN
        plot( [P(1) , A(1)] , [P(2) , A(2)]  , 'LineWidth' , 2); % PA
        plot( [A(1) , B(1)] , [A(2) , B(2)]  , 'LineWidth' , 2); % AB
        plot( [B(1) , C(1)] , [B(2) , C(2)]  , 'LineWidth' , 2); % BC
        plot( [C(1) , D(1)] , [C(2) , D(2)]  , 'LineWidth' , 2); % CD
        plot( [B(1) , E(1)] , [B(2) , E(2)]  , 'LineWidth' , 2); % BE
        plot( [D(1) , E(1)] , [D(2) , E(2)]  , 'LineWidth' , 2); % DE
        plot( [D(1) , Q(1)] , [D(2) , Q(2)]  , 'LineWidth' , 2); % DQ
        plot( [Q(1) , N(1)] , [Q(2) , N(2)]  , 'LineWidth' , 2); % QN
        plot( [N(1) , M(1)] , [N(2) , M(2)]  , 'LineWidth' , 2); % MN
        % 绘制关于y轴对称的上述线段
        plot( [P(1) , -A(1)] , [P(2) , A(2)]  , 'LineWidth' , 2); % PA
        plot( [-A(1) , -B(1)] , [A(2) , B(2)]  , 'LineWidth' , 2); % AB
        plot( [-B(1) , -C(1)] , [B(2) , C(2)]  , 'LineWidth' , 2); % BC
        plot( [-C(1) , -D(1)] , [C(2) , D(2)]  , 'LineWidth' , 2); % CD
        plot( [-B(1) , -E(1)] , [B(2) , E(2)]  , 'LineWidth' , 2); % BE
        plot( [-D(1) , -E(1)] , [D(2) , E(2)]  , 'LineWidth' , 2); % DE
        plot( [-D(1) , -Q(1)] , [D(2) , Q(2)]  , 'LineWidth' , 2); % DQ
        plot( [-Q(1) , -N(1)] , [Q(2) , N(2)]  , 'LineWidth' , 2); % QN
        plot( [-N(1) , -M(1)] , [N(2) , M(2)]  , 'LineWidth' , 2); % MN
        
        % print N 
        % disp(N);
        % plot points : P , A , B , C , D , E , Q , N , M
        % plot( P(1) , P(2) , 'ro' , 'MarkerFaceColor' , 'r' , 'MarkerSize' , 8); % P
        % plot( A(1) , A(2) , 'ro' , 'MarkerFaceColor' , 'r' , 'MarkerSize' , 8); % A
        % plot( B(1) , B(2) , 'ro' , 'MarkerFaceColor' , 'r' , 'MarkerSize' , 8); % B
        % plot( C(1) , C(2) , 'ro' , 'MarkerFaceColor' , 'r' , 'MarkerSize' , 8); % C
        % plot( D(1) , D(2) , 'ro' , 'MarkerFaceColor' , 'r' , 'MarkerSize' , 8); % D
        % plot( E(1) , E(2) , 'ro' , 'MarkerFaceColor' , 'r' , 'MarkerSize' , 8); % E
        % plot( Q(1) , Q(2) , 'ro' , 'MarkerFaceColor' , 'r' , 'MarkerSize' , 8); % Q
        % plot( N(1) , N(2) , 'ro' , 'MarkerFaceColor' , 'r' , 'MarkerSize' , 8); % N
        % plot( M(1) , M(2) , 'ro' , 'MarkerFaceColor' , 'r' , 'MarkerSize' , 8); % M
        % plot circle: O(r = R)
        theta = 0 : 0.01 : 2 * pi;
        x_circle = R * cos(theta);
        y_circle = R * sin(theta);
        plot( x_circle , y_circle , 'LineWidth' , 2 , 'LineStyle' , '--' , 'Color' , [0.2 0.2 0.8]); % pipe');
    end
    %% * pack output
    points = [P ; A ; B ; C ; D ; E ; Q ; N ; M];
end

