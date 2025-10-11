function loss = lossFunction1(theta , X , R , figID , pringtFlag)
    % points = [P ; A ; B ; C ; D ; E ; Q ; N ; M];
    points = forward(X , theta , R , figID);
    N = points(8 , :);
    M = points(9 , :);
    mid_MN = (N + M) / 2; % 轮胎中心
    if isempty(pringtFlag)
        pringtFlag = 0;
    end

    % * 夹持点到轮胎中心损失
    % loss_dis = 0;
    intersections = lineCircleIntersection(M , N , [0 , 0] , R);    % 线段MN与圆的交点
    if isempty(intersections)
        % 直接给固定的惩罚值会牺牲梯度, 改为使用圆心到MN中点的距离与R的差值
        center_to_mid = sqrt( mid_MN(1)^2 + mid_MN(2)^2 );
        loss = abs(center_to_mid - R) * 2 + 100; % 惩罚值
        return; % 提前返回 
    else
        intersections = sum(intersections, 1) / size(intersections, 1); % 多个交点取平均
        loss_dis = sqrt( (mid_MN(1) - intersections(1))^2 + (mid_MN(2) - intersections(2))^2 );
    end
    % * 夹持点与圆周120°位置的角度偏差
    angle = atan2(intersections(2) , intersections(1)); % 交点与x轴正向夹角 , -pi ~ pi 
    angle = -angle + 0.5 * pi; 
    angle_dis = abs( 120 - rad2deg(angle) );
    % * 轮胎夹持姿态与夹持点切线夹角偏差
    target_angle = pi - angle; % 目标夹持姿态角度
    cur_angle = atan2( (M(2) - N(2)) , (M(1) - N(1)) ); % 当前夹持姿态角度(表示向量NM与x轴正向夹角)
    pose_dis = abs( rad2deg(cur_angle - target_angle) );

    % * 加权合并损失
    w1 = 0.2;
    w2 = 0.4;
    w3 = 0.4;
    loss = w1 * loss_dis + w2 * angle_dis + w3 * pose_dis;
    if pringtFlag
        % 打印夹持点坐标与对应的loss_dis
        fprintf('Intersection: (%.2f, %.2f), loss_dis: %.2f\n', ...
            intersections(1), intersections(2), loss_dis);
        % 打印夹持点圆周角度与对应的angle_dis
        fprintf('Angle: %.2f degrees, angle_dis: %.2f\n', ...
            rad2deg(angle), angle_dis);
        % 打印夹持姿态与目标夹持姿态的夹角与对应的pose_dis
        fprintf('Current angle: %.2f degrees, Target angle: %.2f degrees, pose_dis: %.2f\n', ...
            rad2deg(cur_angle), rad2deg(target_angle), pose_dis);
    end
    return;
end