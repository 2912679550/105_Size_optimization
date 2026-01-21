% 注意这个函数假设直线是无限延伸的
function intersection = lineCircleCross(P0, P1, center, radius)
    % 输入：P0, P1 为直线上的两个点 [x,y]；center 为圆心 [x,y]；radius 为半径
    % 输出：intersection 为距离线段P0P1中点最近的一个交点坐标 [x,y]，无交点时为空矩阵
    dx = P1(1) - P0(1);
    dy = P1(2) - P0(2);
    A = dx^2 + dy^2;
    B = 2 * (dx * (P0(1) - center(1)) + dy * (P0(2) - center(2)));
    C = (P0(1) - center(1))^2 + (P0(2) - center(2))^2 - radius^2;
    
    discriminant = B^2 - 4 * A * C;
    intersection = [];
    
    if discriminant >= 0
        % 计算线段 P0P1 的中点
        midPt = (P0 + P1) / 2;
        
        t1 = (-B + sqrt(discriminant)) / (2 * A);
        p1 = [P0(1) + t1 * dx, P0(2) + t1 * dy];
        
        if discriminant > 0  % 两个交点
            t2 = (-B - sqrt(discriminant)) / (2 * A);
            p2 = [P0(1) + t2 * dx, P0(2) + t2 * dy];
            
            % 比较两个交点到中点的距离
            dist1 = sum((p1 - midPt).^2);
            dist2 = sum((p2 - midPt).^2);
            
            if dist1 < dist2
                intersection = p1;
            else
                intersection = p2;
            end
        else  % 一个交点（切点）
            intersection = p1;
        end
    end
end