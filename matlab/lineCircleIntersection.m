%% 计算直线与圆的交点
function intersections = lineCircleIntersection(P0, P1, center, radius)
    % 输入：P0, P1 为直线上的两个点 [x,y]；center 为圆心 [x,y]；radius 为半径
    % 输出：intersections 为交点坐标矩阵，每行一个交点 [x,y]，无交点时为空矩阵
    dx = P1(1) - P0(1);
    dy = P1(2) - P0(2);
    A = dx^2 + dy^2;
    B = 2 * (dx * (P0(1) - center(1)) + dy * (P0(2) - center(2)));
    C = (P0(1) - center(1))^2 + (P0(2) - center(2))^2 - radius^2;
    
    discriminant = B^2 - 4 * A * C;
    intersections = [];
    
    if discriminant >= 0
        t1 = (-B + sqrt(discriminant)) / (2 * A);
        x1 = P0(1) + t1 * dx;
        y1 = P0(2) + t1 * dy;
        intersections = [intersections; x1, y1];
        
        if discriminant > 0  % 两个交点
            t2 = (-B - sqrt(discriminant)) / (2 * A);
            x2 = P0(1) + t2 * dx;
            y2 = P0(2) + t2 * dy;
            intersections = [intersections; x2, y2];
        end
    end
end