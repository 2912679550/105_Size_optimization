%% 计算线段与圆的交点
function intersections = lineCircleIntersection(P0, P1, center, radius)
    % 输入：P0, P1 为线段端点 [x,y]；center 为圆心 [x,y]；radius 为半径
    dx = P1(1) - P0(1);
    dy = P1(2) - P0(2);
    A = dx^2 + dy^2;
    B = 2 * (dx * (P0(1) - center(1)) + dy * (P0(2) - center(2)));
    C = (P0(1) - center(1))^2 + (P0(2) - center(2))^2 - radius^2;
    
    discriminant = B^2 - 4 * A * C;
    intersections = [];
    
    if discriminant >= 0
        t1 = (-B + sqrt(discriminant)) / (2 * A);
        t2 = (-B - sqrt(discriminant)) / (2 * A);
        
        % 检查参数t是否在[0,1]范围内
        if t1 >= 0 && t1 <= 1
            x = P0(1) + t1 * dx;
            y = P0(2) + t1 * dy;
            intersections = [intersections; x, y];
        end
        if discriminant > 0 && t2 >= 0 && t2 <= 1  % 避免重复添加相切点
            x = P0(1) + t2 * dx;
            y = P0(2) + t2 * dy;
            intersections = [intersections; x, y];
        end
    end
end