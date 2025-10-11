%% 辅助函数：点到线段的距离
function dist = pointToLineDistance(point, lineP1, lineP2)
    % 计算点到直线的距离
    v1 = point - lineP1;
    v2 = lineP2 - lineP1;
    v3 = lineP2 - lineP1;
    % 计算投影长度
    t = dot(v1, v2) / dot(v2, v2);
    % 限制投影在线段范围内
    t = max(0, min(1, t));
    % 计算最近点
    projection = lineP1 + t * v3;
    % 计算距离
    dist = norm(point - projection);
end