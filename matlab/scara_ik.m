function q = scara_ik(p, L, elbow, base, th1_offset_deg)
%SCARA_IK 三轴三自由度 SCARA 机械手逆运动学
%   p = [x; y; z] 末端位置(mm，棋盘坐标)
%   L = [L1; L2] 臂长(mm)
%   elbow = +1 或 -1，肘关节配置（θ2 的符号）
%   base = [bx; by]，基座投影位置（棋盘坐标），默认 [0;0]
%   th1_offset_deg：θ1 编码零点偏移(deg)，默认 0
%   返回 q = [theta1; theta2; d3]（弧度 + mm）
%   不可达返回 NaN。

    if nargin < 4, base = [0; 0]; end
    if nargin < 5, th1_offset_deg = 0; end
    off = deg2rad(th1_offset_deg);

    L1 = L(1);
    L2 = L(2);
    x  = p(1) - base(1);
    y  = p(2) - base(2);
    z  = p(3);

    c2 = (x^2 + y^2 - L1^2 - L2^2) / (2*L1*L2);
    if abs(c2) > 1
        q = [NaN; NaN; NaN];
        return;
    end
    s2  = elbow * sqrt(1 - c2^2);
    th2 = atan2(s2, c2);

    phi1 = atan2(y, x) - atan2(L2*sin(th2), L1 + L2*cos(th2));  % 第一段臂方向角
    th1 = phi1 - off;   % 换算回 θ1 编码

    q = [th1; th2; z];
end
