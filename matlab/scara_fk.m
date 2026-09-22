function [T, p] = scara_fk(q, L, base, th1_offset_deg)
%SCARA_FK 三轴三自由度 SCARA 机械手正运动学
%   q = [theta1; theta2; d3]，theta 为弧度，d3 为升降位移(mm)
%   L = [L1; L2]，两段臂长(mm)
%   base = [bx; by]，基座投影位置（棋盘坐标），默认 [0;0]
%   th1_offset_deg：θ1 编码零点相对 +x 方向角的角度偏移(deg)，默认 0
%   返回 T(4x4 齐次变换) 和 p(末端位置 [x;y;z]，mm)

    if nargin < 3, base = [0; 0]; end
    if nargin < 4, th1_offset_deg = 0; end
    off = deg2rad(th1_offset_deg);

    L1 = L(1);
    L2 = L(2);
    th1 = q(1);
    th2 = q(2);
    d3  = q(3);

    phi1 = th1 + off;   % 第一段臂的方向角（+x 基准）

    x = base(1) + L1*cos(phi1) + L2*cos(phi1 + th2);
    y = base(2) + L1*sin(phi1) + L2*sin(phi1 + th2);
    z = d3;

    p = [x; y; z];

    T = [ cos(phi1+th2), -sin(phi1+th2), 0, x;
          sin(phi1+th2),  cos(phi1+th2), 0, y;
          0,             0,            1, z;
          0,             0,            0, 1 ];
end
