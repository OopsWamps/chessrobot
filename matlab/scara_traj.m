function [q, qd, qdd, t] = scara_traj(q0, q1, T, dt)
%SCARA_TRAJ 五次多项式关节空间轨迹（rest-to-rest）
%   q0, q1: 起始/目标关节角，3x1（theta1, theta2, d3）
%   T: 运动时间(s)，dt: 采样周期(s)
%   返回 q, qd, qdd（Nx3）和 t（Nx1）
%   初末速度、加速度均为 0（五次多项式保证 C2 连续）

    t = (0:dt:T)';
    n = length(t);
    q = zeros(n, 3);
    qd = zeros(n, 3);
    qdd = zeros(n, 3);

    for j = 1:3
        dq = q1(j) - q0(j);
        a0 = q0(j);
        a3 = 10*dq / T^3;
        a4 = -15*dq / T^4;
        a5 = 6*dq / T^5;
        q(:,j)   = a0 + a3*t.^3 + a4*t.^4 + a5*t.^5;
        qd(:,j)  = 3*a3*t.^2 + 4*a4*t.^3 + 5*a5*t.^4;
        qdd(:,j) = 6*a3*t + 12*a4*t.^2 + 20*a5*t.^3;
    end
end
