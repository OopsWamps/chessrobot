% scara_pick_place.m
% SCARA 机械手取放子仿真（pick-and-place），基座在棋盘上边缘中点
% 流程：初始位形 → 移到棋子A上方 → 下降吸子 → 提升 → 移到B上方 → 下降放子 → 提升

clear; clc; close all;

%% 参数
L = [195; 185];          % 臂长 (mm)（2026-09-08 复核: L1=195, L2=185）
base = [0; 230];         % 基座轴心投影（棋盘上边缘中点外 65mm，2026-09-08 实测）
d3_safe = 20;            % 安全高度（吸盘缩回，向下伸 20mm）
d3_pick = 90;            % 取放子高度（吸盘伸出 90mm，接触棋子）
elbow = 1;               % 肘关节配置
T_seg = 1.0;             % 每段运动时间 (s)
dt = 0.02;               % 采样周期 (s)
board = 330;             % 棋盘边长 (mm)
half = board / 2;

% 棋子位置（棋盘坐标 mm，棋盘中心为原点）
pA = [-80; 0];           % 起始棋子
pB = [ 80; -100];        % 目标落点

%% 逆解路径点关节角
thA = scara_ik([pA; d3_safe], L, elbow, base, -90);
thB = scara_ik([pB; d3_safe], L, elbow, base, -90);
if any(isnan(thA)) || any(isnan(thB))
    error('棋子位置超出工作空间，不可达');
end
% θ1 关节范围检查 [-150°, 15°]
if mod(rad2deg(thA(1)) + 150, 360) > 165 || mod(rad2deg(thB(1)) + 150, 360) > 165
    error('棋子超出 θ1 关节范围 [-150, 15]');
end

%% 7 个路径点（关节空间）
% [初始, A上, A取, A上, B上, B放, B上]
wp = [ ...
    0,        thA(1), thA(1), thA(1), thB(1), thB(1), thB(1);
    pi,       thA(2), thA(2), thA(2), thB(2), thB(2), thB(2);
    d3_safe,  d3_safe,d3_pick,d3_safe,d3_safe,d3_pick,d3_safe ];

% 关节角连续性（消除 2*pi 跳变）
wp(1,:) = unwrap(wp(1,:));
wp(2,:) = unwrap(wp(2,:));

%% 逐段五次多项式拼接
q = []; qd = []; qdd = []; t = [];
for k = 1:size(wp, 2) - 1
    [qk, qdk, qddk, tk] = scara_traj(wp(:,k), wp(:,k+1), T_seg, dt);
    if ~isempty(t), tk = tk + t(end); end
    q   = [q;   qk];   %#ok<AGROW>
    qd  = [qd;  qdk];  %#ok<AGROW>
    qdd = [qdd; qddk]; %#ok<AGROW>
    t   = [t;   tk];   %#ok<AGROW>
end

%% 正运动学计算末端轨迹
N = length(t);
p = zeros(N, 3);
for i = 1:N
    [~, p(i,:)] = scara_fk(q(i,:)', L, base, -90);
end

fprintf('总时长 = %.1f s，采样点 = %d\n', t(end), N);
fprintf('末端起点 = [%.1f, %.1f, %.1f]\n', p(1,:));
fprintf('末端终点 = [%.1f, %.1f, %.1f]\n', p(end,:));
fprintf('关节角范围: th1=[%.0f, %.0f] deg, th2=[%.0f, %.0f] deg\n', ...
    rad2deg(min(q(:,1))), rad2deg(max(q(:,1))), ...
    rad2deg(min(q(:,2))), rad2deg(max(q(:,2))));

%% 绘图
fig = figure('Color', 'w', 'Position', [100 100 800 750]);

subplot(2,1,1); hold on; grid on; axis equal;
plot(p(:,1), p(:,2), 'b-', 'LineWidth', 1);
plot(pA(1), pA(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
plot(pB(1), pB(2), 'g^', 'MarkerSize', 10, 'LineWidth', 2);
plot(base(1), base(2), 'ks', 'MarkerSize', 12, 'LineWidth', 2);
rectangle('Position', [-half, -half, board, board], ...
    'EdgeColor', [.5 .5 .5], 'LineStyle', '--');
xlabel('X (mm)'); ylabel('Y (mm)'); title('末端 XY 轨迹（取放子）');
legend({'末端轨迹', '起点棋子A', '目标落点B', '基座', '棋盘'}, 'Location', 'best');

subplot(2,1,2); hold on; grid on;
plot(t, rad2deg(q(:,1)), 'LineWidth', 1.5);
plot(t, rad2deg(q(:,2)), 'LineWidth', 1.5);
plot(t, q(:,3), 'LineWidth', 1.5);
xlabel('时间 t (s)'); ylabel('角度 (deg) / 升降 (mm)');
title('关节轨迹');
legend({'\theta_1', '\theta_2', 'd_3'}, 'Location', 'best');

saveas(fig, 'pick_place_result.png');
fprintf('结果图已保存: pick_place_result.png\n');
