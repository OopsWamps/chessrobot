% scara_workspace.m
% SCARA 机械手工作空间验证（受限关节 + 255×255 网格）
% θ1 ∈ [-150°, 15°]（165°，θ1=0 指向正下），θ2 ∈ [0°, 180°]（180°），基座在上边缘中点

clear; clc; close all;

L = [195; 185];          % 臂长 (mm)（2026-09-08 复核: L1=195, L2=185）
base = [0; 230];         % 基座轴心投影（棋盘上边缘中点外 65mm，2026-09-08 实测）
board = 330;             % 棋盘物理尺寸 (mm)
grid = 255;              % 网格（棋子落点区域）(mm)
bhalf = board / 2;       % 165
ghalf = grid / 2;        % 127.5
off = deg2rad(-90);       % θ1 零点偏移（θ1=0 → 正下/棋盘中心）

% 关节限制
th1 = deg2rad(linspace(-150, 15, 240));
th2 = deg2rad(linspace(0, 180, 240));
[TH1, TH2] = meshgrid(th1, th2);

% 可达点云
X = base(1) + L(1)*cos(TH1 + off) + L(2)*cos(TH1 + off + TH2);
Y = base(2) + L(1)*sin(TH1 + off) + L(2)*sin(TH1 + off + TH2);

% 网格四角可达性判断（逆解 + 关节范围检查）
gc = [-ghalf -ghalf; ghalf -ghalf; ghalf ghalf; -ghalf ghalf];
fprintf('关节限制: th1=[-150,15] deg (165), th2=[0,180] deg (180)\n');
fprintf('网格: %d x %d mm (棋子落点区域)\n', grid, grid);
for i = 1:4
    reachable = false;
    for el = [1, -1]
        q = scara_ik([gc(i,1); gc(i,2); 0], L, el, base, -90);
        if any(isnan(q)), continue; end
        t1 = rad2deg(q(1));
        t2 = rad2deg(q(2));
        if mod(t1 + 150, 360) <= 165 && t2 >= 0 && t2 <= 180
            reachable = true;
            fprintf('  网格角(%.0f, %.0f) 可达 (th1=%.0f, th2=%.0f)\n', gc(i,1), gc(i,2), t1, t2);
            break;
        end
    end
    if ~reachable
        fprintf('  网格角(%.0f, %.0f) 不可达\n', gc(i,1), gc(i,2));
    end
end

% 绘图
figure('Color', 'w'); hold on; grid on; axis equal;
plot(X(:), Y(:), '.', 'MarkerSize', 2, 'Color', [0.75 0.75 0.75]);
rectangle('Position', [-bhalf, -bhalf, board, board], ...
    'EdgeColor', [.5 .5 .5], 'LineStyle', '--', 'LineWidth', 1);
rectangle('Position', [-ghalf, -ghalf, grid, grid], ...
    'EdgeColor', 'r', 'LineWidth', 2);
plot(base(1), base(2), 'ks', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('X (mm)'); ylabel('Y (mm)');
title('元萝卜活动空间(255mm)');
legend({'可达点云', '棋盘330', '网格255', '基座'}, 'Location', 'best');
saveas(gcf, 'workspace_result.png');
fprintf('结果图已保存: workspace_result.png\n');
