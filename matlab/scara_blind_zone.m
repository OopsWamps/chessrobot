% scara_blind_zone.m
% SCARA 机械手盲区地图：在 255mm 网格内逐点采样，标出可达/盲区

clear; clc; close all;

L = [195; 185];          % 臂长 (mm)（2026-09-08 复核: L1=195, L2=185）
base = [0; 230];         % 基座轴心投影（棋盘上边缘中点外 65mm，2026-09-08 实测）
th1_off = -90;           % θ1 零点偏移(deg)，θ1=0 → 正下/棋盘中心
grid = 255;              % 网格（棋子落点区域）(mm)
ghalf = grid / 2;

% 采样步长
step = 5;                % mm
xs = -ghalf:step:ghalf;
ys = -ghalf:step:ghalf;
nx = length(xs);
ny = length(ys);

% 可达性矩阵
reachable = false(ny, nx);

for i = 1:nx
    for j = 1:ny
        p = [xs(i); ys(j); 0];
        ok = false;
        for el = [1, -1]
            q = scara_ik(p, L, el, base, th1_off);
            if any(isnan(q)), continue; end
            t1 = rad2deg(q(1));
            t2 = rad2deg(q(2));
            % θ1 ∈ [-150,15]（165°），θ2 ∈ [0,180]（单向折叠）
            if mod(t1 + 150, 360) <= 165 && t2 >= 0 && t2 <= 180
                ok = true;
                break;
            end
        end
        reachable(j, i) = ok;
    end
end

n_total = numel(reachable);
n_reach = sum(reachable(:));
n_blind = n_total - n_reach;
fprintf('网格采样: %.0f mm 步长, 共 %d 点\n', step, n_total);
fprintf('可达 %d 点, 盲区 %d 点 (%.1f%%)\n', n_reach, n_blind, 100*n_blind/n_total);

% 盲区地图
figure('Color', 'w', 'Position', [100 100 700 640]);
hold on; grid on; axis equal;
imagesc(xs, ys, reachable);
colormap([0.85 0.2 0.2; 0.2 0.75 0.3]);   % 红=盲区, 绿=可达
set(gca, 'YDir', 'normal');
rectangle('Position', [-ghalf, -ghalf, grid, grid], ...
    'EdgeColor', 'k', 'LineWidth', 1.5);
plot(base(1), base(2), 'ks', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'y');
xlabel('X (mm)'); ylabel('Y (mm)');
title('机械手盲区地图（红=盲区，绿=可达）');
xlim([-ghalf-20, ghalf+20]); ylim([-ghalf-20, base(2)+20]);
saveas(gcf, 'blind_zone.png');
fprintf('盲区地图已保存: blind_zone.png\n');
