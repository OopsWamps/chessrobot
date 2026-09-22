%SCARA_CORRECTOR_ENVELOPE 扶正附件(喇叭口)动作包线核算
%   配套文档: docs/扶正附件设计方案.md (v0)
%   输出: 位形 d3 表 / 干涉间隙核算 / 扫动包线图 scara_corrector_result.png
%   注意: 中文标签在部分控制台可能乱码, 判读以数字与 PASS/FAIL 为准

clear; close all; clc;

%% ---------- 常量 (params.md + 方案文档 v0) ----------
L1 = 195; L2 = 185; base = [0; 230];     % SCARA 主体
pitch    = 255/12;      % 围棋格距假设 21.25 mm (P0 待实测)
r_piece  = 19/2;        % 围棋子半径 (平躺)
h_lay    = 4.5;         % 平躺高
h_stand  = 19;          % 侧立高 (透镜立起, 约等于直径)
R_out    = 21/2;        % 喇叭口下口沿外半径 (D_out=21, 顶格用满)
R_in     = 20/2;        % 下口内半径 (壁厚 0.5)
R_noz    = 17/2;        % 吸嘴半径
dz_rim   = 2;           % 口沿低于吸嘴口 2 mm (口沿 z = 嘴口 z - 2)
skirt_len = 27;         % 裙筒内腔长 (口沿 -> 上端), 需 >= 侧立高 + 余量
d3_max   = 35;          % 升降行程
z_noz_top = 30;         % 吸嘴口最高位 z

fprintf('===== 1. working postures (d3 check) =====\n');
% 吸嘴口 z = 30 - d3 ; 口沿 z = 吸嘴口 z - 2
postures = { % name, rim_z(口沿目标高度)
  'standby',   28;
  'sweep',      1;
  'engage',     2.5;   % 吸合: 嘴口贴子顶 4.5
};
for i = 1:size(postures,1)
    rim_z = postures{i,2};
    noz_z = rim_z + dz_rim;
    d3    = z_noz_top - noz_z;
    ok = (d3 >= 0) && (d3 <= d3_max);
    fprintf('%-8s rim_z=%6.2f  noz_z=%6.2f  d3=%6.2f  [%s]\n', ...
        postures{i,1}, rim_z, noz_z, d3, pass(ok));
end

fprintf('\n===== 2. dense-grid static clearance =====\n');
% 静止下探: 口沿外缘 vs 相邻棋子 (中心距 = pitch)
gap_static = pitch - (R_out + r_piece);
fprintf('static gap rim-to-neighbor = %6.3f mm  [%s]\n', gap_static, pass(gap_static > 0.3));

fprintf('\n===== 3. dense-grid sweep limit =====\n');
% 扫动偏移 s 后仍不碰邻子: s <= pitch - (R_out + r_piece)
s_max = pitch - (R_out + r_piece);
fprintf('s_max (dense grid) = %6.3f mm\n', s_max);
% 推倒侧立子所需偏移: 口沿内缘越过子中心线 -> 半宽 2.25
s_tip = h_lay/2;    % 透镜立着接地宽近似 = 平躺高/2 (保守)
fprintf('s needed to topple (rim inner edge passes piece CL) ~ %6.3f mm\n', s_tip);
if s_tip > s_max
    fprintf(['NOTE: topple-by-sweep (%.2fmm) exceeds dense-grid limit (%.2fmm).\n' ...
             '      -> dense cells must rely on cone guidance (press-in), not sweep.\n'], s_tip, s_max);
end

fprintf('\n===== 4. standing piece inside skirt =====\n');
% 内径沿高度线性收窄: z=0(口沿)->R_in, z=skirt_len->R_noz+0.2
for zk = [h_stand, skirt_len]
    Rz = R_in - (R_in - (R_noz+0.2)) * zk/skirt_len;
    fprintf('skirt inner R at z=%4.1f : %6.3f mm\n', zk, Rz);
end
% 侧立子顶所处高度 = h_stand (从口沿起算)
R_top = R_in - (R_in - (R_noz+0.2)) * h_stand/skirt_len;
clearance_noz = R_top - R_noz;   % 立子中心对中时, 立子表面到吸嘴表面... 保守用半径差
fprintf('top clearance (piece side vs nozzle, centered) = %6.3f mm\n', clearance_noz);
fprintf('skirt_len(%4.1f) vs standing piece(%4.1f)+margin: [%s]\n', skirt_len, h_stand, pass(skirt_len >= h_stand + 2));

fprintf('\n===== 5. isolated-piece sweep envelope (figure) =====\n');
% 孤立子(棋盒/散乱区): 口沿中心绕子中心走半径 s_iso=5 的小圆
s_iso = 5;
figure('Color','w','Position',[100 100 640 640]); hold on; axis equal;
th = linspace(0, 2*pi, 200);
% 棋子阵列 5x5 (间隔 pitch), 中心为目标子
for i = -2:2
    for j = -2:2
        rectangle('Position',[i*pitch-r_piece, j*pitch-r_piece, 2*r_piece, 2*r_piece], ...
                  'Curvature',[1 1], 'EdgeColor',[.4 .4 .4], 'FaceColor',[.85 .85 .85]);
    end
end
% 口沿扫动包络 = 半径 R_out + s_iso 的圆环带
rectangle('Position',[-s_iso-R_out, -s_iso-R_out, 2*(R_out+s_iso), 2*(R_out+s_iso)], ...
          'Curvature',[1 1], 'EdgeColor',[.85 .2 .2], 'LineStyle','--', 'LineWidth',1.5);
rectangle('Position',[-R_out, -R_out, 2*R_out, 2*R_out], ...
          'Curvature',[1 1], 'EdgeColor','r', 'LineWidth',1.5);
rectangle('Position',[-R_in, -R_in, 2*R_in, 2*R_in], ...
          'Curvature',[1 1], 'EdgeColor',[.95 .6 .1], 'LineWidth',1.2);
grid on;
xlim([-35 35]); ylim([-35 35]);
xlabel('x (mm)'); ylabel('y (mm)');
title(sprintf('sweeper rim envelope: R_{out}=%.1f, s_{iso}=%.1f, pitch=%.2f', R_out, s_iso, pitch));
legend({'pieces','envelope (R+s)','rim R_{out}','rim R_{in}'}, 'Location','southoutside');
saveas(gcf, 'scara_corrector_result.png');
fprintf('figure saved: scara_corrector_result.png\n');

fprintf('\n===== done =====\n');

function s = pass(ok)
    if ok, s = 'PASS'; else, s = 'FAIL'; end
end
