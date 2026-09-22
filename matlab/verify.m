% verify.m - 快速验证 SCARA 正逆运动学和工作空间覆盖（基座在上边缘中点）
L = [195; 185];
base = [0; 230];

% 1. 正运动学
q1 = [0; 0; 0];
[~, p1] = scara_fk(q1, L, base);
fprintf('FK q=[0,0,0]   -> p=[%.1f, %.1f, %.1f]\n', p1);

q2 = [0; pi/2; 0];
[~, p2] = scara_fk(q2, L, base);
fprintf('FK q=[0,90,0]  -> p=[%.1f, %.1f, %.1f]\n', p2);

% 2. 逆运动学往返（应接近 0）
p3 = [-80; 0; 30];
q3 = scara_ik(p3, L, 1, base);
[~, p3b] = scara_fk(q3, L, base);
fprintf('IK roundtrip err = %.4f mm\n', norm(p3 - p3b));

% 3. 工作空间覆盖（255 网格最远下角，基座在 y=230）
Rmax = L(1) + L(2);
far = sqrt(127^2 + (230+127)^2);        % 网格下角 (127,-127) 到基座
fprintf('Rmax=%.1f  网格最远下角=%.1f  cover=%d\n', Rmax, far, Rmax >= far);
