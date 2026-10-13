clear;
clc;
close all;

%% =========================================================
%  1. 文件设置
% ==========================================================

file_x = '水管X分量位移.csv';
file_y = '水管Y分量位移.csv';
file_motor = '电机转角-30°.csv';

%% =========================================================
%  2. 读取 SolidWorks Motion 导出的 CSV
% ==========================================================

Tx = readtable(file_x, ...
    'NumHeaderLines', 1, ...
    'VariableNamingRule', 'preserve');

Ty = readtable(file_y, ...
    'NumHeaderLines', 1, ...
    'VariableNamingRule', 'preserve');

Tm = readtable(file_motor, ...
    'NumHeaderLines', 1, ...
    'VariableNamingRule', 'preserve');

t_x = Tx{:,1};
dx  = Tx{:,2};

t_y = Ty{:,1};
dy  = Ty{:,2};

t_m = Tm{:,1};
motor_abs_deg = Tm{:,2};

%% =========================================================
%  3. 检查时间轴
% ==========================================================

if length(t_x) ~= length(t_y) || length(t_x) ~= length(t_m)
    error('三个 CSV 的数据点数量不一致。');
end

if max(abs(t_x - t_y)) > 1e-9 || ...
   max(abs(t_x - t_m)) > 1e-9
    error('三个 CSV 的时间轴不一致。');
end

t = t_x;
dt = diff(t);

fprintf('========== 数据基本信息 ==========\n');
fprintf('数据点数：%d\n', length(t));
fprintf('仿真时长：%.6f s\n', t(end) - t(1));
fprintf('平均采样周期：%.6f s\n', mean(dt));
fprintf('平均采样频率：%.3f Hz\n\n', 1 / mean(dt));

%% =========================================================
%  4. 水管长度检查
% ==========================================================

pipe_length = sqrt(dx.^2 + dy.^2);

fprintf('========== 水管长度检查 ==========\n');
fprintf('平均长度：%.9f mm\n', mean(pipe_length));
fprintf('最小长度：%.9f mm\n', min(pipe_length));
fprintf('最大长度：%.9f mm\n', max(pipe_length));
fprintf('长度极差：%.12f mm\n\n', ...
    max(pipe_length) - min(pipe_length));

%% =========================================================
%  5. 定义电机控制角
%
%  SolidWorks 当前数据：
%      105.6 deg -> 75.6 deg
%
%  也就是原始变化：
%      0 -> -30 deg
%
%  但我们希望控制定义为：
%      0 -> +30 deg
%
%  所以取负号
% ==========================================================

motor_deg = -(motor_abs_deg - motor_abs_deg(1));

fprintf('========== 电机控制角定义 ==========\n');
fprintf('电机初始绝对角：%.6f deg\n', motor_abs_deg(1));
fprintf('电机控制初始角：%.6f deg\n', motor_deg(1));
fprintf('电机控制最终角：%.6f deg\n\n', motor_deg(end));

%% =========================================================
%  6. 定义水管真实几何角
%
%  当前 atan2d(dy, dx) 得到的是：
%      +5.34 deg -> -4.91 deg
%
%  但我们希望实际控制定义是：
%      初始为负
%      后续逐渐变正
%
%  因此取负号：
%
%      pipe_deg = -atan2d(dy, dx)
%
%  最终得到：
%      -5.34 deg -> +4.91 deg
%
%  注意：
%  水管这里不做相对归零
% ==========================================================

pipe_deg = -atan2d(dy, dx);

fprintf('========== 水管真实几何角 ==========\n');
fprintf('水管初始角：%.6f deg\n', pipe_deg(1));
fprintf('水管最终角：%.6f deg\n\n', pipe_deg(end));

%% =========================================================
%  7. 一次线性拟合
%
%      pipe = k * motor + b
%
%  这里 b 有明确物理意义：
%
%      motor = 0 时
%      pipe  = b
%
%  即水管初始真实角度
% ==========================================================

p1 = polyfit(motor_deg, pipe_deg, 1);

k = p1(1);
b = p1(2);

pipe_fit_linear = polyval(p1, motor_deg);

residual_linear = pipe_deg - pipe_fit_linear;

SS_res = sum(residual_linear.^2);
SS_tot = sum((pipe_deg - mean(pipe_deg)).^2);

R2_linear = 1 - SS_res / SS_tot;
RMSE_linear = sqrt(mean(residual_linear.^2));
max_error_linear = max(abs(residual_linear));

fprintf('========== 一次线性拟合 ==========\n');
fprintf('pipe = k * motor + b\n\n');

fprintf('k = %.10f\n', k);
fprintf('b = %.10f deg\n', b);
fprintf('R^2 = %.10f\n', R2_linear);
fprintf('RMSE = %.10f deg\n', RMSE_linear);
fprintf('最大绝对误差 = %.10f deg\n\n', ...
    max_error_linear);

%% =========================================================
%  8. 二次拟合
%
%      pipe = a*motor^2 + b*motor + c
%
%  用于判断非线性是否明显
% ==========================================================

p2 = polyfit(motor_deg, pipe_deg, 2);

pipe_fit_quad = polyval(p2, motor_deg);

residual_quad = pipe_deg - pipe_fit_quad;

SS_res2 = sum(residual_quad.^2);
R2_quad = 1 - SS_res2 / SS_tot;

RMSE_quad = sqrt(mean(residual_quad.^2));
max_error_quad = max(abs(residual_quad));

fprintf('========== 二次拟合 ==========\n');
fprintf('pipe = a*motor^2 + b*motor + c\n\n');

fprintf('a = %.12f\n', p2(1));
fprintf('b = %.12f\n', p2(2));
fprintf('c = %.12f\n', p2(3));
fprintf('R^2 = %.10f\n', R2_quad);
fprintf('RMSE = %.10f deg\n', RMSE_quad);
fprintf('最大绝对误差 = %.10f deg\n\n', ...
    max_error_quad);

%% =========================================================
%  9. 计算水管过水平位置
%
%  令：
%
%      pipe = 0
%
%  则：
%
%      motor_zero_pipe = -b / k
% ==========================================================

motor_at_pipe_zero = -b / k;

fprintf('========== 水管水平位置 ==========\n');
fprintf('当水管角 = 0 deg 时：\n');
fprintf('电机控制角约为 %.6f deg\n\n', ...
    motor_at_pipe_zero);

%% =========================================================
%  10. 图1：电机角 -> 水管真实角
% ==========================================================

figure;

plot(motor_deg, pipe_deg, '.', ...
    'DisplayName', 'SolidWorks仿真数据');

hold on;

plot(motor_deg, pipe_fit_linear, ...
    'LineWidth', 1.5, ...
    'DisplayName', '一次拟合');

plot(motor_deg, pipe_fit_quad, ...
    ':', ...
    'LineWidth', 1.5, ...
    'DisplayName', '二次拟合');

yline(0, '--', ...
    'DisplayName', '水管水平位置');

grid on;

xlabel('电机控制角度 (deg)');
ylabel('水管真实几何角度 (deg)');

title('电机角度 - 水管真实角度映射');

legend('Location', 'best');

%% =========================================================
%  11. 图2：一次拟合残差
% ==========================================================

figure;

plot(motor_deg, residual_linear, '.-');

hold on;
yline(0, '--');

grid on;

xlabel('电机控制角度 (deg)');
ylabel('拟合残差 (deg)');

title('一次线性拟合残差');

%% =========================================================
%  12. 图3：二次拟合残差
% ==========================================================

figure;

plot(motor_deg, residual_quad, '.-');

hold on;
yline(0, '--');

grid on;

xlabel('电机控制角度 (deg)');
ylabel('拟合残差 (deg)');

title('二次拟合残差');

%% =========================================================
%  13. 图4：水管长度检查
% ==========================================================

figure;

plot(t, pipe_length);

grid on;

xlabel('时间 (s)');
ylabel('两端点距离 (mm)');

title('水管刚性长度检查');

%% =========================================================
%  14. 图5：电机角与水管角随时间变化
% ==========================================================

figure;

plot(t, motor_deg, ...
    'DisplayName', '电机控制角');

hold on;

plot(t, pipe_deg, ...
    'DisplayName', '水管真实角');

grid on;

xlabel('时间 (s)');
ylabel('角度 (deg)');

title('电机角与水管角随时间变化');

legend('Location', 'best');

%% =========================================================
%  15. 保存结果
% ==========================================================

result = table( ...
    t, ...
    motor_abs_deg, ...
    motor_deg, ...
    dx, ...
    dy, ...
    pipe_length, ...
    pipe_deg, ...
    pipe_fit_linear, ...
    pipe_fit_quad, ...
    residual_linear, ...
    residual_quad);

result.Properties.VariableNames = { ...
    'time_s', ...
    'motor_abs_deg', ...
    'motor_control_deg', ...
    'pipe_dx_mm', ...
    'pipe_dy_mm', ...
    'pipe_length_mm', ...
    'pipe_angle_deg', ...
    'linear_fit_deg', ...
    'quadratic_fit_deg', ...
    'linear_residual_deg', ...
    'quadratic_residual_deg'};

writetable(result, ...
    'motor_pipe_absolute_mapping_result.csv');

fprintf('结果已保存：motor_pipe_absolute_mapping_result.csv\n');

%% =========================================================
%  16. 控制器最终可用形式
% ==========================================================

fprintf('\n========== 控制器可用形式 ==========\n');

fprintf('正向映射：\n');
fprintf('pipe_angle = %.10f * motor_angle %+ .10f\n\n', ...
    k, b);

fprintf('反向映射：\n');
fprintf('motor_angle = (pipe_target - (%+.10f)) / %.10f\n\n', ...
    b, k);

fprintf('水管初始角：\n');
fprintf('pipe_angle(motor = 0) = %.10f deg\n\n', ...
    b);

fprintf('水管水平时：\n');
fprintf('motor_angle = %.10f deg\n', ...
    motor_at_pipe_zero);