%% Plot script
clear
close all
clc

%% Global Plot Settings
set(0, 'DefaultTextInterpreter', 'latex')
set(0, 'DefaultLegendInterpreter', 'latex')
set(0, 'DefaultAxesTickLabelInterpreter', 'tex') 
lw = 1.5; % linewidth
fs = 14;  % Font size

% Palette
dark_grey = [0.2, 0.2, 0.2]; % Dark Grey
red       = [0.8, 0.1, 0.1]; % Red
green     = [0.1, 0.8, 0.1]; % Green
blue      = [0.1, 0.4, 0.7]; % Blue

colors = {red, green, blue, dark_grey, red, green, blue, dark_grey};

%% Simulation
% Simulink file names:
% OpenAUV_v1dist
% OpenAUV_v2OBS
model_name = "OpenAUV_v1dist";
out = sim(model_name + ".slx");

%% Data extraction
time = out.err_p.time;
time_w = out.omega.time;
err_p = out.err_p.signals.values;
err_R = squeeze(out.err_R.signals.values)';
w_b = squeeze(out.w_b.signals.values)';
w = squeeze(out.omega.signals.values)';
u_data = squeeze(out.u.signals.values);

%% FIGURE 1: Tracking Errors (Position and Orientation)
h1 = figure('Position', [100 100 1000 400]);
subplot(1, 2, 1)
plot(time, err_p(:,1), 'Color', red, 'LineWidth', lw); hold on;
plot(time, err_p(:,2), 'Color', green, 'LineWidth', lw);
plot(time, err_p(:,3), 'Color', blue, 'LineWidth', lw);
grid on; box on;
xlabel('$t$ [s]')
ylabel('$e_p$ [m]')
legend('$e_x$', '$e_y$', '$e_z$', 'Location', 'northeast')
title('Position Error')
set(gca, 'FontSize', fs);
xlim([0, 80]);


subplot(1, 2, 2)
plot(time, err_R(:,1), 'Color', red, 'LineWidth', lw); hold on;
plot(time, err_R(:,2), 'Color', green, 'LineWidth', lw);
plot(time, err_R(:,3), 'Color', blue, 'LineWidth', lw);
grid on; box on;
xlabel('$t$ [s]')
ylabel('$e_R$ [rad]')
legend('$e_{R,1}$', '$e_{R,2}$', '$e_{R,3}$', 'Location', 'northeast')
title('Orientation Error')
set(gca, 'FontSize', fs);
set(gcf, 'color', 'w');
xlim([0, 80]);

%% FIGURE 2: Control Wrench (Forces and Torques)
h2 = figure('Position', [100 100 1000 400]);
subplot(1, 2, 1)
plot(time, w_b(:,1), 'Color', red, 'LineWidth', lw); hold on;
plot(time, w_b(:,2), 'Color', green, 'LineWidth', lw);
plot(time, w_b(:,3), 'Color', blue, 'LineWidth', lw);
grid on; box on;
xlabel('$t$ [s]')
ylabel('$f^b$ [N]')
legend('$f_x^b$', '$f_y^b$', '$f_z^b$', 'Location', 'northeast')
title('$f^b$')
set(gca, 'FontSize', fs);
xlim([0, 70]);


subplot(1, 2, 2)
plot(time, w_b(:,4), 'Color', red, 'LineWidth', lw); hold on;
plot(time, w_b(:,5), 'Color', green, 'LineWidth', lw);
plot(time, w_b(:,6), 'Color', blue, 'LineWidth', lw);
grid on; box on;
xlabel('$t$ [s]')
ylabel('$\tau^b$ [Nm]')
legend('$\tau^b_\phi$', '$\tau^b_\theta$', '$\tau^b_\psi$', 'Location', 'northeast')
title('$\tau^b$')
set(gca, 'FontSize', fs);
set(gcf, 'color', 'w');
xlim([0, 70]);


%% FIGURE 3: Control Inputs (Actuators Allocation)
h3 = figure('Position', [100 100 1000 400]);
plot(time_w, w(:,1), 'Color', red, 'LineWidth', lw); hold on;
plot(time_w, w(:,2), 'Color', green, 'LineWidth', lw);
plot(time_w, w(:,3), 'Color', blue, 'LineWidth', lw); 
plot(time_w, w(:,4), 'Color', dark_grey, 'LineWidth', lw); 
grid on; box on;
xlabel('$t$ [s]')
ylabel('$\omega$ [rad/s]')
legend('$\omega_1$', '$\omega_2$', '$\omega_3$', '$\omega_4$', 'Location', 'northeast')
title('Spinning velocities')
set(gca, 'FontSize', fs);

if strcmp(model_name, "OpenAUV_v2OBS")
    
    % Data extraction
    T_real_data = squeeze(out.T_real.signals.values)';
    w_dist_hat = squeeze(out.w_dist_hat.signals.values)';
    time_w_dist_hat = out.w_dist_hat.time;
    
    %% FIGURE 4: Thrusters 1-4 (u vs T_real)
    figure('Position', [100 100 700 500], 'Name', 'Thrusters 1-4', 'Color', 'w');
    for i = 1:4
        subplot(2, 2, i);
        plot(out.u.time, u_data(:, i), '--', 'Color', colors{i}, 'LineWidth', lw); hold on;
        plot(out.T_real.time, T_real_data(:, i), 'Color', colors{i}, 'LineWidth', lw);
        grid on; box on;
        title(['Thruster ', num2str(i)]);
        legend(['$u_{', num2str(i), '}$ (Des)'], ['$T_{', num2str(i), '}$ (Act)'], 'Location', 'northeast');
        xlabel('$t$ [s]'); ylabel(['$T_{', num2str(i), '}$ [N]']); 
        xlim([0, 30]);
    end
    
    %% FIGURE 5: Thrusters 5-8 (u vs T_real)
    figure('Position', [100 100 700 500], 'Name', 'Thrusters 5-8', 'Color', 'w');
    for i = 5:8
        subplot(2, 2, i-4);
        plot(out.u.time, u_data(:, i), '--', 'Color', colors{i}, 'LineWidth', lw); hold on;
        plot(out.T_real.time, T_real_data(:, i), 'Color', colors{i}, 'LineWidth', lw);
        grid on; box on;
        title(['Thruster ', num2str(i)]);
        legend(['$u_{', num2str(i), '}$ (Des)'], ['$T_{', num2str(i), '}$ (Act)'], 'Location', 'northeast');
        xlabel('$t$ [s]'); ylabel(['$T_{', num2str(i), '}$ [N]']); 
        xlim([0, 30]);
    end

   %% FIGURE 6: Estimated Disturbances
    h6 = figure('Position', [100 100 1000 400]);
    subplot(1, 2, 1)
    plot(time_w_dist_hat, w_dist_hat(:,1), 'Color', red, 'LineWidth', lw); hold on;
    plot(time_w_dist_hat, w_dist_hat(:,2), 'Color', green, 'LineWidth', lw);
    plot(time_w_dist_hat, w_dist_hat(:,3), 'Color', blue, 'LineWidth', lw); 
    legend('$\hat{f}_{dist,x}$', '$\hat{f}_{dist,y}$', '$\hat{f}_{dist,z}$', 'Location', 'northeast', 'Interpreter', 'latex');    
    xlabel('$t$ [s]'); 
    ylabel('Disturbance [N]');
    grid on; box on;
    title('Force Disturbances');
    set(gca, 'FontSize', fs);
    set(gcf, 'color', 'w');
    xlim([0, 30]);

    subplot(1, 2, 2)
    plot(time_w_dist_hat, w_dist_hat(:,4), 'Color', red, 'LineWidth', lw); hold on;
    plot(time_w_dist_hat, w_dist_hat(:,5), 'Color', green, 'LineWidth', lw);
    plot(time_w_dist_hat, w_dist_hat(:,6), 'Color', blue, 'LineWidth', lw);
    grid on; box on;
    xlabel('$t$ [s]')
    ylabel('Disturbance [N]');
    legend('$\hat{\tau}_{dist,x}$', '$\hat{\tau}_{dist,y}$', '$\hat{\tau}_{dist,z}$', 'Location', 'northeast', 'Interpreter', 'latex');    
    ylabel('Disturbance [N]');
    grid on; box on;
    title('Torque Disturbances');
    set(gca, 'FontSize', fs);
    set(gcf, 'color', 'w');
    xlim([0, 30]);

end