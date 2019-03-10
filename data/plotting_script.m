% Variable for plot_options
clear
clf
clf
clf
coeff_together = 1;

% Load variables
load('samples.mat');

load('x_pos.mat')
load('roll_data.mat');
load('px_data.mat');
load('ix_data.mat');
load('dx_data.mat');

load('y_pos.mat');
load('pitch_data.mat');
load('py_data.mat');
load('iy_data.mat');
load('dy_data.mat');

load('z_pos.mat')
load('thrust_data.mat');
load('pz_data.mat');
load('iz_data.mat');
load('dz_data.mat');

load('curr_vel_data.mat')
load('vel_data.mat');
load('pvel_data.mat');
load('ivel_data.mat');
load('dvel_data.mat');

%load('feedforward_thrust_data.mat');
load('pid_x_sp_data.mat');
pid_x_sp_dat = cast(pid_x_sp_data,'double')*ones(1, length(samples));
load('pid_y_sp_data.mat');
pid_y_sp_dat = cast(pid_y_sp_data,'double')*ones(1, length(samples));
load('pid_z_sp_data.mat');
pid_z_sp_dat = cast(pid_z_sp_data,'double')*ones(1, length(samples));
load('pid_vel_sp_data.mat');
pid_vel_sp_dat = cast(pid_vel_sp_data,'double')*ones(1, length(samples));


% Plots from System
figure(1)
suptitle("X-Position PID-Controller")
if coeff_together == 0
    subplot(5,1,1)
    grid on
    hold on
    plot(samples, x_pos)
    plot(samples, pid_x_sp_dat)
    xlabel("Samples")
    ylabel("X-Position [m]")
    legend("Input","Setpoint")
    subplot(5,1,2)
    grid on
    hold on
    plot(samples, roll_data)
    xlabel("Samples")
    ylabel("Output Roll [degrees]")
    subplot(5,1,3)
    grid on
    hold on
    plot(samples, px_data)
    xlabel("Samples")
    ylabel("P")
    subplot(5,1,4)
    grid on
    hold on
    plot(samples, ix_data)
    xlabel("Samples")
    ylabel("I")
    subplot(5,1,5)
    grid on
    hold on
    plot(samples, dx_data)
    xlabel("Samples")
    ylabel("D")
else
    subplot(3,1,1)
    grid on
    hold on
    plot(samples, x_pos)
    plot(samples, pid_x_sp_dat)
    xlabel("Samples")
    ylabel("X-Position [m]")
    legend("Input","Setpoint")
    subplot(3,1,2)
    grid on
    hold on
    plot(samples, roll_data)
    xlabel("Samples")
    ylabel("Output Roll [degrees]")
    subplot(3,1,3)
    grid on
    hold on
    plot(samples, px_data)
    plot(samples, ix_data)
    plot(samples, dx_data)
    xlabel("Samples")
    ylabel("Coefficients")
    legend("P", "I", "D")
end

figure(2)
suptitle("Y-Position PID-Controller")
if coeff_together == 0
    subplot(5,1,1)
    grid on
    hold on
    plot(samples, y_pos)
    plot(samples, pid_y_sp_dat)
    xlabel("Samples")
    ylabel("Y-Position [m]")
    legend("Input","Setpoint")
    subplot(5,1,2)
    grid on
    hold on
    plot(samples, pitch_data)
    xlabel("Samples")
    ylabel("Output Pitch [degrees]")
    subplot(5,1,3)
    grid on
    hold on
    plot(samples, py_data)
    xlabel("Samples")
    ylabel("P")
    subplot(5,1,4)
    grid on
    hold on
    plot(samples, iy_data)
    xlabel("Samples")
    ylabel("I")
    subplot(5,1,5)
    grid on
    hold on
    plot(samples, dy_data)
    xlabel("Samples")
    ylabel("D")
else
    subplot(3,1,1)
    grid on
    hold on
    plot(samples, y_pos)
    plot(samples, pid_y_sp_dat)
    xlabel("Samples")
    ylabel("Y-Position [m]")
    legend("Input","Setpoint")
    subplot(3,1,2)
    grid on
    hold on
    plot(samples, pitch_data)
    xlabel("Samples")
    ylabel("Output Pitch [degrees]")
    subplot(3,1,3)
    grid on
    hold on
    plot(samples, py_data)
    plot(samples, iy_data)
    plot(samples, dy_data)
    xlabel("Samples")
    ylabel("Coefficients")
    legend("P", "I", "D")
end

figure(3)
suptitle("Z-Position PID-Controller")
if coeff_together == 0
    subplot(5,1,1)
    grid on
    hold on
    plot(samples, z_pos)
    plot(samples, pid_z_sp_dat)
    xlabel("Samples")
    ylabel("Z-Position [m]")
    legend("Input","Setpoint")
    subplot(5,1,2)
    grid on
    hold on
    plot(samples, thrust_data)
    xlabel("Samples")
    ylabel("Output Thrust [%]")
    subplot(5,1,3)
    grid on
    hold on
    plot(samples, pz_data)
    xlabel("Samples")
    ylabel("P")
    subplot(5,1,4)
    grid on
    hold on
    plot(samples, iz_data)
    xlabel("Samples")
    ylabel("I")
    subplot(5,1,5)
    grid on
    hold on
    plot(samples, dz_data)
    xlabel("Samples")
    ylabel("D")
else
    subplot(3,1,1)
    grid on
    hold on
    plot(samples, z_pos)
    plot(samples, pid_z_sp_dat)
    xlabel("Samples")
    ylabel("Z-Position [m]")
    legend("Input","Setpoint")
    subplot(3,1,2)
    grid on
    hold on
    plot(samples, thrust_data)
    xlabel("Samples")
    ylabel("Output Thrust [%]")
    subplot(3,1,3)
    grid on
    hold on
    plot(samples, pz_data)
    plot(samples, iz_data)
    plot(samples, dz_data)
    xlabel("Samples")
    ylabel("Coefficients")
    legend("P", "I", "D")
end

figure(4)
suptitle("Z-Velocity PID-Controller")
if coeff_together == 0
    subplot(5,1,1)
    grid on
    hold on
    plot(samples, curr_vel_data)
    plot(samples, pid_vel_sp_dat)
    xlabel("Samples")
    ylabel("Vel [m/s]")
    legend("Input","Setpoint")
    subplot(5,1,2)
    grid on
    hold on
    plot(samples, thrust_data)
    xlabel("Samples")
    ylabel("Output Thrust [%]")
    subplot(5,1,3)
    grid on
    hold on
    plot(samples, pz_data)
    xlabel("Samples")
    ylabel("P")
    subplot(5,1,4)
    grid on
    hold on
    plot(samples, iz_data)
    xlabel("Samples")
    ylabel("I")
    subplot(5,1,5)
    grid on
    hold on
    plot(samples, dz_data)
    xlabel("Samples")
    ylabel("D")
else
    subplot(3,1,1)
    grid on
    hold on
    plot(samples, z_pos)
    plot(samples, pid_z_sp_dat)
    plot(samples, vel_data)
    xlabel("Samples")
    ylabel("Z-Position [m]")
    legend("Input","Setpoint", "Velocity-Out")
    subplot(3,1,2)
    grid on
    hold on
    plot(samples, vel_data)
    plot(samples, thrust_data + vel_data)
    xlabel("Samples")
    ylabel("Output Thrust [%]")
    subplot(3,1,3)
    grid on
    hold on
    plot(samples, pz_data)
    plot(samples, iz_data)
    plot(samples, dz_data)
    plot(samples, pvel_data)
    plot(samples, ivel_data)
    plot(samples, dvel_data)
    xlabel("Samples")
    ylabel("Coefficients")
    legend("P","I","D","Pvel","Ivel","Dvel")
end



