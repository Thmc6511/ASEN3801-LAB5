%% ASEN 3801 Lab 4 - Quadrotor Simulation and Control
% Main simulation script
clear; clc; close all;

%% Quadrotor Parameters (Parrot Mambo Minidrone)
g = 9.81;           % gravity (m/s^2)
m = 0.068;          % mass (kg)
d = 0.06;           % distance from CG to rotor (m)
km = 0.0024;        % control moment coefficient (N·m/N)
nu = 1e-3;          % aerodynamic force coefficient (N/(m/s)^2)
mu = 2e-6;          % aerodynamic moment coefficient (N·m/(rad/s)^2)
I = diag([5.8e-5, 7.2e-5, 1.0e-4]); % inertia matrix (kg·m^2)

% Initial Conditions (Hover Trim)
% State vector: [x y z phi theta psi u v w p q r]'
var0 = zeros(12,1); % Start at origin with zero velocities/angles

% Trim motor forces for hover
motor_forces_trim = (m*g/4) * ones(4,1);

% Simulation Parameters
tspan = [0 10];     % simulation time (s)
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
T = load("RSdata_nocontrol.mat");
time = T.rt_estim.time;
var = T.rt_estim.signals.values;
%% Task 1: Nonlinear Simulation (Hover)

% Trim motor forces for hover
motor_forces_trim = (m*g/4) * ones(4,1);
fprintf('Running Task 1: Nonlinear Hover Simulation...\n');
tspan = [0 10];
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
[t_nl, y_nl] = ode45(@(t,var) QuadrotorEOM(t,var,g,m,I,d,km,nu,mu,motor_forces_trim), ...
                     tspan, var0, options);

% Calculate control inputs for plotting
control_inputs_nl = zeros(length(t_nl), 4);
for i = 1:length(t_nl)
    f1 = motor_forces_trim(1);
    f2 = motor_forces_trim(2);
    f3 = motor_forces_trim(3);
    f4 = motor_forces_trim(4);
    
    Zc = -f1 - f2 - f3 - f4;
    Lc = (d/sqrt(2)) * (-f1 - f2 + f3 + f4);
    Mc = (d/sqrt(2)) * (f1 - f2 - f3 + f4);
    Nc = km * (f1 - f2 + f3 - f4);
    
    control_inputs_nl(i,:) = [Zc, Lc, Mc, Nc];
end

% Plot results
fig_numbers = [1 2 3 4 5 6];
PlotAircraftSim(t_nl, y_nl, control_inputs_nl, fig_numbers, 'b-');

%% Task 2: Linearized Simulation
fprintf('Running Task 2: Linearized Simulation...\n');

% Linearized about hover trim
deltaFc = [0; 0; 0];  % No control force deviation
deltaGc = [0; 0; 0];  % No control moment deviation

[t_lin, y_lin] = ode45(@(t,var) QuadrotorEOM_Linearized(t,var,g,m,I,deltaFc,deltaGc), ...
                       tspan, var0, options);

%% Task 2.3: Rotation Derivative Feedback
fprintf('Running Task 2.3: Rotation Derivative Feedback...\n');

% Test with initial angular rate disturbance
var0_disturbed = var0;
var0_disturbed(10) = 0.1; % +0.1 rad/s roll rate

% Without control
[t_nl_dist, y_nl_dist] = ode45(@(t,var) QuadrotorEOM(t,var,g,m,I,d,km,nu,mu,motor_forces_trim), ...
                              tspan, var0_disturbed, options);

% With control - need to create the controlled EOM function
% [t_controlled, y_controlled] = ode45(@(t,var) QuadrotorEOMwithRateFeedback(t,var,g,m,I,nu,mu), ...
%                                     tspan, var0_disturbed, options);
%% 2.5
% d
var01 = [0 0 0 0 0 0 0 0 0 0.1 0 0]; % Start at origin with zero velocities/angles

% Trim motor forces for hover
motor_forces_trim = (m*g/4) * ones(4,1);
fprintf('Running Task 2: Nonlinear Hover Simulation...\n');
tspan = [0 10];
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
[t_nl, y_nl] = ode45(@(t,var) QuadrotorEOMwithRateFeedback(t,var,g,m,I,nu,mu), tspan, var01, options);

% Calculate control inputs for plotting
control_inputs_nl = zeros(length(t_nl), 4);
for i = 1:length(t_nl)
    f1 = motor_forces_trim(1);
    f2 = motor_forces_trim(2);
    f3 = motor_forces_trim(3);
    f4 = motor_forces_trim(4);
    
    Zc = -f1 - f2 - f3 - f4;
    Lc = (d/sqrt(2)) * (-f1 - f2 + f3 + f4);
    Mc = (d/sqrt(2)) * (f1 - f2 - f3 + f4);
    Nc = km * (f1 - f2 + f3 - f4);
    
    control_inputs_nl(i,:) = [Zc, Lc, Mc, Nc];
end

% Plot results
fig_numbers = [1 2 3 4 5 6];
PlotAircraftSim(t_nl, y_nl, control_inputs_nl, fig_numbers, 'b-');
%% e

var02 = [0 0 0 0 0 0 0 0 0 0 0.1 0]; % Start at origin with zero velocities/angles

% Trim motor forces for hover
motor_forces_trim = (m*g/4) * ones(4,1);
fprintf('Running Task 2: Nonlinear Hover Simulation...\n');
tspan = [0 10];
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
[t_nl, y_nl] = ode45(@(t,var) QuadrotorEOMwithRateFeedback(t,var,g,m,I,nu,mu), tspan, var02, options);

% Calculate control inputs for plotting
control_inputs_nl = zeros(length(t_nl), 4);
for i = 1:length(t_nl)
    f1 = motor_forces_trim(1);
    f2 = motor_forces_trim(2);
    f3 = motor_forces_trim(3);
    f4 = motor_forces_trim(4);
    
    Zc = -f1 - f2 - f3 - f4;
    Lc = (d/sqrt(2)) * (-f1 - f2 + f3 + f4);
    Mc = (d/sqrt(2)) * (f1 - f2 - f3 + f4);
    Nc = km * (f1 - f2 + f3 - f4);
    
    control_inputs_nl(i,:) = [Zc, Lc, Mc, Nc];
end

% Plot results
fig_numbers = [1 2 3 4 5 6];
PlotAircraftSim(t_nl, y_nl, control_inputs_nl, fig_numbers, 'b-');
%% f

var03 = [0 0 0 0 0 0 0 0 0 0 0 0.1]; % Start at origin with zero velocities/angles

% Trim motor forces for hover
motor_forces_trim = (m*g/4) * ones(4,1);
fprintf('Running Task 1: Nonlinear Hover Simulation...\n');
tspan = [0 10];
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
[t_nl, y_nl] = ode45(@(t,var) QuadrotorEOMwithRateFeedback(t,var,g,m,I,nu,mu), tspan, var03, options);

% Calculate control inputs for plotting
control_inputs_nl = zeros(length(t_nl), 4);
for i = 1:length(t_nl)
    f1 = motor_forces_trim(1);
    f2 = motor_forces_trim(2);
    f3 = motor_forces_trim(3);
    f4 = motor_forces_trim(4);
    
    Zc = -f1 - f2 - f3 - f4;
    Lc = (d/sqrt(2)) * (-f1 - f2 + f3 + f4);
    Mc = (d/sqrt(2)) * (f1 - f2 - f3 + f4);
    Nc = km * (f1 - f2 + f3 - f4);
    
    control_inputs_nl(i,:) = [Zc, Lc, Mc, Nc];
end

% Plot results
fig_numbers = [1 2 3 4 5 6];
PlotAircraftSim(t_nl, y_nl, control_inputs_nl, fig_numbers, 'b-');
%% Lab Task 3
%% 3.3a
var04 = [0 0 0 5 0 0 0 0 0 0 0 0];
tspan = [0 10];
[t_linc, y_linc] = ode45(@(t,var) QuadrotorEOMclosed_Linearized(t,var,g,m,I),tspan, var04, options);
control_inputs_linc = zeros(length(t_linc), 4);
for i = 1:length(t_linc)
    [Fc, Gc] = InnerLoopFeedback(y_linc(i,:)');
    Zc = Fc(3);
    Lc = Gc(1);
    Mc = Gc(2);
    Nc = Gc(3);
    control_inputs_linc(i,:) = [Zc Lc Mc Nc];
end
fig_numbers = [1 2 3 4 5 6];
PlotAircraftSim(t_linc, y_linc, control_inputs_linc, fig_numbers, 'b-');
%% 3.3b

var05 = [0 0 0 0 5 0 0 0 0 0 0 0];
tspan = [0 10];
[t_linc, y_linc] = ode45(@(t,var) QuadrotorEOMclosed_Linearized(t,var,g,m,I),tspan, var05, options);
control_inputs_linc = zeros(length(t_linc), 4);
for i = 1:length(t_linc)
    [Fc, Gc] = InnerLoopFeedback(y_linc(i,:)');
    Zc = Fc(3);
    Lc = Gc(1);
    Mc = Gc(2);
    Nc = Gc(3);
    control_inputs_linc(i,:) = [Zc Lc Mc Nc];
end
fig_numbers = [1 2 3 4 5 6];
PlotAircraftSim(t_linc, y_linc, control_inputs_linc, fig_numbers, 'b-');
%% 3.3c
var06 = [0 0 0 0 0 0 0 0 0 0.1 0 0];
tspan = [0 10];
[t_linc, y_linc] = ode45(@(t,var) QuadrotorEOMclosed_Linearized(t,var,g,m,I),tspan, var06, options);
control_inputs_linc = zeros(length(t_linc), 4);
for i = 1:length(t_linc)
    [Fc, Gc] = InnerLoopFeedback(y_linc(i,:)');
    Zc = Fc(3);
    Lc = Gc(1);
    Mc = Gc(2);
    Nc = Gc(3);
    control_inputs_linc(i,:) = [Zc Lc Mc Nc];
end
fig_numbers = [1 2 3 4 5 6];
PlotAircraftSim(t_linc, y_linc, control_inputs_linc, fig_numbers, 'b-');
%% 3.3d
var07 = [0 0 0 5 0 0 0 0 0 0 0.1 0];
tspan = [0 10];
[t_linc, y_linc] = ode45(@(t,var) QuadrotorEOMclosed_Linearized(t,var,g,m,I),tspan, var07, options);
control_inputs_linc = zeros(length(t_linc), 4);
for i = 1:length(t_linc)
    [Fc, Gc] = InnerLoopFeedback(y_linc(i,:)');
    Zc = Fc(3);
    Lc = Gc(1);
    Mc = Gc(2);
    Nc = Gc(3);
    control_inputs_linc(i,:) = [Zc Lc Mc Nc];
end
fig_numbers = [1 2 3 4 5 6];
PlotAircraftSim(t_linc, y_linc, control_inputs_linc, fig_numbers, 'b-');
%% 3.4a
[t_linc, y_linc] = ode45(@(t,var) QuadrotorEOMclosed(t,var,g,m,I,d,km,nu,mu,motor_forces_trim),tspan, var04, options);
control_inputs_linc = zeros(length(t_linc), 4);
for i = 1:length(t_linc)
    [Fc, Gc] = InnerLoopFeedback(y_linc(i,:)');
    Zc = Fc(3);
    Lc = Gc(1);
    Mc = Gc(2);
    Nc = Gc(3);
    control_inputs_linc(i,:) = [Zc Lc Mc Nc];
end
fig_numbers = [1 2 3 4 5 6];
PlotAircraftSim(t_linc, y_linc, control_inputs_linc, fig_numbers, 'b-');
%% 3.4b
[t_linc, y_linc] = ode45(@(t,var) QuadrotorEOMclosed(t,var,g,m,I,d,km,nu,mu,motor_forces_trim),tspan, var05, options);
control_inputs_linc = zeros(length(t_linc), 4);
for i = 1:length(t_linc)
    [Fc, Gc] = InnerLoopFeedback(y_linc(i,:)');
    Zc = Fc(3);
    Lc = Gc(1);
    Mc = Gc(2);
    Nc = Gc(3);
    control_inputs_linc(i,:) = [Zc Lc Mc Nc];
end
fig_numbers = [1 2 3 4 5 6];
PlotAircraftSim(t_linc, y_linc, control_inputs_linc, fig_numbers, 'b-');

%% 3.4c
[t_linc, y_linc] = ode45(@(t,var) QuadrotorEOMclosed(t,var,g,m,I,d,km,nu,mu,motor_forces_trim),tspan, var06, options);
control_inputs_linc = zeros(length(t_linc), 4);
for i = 1:length(t_linc)
    [Fc, Gc] = InnerLoopFeedback(y_linc(i,:)');
    Zc = Fc(3);
    Lc = Gc(1);
    Mc = Gc(2);
    Nc = Gc(3);
    control_inputs_linc(i,:) = [Zc Lc Mc Nc];
end
fig_numbers = [1 2 3 4 5 6];
PlotAircraftSim(t_linc, y_linc, control_inputs_linc, fig_numbers, 'b-');

%% 3.4d
T = load("RSdata_nocontrol.mat");
time = T.rt_estim.time;
var = T.rt_estim.signals.values;
g = 9.81;           % gravity (m/s^2)
m = 0.068;          % mass (kg)
d = 0.06;           % distance from CG to rotor (m)
km = 0.0024;        % control moment coefficient (N·m/N)
nu = 1e-3;          % aerodynamic force coefficient (N/(m/s)^2)
mu = 2e-6;          % aerodynamic moment coefficient (N·m/(rad/s)^2)
I = diag([5.8e-5, 7.2e-5, 1.0e-4]); % inertia matrix (kg·m^2)
var0 = [0 0 0 0 0 0 0 0 0 0 0.1 0];
tspan = [0 10];
[t_linc, y_linc] = ode45(@(t,var) QuadrotorEOMclosed(t,var,g,m,I,d,km,nu,mu,motor_forces_trim),tspan, var0, options);
control_inputs_linc = zeros(length(t_linc), 4);
for i = 1:length(t_linc)
    [Fc, Gc] = InnerLoopFeedback(y_linc(i,:)');
    Zc = Fc(3);
    Lc = Gc(1);
    Mc = Gc(2);
    Nc = Gc(3);
    control_inputs_linc(i,:) = [Zc Lc Mc Nc];
end
fig_numbers = [1 2 3 4 5 6];
PlotAircraftSim(t_linc, y_linc, control_inputs_linc, fig_numbers, 'b-');


%% 3.5
% Parameters
g = 9.81;
Ix = 5.8e-5;
Iy = 7.2e-5;

% Inner-loop gains (example values from Problem 3.1)
K1 = 3.96e-5;    % angle feedback
K2 = 1.8e-6;   % rate feedback

% Range of K3 gains to test
K3_range = linspace(0, 5, 200);

eig_lat = zeros(3, length(K3_range));
eig_lon = zeros(3, length(K3_range));

for i = 1:length(K3_range)
    K3 = K3_range(i);

    % Lateral closed-loop matrix
    A_lat = [0, g, 0;
             0, 0, 1;
            -(K1*K3*g)/Ix, -K1/Ix, -K2/Ix];
    eig_lat(:,i) = eig(A_lat);

    % Longitudinal closed-loop matrix
    A_lon = [0, -g, 0;
             0, 0, 1;
             (K1*K3*g)/Iy, -K1/Iy, -K2/Iy];
    eig_lon(:,i) = eig(A_lon);
end
disp(K3)
figure;
plot(real(eig_lat(:)), imag(eig_lat(:)), 'b.', 'DisplayName','Lateral');
hold on;
plot(real(eig_lon(:)), imag(eig_lon(:)), 'r.', 'DisplayName','Longitudinal');
xlabel('Real Part'); ylabel('Imaginary Part');
title('Eigenvalue Locus vs K3');
legend; grid on;
%% 3.7

T = load("RSdata_nocontrol.mat");
time = T.rt_estim.time;
var = T.rt_estim.signals.values;
g = 9.81;           % gravity (m/s^2)
m = 0.068;          % mass (kg)
d = 0.06;           % distance from CG to rotor (m)
km = 0.0024;        % control moment coefficient (N·m/N)
nu = 1e-3;          % aerodynamic force coefficient (N/(m/s)^2)
mu = 2e-6;          % aerodynamic moment coefficient (N·m/(rad/s)^2)
I = diag([5.8e-5, 7.2e-5, 1.0e-4]); % inertia matrix (kg·m^2)
var0 = [0 0 0 0 0 0 0 0 0 0.1 0 0]; % Start at origin with zero velocities/angles

% Trim motor forces for hover
motor_forces_trim = (m*g/4) * ones(4,1);
fprintf('Running Task 2: Nonlinear Hover Simulation...\n');
tspan = [0 10];
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
[t_nl, y_nl] = ode45(@(t,var) QuadrotorEOMopen(t,var,g,m,I,nu,mu), tspan, var0, options);

% Calculate control inputs for plotting
control_inputs_nl = zeros(length(t_nl), 4);
for i = 1:length(t_nl)
    f1 = motor_forces_trim(1);
    f2 = motor_forces_trim(2);
    f3 = motor_forces_trim(3);
    f4 = motor_forces_trim(4);
    
    Zc = -f1 - f2 - f3 - f4;
    Lc = (d/sqrt(2)) * (-f1 - f2 + f3 + f4);
    Mc = (d/sqrt(2)) * (f1 - f2 - f3 + f4);
    Nc = km * (f1 - f2 + f3 - f4);
    
    control_inputs_nl(i,:) = [Zc, Lc, Mc, Nc];
end

% Plot results
fig_numbers = [1 2 3 4 5 6];
PlotAircraftSim(t_nl, y_nl, control_inputs_nl, fig_numbers, 'b-');
%% Display Results Summary
fprintf('\n=== Simulation Complete ===\n');
fprintf('Nonlinear simulation: %d time points\n', length(t_nl));
fprintf('Linearized simulation: %d time points\n', length(t_lin));
fprintf('Final position: x=%.3f m, y=%.3f m, z=%.3f m\n', ...
        y_nl(end,1), y_nl(end,2), y_nl(end,3));