% Problem 3 
% (Simulating Aircraft Longitudinal Dynamics using ode45) with an elevator doublet input.

clc; clear; close all;

% givens
% Doublet Parameters
doublet_size = deg2rad(15); % Doublet size: 15 degrees, into radians
doublet_time = 0.25;      % Duration of each half-pulse

% Define Trim and Initial State (from Problem 2.2/typical linear dynamics)
% x = [u, w, q, theta]' (Perturbation state)
% The trim *perturbation* state x_trim is zero.
x_trim = zeros(4, 1);     % Assuming trim state is the origin (zero perturbations)
% From Problem 2.2, the trim elevator input is 0.1079 rad
delta_e_trim = 0.1079; 

% Initial state for the simulation (Aircraft starts at trim state)
x0 = x_trim;

% A is the 4x4 stability matrix
A = [-0.1, 0, 0, -9.81; 
     0, -0.5, 300, 0;
     0, -0.01, -2, 0;
     0, 0, 1, 0]; 
% B is the 4x1 control input matrix
B = [0; -0.1; -1.5; 0]; 

%parameters into a struct to pass to the EOM function
P.doublet_size = doublet_size;
P.doublet_time = doublet_time;
P.delta_e_trim = delta_e_trim;
P.x_trim = x_trim;
P.A = A; % Placeholder matrix
P.B = B; % Placeholder matrix

%tolerance
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);

%Short Period (SP) Mode with tspan --> 3 seconds
disp('Starting Short Period Simulation (0 to 3 seconds)...');
t_span_sp = [0 3]; % Short time span to observe the fast Short Period mode

% ode45 call
[t_sp, x_sp] = ode45(@(t, x) AircraftEQMDoulet(t, x, P), t_span_sp, x0, options);

%plots Short Period Results
figure(1);
subplot(2, 2, 1);
plot(t_sp, x_sp(:, 1)); title('Perturbation Velocity u (m/s)'); ylabel('u'); grid on;
subplot(2, 2, 2);
plot(t_sp, x_sp(:, 2)); title('Perturbation Velocity w (m/s)'); ylabel('w'); grid on;
subplot(2, 2, 3);
plot(t_sp, rad2deg(x_sp(:, 3))); title('Pitch Rate q (deg/s)'); ylabel('q'); xlabel('Time (s)'); grid on;
subplot(2, 2, 4);
plot(t_sp, rad2deg(x_sp(:, 4))); title('Pitch Angle \theta (deg)'); ylabel('\theta'); xlabel('Time (s)'); grid on;
sgtitle('Aircraft Longitudinal Response to Elevator Doublet (Short Period)');

disp('Short Period Analysis: The mode is most visible in q and w, and settles quickly (e.g., within the first 1-2 seconds).');
disp('To estimate natural frequency (\omega_n) and damping ratio (\zeta):');
disp('  1. Use the pitch rate (q) plot.');
disp('  2. Measure the period (T) of oscillation to find damped natural frequency: \omega_d = 2\pi/T.');
disp('  3. Use logarithmic decrement method to find \zeta from the decay envelope.');

% Phugoid Mode (phu) with tspan --> 100 seconds
disp('Starting Phugoid Simulation (0 to 100 seconds)...');
t_span_phu = [0 100]; % Long time span to observe the slow Phugoid mode

% ode45 call
[t_phu, x_phu] = ode45(@(t, x) AircraftEQMDoulet(t, x, P), t_span_phu, x0, options);

% plots Phugoid Results
figure(2);
subplot(2, 2, 1);
plot(t_phu, x_phu(:, 1)); title('Perturbation Velocity u (m/s)'); ylabel('u'); grid on;
subplot(2, 2, 2);
plot(t_phu, x_phu(:, 2)); title('Perturbation Velocity w (m/s)'); ylabel('w'); grid on;
subplot(2, 2, 3);
plot(t_phu, rad2deg(x_phu(:, 3))); title('Pitch Rate q (deg/s)'); ylabel('q'); xlabel('Time (s)'); grid on;
subplot(2, 2, 4);
plot(t_phu, rad2deg(x_phu(:, 4))); title('Pitch Angle \theta (deg)'); ylabel('\theta'); xlabel('Time (s)'); grid on;
sgtitle('Aircraft Longitudinal Response to Elevator Doublet (Phugoid)');

disp('Phugoid Analysis: The mode is most visible in u and \theta, and has a very long period.');
disp('Follow the same estimation steps as above, using the u or \theta plot over the 100s period.');


% Aircraft EOM (For Problem 3)
function xdot = AircraftEQMDoulet(t, x, P)

% Calculates the linearized longitudinal EOM for the aircraft with a doublet
% applied to the elevator.

% State vector x = [u, w, q, theta]' (perturbations)
% Control input is delta_e (perturbation from trim)

% Unpack parameters
delta_e_trim = P.delta_e_trim;
doublet_size = P.doublet_size;
doublet_time = P.doublet_time;
A = P.A;
B = P.B;

% 1. Calculate the elevator command delta_e(t) (total value) using the doublet logic
% Equation 7 on lab assignment
if t <= doublet_time
    delta_e_total = delta_e_trim + doublet_size;
elseif t <= 2 * doublet_time
    delta_e_total = delta_e_trim - doublet_size;
else
    delta_e_total = delta_e_trim;
end

% 2. Calculate the perturbation control input, delta_e_pert.
% The linearized EOM is $\dot{x} = Ax + B\delta_{e,pert}$, where $\delta_{e,pert} = \delta_{e,total} - \delta_{e,trim}$.
delta_e_pert = delta_e_total - delta_e_trim;

% 3. Calculate the state derivative (linearized EOM)
xdot = A * x + B * delta_e_pert;

end