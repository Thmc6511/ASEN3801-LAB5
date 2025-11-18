clc
clear
close all

%% Problem 2.1
%xdot = AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial,aircraft_parameters)

% Define variables
x0 = zeros(12,1); % aircraft state
x0(3) = -1609.34;% altitude (m)
x0(7) = 21.0;% u (m/s)
u = [0; 0; 0; 0];% control surface deflections in rad
wind = [0; 0; 0];% inertial wind (m/s)

ap = ttwistor();

tspan = [0 200]; % time span for ode45

%% Ode call
odefun = @(t,x) AircraftEOM_local(t, x, u, wind, ap);
opts = odeset('RelTol',1e-6, 'AbsTol',1e-8);
[t, x] = ode45(odefun, tspan, x0, opts);

