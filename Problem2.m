clc
clear
close all

%% Problem 2.1
% Define variables
x0 = zeros(12,1); % aircraft state
x0(3) = -1609.34;% altitude (m)
x0(7) = 21.0;% u (m/s)
u = [0; 0; 0; 0];% control surface deflections in rad
wind = [0; 0; 0];% inertial wind (m/s)

ttwistor; % Load in aircraft parameters from ttwistor.m
ap = aircraft_parameters;

tspan = [0 200]; % time span for ode45

%% Ode call
odefun = @(t,x) AircraftEOM(t, x, u, wind, ap);
[t, x] = ode45(odefun, tspan, x0);

fig = 1:6;

u_hist = repmat(u, 1, numel(t));
PlotAircraftSim(t, x, u_hist, fig, "b");