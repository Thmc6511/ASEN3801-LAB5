clc
clear
close all

ttwistor; % Load in aircraft parameters from ttwistor.m
ap = aircraft_parameters;

tspan = [0 200]; % time span for ode45
%% Problem 2.1
%{
% Define variables
x0 = zeros(12,1); % aircraft state
x0(3) = -1609.34;% altitude (m)
x0(7) = 21.0;% u (m/s)
u = [0; 0; 0; 0];% control surface deflections in rad
wind = [0; 0; 0];% inertial wind (m/s)
%}

%% Problem 2.2
%{
x0 = [0; 0; -1800; 0; 0.02780; 0; 20.99; 0; 0.5837; 0; 0; 0];
u = [0.1079; 0; 0; 0.3182];
wind = [0; 0; 0];
%}

%% Problem 2.3
x0 = [0; 0; -1800; deg2rad(15); deg2rad(-12); deg2rad(270); 19; 3; -2; 0.08; -0.2; 0];
u = [deg2rad(5); deg2rad(2); deg2rad(-13); 0.3];
wind = [0; 0; 0];

%% Ode call
odefun = @(t,x) AircraftEOM(t, x, u, wind, ap);
[t, x] = ode45(odefun, tspan, x0);

fig = 1:6;

u_hist = repmat(u, 1, numel(t));
PlotAircraftSim(t, x, u_hist, fig, "b");