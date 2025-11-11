%% 2.2

%% Initial Conditions
x0 = ...
    [0,0,-1800 %m
    deg2rad(15), deg2rad(-12), deg2rad(270) %rad
    19, 3, -2 %m/s
    deg2rad(0.08), deg2rad(-0.2) 0 %rad/s
    ]';
u0 = [deg2rad(5), deg2rad(2), deg2rad(-13), 0.3]'; %rad