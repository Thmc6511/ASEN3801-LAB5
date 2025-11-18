function motor_forces = ComputeMotorForces(Fc, Gc, d, km)
% ComputeMotorForces
% Calculates individual rotor thrusts given control force & moments.
%
% INPUTS:
%   Fc  - 3x1 control force vector [Fx; Fy; Fz]  (only Fz used)
%   Gc  - 3x1 control moment vector [L; M; N]
%   d   - distance from CG to each rotor (m)
%   km  - control moment coefficient (N·m / N)
%
% OUTPUT:
%   motor_forces - 4x1 vector [f1; f2; f3; f4] of rotor thrusts (N)

    % Extract relevant control variables
    Zc = Fc(3);   % body z control force (usually -sum of thrusts)
    Lc = Gc(1);   % roll moment
    Mc = Gc(2);   % pitch moment
    Nc = Gc(3);   % yaw moment

    % Construct control allocation matrix (from lab handout)
    A = [ -1,  -1,  -1,  -1;
          -d/sqrt(2), -d/sqrt(2),  d/sqrt(2),  d/sqrt(2);
           d/sqrt(2), -d/sqrt(2), -d/sqrt(2),  d/sqrt(2);
           km, -km, km, -km ];

    % Desired control vector
    control_vec = [Zc; Lc; Mc; Nc];

    % Solve for motor thrusts
    motor_forces = A \ control_vec;  % equivalent to inv(A)*control_vec
end
