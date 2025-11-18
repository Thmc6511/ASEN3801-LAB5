function xdot = AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters)
% AircraftEOM
%   xdot = AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters)
%
%   aircraft_state   = [xi, yi, zi, roll, pitch, yaw, u, v, w, p, q, r]^T
%   aircraft_surfaces= [de, da, dr, dt]^T
%   wind_inertial    = [wx, wy, wz]^T in inertial coordinates
%   aircraft_parameters = struct from ttwistor.m
%
%   Output xdot is the 12x1 derivative of the state.
%   Uses AeroForcesAndMoments.m as required in the problem.

ap = aircraft_parameters;   %#ok<NASGU> (just shorthand)

% --- Unpack state ---
xi    = aircraft_state(1);
yi    = aircraft_state(2);
zi    = aircraft_state(3);
phi   = aircraft_state(4);
theta = aircraft_state(5);
psi   = aircraft_state(6);
u     = aircraft_state(7);
v     = aircraft_state(8);
w     = aircraft_state(9);
p     = aircraft_state(10);
q     = aircraft_state(11);
r     = aircraft_state(12);

% --- Constant density (approx at 1 mile ≈ 1609 m) ---
rho = 1.06;   % kg/m^3, you can refine this later if you want

% --- Aerodynamic + propulsive forces/moments in body frame ---
[aero_forces, aero_moments] = AeroForcesAndMoments(aircraft_state, ...
                                                   aircraft_surfaces, ...
                                                   wind_inertial, ...
                                                   rho, ...
                                                   aircraft_parameters);

X = aero_forces(1);
Y = aero_forces(2);
Z = aero_forces(3);

L = aero_moments(1);
M = aero_moments(2);
N = aero_moments(3);

% --- Parameters ---
m  = ap.m;
g  = ap.g;
Ix = ap.Ix;
Iy = ap.Iy;
Iz = ap.Iz;
Ixz = ap.Ixz;

% --- Translational dynamics in body frame (including gravity) ---
udot = r*v - q*w + X/m - g*sin(theta);
vdot = p*w - r*u + Y/m + g*sin(phi)*cos(theta);
wdot = q*u - p*v + Z/m + g*cos(phi)*cos(theta);

% --- Precompute inertia combinations (Gammas from Beard & McLain) ---
Gamma  = Ix*Iz - Ixz^2;
Gamma1 = (Ixz*(Ix - Iy + Iz))/Gamma;
Gamma2 = (Iz*(Iz - Iy) + Ixz^2)/Gamma;
Gamma3 = Iz/Gamma;
Gamma4 = Ixz/Gamma;
Gamma5 = (Iz - Ix)/Iy;
Gamma6 = Ixz/Iy;
Gamma7 = (Ix*(Ix - Iy) + Ixz^2)/Gamma;
Gamma8 = Ix/Gamma;

% --- Rotational dynamics ---
pdot = Gamma1*p*q - Gamma2*q*r + Gamma3*L + Gamma4*N;
qdot = Gamma5*p*r - Gamma6*(p^2 - r^2) + M/Iy;
rdot = Gamma7*p*q - Gamma1*q*r + Gamma4*L + Gamma8*N;

% --- Euler angle kinematics ---
phi_dot   = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
theta_dot = q*cos(phi) - r*sin(phi);
psi_dot   = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);

% --- Position kinematics: body -> inertial ---
cphi = cos(phi);   sphi = sin(phi);
cth  = cos(theta); sth  = sin(theta);
cpsi = cos(psi);   spsi = sin(psi);

R_bi = [ cth*cpsi,  sphi*sth*cpsi - cphi*spsi,  cphi*sth*cpsi + sphi*spsi;
         cth*spsi,  sphi*sth*spsi + cphi*cpsi,  cphi*sth*spsi - sphi*cpsi;
        -sth,       sphi*cth,                   cphi*cth ];

vel_inertial = R_bi * [u; v; w];

xidot = vel_inertial(1);
yidot = vel_inertial(2);
zidot = vel_inertial(3);

% --- Pack derivative vector ---
xdot = zeros(12,1);
xdot(1)  = xidot;
xdot(2)  = yidot;
xdot(3)  = zidot;
xdot(4)  = phi_dot;
xdot(5)  = theta_dot;
xdot(6)  = psi_dot;
xdot(7)  = udot;
xdot(8)  = vdot;
xdot(9)  = wdot;
xdot(10) = pdot;
xdot(11) = qdot;
xdot(12) = rdot;

end
