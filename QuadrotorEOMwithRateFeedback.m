function var_dot = QuadrotorEOMwithRateFeedback(t,var,g,m,I,nu,mu)
[Fc Gc] = RotationDerivativeFeedback(var, m, g);
% Extract states
x_e = var(1);
y_e = var(2);
z_e = var(3);
phi = var(4);
theta = var(5);
psi = var(6);
u_e = var(7);
v_e = var(8);
w_e = var(9);
p = var(10);
q = var(11);
r = var(12);
% Extract inertia
Ix = I(1,1); 
Iy = I(2,2); 
Iz = I(3,3);
% Rotational Derivatives
Va = sqrt(u_e^2 + v_e^2 + w_e^2);
X = -nu*Va*u_e;
Y = -nu*Va*v_e;
Z = -nu*Va*w_e;
% Compute control forces and moments
Z_c = -Fc(3);
L_c = Gc(1);
M_c = Gc(2);
N_c = Gc(3);
% Translational accelerations (body frame)
u_dot = r*v_e - q*w_e - g*sin(theta) + (X/m);
v_dot = p*w_e - r*u_e + g*sin(phi)*cos(theta) + (Y/m);
w_dot = q*u_e - p*v_e + g*cos(phi)*cos(theta) + (Z/m) + (Z_c/m);
% Angular accelerations
p_dot = (L_c - (Iz - Iy)*q*r)/Ix;
q_dot = (M_c - (Ix - Iz)*p*r)/Iy;
r_dot = (N_c - (Iy - Ix)*p*q)/Iz;
% Euler angle rate kinematics
E = [1, sin(phi)*tan(theta),  cos(phi)*tan(theta);
    0, cos(phi),            -sin(phi);
    0, sin(phi)/cos(theta),  cos(phi)/cos(theta)];
ang_rates = E * [p; q; r];
phi_dot   = ang_rates(1);
theta_dot = ang_rates(2);
psi_dot   = ang_rates(3);
% Rotation matrix from body to inertial frame
R = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
    cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
   -sin(theta),          sin(phi)*cos(theta),                            cos(phi)*cos(theta)];
vel_inertial = R * [u_e; v_e; w_e];
x_dot = vel_inertial(1);
y_dot = vel_inertial(2);
z_dot = vel_inertial(3);
% Output derivative vector
var_dot = [x_dot; y_dot; z_dot; phi_dot; theta_dot; psi_dot; u_dot; v_dot; w_dot; p_dot; q_dot; r_dot];


end