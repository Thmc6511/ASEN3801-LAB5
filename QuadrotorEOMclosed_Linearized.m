function var_dot = QuadrotorEOMclosed_Linearized(t, var, g, m, I)
[Fc,Gc] = InnerLoopFeedback(var);
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
%Controls
Zc = Fc(3);
Lc = Gc(1);
Mc = Gc(2);
Nc = Gc(3);
% Translational accelerations (inertial frame)
x_dot = u_e;
y_dot = v_e;
z_dot = w_e;
% Translational accelerations (body frame)
u_dot = - g*theta;
v_dot = g*phi;
w_dot = Zc/m;
% Angular accelerations
p_dot = (Lc)/Ix;
q_dot = (Mc)/Iy;
r_dot = (Nc)/Iz;
% Angle Derivatives
phi_dot   = p;
theta_dot = q;
psi_dot   = r;

% Output derivative vector
var_dot = [x_dot; y_dot; z_dot; phi_dot; theta_dot; psi_dot; u_dot; v_dot; w_dot; p_dot; q_dot; r_dot];
end

