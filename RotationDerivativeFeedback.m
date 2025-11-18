function [Fc, Gc] = RotationDerivativeFeedback(var, m, g)

p = var(10);
q = var(11);
r = var(12);

% Control moments (damping torque)
k = 0.004;  % Nm/(rad/s)
Gc = -k * [p; q; r];

% Control force in body z (balance weight)
Fc = [0; 0; m * g];


