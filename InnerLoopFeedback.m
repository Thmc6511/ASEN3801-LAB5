function [Fc, Gc] = InnerLoopFeedback(var)
phi = var(4);
theta = var(5);
p = var(10);
q = var(11);
r = var(12);
g = 9.81;
m = 0.068;
Zc = m*g;
Ix = 5.800000000000000e-05;
Iy = 7.200000000000000e-05;
lambda1 = -0.5;
lambda2 = -0.05;
a0 = -(lambda1+lambda2);
a1 = lambda1*lambda2;
kp = a1*Ix;
kphi = a0*Ix;
kq = a1*Iy;
ktheta = a0*Iy;
kr = 0.004;
Lc = (-kphi*phi)-(kp*p);
Mc = (-ktheta*theta)-(kq*q);
Nc = -kr*r;
disp(kphi)
disp(kp)
disp(ktheta)
disp(kq)
Fc = [0 0 Zc];
Gc = [Lc Mc Nc];