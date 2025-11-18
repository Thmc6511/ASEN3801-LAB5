function wind_body = TransformFromInertialToBody(wind_inertial, euler_angles)
phi   = euler_angles(1);
theta = euler_angles(2);
psi   = euler_angles(3);

cphi = cos(phi); sphi = sin(phi);
cth = cos(theta); sth = sin(theta);
cpsi = cos(psi); spsi = sin(psi);

R_bi = [ cth*cpsi,  sphi*sth*cpsi - cphi*spsi,  cphi*sth*cpsi + sphi*spsi;
         cth*spsi,  sphi*sth*spsi + cphi*cpsi,  cphi*sth*spsi - sphi*cpsi;
        -sth,       sphi*cth,                   cphi*cth ];

wind_body = R_bi' * wind_inertial;
end
