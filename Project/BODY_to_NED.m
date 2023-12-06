%transform BODY to NED
%pdot = linear velocities in NED
%theta_dot = angular velocities in NED
%integrate these to get positions and angles in NED?
%eulerang function is Fossen's that calculates J matrix where etadot = J*nu
%nu = linear and angular velocities in BODY
%etadot = vector of pdot and theta_dot

[J,J11,J22] = eulerang(x(10),x(11),x(12));  %x(10-12) are phi, theta, psi - euler angles (pg 31)
pdot = J11 * v_b;                           %v_b = [u,v,w]', linear velocities in BODY
theta_dot = J22 * w_nb;                     %w_nb = [p,q,r]', angular velocities in BODY
