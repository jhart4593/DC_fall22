%transform NED to BODY
%phi, theta, psi are euler angles
%v_b = linear velocities in BODY [u,v,w]'
%w_nb = angular velocities in BODY [p,q,r]'
%to get pdot and theta_dot need to take derivative of NED positions?
%(etadot matrix)

phi = x(10);
theta = x(11);
psi = x(12);

R_ntob = Rzyx(phi,theta,psi)';  %rotation matrix transpose (which equals it's inverse)
v_b = R_ntob * pdot;            %pdot = linear velocities in NED [xdot,ydot,zdot]' 


inv_T = [1 0 -sin(theta);0 cos(phi) cos(theta)*sin(phi);  %inverse transformation matrix 
    0 -sin(phi) cos(theta)*cos(phi)];

w_nb = inv_T * theta_dot;            %theta_dot = angular velocities in NED [phidot,thetadot,psidot]'

    
       
