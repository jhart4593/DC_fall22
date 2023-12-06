syms z

T = 0.1;
% s = ((2*z-2)/(T*z+T));
zeta = [0.5];
w_n = [20];

eqn = ((2*z-2)/(T*z+T))^2+2*zeta*w_n*((2*z-2)/(T*z+T))+w_n^2 == 0;

vpasolve(eqn,z)