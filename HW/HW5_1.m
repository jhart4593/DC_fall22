%find G(w)
% syms w
% z = (1+.05*w)/(1-.05*w);
% 
% G = (.01*z + .01)/(2*z^2-4*z+2);
% Gnew = simplify(G)

%G(w) bode plot
num = [0 -0.1 2];
den = [2 0 0];
sys = tf(num,den);

%margin(sys)

%KG(w) 
K = 5;
sys_K = K*sys;

%margin(sys_K)

%compensated system in w
numc = [.898 1];
denc = [.056 1];
sys_c = tf(numc,denc);

loop_sys = sys_c*sys_K;
%margin(loop_sys)

%compensator in z domain
numGz = [0 .01 .01];
denGz = [2 -4 2];
Gz = tf(numGz,denGz,0.1);
K = 5;
numcz = [1.896 -1.696];
dencz = [.212 -.012];
sys_cz = tf(numcz,dencz,0.1);

loop_sysz = K*sys_cz*Gz;
%margin(loop_sysz)

sys_fb = feedback(loop_sysz,1);
step(sys_fb)



