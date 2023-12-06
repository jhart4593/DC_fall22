syms w
z = (1+.05*w)/(1-.05*w);

G = (.0906*z + .082)/(4*z^2-6.89*z+2.96);
Gnew = simplify(G);

%G(w)
num = [-.0006205 -.2365 4.979];
den = [1 2.99 1.962];
sys = tf(num,den);

% margin(sys)

%KG(w) 
K = 2;
sys_K = K*sys;

% margin(sys_K)

%compensated system in w
numc = [.165 1];
denc = [.08 1];
sys_c = tf(numc,denc);

loop_sys = sys_c*sys_K;
% margin(loop_sys)

%compensator in z domain
numGz = [0 .0906 .082];
denGz = [4 -6.89 2.96];
Gz = tf(numGz,denGz,0.1);
numcz = [.86 -.46];
dencz = [.26 -.06];
sys_cz = tf(numcz,dencz,0.1);

loop_sysz = sys_cz*Gz;
margin(loop_sysz)