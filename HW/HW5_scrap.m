% syms z
% w = (2*z-2)/(0.1*z+0.1);
% G = (.898*w+1)/(.056*w+1);
% Gnew = simplify(G)

syms v
eqn1 = sqrt((-.006205*v^2-2.365*v)^2+49.79^2)/sqrt((v^2+2.99*v)^2+1.962^2)==0.27;
vpasolve(eqn1,v)

% syms z
% G = (z-1)/z*(5*z/(z-1)-5*z/(z-exp(-0.1))+5*z/(2*z-2*exp(-0.2)));
% Gnew = simplify(G)