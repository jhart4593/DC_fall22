num = 1;
den = [1 0 0];
sys = tf(num,den,'InputDelay',0.1)
syms s t z
F = 1/s^2;
f = ilaplace(F,s,t)
zee = ztrans(f,t,z)
