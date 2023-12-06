t = [0:10];
unitstep1 = t>=2;
unitstep2 = t>=6;

x = 0.25*unitstep1.*(t-2) - 0.25*unitstep2.*(t-6);
plot(t,x)

