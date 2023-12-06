num = [500 2000];
den = [1 80];
C = tf(num,den)
Cz = c2d(C,0.1)
Cz1 = c2d(C,0.02)