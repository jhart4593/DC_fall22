num = [0 0 10];
den = [1 11 10];
sysc = tf(num,den);
sysd = c2d(sysc,.05)

num1 = [0 0 0 10];
den1 = [1 11 10 0];