num = [0 0 0 5];
den = [.238 1.0476 1 0];
sys = tf(num,den);

bode(sys)
margin(sys)