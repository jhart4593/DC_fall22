num = [0 0 0 1];
den = [1 11 10 0];
sys = tf(num,den);

% bode(sys)
% margin(sys)

bode(21*sys)
margin(21*sys)