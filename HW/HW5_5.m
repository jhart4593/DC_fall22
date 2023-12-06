%initial system in state space form
A = [0 1 0;
    0 0 1;
    0 -.558 1.527];
B = [0;0;1];
C = [0.0142 0.0173 0];
D = 0;

sys = ss(A,B,C,D,5);  %state space system

P = [0.8+0.25j 0.8-0.25j 0.697];  %vector of desired poles
[K,PREC,message] = place(A,B,P)  %determine gain matrix to achieve desired poles

Acl = A-B*K; %new A matrix with closed loop poles
syscl = ss(Acl,B,C,D,5);  %closed loop system
Pcl = pole(syscl) %check if desired poles achieved

step(sys)
hold on
step(syscl)
legend('Initial System','Pole Placement Design')

