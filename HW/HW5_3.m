syms a1 a0 b1 b0  

eqn1 = a0+b0 == 0;
eqn2 = 0.2*b0+b1-0.7*a0+a1 == 0;
eqn3 = 0.1*a1-0.7*a1-0.24*b0+0.2*b1 == 0;
eqn4 = 0.1*a1-0.24*b1 ==1;

sol = solve([eqn1,eqn2,eqn3,eqn4],[a1,a0,b1,b0]);
a1sol = sol.a1
a0sol = sol.a0
b1sol = sol.b1
b0sol = sol.b0


