% Digital Control
% homework1 Problem 2.1 Bode Analysis

clear all;
num=[1];
den=[1 1 0];
h1=tf(num,den);

%K=3.53; theoretical results
K=3.47;

figure;
margin(h1);grid;
figure;
margin(K*h1);grid;
[Gm,Pm,Wcg,Wcp]=margin(K*h1)

%figure;
%[numc,denc]=cloop(K*num,den,-1); 
%step(numc,denc);

% homework1 Problem 2.2 Bode design

%Bode plot design;
num=[5];
den1=[1/1.4 1 0];
den2=[1/3 1];
den=conv(den1,den2);
h1=tf(num,den);

figure;
bode(h1);grid;
%margin(h1);grid; 
[Gm,Pm,Wcg,Wcp]=margin(h1)

%axis([0.01 1000 150 50]);

%lead controller
%alfa=0.1461;
alfa=0.014;
Wmax=5.5;
T=1/Wmax/sqrt(alfa)/2
numc=[T 1];
denc=[alfa*T 1];
num=conv(num,numc);
den=conv(den,denc);
h2=tf(num,den);

margin(h2);grid;
[Gm,Pm,Wcg,Wcp]=margin(h2)
