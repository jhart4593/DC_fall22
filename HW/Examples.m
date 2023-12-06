clc; clear; close all;

%% Example 1
zeta=[0.5,0.8];
omega_d=[1,2];
sigma=[0.5,2];
szmap(sigma,omega_d,zeta,0.5);
szmap(sigma,omega_d,zeta,1);

%% Example 2
zeta=[0.5,0.8];
omega_d=[];
sigma=[];
szmap(sigma,omega_d,zeta);

%% Example 3
zeta=0.5;
omega_d=1;
sigma=1;
szmap(sigma,omega_d,zeta);
