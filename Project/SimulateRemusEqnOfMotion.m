%% USER INPUTS
N  = 10000;     % number of samples

if TsSelection == 1
    Ts  = 0.05;     % sample time (s)
else
    Ts = .15; 
end

% Initial state vector
phi = 0; 
theta = 0; 
psi = 0;

x = [zeros(9,1); phi; theta; psi];

% Intialize integral summation 
z_int = 0;
theta_int = 0;
psi_int = 0;

% Propellor profile
n_d = 1525;  % desired propeller revolution, max 1525 rpm

nRampDur = 76.25; % ramp duration (sec)
nSlop = n_d/nRampDur; % slop of ramp
nRamp = @(t) nSlop*t+1; 

% Depth Profile and Controller coefficients 
z_step = 30; % desired depth (max 100 m)

tzStart = 100; % start of depth trajectory (sec)
zRampDur = 100; % ramp duration (sec)
zSlop = z_step/zRampDur; % slop of ramp
zRamp = @(t) zSlop*(t-tzStart);

Kp_z = 0.1;                 % proportional gain
T_z = 100;                  % integral gain

Kp_theta = 2;            
Kd_theta = 3;
Ki_theta = 0.1;

% Heading Profiles and controller coefficients  

if traj == 1 % Ramp to constant value
    psi_step = -60*pi/180; % target value
    tpsiStart = 250;  % start of yaw trajectory (sec)
    psiRampDur = 50; % ramp duration (sec)
    psiSlop = psi_step/psiRampDur;  % slop of ramp
    psiTraj = @(t) psiSlop*(t-tpsiStart); 

elseif traj == 2 % Follow sine path
    tpsiStart = 250; % start of yaw trajectory (sec)
    psiAmp = 30*pi/180; % sin amplitude (rad)
    psiTrajFreq = 1/250; % sin wave frequency (1/sec)
    psiTraj = @(t) psiAmp*sin(2*pi*psiTrajFreq*(t-tpsiStart));

elseif traj == 3 % unbounded ramp 
    tpsiStart = 250; % start of yaw trajectory (sec)
    Rate = 2*pi/125; % ramp rate (m/sec)
    psiTraj = @(t) (t-tpsiStart)*Rate;
end

wn_d_psi = 1/5;             % desired natural frequency, reference model 
wn_b_psi = 1;               % bandwidth, pole placement algorithm 
m66 = 7.5;                  % moment of inertia, yaw 
Kp_psi = m66 * wn_b_psi^2;                 
Kd_psi = m66 * 2*wn_b_psi; 
Ki_psi = Kp_psi * (wn_b_psi/10);

for i = 1:N+1
   t = (i-1)*Ts; % time

   % Relevant Statess
   q = x(5); % pitch rate
   r = x(6); % yaw rate
   z = x(9); % z-position (depth)
   phi = x(10);  % angle about x
   theta = x(11); % angle about y
   psi = x(12);  % angle about z

   % Propellor Rev Profile
   n = nRamp(t);
   n = min(n,n_d); 
   
   % Depth Profile
   if t > tzStart 
       z_d = min(zRamp(t),z_step);
   else
       z_d = 0;
   end
   
   theta_d = Kp_z * ( (z - z_d) + (1/T_z) * z_int ); % pitch angle
   delta_s = -Kp_theta *(theta - theta_d) - Kd_theta * q- Ki_theta * theta_int; % stern plane angle
   
   % Yaw trajectory 
   if t > tpsiStart
       if traj == 1
           psi_d = max(psiTraj(t),psi_step);
       else
           psi_d = psiTraj(t); 
       end
   else
       psi_d = 0;
   end       
   
   delta_r = -Kp_psi *(psi - psi_d) - Kd_psi * r - Ki_psi * psi_int; % rudder angle


   Inputs = [delta_r delta_s n]';

   % store simulation data in a table 
   simdata(i,:) = [t z_d theta_d psi_d Inputs' x'];   
   
   % solve ODE equation for plant model state space
   tSpan = [t i*Ts];
   xdot = @(t,x) RemusEqnOfMotionMod(x,Inputs,Density,CurrBetaSel,CurrVelSel);
  
   [t,x] = ode45(xdot, tSpan,x);
   x = x(end,:)';

   xStore(i,:) = x'; 

   % Euler's integration method (k+1) for integral error terms
   z_int = z_int + Ts * (z - z_d);
   theta_int = theta_int + Ts * (theta - theta_d);
   psi_int = psi_int + Ts * (psi - psi_d);   
end

%% PLOTS
time = simdata(:,1);    
z_d = simdata(:,2); 
theta_d = simdata(:,3); 
psi_d = simdata(:,4); 
u = simdata(:,5:7); 
Velocity = simdata(:,8:13);
Position = simdata(:,14:19);

if PlotProfiles
    % Propellor revolution profile
    figure; 
    plot(time,u(:,3))
    xlabel('Time (s)')
    ylabel('Revolutions (rpm)')
    title('Propeller Revolutions n (rpm) Profile')
    grid on

    hold on; 
    plot(0,0,'g*')
    plot(nRampDur,0,'r*')
    plot(tzStart,0,'g+')
    plot(tzStart+zRampDur,0,'r+')
    plot(tpsiStart,0,'go')

    legend('Reference Input','Prop Profile Start','Prop Profile End',...
        'Depth Profile Start','Depth Profile End','Heading Profile Start')
    set(findall(gcf,'type','line'),'linewidth',2)
    set(findall(gcf,'type','text'),'FontSize',14)

    % Depth profile
    figure; 
    plot(time,z_d)
    xlabel('Time (s)')
    ylabel('Depth (m)')
    title('Depth (m) Profile')
    grid on

    hold on; 
    plot(0,0,'g*')
    plot(nRampDur,0,'r*')
    plot(tzStart,0,'g+')
    plot(tzStart+zRampDur,0,'r+')
    plot(tpsiStart,0,'go')

    legend('Reference Input','Prop Profile Start','Prop Profile End',...
        'Depth Profile Start','Depth Profile End','Heading Profile Start')
    set(findall(gcf,'type','line'),'linewidth',2)
    set(findall(gcf,'type','text'),'FontSize',14)

    figure; 
    for j = 1:3
        for i = 1:numel(time)
        t = time(i);
        if j == 1 % Ramp to constant value
            psi_step = -60*pi/180; % target value
            tpsiStart = 250;  % start of yaw trajectory (sec)
            psiRampDur = 50; % ramp duration (sec)
            psiSlop = psi_step/psiRampDur;  % slop of ramp
            psiTraj = @(t) psiSlop*(t-tpsiStart); 
    
        elseif j == 2 % Follow sine path
            tpsiStart = 250; % start of yaw trajectory (sec)
            psiAmp = 30*pi/180; % sin amplitude (rad)
            psiTrajFreq = 1/250; % sin wave frequency (1/sec)
            psiTraj = @(t) psiAmp*sin(2*pi*psiTrajFreq*(t-tpsiStart));
        
        elseif j == 3 % unbounded ramp 
            tpsiStart = 250; % start of yaw trajectory (sec)
            Rate = 2*pi/125; % ramp rate (m/sec)
            psiTraj = @(t) (t-tpsiStart)*Rate;
        end

           if t > tpsiStart
               psi(i) = max(psiTraj(t),psi_step);
           else
               psi(i) = 0;
           end       
    
        end
    
         
        plot(time,psi*180/pi)
        hold on; 
        
    end
    xlabel('Time (s)')
    ylabel('Yaw Angle (deg)')
    title('Yaw Angle (deg) Trajectories')
    grid on

    plot(0,0,'g*')
    plot(nRampDur,0,'r*')
    plot(tzStart,0,'g+')
    plot(tzStart+zRampDur,0,'r+')
    plot(tpsiStart,0,'go')
    
    legend('Trajectory 1','Trajectory 2','Trajectory 3','Prop Profile Start','Prop Profile End',...
        'Depth Profile Start','Depth Profile End','Heading Profile Start')
    set(findall(gcf,'type','line'),'linewidth',2)
    set(findall(gcf,'type','text'),'FontSize',14)
end

% Plot Velocity/Angular velocity states
figure; clf
subplot(3,2,1)
plot(time,Velocity(:,1))
xlabel('Time (s)')
ylabel('Linear Velocity (m/s)')
title('Surge (along x) Velocity (m/s)')
grid on

subplot(3,2,3)
plot(time,Velocity(:,2))
xlabel('Time (s)')
ylabel('Linear Velocity (m/s)')
title('Sway (along y) Velocity (m/s)')
grid on

subplot(3,2,5)
plot(time,Velocity(:,3))
xlabel('Time (s)')
ylabel('Linear Velocity (m/s)')
title('Heave (along z) Velocity (m/s)')
grid on

subplot(3,2,2)
plot(time,Velocity(:,4))
xlabel('Time (s)')
ylabel('Angular Velocity (deg/s)')
title('Roll (about x) Rate (deg/s)')
grid on

subplot(3,2,4)
plot(time,Velocity(:,5))
xlabel('Time (s)')
ylabel('Angular Velocity (deg/s)')
title('Pitch (about y) Rate (deg/s)')
grid on

subplot(3,2,6)
plot(time,Velocity(:,6))
xlabel('Time (s)')
ylabel('Angular Velocity (deg/s)')
title('Yaw (about z) Rate (deg/s)')
grid on

set(findall(gcf,'type','line'),'linewidth',2)
set(findall(gcf,'type','text'),'FontSize',14)

% Plot Position/Angle States
figure; clf
subplot(3,2,1)
plot(time,Position(:,1))
xlabel('Time (s)')
ylabel('Position (m)')
title('Surge (x) Position (m)')
grid on

subplot(3,2,3)
plot(time,Position(:,2))
xlabel('Time (s)')
ylabel('Position (m)')
title('Sway (y) Position (m)')
grid on

subplot(3,2,5)
plot(time,Position(:,3))
hold on 
plot(time,z_d)

xlabel('Time (s)')
ylabel('Position (m)')
title('Heave (z) Position (m)')
legend('True','Desired')

grid on

subplot(3,2,2)
plot(time,Position(:,4)*180/pi)
xlabel('Time (s)')
ylabel('Angle (deg)')
title('Roll (from x) Angle (deg)')
grid on

subplot(3,2,4)
plot(time,Position(:,5)*180/pi)
hold on 
plot(time,theta_d*180/pi)

xlabel('Time (s)')
ylabel('Angle (deg)')
title('Pitch (from y) Angle (deg)')
legend('True','Desired')
grid on

subplot(3,2,6)
plot(time,Position(:,6)*180/pi)
hold on 
plot(time,psi_d*180/pi)

xlabel('Time (s)')
ylabel('Angle (deg)')
title('Yaw (from z) Angle (deg)')
legend('True','Desired')
grid on

set(findall(gcf,'type','line'),'linewidth',2)
set(findall(gcf,'type','text'),'FontSize',14)
set(findall(gcf,'type','legend'),'FontSize',16)


figure; clf 
subplot(2,2,1)
plot(time,u(:,3))
xlabel('Time (s)')
ylabel('Revolutions (rpm)')
title('Propeller Revolutions n (rpm)')
grid on

subplot(2,2,2)
plot(time, vecnorm(Velocity(:,1:3),2,2));
xlabel('Time (s)')
ylabel('Speed (m/s)')
title('Speed (m/s)')
grid on

subplot(2,2,3)
plot(time,(180/pi)*u(:,1))
xlabel('Time (s)')
ylabel('Angle (deg)')
title('Rudder \delta_r (deg)')
grid on

subplot(2,2,4)
plot(time,(180/pi)*u(:,2))
xlabel('Time (s)')
ylabel('Angle (deg)')
title('Stern planes \delta_s (deg)')
grid on

set(findall(gcf,'type','line'),'linewidth',2)
set(findall(gcf,'type','text'),'FontSize',14)


