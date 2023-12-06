function [stateVecd] = RemusEqnOfMotionMod(stateVec,Inputs,Density,CurrBetaSel,CurrVelSel)
% Unwrap controllable inputs
delta_r = Inputs(1); % Rudder angle (rad) 
delta_s = Inputs(2); % Stern plane angle (rad)
n = Inputs(3); % Propellor revolution (rpm)

%% Allocate Parameters
rhoSeaWater = 1026; % density of sea water (kg/m^3)
Length_AUV = 1.6; % AUV length (m) 
Dia_AUV = .19; % AUV diameter (m)
rhoFront = Density(1); % density of front section of AUV (kg/m^3)
rhoRear = Density(2); % density of rear section of AUV (kg/m^3)

perVol = .90; % percent volume of the front piece of the AUV
g = 9.8; % gravitational acceleration constant (m/s^2)


CL_0 = 0; % Initial lift coefficient

S = .7*Length_AUV*Dia_AUV; % planform area of wing, 70% of rectangel Length_AUV*Dia_AUV
OswEffFac = .7; % Oswald efficiency factor
AR = (Dia_AUV)^2/S; % Aspect ratio
CD_p = .42*pi*AR/4;  % drag coefficient 


%% Define rotation matrices/coordinate frame transformations
Rz = @(psi) [cos(psi) -sin(psi) 0
    sin(psi) cos(psi) 0 
    0 0 1]; 

Ry = @(phi) [cos(phi) 0 sin(phi) 
    0 1 0 
    -sin(phi) 0 cos(phi)];

Rx = @(theta) [1 0 0
    0 cos(theta) -sin(theta)
    0 sin(theta) cos(theta)];

R_Body2NED = Rz(stateVec(12))*Ry(stateVec(11))*Rx(stateVec(10)); 

TMat  = @(phi,theta) [1 0 -sin(theta)
        0 cos(phi) cos(theta)*sin(phi)
     0 -sin(phi) cos(phi)*cos(theta)];
%% Current selection and relative velcity computations

if CurrBetaSel == 1 % beta angle selection
    betaVc = 33.08*pi/180; % horizontal current angle
elseif CurrBetaSel == 2
    if round(stateVec(12),5) == 0 % avoids divide by zero
        betaVc = 90*pi/180; 
    else
        betaVc = -1/tan(stateVec(12)); % angle orthogonal to the vehicle 
    end
end

if CurrVelSel == 1 % Current speed selection
    Vc = 0.5;
elseif CurrVelSel == 2
    Vc = 1.8;
end

alphaVc = 0; % vertical current angle  
v_cn = [Vc*cos(alphaVc)*cos(betaVc);Vc*sin(betaVc);Vc*sin(alphaVc)*cos(betaVc)]; %current vector in NED

% Ocean currents expressed in body
LinVel_CurrentB = R_Body2NED'*v_cn;  %current linear velocity vector in body
AngVel_CurrentB = zeros(3,1);  %current angular velocity vector in body

Speed = norm(stateVec(1:3)); % speed 
LinVelRel = stateVec(1:3)-LinVel_CurrentB; % linear relative velocity  
RelSpeed = norm(LinVelRel); % relative speed
AngVelRel = stateVec(4:6)-AngVel_CurrentB; % angular relative velocity
velRel = [LinVelRel; AngVelRel]; % relative velocity
SpeedRel = sqrt( LinVelRel(1)^2 + LinVelRel(2)^2 + LinVelRel(3)^2 );  % relative speed (m/s)
%% Derive Rigid Body Mass and Rigid Body Coriolis

[I_CG,m,xCG,yCG,zCG] = EllipsoidInertiaCG(Length_AUV/2,Dia_AUV/2,Dia_AUV/2,rhoFront,rhoRear,perVol); % compute inertia matrix 

M_RB = [m*eye(3) zeros(3,3)
    zeros(3,3) I_CG]; % Rigid body mass at the center of gravity 

C_RB = [m*Smat(stateVec(4:6)) zeros(3,3)
    zeros(3,3) -Smat(I_CG*stateVec(4:6))]; %Rigid body coriolis matrix at center of gravity

r_CGinB = [xCG; yCG; zCG]; % vector from CG to CO
r_CGinB = r_CGinB+[0; 0; .02]; % vector from CG to CO (shifted up to account for inability to control roll)

H = [eye(3) Smat(r_CGinB)'
    zeros(3,3) eye(3)]; % Transformation matrix from CG to CO 

M_RB = H'*M_RB*H; % Rigid body mass at CO 
C_RB = H'*C_RB*H; % Rigid body coriolis at CO 

%% Derive Added Mass Terms
a = Length_AUV/2;
b = Dia_AUV/2; 

e = sqrt(1- (b/a)^2); 
beta0= 1/e^2-(1-e^2)/(2*e^3)*log((1+e)/(1-e));
alpha0 = 2*(1-e^2)/e^3*(.5*log((1+e)/(1-e))-e); 

Xud = -alpha0/(2-alpha0)*(4/3*pi*rhoSeaWater*a*b^2); 
Yvd = -beta0/(2-beta0)*(4/3*pi*rhoSeaWater*a*b^2);
Zwd = Yvd; 
Kpd = 0 ;
Nrd = -1/5*(b^2-a^2)^2*(alpha0-beta0)/(2*(b^2-a^2)+(b^2+a^2)*(beta0-alpha0))*(4/3*pi*rhoSeaWater*a*b^2); 
Mqd = Nrd;

MA = -[Xud 0 0 0 0 0
    0 Yvd 0 0 0 0 
    0 0 Zwd 0 0 0 
    0 0 0 Kpd 0 0 
    0 0 0 0 Mqd 0 
    0 0 0 0 0 Nrd];  % Added mass matrix

M11 = MA(1:3,1:3); 
M12 = MA(1:3,4:6); 
M21 = MA(4:6,1:3); 
M22 = MA(4:6,4:6); 

CA = [zeros(3,3) -Smat(M11*LinVelRel+M12*AngVelRel)
    -Smat(M11*LinVelRel+M12*AngVelRel) -Smat(M21*LinVelRel+M22*AngVelRel)]; % Added coriolis matrix 

% Linear rotational damping assumption, set nonlinear quadratic terms to zero 
CA(5,1) = 0;   
CA(5,4) = 0;
CA(6,1) = 0;
CA(6,2) = 0;

M = M_RB+MA; % total mass matrix
C = C_RB+CA; % Total coriolis matrix
%% Hydrostatic 
[xCB,yCB,zCB,volWaterDis] = PartEllipsoidCentroid(Length_AUV/2,Dia_AUV/2,Dia_AUV/2,[-Length_AUV/2 Length_AUV/2]); % compute center of buoyancy and volume of water displaced

r_CBinB = [xCB; yCB; zCB]; % center of buoyancy in body frame

FgravN = [0; 0 ; m*g]; % force of gravity in NED
FBuoyN = -[0; 0; rhoSeaWater*g*volWaterDis]; % buoyancy force in NED

gEta = -[R_Body2NED'*(FgravN+FBuoyN)
    cross(r_CGinB,R_Body2NED'*FgravN)+cross(r_CBinB,R_Body2NED'*FBuoyN)]; % hydrostatic contribution in body frame

%% Drag Force, Lift Force

AngOfAtk = atan2(LinVelRel(3),LinVelRel(1) );                % angle of attack (rad)

CL_alpha = pi*AR/(1+sqrt(1+(AR/2)^2));
CD_alpha = CD_p + (CL_0+CL_alpha*AngOfAtk)^2/(pi*OswEffFac*AR); 

FDrag = .5*rhoSeaWater*RelSpeed^2*S*CD_alpha; 
FLift = .5*rhoSeaWater*RelSpeed^2*S*(CL_0+CL_alpha*AngOfAtk); 

FDragLift_xB = -FDrag*cos(AngOfAtk)+FLift*sin(AngOfAtk);
FDragLift_zB = -FDrag*sin(AngOfAtk)-FLift*cos(AngOfAtk);

tauLiftDrag =  [FDragLift_xB; 0; FDragLift_zB; 0; 0; 0];

%% Linear Damping
T1 = 20; % time constant in sway (s) 
T2 = 20; % time constant in surge (s) 
T3 = T1;  
zeta4 = .3; % relative damping ratio in roll 
zeta5 = .8; % relative damping ratio in pitch 
T6 = 5; % time constant in yaw (s) 

w4 = sqrt(m*g*r_CGinB(3)/M(4,4));
w5 = sqrt(m*g*r_CGinB(3)/M(5,5));

D = [M(1,1)/T1 0 0 0 0 0 
    0 M(2,2)/T2 0 0 0 0 
    0 0 M(3,3)/T3 0 0 0
    0 0 0 2*zeta4*w4*M(4,4) 0 0
    0 0 0 0 2*zeta5*w5*M(5,5) 0
    0 0 0 0 0 M(6,6)/T6];

D(1,1) = D(1,1) * exp(-3*SpeedRel);   % vanish at high speed where quadratic drag and lift forces dominates
D(2,2) = D(2,2) * exp(-3*SpeedRel);   
D(6,6) = D(6,6) * exp(-3*SpeedRel);


%% Cross Flow Drag
NumSeg = 20; % number of segments for the integral 
dx = Length_AUV/NumSeg;
T = Dia_AUV; 
B = Dia_AUV; 

xBnds = -Length_AUV/2:dx:Length_AUV/2; 
Y_CFD = 0; 
N_CFD = 0; 
Z_CFD = 0; 
M_CFD = 0; 
Cd_2D = Hoerner(B,T); 
for i = 1:numel(xBnds)
    velSway = LinVelRel(2); 
    velHeave = LinVelRel(3); 
    velYaw = AngVelRel(3);

    Y_CFD = Y_CFD  -.5*rhoSeaWater*T*Cd_2D*abs(velSway+xBnds(i)*velYaw)*(velSway+xBnds(i)*velYaw)*dx;
    N_CFD = N_CFD  -.5*rhoSeaWater*T*Cd_2D*xBnds(i)*abs(velSway+xBnds(i)*velYaw)*(velSway+xBnds(i)*velYaw)*dx;
    Z_CFD = Z_CFD  -.5*rhoSeaWater*T*Cd_2D*abs(velHeave+xBnds(i)*velYaw)*(velSway+xBnds(i)*velYaw)*dx;
    M_CFD = M_CFD  -.5*rhoSeaWater*T*Cd_2D*xBnds(i)*abs(velHeave+xBnds(i)*velYaw)*(velSway+xBnds(i)*velYaw)*dx;
end

tauCrossFlow = [0; Y_CFD; Z_CFD;0; M_CFD; N_CFD];

%% Convesion of controllable inputs to forces and moments 
tau = zeros(6,1); 

U_rh = sqrt( LinVelRel(1)^2 + LinVelRel(2)^2 );  % horizontal plane relative speed
U_rv = sqrt( LinVelRel(1)^2 + LinVelRel(3)^2 );  % vertical plane relative speed

% Generalized propulsion force         
C1 = 4.4992e-05;
C2 = -0.0166;
Cr = -1.2825;
Cs = -3.5910;
tau(1) = C1*n.^2+C2*n.*Speed+Cr*(delta_r.^2).*(U_rh.^2)+Cs*delta_s.^2.*(U_rv.^2);

% Rudder (1 rudder) 
CL_delta_r = 0.5;        % rudder lift coefficient 
A_r = 0.10 * 0.05;       % rudder area (m^2)
x_r = -Length_AUV/2;     % rudder x-position (m)
 
Y_r = -0.5 * rhoSeaWater * U_rh^2 * A_r * CL_delta_r * delta_r; % Rudder sway force

tau(2) = Y_r;
tau(6) = x_r * Y_r;

% Stern plane (2 stern planes)
CL_delta_s = 0.7;       % stern plane lift coefficient
A_s = 2 * 0.10 * 0.05;  % stern plane area (m^2)
x_s = -Length_AUV/2;    % stern plane x-position (m)

Z_s = -0.5 * rhoSeaWater * U_rv^2 * A_s * CL_delta_s * delta_s; % Stern-plane heave force

tau(3) = Z_s;
tau(5) = x_s * Z_s;

%% State Space representation of plant model Plant Model 
vrdot = inv(M)*(tau+tauCrossFlow+tauLiftDrag-C*velRel-D*velRel-gEta); 

Dnu_c = [-Smat(stateVec(4:6))*LinVel_CurrentB; zeros(3,1)];

vdot = Dnu_c +vrdot;
J = [ R_Body2NED  zeros(3,3)
      zeros(3,3) inv(TMat(stateVec(10),stateVec(11)))];

stateVecd = [vdot; J*stateVec(1:6)]; 

end