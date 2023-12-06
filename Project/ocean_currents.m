%assumes input available is x vector of states (vector of 12 values as Fossen uses
%in his remus100 file)

%inputs
Vc = 1.8.*rand(1,1);             %input random value for current speed (m/s), max is 1.8 https://oceanservice.noaa.gov/facts/gulfstreamspeed.html
alphaVc = -3.14+6.28.*rand(1,1); %input random value for vertical current direction (rad), max is 180deg or 3.14rad, min is -3.14rad (-180deg)
betaVc = -3.14+6.28.*rand(1,1);  %input random value for horizontal current direction (rad), max is 180deg or 3.14rad, min is -3.14rad (-180deg)

%x = [0 0 0 .25 .5 .75 0 0 0 .7 1 .3];  %used to test 

%transformation from NED to BODY - pg 304 fossen
R = Rzyx(x(10),x(11),x(12));  %R transformation matrix from NED to BODY - using euler angles
v_cn = [Vc*cos(alphaVc)*cos(betaVc);Vc*sin(betaVc);Vc*sin(alphaVc)*cos(betaVc)]; %current vector in NED
v_cb = R'*v_cn;  %current vector in BODY

nu_c = [v_cb;0;0;0];  %full current velocity vector [uc,vc,wc,0,0,0]'

%next calculate time derivative of nu_c - pg 303 fossen
omega_nb = [x(4),x(5),x(6)];        %these are p, q, r from state matrix (angular velocities)
S_omega_nb = Smtrx(omega_nb);       %use Smtrx function from fossen to get omega_nb x current velocities
Dnu_c = [(-S_omega_nb*v_cb);0;0;0]; %derivative of nu_c according to equation pg 303 Fossen

%relative velocity vector - this is used in state space model
%nu_r = nu - nu_c; %where nu = x(1:6)



