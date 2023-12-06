function []=szmap(sigma,wd,zeta,...
                  T)        %optional input arguments
% SZMAP(Sigma,Wd,Zeta,T,N) plots the design area of the S-plane and 
% maps/plots it into Z-plane. This design area is the area bounded by 
% the limits of relative stabilty parameters (i.e. damping ratio, etc... ).
%
%   Sigma is a 2X1 or 1X2 vector that contains the limits of real part
%   of required poles. OR it can be just a 1X1 vector that represents
%   the maximum value of desired poles real part. 
%
%   Wd is a 2X1 or 1X2 vector that contains the limit values of damped
%   natural frequency of required poles. OR it can be just a 1X1 vector
%   that represents the maximum value of desired poles damped natural
%   frequency
%
%   ZETA is a 2X1 or 1X2 vector contains the limit values of damping ratio of
%   required poles. OR it can be just a 1X1 vector that represents the
%   minimum value of desired poles damping ratio.
%
%   T is the sampling time
%
% SZMAP(Sigma,Wd,Zeta) maps the region on the s-plan defined by the limits
% Zeta, Wd and Sigma for a sampling time T=1.
% 
% if there are no constraints on say Wd, use SZMAP(Sigma,[],Zeta,T)


%%%%%%%%%%%%%Created by A.M.Taha, Cairo university, 2017%%%%%%%%%%%%%%%%

if nargin<4
    T=1;
    if nargin<3
        zeta=[];
        if nargin<2
            wd=[];
        end
    end
end
n=100;
m=numel(sigma)+numel(wd)+numel(zeta);

%% S-plane limits calculation
s=zeros(n,m);
x=zeros(n,m);
y=zeros(n,m);
[lim_x,lim_y]=xylimits(sigma,wd,zeta);
if ~isempty(sigma)
    for i=1:numel(sigma)
        y(:,i)=linspace(0,lim_y+1,n);
        x(:,i)=-sigma(i)*ones(n,1);
        s(:,i)=x(:,i)+1i*y(:,i);
    end
end

if ~isempty(wd)
    for i=numel(sigma)+1:numel(wd)+numel(sigma)
        x(:,i)=linspace(-lim_x-1,0,n);
        y(:,i)=wd(i-numel(sigma))*ones(n,1);
        s(:,i)=x(:,i)+1i*y(:,i);
    end
end

if ~isempty(zeta)
    r_s=zeros(n,m);
    theta_s=zeros(n,m);
    for i=numel(sigma)+numel(wd)+1:numel(zeta)+numel(sigma)+numel(wd)
        zeta1=zeta(i-numel(sigma)-numel(wd));
        r_s(:,i)=linspace(0,sqrt(lim_x^2+lim_y^2)+1,n);
        theta_s(:,i)=(pi-acos(zeta1))*ones(n,1);
        s(:,i)=r_s(:,i).*cos(theta_s(:,i))+1i*r_s(:,i).*sin(theta_s(:,i));
    end
end

%% z-plane limits calculation
z=zeros(n,m);
r=zeros(n,m);
theta=zeros(n,m);
if ~isempty(sigma)
    for i=1:numel(sigma)
        theta(:,i)=linspace(0,pi,n);
        r(:,i)=exp(-sigma(i)*T)*ones(n,1);
        z(:,i)=r(:,i).*cos(theta(:,i))+1i*r(:,i).*sin(theta(:,i));
    end
end

if ~isempty(wd)
    for i=numel(sigma)+1:numel(wd)+numel(sigma)
        r(:,i)=linspace(0,2,n);
        theta(:,i)=wd(i-numel(sigma))*T*ones(n,1);
        z(:,i)=r(:,i).*cos(theta(:,i))+1i*r(:,i).*sin(theta(:,i));
    end
end

if ~isempty(zeta)
    for i=numel(sigma)+numel(wd)+1:numel(zeta)+numel(sigma)+numel(wd)
        zeta1=zeta(i-numel(sigma)-numel(wd));
        theta(:,i)=linspace(0,pi,n);
        sigma1=theta(:,i)/T/tan(acos(zeta1));
        r(:,i)=exp(-sigma1*T);
        z(:,i)=r(:,i).*cos(theta(:,i))+1i*r(:,i).*sin(theta(:,i));
    end
end

%% Plotting S-plane
figure;
hold on; grid minor; box on;
% Axes
plot([-lim_x-1,1],[0,0],'k-.',[0,0],[-lim_y-1,lim_y+1],'k-.');
%Constant requirements lines
for i=1:m
    plot(s(:,i),'k');
    plot(conj(s(:,i)),'k');
end
% Design region
[s_design]=region(sigma,wd,zeta,T,'s');
scatter(real(s_design),imag(s_design),'k.')
scatter(real(s_design),imag(-s_design),'k.')
if isempty(sigma) && isempty(wd)
    xlim([-lim_x,1]); ylim([-lim_y,lim_y]);
else
    xlim([-lim_x-1,1]); ylim([-lim_y-1,lim_y+1]);
end
title('s-plane');
xlabel('$\Re{(s)}$','interpreter','latex');
ylabel('$\Im{(s)}$','interpreter','latex');

%% Plotting Z-plane
figure;
hold on; grid minor; box on;
% unit circle
plot(cos(linspace(0,2*pi))+1i*sin(linspace(0,2*pi)),'k');
axis equal; xlim([-1,1]); ylim([-1,1]);
plot([-5,5],[0,0],'k-.',[0,0],[-5,5],'k-.');
%Constant requirements lines
for i=1:m
    plot(z(:,i),'k');
    plot(conj(z(:,i)),'k');
end
% Design region
[z_design]=region(sigma,wd,zeta,T,'z');
scatter(real(z_design),imag(z_design),'k.')
scatter(real(z_design),imag(-z_design),'k.')
title('Z-plane');
xlabel('$\Re{(z)}$','interpreter','latex');
ylabel('$\Im{(z)}$','interpreter','latex');

end