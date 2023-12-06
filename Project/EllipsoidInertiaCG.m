function [I_CG,m,xCG,yCG,zCG] = EllipsoidInertiaCG(xr,yr,zr,rho1,rho2,perVol)
% xr,yr,zr are semi axes of an ellipsoid, the major axis is along the y
% axis
% Ellipsoid is split into 2 pieces, 
% rho1 - the uniform density of the first piece
% rho2 - the uniform density of the second piece
% perVol - percent of total volume that makes up the first piece
% First piece is from [lowerBnd yr]

f = @(LowBnd) LowBnd^3/3-LowBnd*xr^2 +2*xr^3/3*(1-2*perVol); % function to compute lower bound of segment 1
LowBnd = fzero(f,[-xr xr]); % Low bound of segment 1

indefIntXsqrd = @(bnd) (xr^3*zr*yr*pi)*(bnd^3/3-bnd^5/5); % indefinite integral of x^2
indefIntYsqrd = @(bnd) (xr*zr*yr^3*pi/4)*(bnd^5/5-2*bnd^3/3+bnd); % indefinite integral of y^2
indefIntZsqrd = @(bnd) (xr*zr^3*yr*pi/4)*(bnd^5/5-2*bnd^3/3+bnd); % indefinite integral of z^2

Bnds = [-xr LowBnd xr]/xr; 

Ix1 = rho1*(indefIntYsqrd(Bnds(3))-indefIntYsqrd(Bnds(2))+indefIntZsqrd(Bnds(3))-indefIntZsqrd(Bnds(2))); 
Ix2 = rho2*(indefIntYsqrd(Bnds(2))-indefIntYsqrd(Bnds(1))+indefIntZsqrd(Bnds(2))-indefIntZsqrd(Bnds(1))); 

Iy1 = rho1*(indefIntXsqrd(Bnds(3))-indefIntXsqrd(Bnds(2))+indefIntZsqrd(Bnds(3))-indefIntZsqrd(Bnds(2))); 
Iy2 = rho2*(indefIntXsqrd(Bnds(2))-indefIntXsqrd(Bnds(1))+indefIntZsqrd(Bnds(2))-indefIntZsqrd(Bnds(1))); 

Iz1 = rho1*(indefIntXsqrd(Bnds(3))-indefIntXsqrd(Bnds(2))+indefIntYsqrd(Bnds(3))-indefIntYsqrd(Bnds(2))); 
Iz2 = rho2*(indefIntXsqrd(Bnds(2))-indefIntXsqrd(Bnds(1))+indefIntYsqrd(Bnds(2))-indefIntYsqrd(Bnds(1))); 

[xbar1,ybar1,zbar1,vol1] = PartEllipsoidCentroid(xr,yr,zr,[-xr LowBnd]);
[xbar2,ybar2,zbar2,vol2] = PartEllipsoidCentroid(xr,yr,zr,[LowBnd xr]);

m1 = rho1*vol1; % mass of first piece
m2 = rho2*vol2; % mass of second piece 
m = m1+m2; 

xCG = (xbar1*m1+xbar2*m2)/(m1+m2);
yCG = (ybar1*m1+ybar2*m2)/(m1+m2);
zCG = (zbar1*m1+zbar2*m2)/(m1+m2);

Ixy = 0; 
Iyx = Ixy;
Iyz = 0; 
Izy = Iyz;
Ixz = 0; 
Izx = Ixz; 

r = [0-xCG; 0-yCG; 0-zCG]; % distance from origin (where inertia is computed in reference to) to CG

I_CG1 = [Ix1 Ixy Ixz
    Iyx Iy1 Iyz
    Izx Izy Iz1];

I_CG2 = [Ix2 Ixy Ixz
    Iyx Iy2 Iyz
    Izx Izy Iz2];
I_CG = I_CG1-m1*Smat(r)^2 + I_CG2-m2*Smat(r)^2;

end