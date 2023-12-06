function [xbar,ybar,zbar,vol] = PartEllipsoidCentroid(xr,yr,zr,MA_Range)
% Centroid of a partial ellipsoid, reduced in along the major axis
% Major axis lies along the y axis
% Equation of ellipsoid x^2/xr^2 + y^2/yr^2 + z^2/zr^2 = 1
ybar = 0; % centroid along the x axis

xdV_indefInt = @(x) (yr*zr*pi/xr^2)*(xr^2*x^2/2 - x^4/4); % indefinite integral of int(ydV) 
int_xdV =  xdV_indefInt(MA_Range(2))-xdV_indefInt(MA_Range(1)); % definite integral of int(ydV) from [lowBnd UppBnd]

vol_indefInt = @(x) (yr*zr*pi/xr^2)*(xr^2*x - x^3/3); % indefinite integral of int(dV) 
vol = vol_indefInt(MA_Range(2))-vol_indefInt(MA_Range(1)); % definite integral of int(dV) from [lowBnd UppBnd]

xbar = int_xdV/vol; % centroid along the major axis
zbar = 0; % centroid along the z axis
end