function [S] = Smat(vec)
% Computes the 3x3 skew symmetric matrix for solving for the cross product
% per equation 2.10 in Fossen Marine Craft book
% Cross product vec X A = S(vec)a
S = [0 -vec(3) vec(2) 
    vec(3) 0 -vec(1) 
    -vec(2) vec(1) 0];
end