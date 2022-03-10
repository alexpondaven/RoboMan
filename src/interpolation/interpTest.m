% t1=1;
% t2=1;
% b3=2;
% b2=3;
% b1=0;
% 
% % coeffs = [a12,a13,a21,a22,a23]
% 
% A=[t1^2 t1^3 0 0 0
%    2*t1 3*t1^2 -1 0 0
%    0 0 t2 t2^2 t2^3
%    0 0 1 2*t2 3*t2^2
%    2 6*t1 0 -2 0];
% 
% b = [b2-b1
%      0
%      b3-b2
%      0
%      0];
% 
% linsolve(A,b)

vias = [0 0 0 0
        1 1 1 1
        2 2 2 2];
Tend = 2;

interpTraj(vias,Tend, 1)