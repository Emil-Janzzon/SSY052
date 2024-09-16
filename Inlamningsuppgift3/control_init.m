% function [phi,theta]=control_init(Fd)
% 
% A=Fd.den{1};      % A=denominator polynomial
% B=Fd.num{1};      % B=numerator polynomial
% 
% % The following lines are only valid for the PD controller
% % and therefore need to be generalized for the PID and PIDf controllers
% 
% theta=[A(2) B(1) B(2)];   
% phi=[0 0 0];

function [phi,theta]=control_init(Fd)

A=Fd.den{1};       % A=denominator polynomial
B=Fd.num{1};      % B=numerator polynomial

n=length(A)-1;
theta=[A(2:n+1) B];   
phi=zeros(1,2*n+1);