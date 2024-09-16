% function [u,phi]=control_update(phi,theta,e)    % First call gives u at time t=0
% 
% % The following lines are only valid for the PD controller
% % and therefore need to be generalized for the PID and PIDf controllers
% 
% phi(2)=e;  % e = control error = r-y
% u=phi(1)*theta(1)+phi(2)*theta(2)+phi(3)*theta(3);
% 
% % Prepare for the next control update
% phi(3)=phi(2);
% phi(1)=-u;

function [u,phi]=control_update(phi,theta,e)
npar = length(theta);
n = (npar-1)/2;
phi(n+1) = e;
u= theta*phi';
phi = [-u phi(1:npar-1)];
end


