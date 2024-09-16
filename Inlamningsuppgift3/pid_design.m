function [F,Ki,Kinf,tau,err]=pid_design(G,wc,phim,beta,zeta)

%  [F,Ki,Kinf,tau,err]=PID_BETA(G,wc,phim,beta,zeta) returns controller 
%  parameters for a PID controller
%
%  F=Ki*(1+2*zeta*s*tau+(s*tau)^2)/(s*(1+s*tau/beta))
%  Kinf=Ki*tau*beta
%  Jv=1/Ki
%
%  err=1 => desired phase margin not achievable   OK R 020223

err=0;
[Gwc,phiGwc]=bode(G,[wc]);
if (phiGwc>90), phiGwc=phiGwc-360; end
phiL=-180+phim;
phiF=phiL-phiGwc;
[wctau,Kinf,phiF0]=pid_design_beta(Gwc,phiF,beta,zeta);
if (abs(phiF-phiF0)>1e-4)
    err=1; 
    warning('Desired stability margin not achievable')
end
phiF;
tau=wctau/wc;
Ki=Kinf/tau/beta;
F=tf([Ki*tau^2 Ki*2*zeta*tau Ki],[tau/beta 1 0]);
if err==0
    err = abs(norm(feedback(G*F,1)))==inf;
    if err
        warning('Unstable closed loop system')
    end
end

function [wctau,Kinf,phiF0]=pid_design_beta(Gwc,phiF,beta,zeta)
%
%  [wctau,Kinf,phiF0]=pid_design_beta(Gwc,phiF,beta,zeta)   OK R 020223
%

wctau=logspace(-1,1,200);

for iter=1:2,

   r=sqrt((1-wctau.^2).^2+(2*zeta*wctau).^2);
   res = acos((1-wctau.^2)./r) - atan(wctau/beta) - phiF*pi/180;
   wctau=inversefcn1(wctau,res,pi/2);

   if isempty(wctau)
       phiF0=phiF+1;
       Kinf=1;
       wctau=1;    
       return
   end
   if iter==1
       wctau=wctau*[0.9805:0.001:1.02];
   end
end

r=sqrt((1-wctau^2)^2+(2*zeta*wctau)^2);
Kinf=beta*wctau*sqrt(1+(wctau/beta)^2)/Gwc/r;

phiF0=(acos((1-wctau^2)/r) - atan(wctau/beta))*180/pi-90;


function [xval,xival,zval]=inversefcn1(x,y,yconst,z)  % OKR o test 020302

if y(1)<=yconst
    ix=find(y>=yconst);
else
    ix=find(y<=yconst);
end
N=length(x);

if isempty(ix) | (N<2)
    xval=[];
    zval=[];
    xival=[];
    return
else
    i=min(ix(1),N-1);
    xival=i;
end

if abs(y(i+1)-y(i))>=1e-12
    xval=x(i)+(x(i+1)-x(i))/(y(i+1)-y(i))*(yconst-y(i));
else
    xval=x(i);
end

if nargin==4 & nargout==3
    zval=z(i)+(z(i+1)-z(i))/(x(i+1)-x(i))*(xval-x(i));
end
