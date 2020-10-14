function [sys,x0,str,ts] = quadrotoradrc(t,x,u,flag)

switch flag

  case 0
    [sys,x0,str,ts] = mdlInitializeSizes;

  case 1
    sys=mdlDerivatives(t,x,u);

  case 3
    sys=mdlOutputs(t,x,u);

  case { 2, 4, 9 }
    sys = [];

  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 24;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 16;
sizes.NumInputs      = 16;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);

x0  = [0; 0; 0; 0; 0; 0; 0; 0; 5; 0; 0;
    0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0;];
% x0 = zeros(1,24);
str = [];
ts  = [0 0];

function sys = mdlDerivatives(t,x,u)
%% Defining system parameters
g   = 9.81;
l   = 0.45;
omega_r  = 0.1;        
Jrr  = 6e-3;      
Ixx = 0.018125; 
Iyy = 0.018125; 
Izz = 0.035;
m   = 2;
dz = 0;
a11 = (Iyy-Izz)/Ixx;
a22 = Jrr/Ixx;
a33 = (Izz-Ixx)/Iyy;
a44 = Jrr/Iyy;
a55 = (Ixx-Iyy)/Izz;
b11 = l/Ixx;
b22 = l/Iyy;
b33 = 1/Izz;
bb = -(cosd(x(1))*cosd(x(3)))/m;
%% Gain variables for linear observer

L1 = 29.5659;
L2 = 2907;
L3 = 3000;

%% Defining quadcopter system dynamics
 
xdot    = [x(2);
           x(4)*x(6)*a11 - x(4)*omega_r*a22 + b11*u(2) + dz1;
           x(4);
           x(2)*x(6)*a33 + x(2)*omega_r*a44 + b22*u(3) + dz2;
           x(6);
           x(2)*x(4)*a55 + b33*u(4) + dz3;
           x(8);
           - g - (u(1)/m)*(cosd(x(1))*cosd(x(3))) + dz;     
           x(10)
- (u(1)/m)*(sind(x(1))*sind(x(5)) + cosd(x(1))*sind(x(3))*cosd(x(5))) + dz;
           x(12);
- (u(1)/m)*(sind(x(1))*cosd(x(5)) - cosd(x(1))*sind(x(3))*sind(x(5))) + dz;
           x(14) + L1*(x(1)-x(13));
           x(15) + L2*(x(1)-x(13)) + b11*u(2);
                   L3*(x(1)-x(13));
           x(17) + L1*(x(3)-x(16));
           x(18) + L2*(x(3)-x(16)) + b22*u(3);
                   L3*(x(3)-x(16));
           x(20) + L1*(x(5)-x(19));
           x(21) + L2*(x(5)-x(19)) + b33*u(4);
                   L3*(x(5)-x(19));
           x(23) + L1*(x(7)-x(22));
           x(24) + L2*(x(7)-x(22)) + bb*u(1);
                   L3*(x(7)-x(22))];
sys      = xdot;

function sys = mdlOutputs(t, x, u)
m   = 2;
l   = 0.45;
Ixx = 0.018125; 
Iyy = 0.018125; 
Izz = 0.035;
b11  = l/Ixx;
b22  = l/Iyy;
b33  = 1/Izz;
bb   = -(cosd(x(1))*cosd(x(3)))/m;

%% Defining x and y position and velocity state

x_pos = 5*cos(2*pi*0.1*t);  % desired x position
y_pos = 5*sin(2*pi*0.1*t);  % desired y position 

x_vel = 0; % desired x velocity
y_vel = 0; % desired y velocity


%% Assigning PD controller gain variables

kpp = u(5);  kdp = u(6);
kpt = u(7);  kdt = u(8);
kpk = u(9);  kdk = u(10);
kpz = u(11); kdz = u(12);
kpx = u(13); kdx = u(14);
kpy = u(15); kdy = u(16);

%% Defining initial y-phi and x-theta values

 phid   =  kpy*(y_pos - x(11)) + kdy*(y_vel - x(12));
 thetad =  kpx*(x_pos - x(9))   + kdx*(x_vel - x(10));
 
%% Control Action for Theta, Phi, Psi and Altitude Channels
% 
zd    = t; % desired z position
zddot = 0; 
u01     = kpz*(zd - x(7)) + kdz*(zddot - x(8));
u1      = (u01 - x(24))/bb;

phid    = cosd(x(5))*phid - sind(x(5))*thetad; % desired psiangle
phiddot = 0; 
u02     = kpp*(phid - x(1)) + kdp*(phiddot - x(2));
u2      = (u02 - x(15))/b11;

thetad    = sind(x(5))*phid + cosd(x(5))*thetad; % desired theta angle
thetaddot = 0; 
u03       = kpt*(thetad- x(3)) + kdt*(thetaddot - x(4));
u3        = (u03 - x(18))/b22;


psid    = 1; % desired psi angle
psiddot = 0; 
u04     = kpk*(psid - x(5)) + kdk*(psiddot - x(6));
u4      = (u04 - x(21))/b33;

%%
sys = [x(1); x(3); x(5); x(7); x(9); x(11); u1; u2; u3; u4; x_pos; y_pos; zd; phid; thetad; psid];
