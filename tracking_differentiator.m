function [sys,x0,str,ts] = trackingdifferentiator(t,x,u,flag)

switch flag

  case 0
    [sys,x0,str,ts]=mdlInitializeSizes();

  case 1
    sys=mdlDerivatives(t,x,u);

  case 3
    sys=mdlOutputs(t,x,u);

  case { 2, 4, 9 }
    sys = [];

  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

function [sys,x0,str,ts]=mdlInitializeSizes()

sizes = simsizes;
sizes.NumContStates  = 2;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = [0;0];
str = [];
ts  = [-1 0];


function sys=mdlDerivatives(t,x,u)
R    = 10;
xdot = [ x(2);
        -1.76*R*x(2) - (R^2)*(x(1)-u(1))];
sys = xdot;

function sys=mdlOutputs(t,x,u)

x(1); 
x(2); 

sys = [x(1); x(2)];
