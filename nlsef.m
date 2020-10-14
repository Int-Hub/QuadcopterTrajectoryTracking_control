function [sys,x0,str,ts] = nlsef(t,x,u,flag)


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
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 1;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [0 0];


function sys=mdlOutputs(t,x,u)
%c  = 2;
r  = 100;
h  = 0.01;


e1 = u(1) - u(2);  
e2 = u(3) - u(4);  
u0 = -fhan(e1,e2,r,h);

sys = u0;
