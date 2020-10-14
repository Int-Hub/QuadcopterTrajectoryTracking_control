function [sys,x0,str,ts] = eso(t,x,u,flag)


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
sizes.NumContStates  = 3;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 4;
sizes.NumInputs      = 3;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = [0; 0; 0];
str = [];
ts  = [0 0];

function sys=mdlDerivatives(t,x,u)
b0 = 0.83;

delta  = 0.001;

hg     = 0.15;
beta01 = 1;
beta02 = 1/(2*(hg^0.5));
beta03 = 2/(25*(hg^1.2));
beta01 = beta01*100;
beta02 = beta02*100;
beta03 = beta03*100;
e      =  x(1) - u(1);

fe = fal(e, 0.5, delta);
fe1 = fal(e, 0.25, delta);

xdot1 = x(2) - beta01 * e;
xdot2 = x(3) - beta02 * fe + b0 * u(2);
xdot3 =  - beta03 * fe1;


sys = [xdot1; xdot2; xdot3];


function sys=mdlOutputs(t,x,u)

x(1); 
x(2); 
x(3); 


sys = [x(1); x(2); x(3); x(1)];