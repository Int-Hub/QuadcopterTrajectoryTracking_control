function [ fhan ] = fhan( x1, x2, r, h )

d  = r*h;
d0 = h*d;
y  = x1 + h*x2;
a0 = sqrt((d^2) + 8*r*(abs(y)));

if abs(y) > d0
    a = x2 + ((a0 - d)/2)*sign(y);    
else
    a = x2 + (y/h);
end

if abs(a) > d
    fhan = -r*sign(a);
else
    fhan = -r*a/d;
end

end