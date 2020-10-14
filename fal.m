function [ fal ] = fal(e,a,delta)
if abs(e) > delta
    fal = ((abs(e))^a) * sign(e);
else
    fal = e/(delta^(1-a));
end

end