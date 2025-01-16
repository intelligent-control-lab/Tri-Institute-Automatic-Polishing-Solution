





















function [dist,varargout] = distLinSeg(point1s,point1e,point2s,point2e)

d1  = point1e - point1s;
d2  = point2e - point2s;
d12 = point2s - point1s;

D1  = d1*d1';
D2  = d2*d2';

S1  = d1*d12';
S2  = d2*d12';
R   = d1*d2';

den = D1*D2-R^2;

if (D1 == 0 || D2 == 0)
    if (D1 ~= 0)
        u = 0;
        t = S1/D1;

        t = fixbound(t);
 
    elseif (D2 ~= 0)
        t = 0;
        u = -S2/D2;

        u = fixbound(u);
  
    else
        t = 0;          
        u = 0;
    end
elseif (den == 0)
    t = 0;
    u = -S2/D2;
    
    uf = fixbound(u);
    
    if (uf~= u)
        t = (uf*R+S1)/D1;
        t = fixbound(t);
    
        u = uf;
    end
else
    
    t = (S1*D2-S2*R)/den;
    
    t = fixbound(t);
    
    u = (t*R-S2)/D2;
    uf = fixbound(u);
    
    if (uf ~= u)
        t = (uf*R+S1)/D1;
        t = fixbound(t);
      
        u = uf;
    end
end


dist = norm(d1*t-d2*u-d12);

if (nargout > 1)
    varargout = {[point1s + d1*t;point2s+d2*u]};
end

end
    
function num = fixbound(num)

if (num < 0)
    num = 0;
elseif (num > 1)
    num = 1;
end

end
