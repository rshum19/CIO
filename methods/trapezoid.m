function [ defects ] = trapezoid(dt,z,f) 

nTime = size(z,2);

idxPrv = 1:(nTime-1);
idxNxt = 2:nTime;

zPrv = z(:,idxPrv);
zNxt = z(:,idxNxt);

fPrv = f(:,idxPrv);
fNxt = f(:,idxNxt);

% This is the key line:  (Trapazoid Rule)
defects = zNxt-zPrv - 0.5*dt*(fPrv+fNxt);

end

