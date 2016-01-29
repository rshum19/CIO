function [ defects ] = euler_def(dt,z,f) 

nTime = size(z,2);

idxPrv = 1:(nTime-1);
idxNxt = 2:nTime;

zPrv = z(:,idxPrv);
zNxt = z(:,idxNxt);

fPrv = f(:,idxPrv);

% This is the key line:  (Euler Rule)
defects = zNxt - zPrv - dt*fPrv;

end

