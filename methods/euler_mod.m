function [ defects ] = euler_mod(dt,q,f) 

nTime = size(q,2);
nStates = size(q,1)/2;

idxPrv = 1:(nTime-1);
idxNxt = 2:nTime;

qPrv = q(:,idxPrv);
qNxt = q(:,idxNxt);

fPrv = f(:,idxPrv);
fNxt = f(:,idxNxt);

% This is the key line:  (Euler Rule)
defects = qPrv - qNxt + dt*fNxt;

end
