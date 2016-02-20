function [ defects ] = euler_backward(dt,y,f) 

nTime = size(y,2);
nStates = size(y,1)/2;

idxPrv = 1:(nTime-1);
idxNxt = 2:nTime;

yPrv = y(:,idxPrv);
yNxt = y(:,idxNxt);

fPrv = f(:,idxPrv);
fNxt = f(:,idxNxt);

% This is the key line:  (Euler Rule)
defects = yPrv - yNxt + dt*fNxt;

end

% Backward Euler Integration sample
% for i = 1:n
%   t = t + h;
%   y = (y + h*(1-t))/(1-4*h);
% end