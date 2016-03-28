function [ cost ] = costFnc(t,x,u,params)
% Inputs:
%   t:          time knots t = [1 X nGrid]
%   x:          state vector x = [2*nStates X nGrid]
%   u:          vector of control inputs u = [nControls x nGrid]
%   params:     structure element containing problem parameters
nTime = length(t);
nState = size(x,1)/2;
nControl = size(u,1);


%---- Step cost
%u = reshape(u,1,numel(u));
%cost = u(1,:).*u(1,:) + u(2,:).*u(2,:) - 50*x(2,:).*x(2,:) ;
cost = -20*x(1,:).*x(1,:) - 20*x(6,:).*x(6,:);

end