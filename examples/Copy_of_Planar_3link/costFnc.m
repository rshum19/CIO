function [ cost ] = costFnc(t,x,u,params)

nTime = length(t);
nState = size(x,1)/2;
nControl = size(u,1);


%---- Step cost
%u = reshape(u,1,numel(u));
cost = u.*u;

end