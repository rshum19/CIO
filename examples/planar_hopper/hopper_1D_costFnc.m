function [ cost ] = hopper_1D_costFnc(t,x,u,params)

nTime = length(t);
nState = size(x,1);
nControl = size(u,1);

%---- Terminal cost
mechJointCost = x(end,:).*u;

%---- Step cost

cost = mechJointCost;

end

