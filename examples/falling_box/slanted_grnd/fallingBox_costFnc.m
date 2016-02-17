function [ cost ] = fallingBox_costFnc(t,x,u,params)

nTime = length(t);
nState = size(x,1);
nControl = size(u,1);
xdes = [0;-10];
%---- Terminal cost
xend = x(1,round(nTime/2):end);
yend = x(2,round(nTime/2):end);
terminal_cost = sum((xend-xdes(1).*ones(size(xend))).^2) + sum((yend-xdes(2).*ones(size(yend))).^2) ;

%---- Step cost
step_cost = 0;

%---- Total cost
cost = step_cost + terminal_cost;

end

