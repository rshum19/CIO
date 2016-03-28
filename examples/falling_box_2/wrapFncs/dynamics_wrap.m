function [dx] = dynamics_wrap(x,lambda,params)

nStates = size(x,1)/2;
p = params;

q = x(1:nStates,:);
dq = x(nStates+1:end,:);

ddq = zeros(nStates,size(x,2));
for i = 1:size(x,2)
    ddq(:,i) = autogen_fallingBox_ddq(q(1,i),q(2,i),q(3,i),...
                                dq(1,i),dq(2,i),dq(3,i),...
                                lambda(1,i),lambda(2,i),lambda(3,i),...
                                p.w,p.h,p.m,p.I,p.g);
end
dx = [dq;ddq];
end

