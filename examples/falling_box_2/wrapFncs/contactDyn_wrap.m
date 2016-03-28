function [Phi,Psi,J] = contactDyn_wrap(x,params)

nStates = size(x,1)/2;
p = params;

q = x(1:nStates,:);
dq = x(nStates+1:end,:);

Phi = zeros(3,size(x,2));
for i = 1:size(x,2)
    [Phi(:,i),Psi,J] = autogen_contactDyn(q(1,i),q(2,i),q(3,i),...
                                     dq(1,i),dq(2,i),dq(3,i),...
                                     p.w,p.h,p.m,p.I,p.g);
end
end