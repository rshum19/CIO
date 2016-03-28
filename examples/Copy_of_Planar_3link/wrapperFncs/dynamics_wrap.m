function [dz] = dynamics_wrap(z,u,lambda,params)

nStates = size(z,1)/2;
p = params;

q = z(1:nStates,:);
dq = z(nStates+1:end,:);

ddq = zeros(nStates,size(z,2));
for i = 1:size(z,2)
    
    [D_mtx,C_mtx,G_vec,B_mtx,J] = autogen_EOM(q(1,i),q(2,i),q(3,i),q(4,i),q(5,i),...
                                              dq(1,i),dq(2,i),dq(3,i),dq(4,i),dq(5,i),...
                                              p.m1,p.m2,p.m3,...
                                              p.I1,p.I2,p.I3,...
                                              p.l1,p.l2,p.l3,...
                                              p.g);
                                          
    ddq(:,i) = D_mtx\(-C_mtx*dq(:,i) - G_vec + B_mtx*u(:,i) + J'*lambda(:,i));                                 
end

dz = [dq;ddq];
end

