function [c_bnd, ceq_bnd] = hopper_1D_bndCst(t,x,u,params)
%
%
% Defines the non-linear boundary constraints to ensure that the output
% gait is periodic.
%

nStates = size(x,1)/2;
q = x(1:nStates,:);
dq = x(nStates+1:end,:);

%--- EQUALITY CONSTRAINTS

% S.T. Initial state = Final state
ceq_temp1 = x(1) - x(end);
ceq_temp1 = reshape(ceq_temp1,numel(ceq_temp1),1);

%---- Consctruct constraint vector
c_bnd = [];
ceq_bnd = ceq_temp1;
end