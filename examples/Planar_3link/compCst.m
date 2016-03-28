  function [ c_comp, ceq_comp] = compCst(Phi,t,lambda)

% Read input
nGrid = length(t);
nLambda = size(Phi,1);
lambdaY = lambda;

%% INEQUALITY CONSTRAINTS
% S.T. No penetatrion
PhiVec = reshape(Phi,numel(Phi),1);
c_nopen = -PhiVec;

% S.T. Positive normal contact force
lambdaYvec = reshape(lambdaY,numel(lambdaY),1);
c_lambda = -lambdaYvec;

% S.T. Static friction limit
c_fric = [];
%% EQUALITY CONSTRAINTS
% S.T. force nonzero if and only if contact is active
% ceq_PhiLambda = zeros(nGrid*nLambda,1);
% for i = 1:nGrid*nLambda
%     ceq_PhiLambda(i) = PhiVec(i).*lambdaYvec(i);
% end
ceq_PhiLambda = PhiVec.*lambdaYvec;

% S.T. No-slip condition
ceq_slip = [];

%% CONCATENATE CONSTRAINTS
c_comp = [c_nopen;c_lambda;c_fric];
ceq_comp = [ceq_PhiLambda;ceq_slip];

end