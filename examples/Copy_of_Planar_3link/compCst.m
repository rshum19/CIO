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

%% EQUALITY CONSTRAINTS
% S.T. force nonzero if and only if contact is active
ceq_PhiLambda = zeros(nGrid*nLambda,1);
for i = 1:nGrid*nLambda
    ceq_PhiLambda(i) = PhiVec(i).*lambdaYvec(i);
end
%ceq_PhiLambda2 = PhiVec.*lambdaYvec;

% Concatenate constraints
c_comp = [c_nopen;c_lambda];
ceq_comp = ceq_PhiLambda;

end