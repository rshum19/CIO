  function [ c_comp, ceq_comp] = fallingBox_compCst(Phi,Psi,t,x,u,lambda,params)

nt = length(t);
% Static friction
mu = params.mu;

% Read input
lambdaX = lambda(1:size(lambda,1)/2,:);
lambdaY = lambda((size(lambda,1)/2+1):end,:);

%% INEQUALITY CONSTRAINTS
% S.T. No penetatrion
PhiVec = reshape(Phi,numel(Phi),1);
c_nopen = -PhiVec;

% S.T. Positive normal contact force
lambdaYvec = reshape(lambdaY,numel(lambdaY),1);
c_lambda = -lambdaYvec;

% S.T. Static friction limit
lambdaXvec = reshape(lambdaX,numel(lambdaX),1);
c_fric = -((mu*lambdaYvec).^2 - lambdaXvec.^2);

%% EQUALITY CONSTRAINTS
% S.T. force nonzero if and only if contact is active
% S.T. No-slip condition
ceq_PhiLambda = zeros(nt,1);
ceq_slip = zeros(nt,1);
for i = 1:nt
    ceq_PhiLambda(i) = Phi(:,i)'*lambdaY(:,i);
    ceq_slip(i) = Psi(:,i)'*lambdaX(:,i);
end

% Concatenate constraints
%c_comp = [c_nopen;c_lambda;c_fric];
%ceq_comp = [ceq_PhiLambda; ceq_slip];
c_comp = [c_nopen;c_lambda];
ceq_comp = ceq_PhiLambda;

end

