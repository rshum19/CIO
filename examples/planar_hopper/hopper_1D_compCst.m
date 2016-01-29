function [c_comp, ceq_comp] = hopper_1D_compCst(Phi,Gamma,t,x,u,lambda,params)


% Static friction
mu = params.mu;

% Read input
lambdaX = lambda(1:size(lambda,1)/2,:);
lambdaY = lambda((size(lambda,1)/2+1):end,:);

%% INEQUALITY CONSTRAINTS
% S.T. No penetatrion
c_nopen = -Phi(1,:)';

% S.T. Positive normal contact force
c_lambda = -lambdaY(1,:)';

% S.T. Static friction limit
c_fric = (mu*lambdaY).^2 - lambdaX.^2;
c_fric = reshape(c_fric,numel(c_fric),1);

%% EQUALITY CONSTRAINTS
% S.T. force nonzero if and only if contact is active
ceq_lambda = Phi.*lambdaY;
ceq_lambda = reshape(ceq_lambda,numel(Phi),1);

% S.T. No-slip condition
ceq_slip = Gamma.*lambdaY;
ceq_slip = reshape(ceq_slip,numel(ceq_slip),1);

% Concatenate constraints
c_comp = [c_nopen;c_lambda;c_fric];
ceq_comp = [ceq_lambda; ceq_slip];
end

