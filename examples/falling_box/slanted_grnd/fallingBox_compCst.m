function [ c_comp, ceq_comp] = fallingBox_compCst(Phi,Psi,t,x,u,lambda,params)

%%---- READ INPUT
nt = length(t);

% Static friction
mu = params.mu;

% Separate contact force into normal and tangential 
lambdaX = lambda(1:size(lambda,1)/2,:);
lambdaY = lambda((size(lambda,1)/2+1):end,:);

%%---- INEQUALITY CONSTRAINTS
% s.t. No penetatrion
PhiVec = reshape(Phi,numel(Phi),1);
c_nopen = -PhiVec;

% s.t. Positive normal contact force
lambdaYvec = reshape(lambdaY,numel(lambdaY),1);
c_lambda = -lambdaYvec;

% s.t. Static friction limit
lambdaXvec = reshape(lambdaX,numel(lambdaX),1);
c_fric = (mu*lambdaYvec).^2 - lambdaXvec.^2;

%%---- EQUALITY CONSTRAINTS
% s.t. force nonzero if and only if contact is active
% s.t. No-slip condition
ceq_lambda = zeros(nt,1);
ceq_slip = zeros(nt,1);
for i = 1:nt
    ceq_lambda(i) = Phi(:,i)'*lambdaY(:,i);
    ceq_slip(i) = Psi(:,i)'*lambdaX(:,i);
end

%%---- Concatenate constraints
c_comp = [c_nopen;c_lambda;c_fric];
ceq_comp = [ceq_lambda; ceq_slip];

end

