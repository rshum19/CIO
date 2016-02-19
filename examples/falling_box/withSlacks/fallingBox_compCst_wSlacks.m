function [ c_comp, ceq_comp] = fallingBox_slack_compCst(Phi,Psi,t,x,u,lambda,slacks,params)

%%---- READ INPUT
nt = length(t);

% Static friction
mu = params.mu;

% Separate contact force into normal and tangential 
gamma = slacks;

%%---- INEQUALITY CONSTRAINTS
% s.t. No penetatrion
PhiVec = reshape(Phi,numel(Phi),1);
c_nopen = -PhiVec;

% s.t. Positive tangential component forces
lambdaXpVec = reshape(lambda.Xp,numel(lambda.Xp),1);
c_lambdaXp = -lambdaXpVec;

lambdaXnVec = reshape(lambda.Xn,numel(lambda.Xn),1);
c_lambdaXn = -lambdaXnVec;

% s.t. Positive normal contact force
lambdaYvec = reshape(lambda.Y,numel(lambda.Y),1);
c_lambdaY = -lambdaYvec;

c_lambda = [c_lambdaXp; c_lambdaXn; c_lambdaY];

% s.t. Slack Variables
gammaVec = reshape(gamma,numel(gamma),1);
c_slack = -gammaVec;

% s.t. Static friction limit
c_fric = -((mu*lambdaYvec) - lambdaXpVec - lambdaXnVec);

% s.t. Slack Variables
for i = 1:nt
    c_slackp = -(gamma(:,i) + Psi(:,i));
    c_slackn = -(gamma(:,i) - Psi(:,i));
end
c_slack = [c_slackp;c_slackn];

%%---- EQUALITY CONSTRAINTS
% s.t. force nonzero if and only if contact is active
% s.t. No-slip condition
ceq_lambda = zeros(nt,1);
ceq_fric = zeros(nt,1);
ceq_slipp = zeros(nt,1);
ceq_slipn = zeros(nt,1);
for i = 1:nt
    ceq_lambda(i) = Phi(:,i)'*lambda.Y(:,i);
    ceq_fric(i) = (mu*lambda.Y(:,i) - lambda.Xp(:,i) - lambda.Xn(:,i))'*gamma(:,i);
    ceq_slipp(i) = (gamma(:,i) + Psi(:,i))'*lambda.Xp(:,i);
    ceq_slipn(i) = (gamma(:,i) - Psi(:,i))'*lambda.Xn(:,i);
end
ceq_slip = [ceq_slipp; ceq_slipn];

%%---- Concatenate constraints
c_comp = [c_nopen; c_lambda; c_fric; c_slack];
ceq_comp = [ceq_lambda; ceq_fric; ceq_slip];

end

