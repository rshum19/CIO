function NLPwCC(Problem)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

%-----------------------------------
% Define problem
%-----------------------------------

P.user.model.params = Problem.model.params;
P.user.model.dynamics = Problem.model.dynamics;

%-----------------------------------
% Construct problem
%-----------------------------------
P.objective =  @(z,Prob)myCostFnc(z,Prob);
P.grad = [];
P.Hessian = [];
P.HessPattern = [];
P.x_L = []; P.x_U = [];

P.Name = [];
P.x_0 = [];
P.pSepFunc = [];
P.fLowBnd = [];


% Linear Constraints
P.A = []; P.b_L = []; P.b_U = [];

% Nonlinear Constraints:
%   1) Dynamics
%   2) Complementary conditions
%   3) Path
%   4) Boundary
P.c = @(z,Prob)nonlcon; 
P.dc = []; P.d2c = []; P.ConsPattern = []; 
P.c_L = zeros(nGrid-1,1); P.c_U = zeros(nGrid-1,1);


P.x_min = []; P.x_max = []; P.f_opt = []; P.x_opt = [];

%% -----------------------------------------------------------
% SNOPT with TOMLAB
% -----------------------------------------------------------
% Construct problem in TOMLAB's format
Prob = conAssign(P.objective,P.grad, P.Hessian, P.HessPattern, P.x_L, P.x_U, P.Name, P.x_0, ...
                            P.pSepFunc, P.fLowBnd, ...
                            P.A, P.b_L, P.b_U,....
                            P.c, P.dc, P.d2c, P.ConsPattern, P.c_L, P.c_U, ...
                            P.x_min, P.x_max, P.f_opt, P.x_opt);

% Solver options
Prob.Warning = 0;    % Turning off warnings.   

% Run TOMLAB
%%TODO add OCP variable to pick Solver
PriLev = 1;
Result = tomRun('snopt', Prob, PriLev);

%zSoln = Result.x_k;
%[tSoln,xSoln,uSoln,lambdaSoln] = unMux(zSoln,muxInfo);

end

%-------------------------------------------------------
% SUB-FUNCTIONS
%-------------------------------------------------------
function cost = myCostFnc(z,Prob)

[t,x,u,lambda] = deMux(z,Prob.user.muxInfo);

cost = (x(2,end)-10).^2;

end

function c = nonlcon(z,Prob)
%NONLCON Adds the Nonlinear Constraints to the problem:
% 
% Types/Group of constraints:
%   1) Dynamics -> ceq_dyn = 0
%   2) Complementary conditions -> ceq_cc 
%   3) Path -> ceq_path
%   4) Boundary -> ceq_bnd

% READ INPUTS
[t,x,u,lambda] = deMux(z,Prob.user.muxInfo);

% DYNAMICS
% ----------------
F = dynamics(t,x,u,lambda);


ceq_dyn = dynamicConstraint(dt,z,F);

c = ceq_dyn;
end

function [z,muxInfo] = mux(t,x,u,lambda)
% MUX constructs vector to pass to NLP solver fmincon

nTime = length(t);
nState = size(x,1);
nCtrl = size(u,1);
nLambda = size(lambda,1);

tSpan = [t(1); t(end)];
stateCol = reshape(x, nState*nTime, 1);
ctrlCol = reshape(u, nCtrl*nTime, 1);
lambdaCol = reshape(lambda,nLambda*nTime,1);
z = [tSpan; stateCol; ctrlCol; lambdaCol];

muxInfo.nTime = nTime;
muxInfo.nState = nState;
muxInfo.nCtrl = nCtrl;
muxInfo.nLambda = nLambda;
end

function [t,x,u,lambda] = deMux(z,muxInfo)

nTime = muxInfo.nTime;
nState = muxInfo.nState;
nCtrl = muxInfo.nCtrl;
nLambda = muxInfo.nLambda;
nx = nState*nTime;
nu = nCtrl*nTime;
nl = nLambda*nTime;

t = linspace(z(1),z(2),nTime);
x = reshape(z((3):(2+nx)),nState,nTime);
u = reshape(z((3+nx):(2+nx+nu)),nCtrl,nTime);
lambda = reshape(z((3+nx+nu):(2+nx+nu+nl)),nLambda,nTime);

end


