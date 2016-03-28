% PURPOSE: Performance a contact invariant optimization of a 3 link 
%          inverted pendulum model of a bipedal robot
% FILENAME: MAIN.m
% AUTHOR:   Roberto Shu
% LAST EDIT: 
%------------------- Instructions ---------------------------
%
% Generalized state: z = [x,y,th,a1,a2]'
%
%

%% ----------------------------------------------------------
%   INITIALIZE WORKSPACE
% -----------------------------------------------------------
% Clear workspace
clc; clear; close all;

% Define folder names
solnFolderName = 'solutions';

% Add paths
add_SNOPT;
addpath('../../');
addpath('../../methods');
addpath('autogenFncs');
addpath('wrapperFncs');
%% ----------------------------------------------------------
%   MODEL PROPERTIES
%   Intialize your dynamic model parameter values and dyanmics
% -----------------------------------------------------------
OCP.model.params = params_planar_3link;
OCP.model.dynamics = @(x,u,lambda)dynamics_wrap(x,u,lambda,OCP.model.params);
OCP.model.contactDyn = @(x)contactDyn_wrap(x,OCP.model.params);
%% ----------------------------------------------------------
%   DEFINE OCP PROBLEM PROPERTIES
% -----------------------------------------------------------

% COST/OBJECTIVE FUNCTIONS
% ------------------------
% e.g.
%   Step cost: OCP.pathCostFnc = @(t,x,u)model_pathcostFnc(t,x,u, OCP.model.params);
%   Terminal cost: OCP.bndCostFnc = @(t,x,u)model_bndcostFnc(t,x,u, OCP.model.params);
OCP.pathCostFnc = @(t,x,u)costFnc(t,x,u,OCP.model.params);
OCP.bndCostFnc = [];

% INITIAL GUESS
% ------------------------

% Time span
t0 = 0;
tF = 2.5;

OCP.ig.time = [t0;tF];
OCP.ig.state = [0, 1.112, 0, 0, 0, zeros(1,5);...
                0, 1.112, 0, 0, 0, zeros(1,5)]';
OCP.ig.control = zeros(2,2);
OCP.ig.lambda = zeros(4,2);

% CONSTRAINTS & BOUNDARIES
% ------------------------
%----- Nonlinear constraints
%%TODO this is the same for all systems this should be defined internally
OCP.compCst = @(Phi,t,lambda)compCst(Phi,t,lambda);
%OCP.pathCst = @(z)pathCst(z,OCP.model.params);
OCP.bndCst = [];%  @(z)bndCst(z,OCP.model.params);

%----- Linear constraints
% You can let time to be free by not setting any bounds on the final time
OCP.bounds.initTime.lb = t0;
OCP.bounds.initTime.ub = t0;
OCP.bounds.finalTime.lb = tF;
OCP.bounds.finalTime.ub = tF;

% State:
OCP.bounds.state.lb = [0; 0; -2*pi; 0; 0; -inf(5,1)]; 
OCP.bounds.state.ub = [5; 10; 2*pi; pi; 2*pi/3; inf(5,1)];

OCP.bounds.initState.lb = [0; 1.112; 0; 0; 0; zeros(5,1)];
OCP.bounds.initState.ub = [0; 1.112; 0; 0; 0; zeros(5,1)];

OCP.bounds.finalState.lb = [0; 0; 0; pi/2; pi/4*0; zeros(5,1)];
OCP.bounds.finalState.ub = [0; 2; 0; pi/2; pi/4*0; zeros(5,1)];

% Control:
OCP.bounds.control.lb = -inf(2,1); 
OCP.bounds.control.ub = inf(2,1);

% Contact forces:
OCP.bounds.lambda.lb = zeros(4,1);
OCP.bounds.lambda.ub = inf(4,1);

%% ----------------------------------------------------------
%   SOLVER OPTIONS
% -----------------------------------------------------------

fminOpt = optimoptions('fmincon','Display','iter','Algorithm','interior-point','MaxIter',1e4,'MaxFunEvals',1e6,'TolFun',1e-6);

%--- Interation 1
options(1).method = 'euler_back';
options(1).nGrid = 20;
options(1).fminOpt = fminOpt;

%--- Interation 2
% options(2).method = 'euler_back';
% options(2).nGrid = 20;
% options(2).fminOpt = fminOpt;

%% ----------------------------------------------------------
%   SOLVE NLP PROBLEM
% -----------------------------------------------------------
for iter = 1:size(options,2)
    fprintf('--------- Optimization Pass No.: %d ---------\n',iter)
    % Set options to pass to solver
    OCP.options = options(iter);
    
    % Solve Optimal control problem
    soln = OptCtrlSolver_wSNOPT2(OCP);
    
    % Update initial condition
    OCP.ig = soln.grid;
end


tgrid = soln(end).grid.time;
zgrid = soln(end).grid.state;
ugrid = soln(end).grid.control;
lambda = soln(end).grid.lambda;
guess = soln(end).guess;

% Save results
% fileName = 'fallingBox_soln';
% overWrite = 0;
% Notes = 'Working solution';
% saveResults(solnFolderName, fileName, overWrite, soln,OCP,Notes)
%% ----------------------------------------------------------
%   PLOT RESULTS
% -----------------------------------------------------------
dyn = OCP.model.params;

figure
subplot(5,1,1)
plot(tgrid,zgrid(1,:))
subplot(5,1,2)
plot(tgrid,zgrid(2,:))
subplot(5,1,3)
plot(tgrid,zgrid(3,:))
subplot(5,1,4)
plot(tgrid,zgrid(4,:))
subplot(5,1,5)
plot(tgrid,zgrid(5,:))


figure
subplot(4,1,1)
plot(tgrid,lambda(1,:))
subplot(4,1,2)
plot(tgrid,lambda(2,:))
subplot(4,1,3)
plot(tgrid,lambda(3,:))
subplot(4,1,4)
plot(tgrid,lambda(4,:))


% Plot results
%fallingBox_plotResults(tgrid,zgrid,ugrid,lambda)

% Animate the results:
A.plotFunc = @(t,z)( draw_planar_3link(t,z,OCP.model.params) );
A.speed = 0.25;
A.figNum = 101;
animate(tgrid,zgrid,A)

%%% TODO
% Draw a stop-action animation:
