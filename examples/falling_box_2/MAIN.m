% PURPOSE: Performance a contact invariant optimization of a 3 link 
%          inverted pendulum model of a bipedal robot
% FILENAME: MAIN.m
% AUTHOR:   Roberto Shu
% LAST EDIT: 
%------------------- Instructions ---------------------------

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
addpath('wrapFncs');
%% ----------------------------------------------------------
%   MODEL PROPERTIES
%   Intialize your dynamic model parameter values and dyanmics
% -----------------------------------------------------------
OCP.model.params = params_fallingBox_model;
OCP.model.dynamics = @(x,lambda)dynamics_wrap(x,lambda,OCP.model.params);
OCP.model.contactDyn = @(x)contactDyn_wrap(x,OCP.model.params);
%% ----------------------------------------------------------
%   DEFINE OCP PROBLEM PROPERTIES
% -----------------------------------------------------------

% COST/OBJECTIVE FUNCTIONS
% ------------------------
% e.g.
%   Step cost: OCP.pathCostFnc = @(t,x,u)model_pathcostFnc(t,x,u, OCP.model.params);
%   Terminal cost: OCP.bndCostFnc = @(t,x,u)model_bndcostFnc(t,x,u, OCP.model.params);
OCP.pathCostFnc = [];
OCP.bndCostFnc = @(t,x)fallingBox_costFnc(t,x,OCP.model.params);

% INITIAL GUESS
% ------------------------
% Load initial guess
data = load('fallingBox_soln5.mat');
data = data.soln.grid;
% Time span
% size: 1x65
t0 = 0;
tF = 2.5;

OCP.ig.time = data.time;
OCP.ig.state = [data.state(1:2,:);zeros(1,65);data.state(3:4,:);zeros(1,65)];
OCP.ig.control = data.control;
OCP.ig.lambda = [data.lambda(2,:);data.lambda(2,:);data.lambda(2,:)];

% CONSTRAINTS & BOUNDARIES
% ------------------------
 
%----- Nonlinear constraints
OCP.compCst = @(Phi,t,lambda)fallingBox_compCst(Phi,t,lambda);

%----- Linear constraints
% You can let time to be free by not setting any bounds on the final time
OCP.bounds.initTime.lb = t0;
OCP.bounds.initTime.ub = t0;
OCP.bounds.finalTime.lb = tF;
OCP.bounds.finalTime.ub = tF;

% State:
OCP.bounds.state.lb = [-10; -50; -2*pi;-inf(3,1)]; 
OCP.bounds.state.ub = [10; 50; 2*pi; inf(3,1)];
OCP.bounds.initState.lb = [0; 7; 0; 0;0;0];
OCP.bounds.initState.ub = [0; 10; 0; 0;0;0];
OCP.bounds.finalState.lb = [-10; -50; -2*pi; -inf(3,1)];
OCP.bounds.finalState.ub = [20; 50; 2*pi; inf(3,1)];

% Control:
OCP.bounds.control.lb = -20; 
OCP.bounds.control.ub = 20;

% Contact forces:
OCP.bounds.lambda.lb = zeros(3,1);
OCP.bounds.lambda.ub = inf(3,1);

%% ----------------------------------------------------------
%   SOLVER OPTIONS
% -----------------------------------------------------------

fminOpt = optimoptions('fmincon','Display','iter','Algorithm','interior-point','MaxIter',1e4,'MaxFunEvals',1e6,'TolFun',1e-6);

%--- Interation 1
options(1).method = 'euler_back';
options(1).nGrid = 30;
options(1).fminOpt = fminOpt;

%--- Interation 2
options(2).method = 'euler_back';
options(2).nGrid = 50;
options(2).fminOpt = fminOpt;

% Display initial guess
%displayIGnBnds(OCP.ig,OCP.bounds,OCP.options(1).nGrid);
%% ----------------------------------------------------------
%   SOLVE NLP PROBLEM
% -----------------------------------------------------------
for iter = 1:size(options,2)
    fprintf('--------- Optimization Pass No.: %d ---------\n',iter)
    % Set options to pass to solver
    OCP.options = options(iter);
    
    % Solve Optimal control problem
    soln = OptCtrlSolver_wSNOPT(OCP);
    
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

% Plot results
fallingBox_plotResults(tgrid,zgrid,ugrid,lambda)

% Animate the results:
% A.plotFunc = @(t,z)( drawModel(t,z,dyn) );
% A.speed = 0.25;
% A.figNum = 101;
% animate(t,z,A)

%%% TODO
% Draw a stop-action animation:
