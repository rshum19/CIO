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
clc; clear;

% Add paths
addpath('wrapperFncs/');
addpath('autogenFncs/');
addpath(genpath('../../'));

%% ----------------------------------------------------------
%   MODEL PROPERTIES
%   Intialize your dynamic model parameter values and dyanmics
% -----------------------------------------------------------
OCP.model.params = params_hopper_1D_model;
OCP.model.dynamics = @(t,x,u,lambda)hopper_1D_DynamicsWrapper(t,x,u,lambda,OCP.model.params);

%% ----------------------------------------------------------
%   DEFINE OCP PROBLEM PROPERTIES
% -----------------------------------------------------------

% COST/OBJECTIVE FUNCTIONS
% ------------------------
% e.g.
%   OCP.pathCostFnc = @(t,x,u)model_pathcostFnc(t,x,u, OCP.model.params);
%   OCP.bndCostFnc = @(t,x,u)model_bndcostFnc(t,x,u, OCP.model.params);
OCP.pathCostFnc = @(t,x,u)hopper_1D_costFnc(t,x,u,OCP.model.params);
OCP.bndCostFnc = [];

% INITIAL GUESS
% ------------------------
% Set the type of initial guess:
%   linear: Just define a intial and final state and OptCtrlSolver will
%           make a linear interpolation between the two points
%   custom: The user provides a custom initial guess "shape". 
%           NOTE: custom initial guess has to be descritized with the same
%           size than OCP.options.nGrid
OCP.options.IGtype = 'linear';

% Time span
t0 = 0;
tF = 2;
OCP.ig.time = [t0,tF];

% State 
% x0, xF:   state vector [q;dq] => [2*n X 1] column vector
%   [y; l; dy; dl];
%  
l0 = OCP.model.params.l0;
x0 = [1.3*l0; 1*l0; 0; 0];
xF = [l0; l0; 0; 0];
OCP.ig.state = [x0,xF];

% Control
ul = [0;0];
OCP.ig.control = ul;

% Contact forces
% For a given contact point i, the contact force is
%   lambda_i = [lambdaX_i;lambdaZ_i]
% expressed in a frame with X tangent and Z normal to 
% the contact surface.
% If there are m contact points then 
%   lambda = [m x 2]
lambda0 = [0;0];
lambdaF = [0;0];
OCP.ig.lambda = [lambda0, lambdaF];

% CONSTRAINTS & BOUNDARIES
% ------------------------
 
% Nonlinear constraints
% e.g.
%   OCP.pathCst = @(t,x,u)pathCst(z);
%   OCP.bndCst = @(t0,x0,u0,tF,xF,uF)bndCst(z);
%   OCP.compCst = @(t0,x0,u0,tF,xF,uF)compCst(z);
OCP.pathCst = [];
OCP.bndCst = [];
OCP.compCst = @(Phi,Gamma,t,x,u,lambda)hopper_1D_compCst(Phi,Gamma,t,x,u,lambda,OCP.model.params);

% You can let time to be free by not setting any bounds on the final time
OCP.bounds.initTime.lb = t0;
OCP.bounds.initTime.ub = t0;
OCP.bounds.finalTime.lb = t0+0.1;
OCP.bounds.finalTime.ub = tF*6;

% State:
%   [y; l; dy; dl];
%  
OCP.bounds.state.lb = [0; 0; -inf(2,1)]; 
OCP.bounds.state.ub = inf(4,1);
OCP.bounds.initState.lb = [1.3*l0; 1*l0; 0; 0];
OCP.bounds.initState.ub = [1.3*l0; 1*l0; 0; 0];
OCP.bounds.finalState.lb = [l0; l0; 0; 0];
OCP.bounds.finalState.ub = [l0; l0; 0; 0];

% Control:
%%TODO
% Change so they are set from model parametes
maxTorque = 25; 
OCP.bounds.control.lb = -maxTorque; 
OCP.bounds.control.ub = maxTorque;

% Contact forces:
OCP.bounds.lambda.lb = -inf(2,1);
OCP.bounds.lambda.ub = inf(2,1);

%% ----------------------------------------------------------
%   SOLVER OPTIONS
% -----------------------------------------------------------

%%% TODO
% add capability for other methods
method = 'euler';
% method = 'trapezoidal';
% method = 'hermiteSimpson';

OCP.options.method = 'trapezoidal';
OCP.options.nGrid = 15;

% For a full list of options refer to :
%   http://www.mathworks.com/help/optim/ug/fmincon.html#inputarg_options
%%% TODO setting options here is not working need to see why
OCP.options.fminOpt.MaxFunEval = 1e5;

%% ----------------------------------------------------------
%   SOLVE NLP PROBLEM
% -----------------------------------------------------------

soln = OptCtrlSolver(OCP);

tGrid = soln(end).grid.time;
t = linspace(tGrid(1),tGrid(end),100);
z = soln(end).interp.state(t);
u = soln(end).interp.control(t);


%% ----------------------------------------------------------
%   PLOT RESULTS
% -----------------------------------------------------------
dyn = OCP.model.params;

figure
subplot(3,1,1)
plot(t,z(1,:));
xlabel('Time [sec]');
ylabel('Mass M1 height [m]');

subplot(3,1,2)
plot(t,z(3,:));
xlabel('Time [sec]');
ylabel('Mass M1 velocity [m/sec]');

subplot(3,1,3)
plot(t,u)
xlabel('Time [sec]');
ylabel('U control input');

% Animate the results:
% A.plotFunc = @(t,z)( drawModel(t,z,dyn) );
% A.speed = 0.25;
% A.figNum = 101;
% animate(t,z,A)

%%% TODO
% Draw a stop-action animation:
