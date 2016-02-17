% PURPOSE: Performance a contact invariant optimization of a box
%          with 3 contact points falling on a slope
% FILENAME: MAIN.m
% AUTHOR:   Roberto Shu
% LAST EDIT: 2/16/16
%------------------- Instructions ---------------------------

%% ----------------------------------------------------------
%   INITIALIZE WORKSPACE
% -----------------------------------------------------------
% Clear workspace
clc; clear; close all;

% Add paths
addpath(genpath('../../..'));
addpath('solutions');
addpath('autogenFncs');
addpath('wrapperFncs');
addpath('helperFncs');
%% ----------------------------------------------------------
%   MODEL PROPERTIES
%   Intialize your dynamic model parameter values and dyanmics
% -----------------------------------------------------------
OCP.model.params = params_fallingBox_model;
OCP.model.dynamics = @(t,x,u,lambda)fallingBox_slantedDyn_wrap(t,x,lambda,OCP.model.params);

%% ----------------------------------------------------------
%   DEFINE OCP PROBLEM PROPERTIES
% -----------------------------------------------------------

% COST/OBJECTIVE FUNCTIONS
% ------------------------
% e.g.
%   Step cost: OCP.pathCostFnc = @(t,x,u)model_StepCostFnc(t,x,u, OCP.model.params);
%   Terminal cost: OCP.bndCostFnc = @(t,x,u)model_TerminalcostFnc(t,x,u, OCP.model.params);
OCP.pathCostFnc = [];
OCP.bndCostFnc = @(t,x,u)fallingBox_costFnc(t,x,u,OCP.model.params);

% INITIAL GUESS
% ------------------------

% Load initial guess
data = load('fallingBox_soln6.mat');
data = data.soln.grid;

% Time span
t0 = 0;
tF = 2.5;
%OCP.ig.time = [t0,tF];
OCP.ig.time = data.time;

% State 
% x0, xF:   state vector [q;dq] => [2*n X 1] column vector
%   [x; y; th; dx; dy; dth];
OCP.ig.state = [data.state(1:2,:);...
                zeros(1,size(data.state,2));...
                data.state(3:4,:);...
                zeros(1,size(data.state,2))];

% Control input
OCP.ig.control = zeros(1,65);

% Contact forces
% For a given contact point i, the contact force is
%   lambda_i = [lambdaX_i;lambdaY_i]
% expressed in a frame with X tangent and Y normal to 
% the contact surface.
% If there are m contact points then 
%   lambda = [2m x nt]

lambdaY = [data.lambda(1,:);...
           data.lambda(1,:);...
           data.lambda(1,:)];
lambdaX = [data.lambda(2,:);...
           data.lambda(2,:);...
           data.lambda(2,:)];
OCP.ig.lambda = [lambdaX;lambdaY];

% CONSTRAINTS & BOUNDARIES
% ------------------------
 
%----- Nonlinear constraints
% e.g.
% Path:
%   OCP.pathCst = @(t,x,u)pathCst(z);
% Boundary: 
%   OCP.bndCst = @(t0,x0,u0,tF,xF,uF)bndCst(z);
% Complementary:
%   OCP.compCst = @(t0,x0,u0,tF,xF,uF)compCst(z);
OCP.pathCst = [];
OCP.bndCst = [];
OCP.compCst = @(Phi,Gamma,t,x,u,lambda)fallingBox_compCst(Phi,Gamma,t,x,u,lambda,OCP.model.params);


%----- Linear constraints
% You can let time to be free by not setting any bounds on the final time
OCP.bounds.initTime.lb = t0;
OCP.bounds.initTime.ub = t0;
OCP.bounds.finalTime.lb = tF;
OCP.bounds.finalTime.ub = tF;

% State:
OCP.bounds.state.lb = [-10; -50; -2*pi; -inf(3,1)]; 
OCP.bounds.state.ub = [10; 50; 2*pi; inf(3,1)];
OCP.bounds.initState.lb = [0; -50; 0; 0; 0;0];
OCP.bounds.initState.ub = [0; 40; 0; 0; 0;0];
OCP.bounds.finalState.lb = [-10; -50; -2*pi; -inf(3,1)];
OCP.bounds.finalState.ub = [10; 50; 2*pi; inf(3,1)];

% Control:
%%TODO
% Change so they are set from model parametes
maxTau = 500;
OCP.bounds.control.lb = 0; 
OCP.bounds.control.ub = 0;

% Contact forces:
OCP.bounds.lambda.lb = -inf(6,1);
OCP.bounds.lambda.ub = inf(6,1);

%% ----------------------------------------------------------
%   SOLVER OPTIONS
% -----------------------------------------------------------

%%% TODO
% add capability for other methods
%method = 'euler';
method = 'euler_mod';
% method = 'trapezoidal';
% method = 'hermiteSimpson';

%--- Interation 1
options(1).method = 'euler_mod';
options(1).nGrid = 20;

%--- Interation 2
options(2).method = 'euler_mod';
options(2).nGrid = 65;


% For a full list of options refer to :
%   http://www.mathworks.com/help/optim/ug/fmincon.html#inputarg_options
%%% TODO setting options here is not working need to see why
OCP.options.fminOpt.MaxFunEval = 1e5;

% Display initial guess
displayIGnBnds(OCP.ig,OCP.bounds,options(1).nGrid);
%% ----------------------------------------------------------
%   SOLVE NLP PROBLEM
% -----------------------------------------------------------

for iter = 1:size(options,2)
    % Set options to pass to solver
    OCP.options = options(iter);
    
    % Solve Optimal control problem
    soln(iter) = OptCtrlSolver(OCP);
    
    % Update initial condition
    OCP.ig = soln(iter).grid;
end

tGrid = soln(end).grid.time;
t = linspace(tGrid(1),tGrid(end),100);
z = soln(end).interp.state(t);
u = soln(end).interp.control(t);
tgrid = soln(end).grid.time;
zgrid = soln(end).grid.state;
ugrid = soln(end).grid.control;
lambda = soln(end).grid.lambda;
guess = soln(end).guess;


%Notes = 'Revised complementary constraint calculation';
%save('solutions/fallingBox_soln6.mat','soln','OCP','Notes')
%% ----------------------------------------------------------
%   PLOT RESULTS
% -----------------------------------------------------------
dyn = OCP.model.params;

% Initial guess
figure
subplot(4,1,1)
plot(guess.time,guess.state(2,:))
xlabel('Time [sec]');
ylabel('Pos [m]');

subplot(4,1,2)
plot(guess.time,guess.state(4,:))
xlabel('Time [sec]');
ylabel('Velocity [m/sec]');

subplot(4,1,3)
plot(guess.time,guess.control)
xlabel('Time [sec]');
ylabel('U control input');

subplot(4,1,4)
plot(guess.time,guess.lambda(1,:),guess.time,guess.lambda(2,:))
xlabel('Time [sec]');
ylabel('Lambda');

% Solution 
figure
subplot(3,2,1)
plot(tgrid,zgrid(2,:));
xlabel('Time [sec]');
ylabel('Post [m]');
title('Vertical direction')

subplot(3,2,3)
plot(tgrid,zgrid(4,:));
xlabel('Time [sec]');
ylabel('Vel. [m/sec]');

subplot(3,2,5)
plot(tgrid,lambda(1,:),tgrid,lambda(2,:),tgrid,lambda(3,:))
xlabel('Time [sec]');
ylabel('Lambda');

subplot(3,2,2)
plot(tgrid,zgrid(1,:));
xlabel('Time [sec]');
ylabel('Pos [m]');
title('Horizontal direction')

subplot(3,2,4)
plot(tgrid,zgrid(3,:));
xlabel('Time [sec]');
ylabel('Vel.[m/sec]');

subplot(3,2,6)
plot(tgrid,lambda(1,:))
xlabel('Time [sec]');
ylabel('Lambda');


% Animate the results:
% A.plotFunc = @(t,z)( drawModel(t,z,dyn) );
% A.speed = 0.25;
% A.figNum = 101;
% animate(t,z,A)

%%% TODO
% Draw a stop-action animation:
