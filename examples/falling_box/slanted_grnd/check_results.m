% PURPOSE: Analyze results
%
% FILENAME: check_results.m
% AUTHOR:   Roberto Shu
% LAST EDIT: 
%------------------- Instructions ---------------------------

%% ----------------------------------------------------------
%   INITIALIZE WORKSPACE
% -----------------------------------------------------------
% Clear workspace
clc; clear; close all;

%% ----------------------------------------------------------
%   LOAD RESULTS
% -----------------------------------------------------------
data = load('fallingBox_slanted_soln1.mat');
soln = data.soln;

OCP.model.params = params_fallingBox_model;
OCP.model.dynamics = @(t,x,u,lambda)fallingBox_dynamics(t,x,lambda,OCP.model.params);
OCP.compCst = @(Phi,Gamma,t,x,u,lambda)fallingBox_compCst(Phi,Gamma,t,x,u,lambda,OCP.model.params);

%% ----------------------------------------------------------
%   CHECK CONSTRAINT SATISFACTION
% -----------------------------------------------

t = soln.grid.time;
x = soln.grid.state;
u = soln.grid.control;
lambda = soln.grid.lambda;


figure
plot(t,x(2,:));
xlabel('time [sec]');
ylabel('y-pos [m]');

figure
subplot(3,2,1)
plot(t,lambda(1,:))

subplot(3,2,2)
plot(t,lambda(2,:))

subplot(3,2,3)
plot(t,lambda(3,:))

subplot(3,2,4)
plot(t,lambda(4,:))

subplot(3,2,5)
plot(t,lambda(5,:))

subplot(3,2,6)
plot(t,lambda(6,:))

% Forward dynamics
[f, Phi, Gamma] = OCP.model.dynamics(t,x,u,lambda(2,:));
figure
plot(t,Phi,'*-')
xlabel('Time [sec]')
ylabel('Phi function value');

% Complementary constraints 
[c_comp, ceq_comp] = OCP.compCst(Phi,Gamma,t,x,u,lambda);

ceq_lambda = Phi.*lambda(2,:);
ceq_lambda = reshape(ceq_lambda,numel(ceq_lambda),1);
ceq_lambda2 = Phi*lambda(2,:)';
ceq_lambda2 = reshape(ceq_lambda2,numel(ceq_lambda2),1);

figure
plot(t,ceq_lambda,'*-')
xlabel('Time [sec]')
ylabel('\Phi \lambda ');

% S.T. No-slip condition
ceq_slip = Gamma.*lambda(1,:);
ceq_slip = reshape(ceq_slip,numel(ceq_slip),1);
figure
plot(t,ceq_slip,'*-')
xlabel('Time [sec]')
ylabel('\Phi \lambda ');

figure
plot(1:length(c_comp),c_comp,'o-',1:length(ceq_comp),ceq_comp,'*-')
xlabel('Index')
ylabel('Constraint value')
legend('inequalities','equalities')
title('Complementary Constraints')