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
data = load('fallingBox_slanted_soln4.mat');
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

fallingBox_plotResults2(t,x,lambda)

% Kinematics
[cntc_pts] = fallingBox_slantedKin_wrap(t,x,OCP.model.params);

figure
% X - position
subplot(1,2,1)
plot(t,cntc_pts(1,:))

% subplot(3,2,3)
% plot(t,cntc_pts(3,:))
% ylabel('X-pos [m]');
% 
% subplot(3,2,5)
% plot(t,cntc_pts(5,:))
% xlabel('time [sec]')
% Y-position
subplot(1,2,2)
plot(t,cntc_pts(2,:))

% subplot(3,2,4)
% plot(t,cntc_pts(4,:))
% ylabel('Y-pos [m]');
% 
% subplot(3,2,6)
% plot(t,cntc_pts(6,:))
% xlabel('time [sec]')


% Forward dynamics
[f, Phi, Psi] = OCP.model.dynamics(t,x,u,lambda(2,:));
figure
plot(t,Phi,'*-')
xlabel('Time [sec]')
ylabel('Phi function value');

% Complementary constraints 
[c_comp, ceq_comp] = OCP.compCst(Phi,Psi,t,x,u,lambda);

nt = length(t);
ceq_lambda = zeros(nt,1);
ceq_slip = zeros(nt,1);
for i = 1:nt
    ceq_lambda(i) = Phi(:,i)'*lambda(2,i);
    ceq_slip(i) = Psi(:,i)'*lambda(1,i);
end

figure
plot(t,ceq_lambda,'*-')
xlabel('Time [sec]')
ylabel('\Phi \lambda ');

% S.T. No-slip condition
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