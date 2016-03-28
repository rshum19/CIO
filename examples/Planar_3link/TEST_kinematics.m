%% TEST KINEMATICS 

% ----------------------------------------------------------
%   INITIALIZE WORKSPACE
% -----------------------------------------------------------
clear all; close all; clc;

addpath('wrapperFncs');
addpath('autogenFncs');

% Load system parameters
params = params_planar_3link;

% Dummy coordinates
z = [0; 1.112; 0; 0; 0]; % vertical position
dz = [0; 0; 2; 0.5; 0];
%z = [0; 0; pi/2; 0; 0]; % horizontal position
%z = [0; 0; pi/4; 0; 0]; % diagonal position
%z = [0; 0; 0; pi/4; pi/4];
[p1,p2,p3,com] = kin_wrt_link2_wrap(z,dz,params);

% w.r.t CoM
%zcom = [com.g;z(4);z(5)];
%[p11,p22,p33,gg] = kin_wrt_CoM_wrap(zcom,dz,params);

figure
plot([p2.g(1);p1.h(1)],[p2.g(2);p1.h(2)],'r')
hold on;
plot([p2.g(1);p3.f(1)],[p2.g(2);p3.f(2)],'m')
plot([p1.h(1);p1.f(1)],[p1.h(2);p1.f(2)],'b')
plot([p3.f(1);p3.h(1)],[p3.f(2);p3.h(2)],'g')
plot(com.g(1),com.g(2),'ok','markersize',10)
