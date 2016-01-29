function [time,state,control,lambda] = gen_initGuess
%clear all; close all; clc;

p = params_hopper_1D_model;


% Time
t = linspace(0,4,50);
time = t;

% State
ang = linspace(0,4*pi,50);
y = (0.3*cos(ang)+1)*p.l0;
dy = 0.03*sin(ang+pi);
l = (0.1*cos(ang)+1)*p.l0;
dl = 0.03*sin(ang+pi);
u = [0;0];
u = interp1([t(1),t(end)]', u',t')';
du = u;
state = [y;l;u;dy;dl;du];

% Control
taul = interp1([t(1),t(end)]', [0;0]',t')';
control = taul;

% Lambda
lambdaX = interp1([t(1),t(end)]', [0;0]',t')';
lambdaZ = interp1([t(1),t(end)]', [0;0]',t')';

lambda = [lambdaX;lambdaZ];

% figure
% subplot(3,1,1)
% plot(ang,y,ang,dy);
% subplot(3,1,2)
% plot(ang,l,ang,dl);
% subplot(3,1,3)
% plot(ang,u,ang,du);

end
