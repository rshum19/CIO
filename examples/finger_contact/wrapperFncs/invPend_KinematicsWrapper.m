function [p1,p2,p3,dp1,dp2,dp3] = invPend_KinematicsWrapper(t,z,params)
%INVPEND_KINEMATICS wrapper to numerically compute 3-link invertered Pendulum's EOMs
% FILENAME: invPend_Dynamics.m
% AUTHOR:   Roberto Shu
% LAST EDIT: 
%
% INPUTS:
%   t:      descritize time vector
%   z:      1st order state vector [6 X n] <--> [q X n;dq X n]
%           n = number of grid/descritazation points
%   params: structure variable with model parameters
%
% OUTPUTS:
%   p1: System state derivative    
%   p2:

%% ----------------------------------------------------------
%   READ AND INTERPRET INPUTS
% -----------------------------------------------------------
nt = length(t);
p = params;
q = z(1:5,:);
dq = z(6:10,:);

[p1,p2,p3,dp1,dp2,dp3] = invPend_3link_kin(q(1),q(2),q(3),q(4),q(5),...
                                           dq(1),dq(2),dq(3),dq(4),dq(5),...
                                           p.l1,p.l2,p.l3);
                                       
end

