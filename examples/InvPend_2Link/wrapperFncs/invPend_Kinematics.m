function [p1,p2,dp1,dp2] = invPend_Kinematics(t,z,params)
%INVPEND_KINEMATICS wrapper to numerically compute 3-link invertered Pendulum's EOMs
% FILENAME: invPend_Dynamics.m
% AUTHOR:   Roberto Shu
% LAST EDIT: 
%
% INPUTS:
%   t:      descritize time vector
%   z:      1st order state vector [4 X n] <--> [q X n;dq X n]
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
q1 = z(1,:);
q2 = z(2,:);
dq1 = z(3,:);
dq2 = z(4,:);

[p1,p2,dp1,dp2] = invPend_2DoF_kin(q1,q2,...
                                   dq1,dq2,...
                                   p.l1,p.l2);
                                       
end

