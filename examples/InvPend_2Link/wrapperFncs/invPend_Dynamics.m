function [ dz ] = invPend_Dynamics(t,z,u,params)
%INVPEND_DYNAMICS wrapper to numerically compute 3-link invertered Pendulum's EOMs
% FILENAME: invPend_Dynamics.m
% AUTHOR:   Roberto Shu
% LAST EDIT: 
%
% INPUTS:
%   t:      descritize time vector
%   z:      1st order state vector [6 X n] <--> [q X n;dq X n]
%           n = number of grid/descritazation points
%   u:      control input [2 X n]
%   params: structure variable with model parameters
%
% OUTPUTS:
%   dz: System state derivative    
%

%% ----------------------------------------------------------
%   READ AND INTERPRET INPUTS
% -----------------------------------------------------------
nt = length(t);
p = params;
q = z(1:2,:);
dq = z(3:4,:);

if isempty(u)
    u = zeros(1,nt);
end

%% ----------------------------------------------------------
%   EQUATION OF MOTION
% -----------------------------------------------------------

% allocate memory space
ddq = zeros(size(q));

% Numerically compute EOMs
for i=1:nt
    
    % EOM matrices
    [D,C,G,B] = autogen_invPend_2DoF_EOM_mtxs(q(1,i),q(2,i),...
                        dq(1,i),dq(2,i),...
                        u(1,i),...
                        p.l1,p.l2,...
                        p.d1,p.d2,...
                        p.m1,p.m2,...
                        p.I1,p.I2,...
                        p.g);
 
    % Invert EOM matrices to get ddq
    ddq(:,i) = D\(B*u(:,i) - C*dq(:,i) - G);                              
end    

% Construct state derivative vector
dz = [dq;ddq];

end