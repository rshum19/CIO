function [ dz, Phi, Gamma] = hopper_1D_DynamicsWrapper(t,z,u,lambda,params)
%INVPEND_DYNAMICS wrapper to numerically compute 3-link invertered Pendulum's EOMs
% FILENAME: invPend_Dynamics.m
% AUTHOR:   Roberto Shu
% LAST EDIT: 1/28/16
%
% INPUTS:
%   t:      descritize time vector
%   z:      1st order state vector [10 X n] <--> [q X n;dq X n]
%           n = number of grid/descritazation points
%   u:      control input [2 X n]
%   lambda: constraint forces [m x n] 
%           m = no. of possible contact points
%   params: structure variable with model parameters
%
% OUTPUTS:
%   dz: System state derivative    
%   P:  contact points Y position for non-penetration constraint
%

%% ----------------------------------------------------------
%   READ AND INTERPRET INPUTS
% -----------------------------------------------------------
nt = length(t);
p = params;
q = z(1:2,:);
dq = z(3:4,:);

if isempty(u)
    u = 0;
end

%% ----------------------------------------------------------
%   EQUATION OF MOTION
% -----------------------------------------------------------

% allocate memory space
ddq = zeros(size(q));
Phi = zeros(size(lambda));
Gamma = zeros(size(lambda));

% Numerically compute EOMs
for i=1:nt
    
    % EOM matrices
    [D,C,G,B,P,J] = hopper_1D_EOM_mtxs(q(1,i),q(2,i),...
                        dq(1,i),dq(2,i),...
                        u(1,i),...
                        p.l0,p.kl,...
                        p.m1,p.m2,...
                        p.g);
 
    % Invert EOM matrices to get ddq
    ddq(:,i) = D\(B*u(:,i) + J'*lambda(:,i) - C*dq(:,i) - G); 
    
    % Concatenate Phi vectors
    Phi(:,i) = P;
    
    % Velocity of contact point
    Gamma(:,i) = J*dq(:,i);
end    

% Construct state derivative vector
dz = [dq;ddq];

end