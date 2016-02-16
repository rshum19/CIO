function [dz, Phi, Psi] = fallingBox_dynamics(t,z,lambda,params)


%% ----------------------------------------------------------
%   READ AND INTERPRET INPUTS
% -----------------------------------------------------------
nt = length(t);
p = params;
x = z(1:2,:);
dx = z(3:4,:);
g = p.g;
h = p.h;

% Gravity vector
G = [0;-g];

J = [0,1];

if isempty(lambda)
    lambda = zeros(1,nt);
end
nc = size(lambda,1);
%% ----------------------------------------------------------
%   EQUATION OF MOTION
% -----------------------------------------------------------
% Allocate memory
Phi = zeros(nc,nt);

for i = 1:nt
 
    % Box acceleration
    ddx(:,i)  = G + J'*lambda(:,i);
    
    % Phi function -- contact point equation
    Phi(:,i) = x(2,i) - h/2;
 
    % Velocity of contact point
    %Psi(:,i) = J*dx(:,i); % Psi_y
     Psi(:,i) = 0; % Psi_x
 
end

dz = [dx;ddx];
end

