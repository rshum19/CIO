function [dz, Phi, Psi] = fallingBox_slantedDyn_wrap(t,z,lambda,params)


%% ----------------------------------------------------------
%   READ AND INTERPRET INPUTS
% -----------------------------------------------------------
nt = length(t);
p = params;
x = z(1:3,:);
dx = z(4:6,:);
g = p.g;

% Mass matrix
M = diag([p.m,p.m,p.I]);

% Gravity vector
G = [0;-g;0];  

if isempty(lambda)
    lambda = zeros(1,nt);
end
nc = size(lambda,1);
%% ----------------------------------------------------------
%   EQUATION OF MOTION
% -----------------------------------------------------------
% Allocate memory
Phi = zeros(nc,nt);
Psi = zeros(nc,nt);
ddx = zeros(size(x,1),nt);
for i = 1:nt
 
    % Contact dynamic matrices Phi, Psi, J
    [Phi(:,i),Psi(:,i),J] = fallingBox_contactDyn_wrapper(z(:,i),p);

    % Box acceleration
    ddx(:,i)  = inv(M)*(G + J'*lambda(:,i));
    
end

dz = [dx;ddx];
end

