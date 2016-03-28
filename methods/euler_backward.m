function [defects] = euler_backward(dt,z,F)
%DYNAMICCONSTRAINTS
%
% Backward Euler integration
%   q_{k}-q_{k+1} + dt*dq_{k+1} = 0
%   dq_{k}-dq_{k+1} + dt*F_{k+1} = 0
% where,
%    F = D\(B*u + J'*lambda - C*q - G) = ddq
%
% INPUTS:
%   dt:         length of the time step
%   z:          stacked state vector z = [q;dq]
%   F:          state second derivative F = ddq
% OUTPUTS:
%   defects:    this

%---------------------------------------------------
% READ INPUT
%---------------------------------------------------
nStates = size(z,1)/2;
nSteps = size(z,2);

q = z(1:nStates,:);
dq = z(nStates+1:end,:);

idxM = 1:(nSteps-1);
idxP = 2:nSteps;

%---------------------------------------------------
% DEFECTS: BACKWARD EULER INTEGRATION
%---------------------------------------------------
%defects = [q(:,idxM) - q(:,idxP) + dt.*dq(:,idxP);...
%           dq(:,idxM) - dq(:,idxP) + dt.*F(:,idxP)];

defects = zeros(size(z,1),size(z,2)-1);
for i = 1:size(z,2)-1
    defects(:,i) = z(:,i) - z(:,i+1) + dt*F(:,i+1);
end
%defects = reshape(defects,[numel(defects),1]);
end

