function [U,T] = hopper_1D_Energy(y,l,ul,dy,dl,dul,l0,k,b,m1,m2,ml,g)
%HOPPER_1D_ENERGY
%    [U,T] = HOPPER_1D_ENERGY(Y,L,UL,DY,DL,DUL,L0,K,B,M1,M2,ML,G)

%    This function was generated by the Symbolic Math Toolbox version 6.3.
%    29-Jan-2016 02:24:22

U = dl.^2.*m2.*(1.0./2.0)+dy.^2.*m1.*(1.0./2.0)+dul.^2.*ml.*(1.0./2.0);
if nargout > 1
    T = g.*m1.*y-g.*m2.*(l-y);
end
