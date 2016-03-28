function [U,T] = autogen_fallingBox_Energy(x,y,th,dx,dy,dth,w,h,m,I,g)
%AUTOGEN_FALLINGBOX_ENERGY
%    [U,T] = AUTOGEN_FALLINGBOX_ENERGY(X,Y,TH,DX,DY,DTH,W,H,M,I,G)

%    This function was generated by the Symbolic Math Toolbox version 6.3.
%    14-Mar-2016 18:51:21

U = I.*dth.^2.*(1.0./2.0)+dx.^2.*m.*(1.0./2.0)+dy.^2.*m.*(1.0./2.0);
if nargout > 1
    T = g.*m.*y;
end
