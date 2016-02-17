function [Phi,Psi,J] = autogen_contactDyn(x,y,th,dx,dy,dth,w,h,m,I,g,a,b,c)
%AUTOGEN_CONTACTDYN
%    [PHI,PSI,J] = AUTOGEN_CONTACTDYN(X,Y,TH,DX,DY,DTH,W,H,M,I,G,A,B,C)

%    This function was generated by the Symbolic Math Toolbox version 6.3.
%    17-Feb-2016 04:52:34

t3 = h.*(1.0./2.0);
t2 = -t3+y;
Phi = [t2;t2;t2];
if nargout > 1
    Psi = [dy;dy;dy];
end
if nargout > 2
    J = reshape([0.0,0.0,0.0,1.0,1.0,1.0,0.0,0.0,0.0],[3,3]);
end
