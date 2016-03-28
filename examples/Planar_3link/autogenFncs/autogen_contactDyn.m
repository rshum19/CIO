function [Phi,Psi] = autogen_contactDyn(x,y,th,a1,a2,dx,dy,dth,da1,da2,m1,m2,m3,I1,I2,I3,l1,l2,l3,G)
%AUTOGEN_CONTACTDYN
%    [PHI,PSI] = AUTOGEN_CONTACTDYN(X,Y,TH,A1,A2,DX,DY,DTH,DA1,DA2,M1,M2,M3,I1,I2,I3,L1,L2,L3,G)

%    This function was generated by the Symbolic Math Toolbox version 6.3.
%    25-Mar-2016 23:25:55

t2 = cos(th);
t3 = l2.*t2.*(1.0./2.0);
t4 = a1+th;
t5 = sin(t4);
t6 = sin(th);
t7 = l2.*t6.*(1.0./2.0);
t8 = a2-th;
Phi = [t3+y+l1.*cos(t4);t3+y;-t3+y;-t3+y-l3.*cos(t8)];
if nargout > 1
    t9 = sin(t8);
    Psi = [dy-dth.*(t7+l1.*t5)-da1.*l1.*t5;dy-dth.*l2.*t6.*(1.0./2.0);dy+dth.*l2.*t6.*(1.0./2.0);dy+dth.*(t7-l3.*t9)+da2.*l3.*t9];
end