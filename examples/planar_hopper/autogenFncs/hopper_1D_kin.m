function [p1,p2,dp1,dp2] = hopper_1D_kin(y,l,ul,dy,dl,dul,l0)
%HOPPER_1D_KIN
%    [P1,P2,DP1,DP2] = HOPPER_1D_KIN(Y,L,UL,DY,DL,DUL,L0)

%    This function was generated by the Symbolic Math Toolbox version 6.3.
%    29-Jan-2016 02:24:22

p1 = y;
if nargout > 1
    p2 = -l+y;
end
if nargout > 2
    dp1 = dy;
end
if nargout > 3
    dp2 = -dl+dy;
end
