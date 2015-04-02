function [xf yf zf] = FootTraj(S,H,W,Duration)
global T
% S   step length
% H   step height
% W   step width
% T  step duration
% interpolate X respect to time
% Wxt = [0, .5, 1];
% Wt = [0, .3, 1];
% interpolate Y Z respect to X
Wz = [ 0, 1, 0.9, 0.4, 0];  
Wx =  [0, .3, 0.5, .9, 1];
Wy = Wx; % Y is proportional to X

tt = T : T :Duration;
% xf = spline(Wt*Duration, [0 Wxt 0]*S,tt);  % plan x 
% xf = poly51(0,S,0,Duration,tt);
xf = poly5(0,S,0,Duration,tt);

yf = spline(Wx*S,Wy*W,xf);  % correlate Y and Z with Z instead of time
zf = spline(Wx*S, Wz*H,xf);
end
% this m file, the xf is always symetric, later i will think how to get the
% non symetric xf profile with 0 acceleration at 0 and T moment.
% Z and Y are correlated wtih xf.