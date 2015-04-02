function [xf yf zf] = FootTraj2(S,H,W,Duration)
global T
% S   step length
% H   step height
% W   step width
% T  step duration
% interpolate X respect to time
Wxt = [0, .5, 1];
Wt = [0, .3, 1];
% interpolate Y Z respect to X
Wz = [ 0, 1, 0.8, 0.35, 0];  
Wx =  [0, .2, 0.5, .9, 1];
% Wz = [ 0, 1, 0];  
% Wx =  [0, 0.5,1];
Wy = Wx; % Y is proportional to X

tt = T : T :Duration;
xf = spline(Wt*Duration, [0 Wxt 0]*S,tt);  % plan x 
% xf = poly51(0,S,0,Duration,tt);

yf = spline(Wx*S,Wy*W,xf);  % correlate Y and Z with Z instead of time
zf = spline(Wx*S, Wz*H,xf);
end
% this m file, the xf is not symetric, i can shift the center point of xf.
% however, i use spline here, so the acceleration is non zero, i will solve
% this problem later. 