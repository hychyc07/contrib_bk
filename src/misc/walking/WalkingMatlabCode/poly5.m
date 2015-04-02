function xt = poly5(xi,xf,Tstart,Tend,time)
T = Tend - Tstart;
t= time/T;
xt = xi + (xf - xi)*( 10*t.^3 - 15*t.^4 + 6*t.^5 );
end

% I can also use pchip which is more smooth than spline
%Examples
% 
% x = -3:3; 
% y = [-1 -1 -1 0 1 1 1]; 
% t = -3:.01:3;
% p = pchip(x,y,t);
% s = spline(x,y,t);
% plot(x,y,'o',t,p,'-',t,s,'-.')
% legend('data','pchip','spline',4)
% i have just tried pchip, it is not as good as 5th order polynomial
% method. the acceleration in pchip is not smooth and starts from non-zero