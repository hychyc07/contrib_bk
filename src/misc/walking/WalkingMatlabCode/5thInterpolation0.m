function f = 5thInterpolation(xi,xf,Tstart,Tend,time)
D = xf - xi;
T = Tend - Tstart;
% t=0:1e-2:1;
t = (time - Tstart)/T;
p = [6, - 15, 10, 0, 0,0]; % coefficients of 5th order polynomial
f = D * polyval(p, t);
end
% plot(f)