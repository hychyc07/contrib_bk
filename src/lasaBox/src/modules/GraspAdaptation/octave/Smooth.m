function S = Smooth(X,W)


if 0
figure(1);
clf
hold on;
X = [1:100];
X = exp(-0.5*X.*X*0.001) ;
plot(X)
X = X + randn(1,100)*0.1 ;
plot(X,'r')
X = [X;X];
end

N = size(X,2);
M = size(X,1);
%W = 10;

R = zeros(M,N);
L = zeros(M,N);

CXR = X;
CXL = X;
for i=1:W    
    R = R + CXR;
    CXSR = shift(CXR,1,2);
    CXSR(:,1) = CXR(:,1);
    CXR = CXSR;

    L = L + CXL;
    CXSL = shift(CXL,1,2);
    CXSL(:,1) = CXL(:,1);
    CXL = CXSL;
end
R = R/W;
L = L/W;

S = (R+L)/2.0;

%plot(S','g');
