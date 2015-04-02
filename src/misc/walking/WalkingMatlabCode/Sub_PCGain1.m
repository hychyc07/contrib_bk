%% Continue Cart table-model
Ac = [0 1 0;
      0 0 1;
      0 0 0];
Bc = [0;0;1];
Cc = [1 0 -z_c/G];
Dc = [0];

% Discrete cart table-model    sysdD = C2D(sysdC,Ts,METHOD)
sysd = c2d(ss(Ac,Bc,Cc,Dc),T);

Atilde = [1 sysd.C*sysd.A; zeros(size(sysd.A,1),1) sysd.A];
Btilde = [sysd.C*sysd.B; sysd.B];
Ctilde = [1 zeros(1,size(sysd.A,1))];
Itilde = [1;0;0;0];

%%%%%%%%% new added for spring model %%%%%%%%%%%
Ytilde = [1 T 0; 0 1 0; 0 0 0];
Ztilde = [T*T/2.0; T ; 1];
%%%%%%%%%%%%%%%%

% Calculation of gains

Qc = 1;
R = 1E-6;

% [K,S,E] = DLQRY(A,B,C,D,Q,R)
[K, P, E] = dlqry(Atilde,Btilde,Ctilde,0,Qc,R);

GI = K(1);
Gd1 = K(2:end);

tussenterm =  R + Btilde'*P*Btilde;

Actilde=Atilde-Btilde/(tussenterm)*Btilde'*P*Atilde;

Xtilde=-Actilde'*P*Itilde;
Gd(1)=-K(1);
for n=2:N_L
   Gd(n)=1/tussenterm*Btilde'*Xtilde;
   Xtilde=Actilde'*Xtilde;
end