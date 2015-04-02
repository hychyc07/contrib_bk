zmpz = zeros(tsize,1);
zmpx_multibody = zeros(tsize,1);
zmpy_multibody = zeros(tsize,1);
zmpx_multibody2 = zeros(tsize,1);
zmpy_multibody2 = zeros(tsize,1);

zmpx2 = zeros(tsize,1);
zmpy2 = zeros(tsize,1);

x_h = zeros(tsize,size(Ac,1));

ezmpx = 0.0;
ezmpy = 0.0;

xh= zeros(tsize,1);
xhd= zeros(tsize,1);
xhdd= zeros(tsize,1);
yh= zeros(tsize,1);
yhd= zeros(tsize,1);
yhdd= zeros(tsize,1);
zh= zeros(tsize,1);
zh(1)=hip_height;
zhd= zeros(tsize,1);
zhdd= zeros(tsize,1);

xh2= zeros(tsize,1);
xhd2= zeros(tsize,1);
xhdd2= zeros(tsize,1);
yh2= zeros(tsize,1);
yhd2= zeros(tsize,1);
yhdd2= zeros(tsize,1);

xfl= zeros(tsize,1);
yfl= zeros(tsize,1);
zfl= zeros(tsize,1);
xfr= zeros(tsize,1);
yfr= zeros(tsize,1);
zfr= zeros(tsize,1);
%joint angular velocity left leg
qdL(:,1)=[0.0;0.0;0.0;0.0;0.0;0.0];
qddL(:,1)=[0.0;0.0;0.0;0.0;0.0;0.0];
qdR(:,1)=[0.0;0.0;0.0;0.0;0.0;0.0];
qddR(:,1)=[0.0;0.0;0.0;0.0;0.0;0.0];
COGd(:,:,1)=[[0.0,0.0,0.0,0.0,0.0,0.0,0.0];[0.0,0.0,0.0,0.0,0.0,0.0,0.0];[0.0,0.0,0.0,0.0,0.0,0.0,0.0]];
COGdd(:,:,1)=[[0.0,0.0,0.0,0.0,0.0,0.0,0.0];[0.0,0.0,0.0,0.0,0.0,0.0,0.0];[0.0,0.0,0.0,0.0,0.0,0.0,0.0]];
        
qdL_link_old=[[0.0,0.0,0.0,0.0,0.0,0.0];[0.0,0.0,0.0,0.0,0.0,0.0];[0.0,0.0,0.0,0.0,0.0,0.0]];
qdR_link_old=[[0.0,0.0,0.0,0.0,0.0,0.0];[0.0,0.0,0.0,0.0,0.0,0.0];[0.0,0.0,0.0,0.0,0.0,0.0]];

%%%%%%%
% initial hip position x y z 
% x = [0 ; 0 ; 0]; 
% x = [-0.0105 ; 0 ; 0];
 x = zeros(size(Ac,1),1);
y = zeros(size(Ac,1),1);
z = [hip_height ; 0 ; 0]; %z(1) is position z(2) is velocity z(3) is acceleration
az_store= zeros(tsize,1);
Pz(1) =  hip_height;
Vz(1) = 0;