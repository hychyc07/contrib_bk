%global MODELPARAMETERS
%global Cp

global L Lmx Lmz mass massTot I Lag Lab Rh Laf Ltb Ltf Lat Ltg theta_F_TD theta_F_endDS K_TJ
global hip_Comy lengthupperleg lengthunderleg footankletotip footankletoside footankletoback
global MassUpperbody MassUpperleg MassUnderleg MassFoot MassTot
global COG_UB COG_UpperLeg COG_UnderLeg COG_foot
global IG_body_loc IG_foot_loc IG_underleg_loc IG_upperleg_loc


hip_Comy        = 0.0800;      % hip displacement to the pelvis center (distance from hip to the pelvis center) ORIGINALLY 0.0726
lengthupperleg  = 0.2258;
lengthunderleg  = 0.201;
footankletotip  = 0.12;
footankletoside = 0.09/2.0;
footankletoback = 0.06;
ankle_height    = 0.0793;       % ankle height is 6.03cm, not that small


MassUpperbody = 10.275;         % for coman 12.5;           %
MassUpperleg  = 3.89;           % 0.9+0.3475+1.522=2.7695
MassUnderleg  = 2.268;          % 1.572+0.898=2.4700
MassFoot      = 0.917;          %

MassTot = MassUpperbody+2.0*MassUpperleg+2.0*MassUnderleg+2.0*MassFoot;
global Mass
Mass = [MassFoot, MassUnderleg, MassUpperleg, MassFoot, MassUnderleg, MassUpperleg, MassUpperbody];
COG_UB = (1*[-0.03;0;0.13]+ 0.26*[-0.0053;0;0.22624]+0.2915*[-0.0053;0;0.17729]+0.8516*[0;0;0.152465]+4.14*[-0.03;0;0.08636])/MassUpperbody;
COG_UpperLeg = (0.9*[0;0;0]+0.3475*[0;0;-0.0405]+1.522*[0;0;-0.157962])/(MassUpperleg);
COG_UnderLeg =(0.898*[0.0092826;0;-0.2013]+1.572*[0;0;-0.08568])/MassUnderleg;
COG_foot = [0.0242615;0;-0.03284];


% IG_body_loc = [[2.5253e-2,0,0];[0,8.538e-3,0];[0,0,2.162978e-2]];
% IG_upperleg_loc = [[0.0090+5.41148e-3,0,0];[0,0.0090+5.4257e-3,0];[0,0,7.4619e-4]];
% IG_underleg_loc = [[6.3829e-3+0.898*0.01,0,0];[0,5.8657e-3+0.898*0.01,0];[0,0,1.199823e-3]];
% IG_foot_loc = [[5.07979e-4,0,0];[0,1.20573e-3,0];[0,0,9.476e-4]];

IG_body_loc     = [[0,0,0];[0,0,0];[0,0,0]];
IG_upperleg_loc = [[0,0,0];[0,0,0];[0,0,0]];
IG_underleg_loc = [[0,0,0];[0,0,0];[0,0,0]];
IG_foot_loc     = [[0,0,0];[0,0,0];[0,0,0]];


Lag=0.1;
Lab=0.1;
Rh=0.05;

Laf=0.15;
Ltb=0.025;
Ltf=0.025;
Lat=Laf-Ltf;
Ltg = 0.02;

theta_F_TD = 10.0*pi/180.0;
theta_F_endDS = -10.0*pi/180.0;
K_TJ = 100;