%global MODELPARAMETERS
%global Cp

global L Lmx Lmz mass massTot I Lag Lab Rh Laf Ltb Ltf Lat Ltg theta_F_TD theta_F_endDS K_TJ
global hip_Comy lengthupperleg lengthunderleg footankletotip footankletoside footankletoback
global MassUpperbody MassUpperleg MassUnderleg MassFoot MassTot
global COG_UB COG_UpperLeg COG_UnderLeg COG_foot
global IG_body_loc IG_foot_loc IG_underleg_loc IG_upperleg_loc


hip_Comy        = 0.075;       % hip displacement to the pelvis center (distance from hip to the pelvis center) ORIGINAL 0.0681
lengthupperleg  = 0.2345;       % taken from module
lengthunderleg  = 0.2005;       % taken from module
footankletotip  = 0.12;         % compute from module 
footankletoside = 0.09/2.0;     % compute from module
footankletoback = 0.06;         % compute from module
ankle_height    = 0.0793;       % compute from module


MassUpperbody = 10.275;       % icub->upperTorso->total_mass_UP + icub->upperTorso->total_mass_LF + icub->upperTorso->total_mass_RT + icub->lowerTorso->total_mass_UP;
MassUpperleg  =  3.199;         % 0.9+0.3475+1.522=2.7695  ... for iCub = 0.695 + 0.982 + 1.522 = 3.199 from iCubLegDynV2
MassUnderleg  =  2.675;         % 1.572+0.898=2.4700       ... for iCub = 2.032 + 0.643 = 2.6750        from iCubLegDynV2
MassFoot      =  0.861;         % 0.917                    ... for iCub 0.861

MassTot = MassUpperbody+2.0*MassUpperleg+2.0*MassUnderleg+2.0*MassFoot;
global Mass
Mass = [MassFoot, MassUnderleg, MassUpperleg, MassFoot, MassUnderleg, MassUpperleg, MassUpperbody];


COG_UB = (1*[-0.03;0;0.13]+ 0.26*[-0.0053;0;0.22624]+0.2915*[-0.0053;0;0.17729]+0.8516*[0;0;0.152465]+4.14*[-0.03;0;0.08636])/MassUpperbody;
% COG_UpperLeg = (0.9*[0;0;0]+0.3475*[0;0;-0.0405]+1.522*[0;0;-0.157962])/(MassUpperleg);

COG_UpperLeg = (0.695*[80.4310e-6;-6.91643e-3;2.30470e-3] + 0.982*[-31.3229e-6;-1.09640e-3;59.8624e-3] + 1.522*[ 3.1143e-3;60.0431e-3;-1.3437e-3] )/MassUpperleg;
% COG_UnderLeg =(0.898*[0.0092826;0;-0.2013]+1.572*[0;0;-0.08568])/MassUnderleg;
COG_UnderLeg = (2.032*[124.3956e-3;-45.0091e-6;-6.2408e-3] + 0.643*[-26.5892e-6;-0.82253e-3;-10.2574e-3])/MassUnderleg;
%COG_foot = [0.0242615;0;-0.03284];
COG_foot = [32.41539e-3,  613.9310e-6,  24.0690e-3];


% IG_body_loc = [[2.5253e-2,0,0];[0,8.538e-3,0];[0,0,2.162978e-2]];
% IG_upperleg_loc = [[0.0090+5.41148e-3,0,0];[0,0.0090+5.4257e-3,0];[0,0,7.4619e-4]];
% IG_underleg_loc = [[6.3829e-3+0.898*0.01,0,0];[0,5.8657e-3+0.898*0.01,0];[0,0,1.199823e-3]];
% IG_foot_loc = [[5.07979e-4,0,0];[0,1.20573e-3,0];[0,0,9.476e-4]];
% 
IG_body_loc     = [[0,0,0];[0,0,0];[0,0,0]];
IG_upperleg_loc = [[0,0,0];[0,0,0];[0,0,0]];
IG_underleg_loc = [[0,0,0];[0,0,0];[0,0,0]];
IG_foot_loc     = [[0,0,0];[0,0,0];[0,0,0]];


str_params = struct('hip_Comy',[],'lengthupperleg',[], 'lengthunderleg',[],'footankletotip',[],'footankletoside',[],'footankletoback',[],'ankle_height',[],'MassUpperbody',[],'MassUpperleg',[],'MassUnderleg',[],'MassFoot',[],'MassTot',[],'COG_UpperLeg',[],'COG_UnderLeg',[],'COG_UB',[],'COG_foot',[])
%% READING FROM FILE
fid = fopen('params_iCub.txt');
texto = textscan(fid, '%s %s %s %s', 'delimiter','\t')
params = [texto{1} texto{2} texto{3} texto{4}]
fclose(fid);
%%

for i=1:size(params,1)
    if(i<13)
        str_params.(genvarname(params{i,1})) = str2num(params{i,2});
    else
        str_params.(genvarname(params{i,1})) = [str2num(params{i,2});str2num(params{i,3});str2num(params{i,4})];
    end
end

display(str_params)

hip_Comy = str_params.hip_Comy;
lengthupperleg = str_params.lengthupperleg;
lengthunderleg = str_params.lengthunderleg;
footankletotip = str_params.footankletotip;
footankletoside = str_params.footankletoside;
footankletoback = str_params.footankletoback;
ankle_height = str_params.ankle_height;
MassUpperbody = str_params.MassUpperbody;
MassUpperleg = str_params.MassUpperleg;
MassFoot = str_params.MassFoot;
MassTot = str_params.MassTot;
COG_UpperLeg = str_params.COG_UpperLeg;
COG_UnderLeg = str_params.COG_UnderLeg;
COG_UB = str_params.COG_UB;
COG_foot = str_params.COG_foot;


        
    