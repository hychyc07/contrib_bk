%%%%%%%%%%% gait locomotion parameters %%%%%%%%%%%%%%%%%%%%%%%%%
hip_height= 0.96*(lengthupperleg+lengthunderleg);
% z_c used in Preview controller
% z_c  = 0.53; % @@@ FOR COMMAN IN ORIGINAL FILE
z_c = 0.48; % JORH: FOR ICUB - Obtained from wholeBodyDynamics/com_foot:o
 
% Qtoe  = 15;     % degree
% Qheel = 10;     % degree
% Qtoe  = 20;     % degree
% Qheel = 10;     % degree
Qtoe  = 0;        % degree
Qheel = 0;        % degree
global PercentToe % percent of toe off angle at 1st step
PercentToe = .5;

% used in inverse kinematics
PatternOffset = 0.0;
    
[LambdaX,LambdaY,alpha_foot,Nu,Lift,wait, LeftFootMoving]=GaitParameter1;

[XGlobalFootL, YGlobalFootL, AlpaGlobalFootL, XGlobalFootR,...
    YGlobalFootR, AlpaGlobalFootR, TSS_start, TDS_start, ...
    TSS_end, TDS_end,  TUA_start, TUA_end] = GlobalPositionsFoot1(LambdaX, ...
    LambdaY,alpha_foot,Nu,Lift,wait, LeftFootMoving);