% programmer: Alex
% updated: 9 June, 2010 

clear all;close all;clc
'Debug Foot Trajectory'
%% set up basic variables
GlobalParameters;

modelparametersV1;

Time_startwalking = 2; %GlobalPositionsFoot function needs it in TSS_start(1) = Time_startwalking;
% Time_startwalking should > Tprev
Tprev = 2.0; 
T = 0.01;

GaitPlan1; %TotalStepTime is calculated in its subfunction GlobalPositionsFoot.m in TotalStepTime = TDS_end(i)
%%%%%
% FootxStore = [XGlobalFootR, XGlobalFootL];
% FootxStore = sort( unique(FootxStore) );
%%%%%
LocomotionTime1;
% Sub_SpringParameter;

%% preset up for preview controller
% [COM_multibody]=CalCOM1;
% zmpPlanning5;
% Sub_PCGain1;  % gains for preview controller
% Sub_VariablesInitial1;
% for PzGenerator2.m
% Atilt=0.03; % Atilt here is the peak to peak magnitude
%% 1st loop of preview control
for n=1:tsize-N_L

%     [WalkingPhase,StartWalkingPhase,EndWalkingPhase,stepnumber]=CheckWalkingPhase(WalkingPhase,StartWalkingPhase,EndWalkingPhase,LeftFootMoving, time(n), stepnumber, TSS_start, TDS_start, TSS_end, TDS_end,  TUA_start, TUA_end,LambdaX);
%     [xfl(n),yfl(n),zfl(n),rotY_fl(n),rotZ_fl(n),xfr(n),yfr(n),zfr(n),rotY_fr(n),rotZ_fr(n)]= TrajFeet( WalkingPhase, StartWalkingPhase, time(n), stepnumber);

    [WalkingPhase,StartWalkingPhase,EndWalkingPhase,stepnumber]=StateMachine3dot1(WalkingPhase,StartWalkingPhase,EndWalkingPhase,LeftFootMoving, time(n), stepnumber, TSS_start, TDS_start, TSS_end, TDS_end,  TUA_start, TUA_end,LambdaX);
	phase_store(n)=WalkingPhase;
    [xfl(n),yfl(n),zfl(n),rotY_fl(n),rotZ_fl(n),xfr(n),yfr(n),zfr(n),rotY_fr(n),rotZ_fr(n)]= TrajFeet3( WalkingPhase, StartWalkingPhase, time(n), stepnumber , TUA_end);
% 	
%     ux = - GI * ezmpx -Gd1 * x - Gd*zmpx(n:n+N_L-1);
%     uy = - GI * ezmpy -Gd1 * y - Gd*zmpy(n:n+N_L-1);    
%     % here i should plugin the ux uy acceleration check, to limit az,
%     % satisfy stability criteria, polygon
%     x = sysd.A * x + sysd.B * ux;
%     y = sysd.A * y + sysd.B * uy;
%     
% 	% here plugin Virtual Force to pick up COM height
% 	PzGenerator2;
% 	
% 	Sub_1stLoopCal1;
%  
%     [zmpx_multibody(n),zmpy_multibody(n),Hdotg_abs_footL(:,n),Hdotg_abs_footR(:,n),qddL_link(:,n),qddR_link(:,n),qdL_link_old,qdR_link_old]=calcZMP1(Body_R(:,:,n),qL(:,n),qR(:,n),qdL(:,n),qdR(:,n),qddL(:,n),qddR(:,n),qd_abs(:,n),qdd_abs(:,n),COG(:,:,n),COGdd(:,:,n),qdL_link_old,qdR_link_old); 
    
end

%% 2nd preview loop
% variables for delta ZMP for compensation
% deltaZmpX=zmpx_multibody-zmpx;
% deltaZmpY=zmpy_multibody-zmpy;
% 
% deltaezmpx = 0.0;
% deltaX = zeros(size(Ac,1),1);
% deltaX=[0.0;0;0];
% deltaezmpy = 0.0;
% deltaY = zeros(size(Ac,1),1);
% deltaY=[0.0;0;0];

%%
beep;  % program ends
% PlotMain2;
% DebugAnalyzeZMP;
% plotrobotFast
figure; hold on;
plotfootholds
plot3(xfl,yfl,zfl,xfr,yfr,zfr); 
