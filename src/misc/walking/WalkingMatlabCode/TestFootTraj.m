% programmer: Alex
% I rewrite this code on Jan 5, 2010 
% updated: 11 Jan
% updated: 4 Feb
% Main2 -> az sine wave profile, plug in Sub_VirtualForce1

clear all;close all;clc
'Main2.m'
%% set up basic variables
GlobalParameters;

modelparametersV1;

GaitPlan1; %TotalStepTime is calculated in its subfunction GlobalPositionsFoot.m in TotalStepTime = TDS_end(i)

LocomotionTime1;

% Sub_SpringParameter;

%% preset up for preview controller
Sub_PCGain1;  % gains for preview controller
Sub_VariablesInitial1;

%% 1st loop of preview control
for n=1:tsize-N_L
%     if time(n)>4; break; end
    [WalkingPhase,StartWalkingPhase,EndWalkingPhase,stepnumber]=StateMachine(WalkingPhase,StartWalkingPhase,EndWalkingPhase,LeftFootMoving, time(n), stepnumber, TSS_start, TDS_start, TSS_end, TDS_end,  TUA_start, TUA_end,LambdaX);

    phase_store(n) =WalkingPhase;
    [xfl(n),yfl(n),zfl(n),rotY_fl(n),rotZ_fl(n),xfr(n),yfr(n),zfr(n),rotY_fr(n),rotZ_fr(n)]= TrajFeet1( WalkingPhase, StartWalkingPhase, time(n), stepnumber);

end

%% 2nd preview loop

%%
beep;  % program ends
figure
% plot3(zmpx,zmpy,zmpz,zmpx_multibody,zmpy_multibody,zmpz,zmpx_multibody2,zmpy_multibody2,zmpz,xh,yh,zh,xh2,yh2,zh,xfl,yfl,zfl,xfr,yfr,zfr,COG_Global(1,:),COG_Global(2,:),COG_Global(3,:))
plot3(zmpx,zmpy,zmpz,xfl,yfl,zfl,xfr,yfr,zfr), legend ('ZMP_{des}','ZMP_{multibody}','Hip','Left foot', 'right foot')
%plot3(zmpx,zmpy,zmpz,xh2,yh2,zh,xfl,yfl,zfl,xfr,yfr,zfr), legend ('ZMP_{des}','ZMP_{multibody}','Hip','Left foot', 'right foot')
plotfootholds;

DebugStateMachineCode;