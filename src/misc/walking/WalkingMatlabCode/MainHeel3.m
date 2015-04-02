% programmer: Alex
% updated: June 15, 2010 
% updated: Feb 7 2013
clear all;close all;clc
disp('cCub walks and turns. Updated 15 March 2011');
disp('Double checked on 14 Nov 2012 to make sure it works');

%% set up basic variables
GlobalParameters;
% ModelParaNew;
ModelParaNew_iCub;

Time_startwalking = 3;            % GlobalPositionsFoot function needs it in TSS_start(1) = Time_startwalking; ORIGINALLY 2

global Tcycle Height steplength
Tcycle     = 12.0;                % @@@ walking cycle Originally 1.0
Height     = 0.03;                % @@@ foot lift height
step_width = 1.0;                 % percentage of step width from mid of pelvis
steplength = 0.05;                % @@@ 0.11; 0.35 Jorh: ORIGINAL WAS 0.1
% Time_startwalking should > Tprev
Tprev      = 2.0;                 % preview time window
T          = 0.01;                % sampling time
% T          = 1e-3;

GaitPlan1;                        % TotalStepTime is calculated in its subfunction GlobalPositionsFoot.m in TotalStepTime = TDS_end(i)
%%%%%
FootxStore = [XGlobalFootR, XGlobalFootL];
FootxStore = sort( unique(FootxStore) );
%%%%%
LocomotionTime1;
% Sub_SpringParameter;

%% preset up for preview controller
% [COM_multibody]=CalCOM1;
zmpPlanning5;
Sub_PCGain1;                                                                % gains for preview controller - TAKES A LOT OF TIME
Sub_VariablesInitial1;
% for PzGenerator2.m
Atilt=0.0;                                                                  % Atilt here is the peak to peak magnitude

%% 1st loop of preview control
Lx_old=0;
Ly_old=0;
Htot_old=0;
for n=1:tsize-N_L
    display(n)
    [WalkingPhase,StartWalkingPhase,EndWalkingPhase,stepnumber]=StateMachine3dot1(WalkingPhase,StartWalkingPhase,EndWalkingPhase,LeftFootMoving, time(n), stepnumber, TSS_start, TDS_start, TSS_end, TDS_end,  TUA_start, TUA_end,LambdaX);
	phase_store(n)=WalkingPhase;
    [xfl(n),yfl(n),zfl(n),rotY_fl(n),rotZ_fl(n),xfr(n),yfr(n),zfr(n),rotY_fr(n),rotZ_fr(n)]= TrajFeet3( WalkingPhase, StartWalkingPhase, time(n), stepnumber , TUA_end);
	
    ux = - GI * ezmpx -Gd1 * x - Gd*zmpx(n:n+N_L-1);
    uy = - GI * ezmpy -Gd1 * y - Gd*zmpy(n:n+N_L-1);    
    % here i should plugin the ux uy acceleration check, to limit az,
    % satisfy stability criteria, polygon
    x = sysd.A * x + sysd.B * ux;
    y = sysd.A * y + sysd.B * uy;
    
	% here plugin Virtual Force to pick up COM height
	PzGenerator2;
	
	Sub_1stLoopCal1;
	[zmpx_multibody(n),zmpy_multibody(n),Ltot(:,n), Htot(:,n), Htot_old,Lx_old,Ly_old]=calcMultiZMP(Body_R(:,:,n),qL(:,n),qR(:,n),qdL(:,n),qdR(:,n),qd_abs(:,n), COG(:,:,n),COGd(:,:,n), COGdd(:,:,n),COG_Global(:,n),COG_Globald(:,n), Lx_old, Ly_old, Htot_old);    
end

%% 2nd preview loop
% variables for delta ZMP for compensation
deltaZmpX=zmpx_multibody-zmpx;
deltaZmpY=zmpy_multibody-zmpy;

deltaezmpx = 0.0;
% deltaX = zeros(size(Ac,1),1);
deltaX=[0.0;0;0];
deltaezmpy = 0.0;
% deltaY = zeros(size(Ac,1),1);
deltaY=[0.0;0;0];

Lx_old2=0;
Ly_old2=0;
Htot_old2=0;
for n=1:tsize-2*N_L
    display(n)
    ux = - GI * deltaezmpx -Gd1 * deltaX - Gd*deltaZmpX(n:n+N_L-1);    %Ks is linked to Gi of the paper. I guess Ks is used in the original paper
    uy = - GI * deltaezmpy -Gd1 * deltaY - Gd*deltaZmpY(n:n+N_L-1);    %Ks is linked to Gi of the paper
    
    deltaX = sysd.A * deltaX + sysd.B * ux;
    deltaY = sysd.A * deltaY + sysd.B * uy;
    
%     record_deltaX(n) =deltaX(3);
%     if n*T < 0.5*Time_startwalking
%         if deltaX(3)> 0.05
%             deltaX(3) =  0.05;
%         elseif deltaX(3)< - 0.05
%             deltaX(3) =  -0.05;
%         end
%     end    
    
    deltaZmpX2(n) = (sysd.C * deltaX)';
    deltaZmpY2(n) = (sysd.C * deltaY)';
    deltaezmpx = (deltaZmpX2(n)-deltaZmpX(n));
    deltaezmpy = (deltaZmpY2(n)-deltaZmpY(n));
%     deltaezmpx = deltaezmpx + (deltaZmpX2(n)-deltaZmpX(n));
%     deltaezmpy = deltaezmpy + (deltaZmpY2(n)-deltaZmpY(n));
    
    deltax_h(n,:) = deltaX';
    x_h2(n,:)=x_h(n,:)-deltax_h(n,:);
    deltay_h(n,:) = deltaY';
    y_h2(n,:)=y_h(n,:)-deltay_h(n,:);

    xh2(n)=x_h2(n,1);
    xhd2(n)=x_h2(n,2);
    xhdd2(n)=x_h2(n,3);
    yh2(n)=y_h2(n,1);
    yhd2(n)=y_h2(n,2);
    yhdd2(n)=y_h2(n,3);

    Body_P2(:,n)=[xh2(n);yh2(n);zh(n)];
	
	% here Body_P2 L2 and so on indicate the parameters used in 2nd preview
	% control loop
    Dt=[0.0;hip_Comy;0];
    [qL2(:,n)]=inversekinematics(Body_P2(:,n),Body_R(:,:,n),Dt, Foot_PL(:,n), Foot_RL(:,:,n));
    Dt=[0.0;-hip_Comy;0];
    [qR2(:,n)]=inversekinematics(Body_P2(:,n),Body_R(:,:,n),Dt, Foot_PR(:,n), Foot_RR(:,:,n));

    [COG2(:,:,n),COG_Global2(:,n)]=calcCOG(Body_P2(:,n),Body_R(:,:,n), Foot_PL(:,n), Foot_RL(:,:,n),Foot_PR(:,n), Foot_RR(:,:,n),qL2(:,n),qR2(:,n));
   
    if (n>1)
            qdL2(:,n)=((qL2(:,n)-qL2(:,n-1))/T);
            qddL2(:,n)=((qdL2(:,n)-qdL2(:,n-1))/T);
            qdR2(:,n)=((qR2(:,n)-qR2(:,n-1))/T);
            qddR2(:,n)=((qdR2(:,n)-qdR2(:,n-1))/T);
            COGd2(:,1,n)=((COG2(:,1,n)-COG2(:,1,n-1))/T);
            COGd2(:,2,n)=((COG2(:,2,n)-COG2(:,2,n-1))/T);
            COGd2(:,3,n)=((COG2(:,3,n)-COG2(:,3,n-1))/T);
            COGd2(:,4,n)=((COG2(:,4,n)-COG2(:,4,n-1))/T);
            COGd2(:,5,n)=((COG2(:,5,n)-COG2(:,5,n-1))/T);
            COGd2(:,6,n)=((COG2(:,6,n)-COG2(:,6,n-1))/T);
            COGd2(:,7,n)=((COG2(:,7,n)-COG2(:,7,n-1))/T);
            %COGd(:,8,n)=((COG(:,8,n)-COG(:,8,n-1))/T);
            COG_Globald2(:,n)=((COG_Global2(:,n)-COG_Global2(:,n-1))/T);
            COGdd2(:,1,n)=((COGd2(:,1,n)-COGd2(:,1,n-1))/T);
            COGdd2(:,2,n)=((COGd2(:,2,n)-COGd2(:,2,n-1))/T);
            COGdd2(:,3,n)=((COGd2(:,3,n)-COGd2(:,3,n-1))/T);
            COGdd2(:,4,n)=((COGd2(:,4,n)-COGd2(:,4,n-1))/T);
            COGdd2(:,5,n)=((COGd2(:,5,n)-COGd2(:,5,n-1))/T);
            COGdd2(:,6,n)=((COGd2(:,6,n)-COGd2(:,6,n-1))/T);
            COGdd2(:,7,n)=((COGd2(:,7,n)-COGd2(:,7,n-1))/T);
            %COGdd(:,8,n)=((COGd(:,8,n)-COGd(:,8,n-1))/T);
            COG_Globaldd2(:,n)=((COG_Globald2(:,n)-COG_Globald2(:,n-1))/T);          
    else
            qdL2(:,1)=[0.0;0.0;0.0;0.0;0.0;0.0];
            qddL2(:,1)=[0.0;0.0;0.0;0.0;0.0;0.0];
            qdR2(:,1)=[0.0;0.0;0.0;0.0;0.0;0.0];
            qddR2(:,1)=[0.0;0.0;0.0;0.0;0.0;0.0];
            COGd2(:,:,1)=[[0.0,0.0,0.0,0.0,0.0,0.0,0.0];[0.0,0.0,0.0,0.0,0.0,0.0,0.0];[0.0,0.0,0.0,0.0,0.0,0.0,0.0]];
            COGdd2(:,:,1)=[[0.0,0.0,0.0,0.0,0.0,0.0,0.0];[0.0,0.0,0.0,0.0,0.0,0.0,0.0];[0.0,0.0,0.0,0.0,0.0,0.0,0.0]];
            COG_Globald2=[0.0;0.0;0.0];
            COG_Globaldd2=[0.0;0.0;0.0];
    end

    time3(n)=time(n);
	[zmpx_multibody2(n),zmpy_multibody2(n),Ltot2(:,n), Htot2(:,n), Htot_old2,Lx_old2,Ly_old2]=calcMultiZMP(Body_R(:,:,n),qL2(:,n),qR2(:,n),qdL2(:,n),qdR2(:,n),qd_abs(:,n), COG2(:,:,n),COGd2(:,:,n), COGdd2(:,:,n),COG_Global2(:,n),COG_Globald2(:,n), Lx_old2, Ly_old2, Htot_old2);
end
%% rest 2s ZMP remain the same
Sub_rest2sZMP1;
%%
qL2_tmp = qL2;
qR2_tmp = qR2;
PlotcCub2;