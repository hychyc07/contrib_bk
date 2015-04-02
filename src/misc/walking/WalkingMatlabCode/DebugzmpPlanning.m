% programmer: Alex
% code written: 
clear all;close all;clc

%% set up basic variables
GlobalParameters;

modelparametersV1;

Time_startwalking = 2.0;%GlobalPositionsFoot function needs it in TSS_start(1) = Time_startwalking;
Tprev = 2.0; 
T = 0.005;

GaitPlan1; %TotalStepTime is calculated in its subfunction GlobalPositionsFoot.m in TotalStepTime = TDS_end(i)

LocomotionTime1;
zmpPlanning5;

close all
figure; hold on
plot(zmpx,zmpy)
plot(zmpx,zmpy,'r.')

figure; 
subplot(2,1,1);hold on
plot(zmpx,''); 
plot(zmpx,'.'); grid on
subplot(2,1,2);hold on
plot(zmpy,'r'); 
plot(zmpy,'r.'); 
grid on