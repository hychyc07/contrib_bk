figure(33);
clf;

time2=1:length(phase_store);
time2=time2*T;

%% COM side view
subplot(2,1,1)

COMx=COG_Global(1,1:4.8/T);
COMy=COG_Global(2,1:4.8/T);
COMz=COG_Global(3,1:4.8/T);
plot3(COMx,COMy,COMz,'r','LineWidth',1.5);hold on
plot3(zmpx(1:4.8/T,1),zmpy(1:4.8/T,1),zeros(length(COMx),1)+.485,'g','LineWidth',1.5);hold on
plot3(COMx,COMy,zeros(length(COMx),1)+.485,'b','LineWidth',1.5);hold on

xlabel('X: m','FontSize',15);
ylabel('Y: m','FontSize',15);
zlabel('Z: m','FontSize',15);
% axis equal; 
axis([-.01 .9 -.08 .05 .485 .49]);
grid on; hold on
title('COM and ZMP Trajectory','FontSize',15);hold on
%% COM front view
subplot(2,1,2)
plot(COG_Global(2,3/T:4/T),COG_Global(3,3/T:4/T),'b','LineWidth',1.5);hold on
axis equal; 
grid on
% axis([0 .8 -.03  .03 .45 .47 ]);
xlabel('Y: m','FontSize',15);
ylabel('Z : m ','FontSize',15);
% zlabel('Z : m ','FontSize',15);
title('Front View of COM Trajectory','FontSize',15);hold on






