figure(33);
clf;

time2=1:length(phase_store);
time2=time2*T;


%% COM side view
subplot(2,2,1)
% ttt=1:length( COG_Global(3,:));
% ttt=ttt*T;
% plot(ttt, COG_Global(3,:)/(lengthupperleg + lengthunderleg + 0.06),'b','LineWidth',1.5);
% hold on
% % plot(time2,(phase_store-1),'r');grid on;
% grid on;
% % axis equal; 
% % axis([3 4 .96 1.00]);
% xlabel('Time: s','FontSize',15);
% ylabel('Normalized COM Height','FontSize',15);
% plot(COG_Global(1,3/T:4/T), COG_Global(3,3/T:4/T),'b','LineWidth',1.5);
COMx=COG_Global(1,1:5/T);
COMy=COG_Global(2,1:5/T);
COMz=COG_Global(3,1:5/T);
plot3(COMx,COMy,COMz,'r','LineWidth',1.5);hold on
plot3(COMx,COMy,zeros(length(COMx),1)+.48,'b','LineWidth',1);hold on
plot3(zmpx(1:5/T,1),zmpy(1:5/T,1),zeros(length(COMx),1)+.48,'g','LineWidth',1);hold on
grid on;
xlabel('X: m','FontSize',15);
ylabel('Y: m','FontSize',15);
zlabel('Z: m','FontSize',15);
% axis equal; 
axis([0 .85 -.072 .045 .482 .495]);hold on
%% COM front view
subplot(2,2,2)
plot(COG_Global(2,3/T:4/T),COG_Global(3,3/T:4/T),'b','LineWidth',1);hold on
axis equal; 
grid on
% axis([0 .8 -.03  .03 .45 .47 ]);
xlabel('Y: m','FontSize',15);
ylabel('Z : m ','FontSize',15);
% zlabel('Z : m ','FontSize',15);
%% Vz
subplot(2,2,3)
plot(time2,COG_Globald(3,:),'b','LineWidth',1);hold on
plot(time2,2*(phase_store-1)-1,'r');grid on;
axis([3 4 -.05 .05]);
xlabel('Time: s','FontSize',15);
ylabel('Vz : m/s ','FontSize',15);
%% Vx
subplot(2,2,4)
plot(time2,COG_Globald(1,:),'b','LineWidth',1);hold on

plot(time2,2*(phase_store-1),'r');grid on;

xlabel('Time: s','FontSize',15);
% ylabel('Vx / Vavg','FontSize',15);axis([4 5 .85 1.2]);
ylabel('Vx : m/s','FontSize',15);
axis([3 4 .22 .38]);






