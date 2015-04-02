close all;
figure(17);
hold on; grid on;
plot(time2(1:(TotalStepTime+Tprev)/T),zmpx(1:(TotalStepTime+Tprev)/T) );
plot(time2(1:(TotalStepTime+Tprev)/T),zmpx2(1:(TotalStepTime+Tprev)/T),'g');

plot(time2(1:(TotalStepTime+Tprev)/T),zmpx_multibody(1:(TotalStepTime+Tprev)/T),'b');
plot(time2(1:(TotalStepTime+Tprev)/T),COG_Global(1,1:(TotalStepTime+Tprev)/T),'g--');

plot(time2(1:(TotalStepTime+Tprev)/T),zmpx_multibody2(1:(TotalStepTime+Tprev)/T),'r');
plot(time2(1:(TotalStepTime+Tprev)/T),COG_Global2(1,1:(TotalStepTime+Tprev)/T),'k--');

xlabel('Time: s','FontSize',15);
ylabel('X: m','FontSize',15);
legend('Desired ZMP','ZMP2','ZMP Multi-Body','COM1','ZMP Multi-Body2','COM');
%%
figure(18);hold on; grid on;
plot(time2(1:(TotalStepTime+Tprev)/T),zmpy(1:(TotalStepTime+Tprev)/T) );
plot(time2(1:(TotalStepTime+Tprev)/T),zmpy2(1:(TotalStepTime+Tprev)/T),'g');

plot(time2(1:(TotalStepTime+Tprev)/T),zmpy_multibody(1:(TotalStepTime+Tprev)/T),'b');
plot(time2(1:(TotalStepTime+Tprev)/T),COG_Global2(2,1:(TotalStepTime+Tprev)/T),'g+');

plot(time2(1:(TotalStepTime+Tprev)/T),zmpy_multibody2(1:(TotalStepTime+Tprev)/T),'r');
plot(time2(1:(TotalStepTime+Tprev)/T),COG_Global2(2,1:(TotalStepTime+Tprev)/T),'k--');

xlabel('Time: s','FontSize',15);
ylabel('Y: m','FontSize',15);

legend('Desired ZMP','ZMP2','ZMP Multi-Body','COM1','ZMP Multi-Body2','COM2');