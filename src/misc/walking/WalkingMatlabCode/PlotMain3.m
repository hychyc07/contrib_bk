figure(1);
clf;
GRF=lowpass(20,COG_Globaldd2(3,:));
GRF=GRF+G;
time=1:length(GRF);
time=time*T;
time2=1:length(phase_store);
time2=time2*T;

% subplot(2,1,1)
title('COG Az'); hold on;
plot(time,(COG_Globaldd2(3,:)+G)/G,'b','LineWidth',1);
hold on; 
plot(time2,.3*(phase_store),'r');grid on;

% subplot(2,1,2)
% plot(time,GRF,'r');grid on;
axis([2.5 6 0.4 1.5]);
%%
figure(3);clf;
title('Hip Height');hold on;
plot(x_h(1:length(z_h(:,1)),1),z_h(:,1),'b');
xlabel('x/m'); ylabel('z/m');

%%
figure(6);clf;
% subplot(2,1,1);title('New Spring Leg Model'); hold on;
% plot(time3,qRiCub(1,:),'r-');legend('Right Hip');grid on;
subplot(2,1,1);
% plot(time3,-qLiCub(4,:),'b-');legend('Left Knee');grid on;
plot(-qLiCub(4,:),'b-');legend('Left Knee');grid on;hold on
plot(10*phase_store,'r');
subplot(2,1,2);
% plot(time3,-qRiCub(4,:),'b-');legend('Right Knee');grid on;
plot(-qRiCub(4,:),'b-');legend('Right Knee');grid on;hold on
plot(10*phase_store,'r');

figure(7);clf;
plot(-qLiCub(4,:),'b-');hold on
plot(-qRiCub(4,:),'r-');
plot(10*phase_store,'g');
legend('Left Knee','Right Knee','Phase');grid on;hold on