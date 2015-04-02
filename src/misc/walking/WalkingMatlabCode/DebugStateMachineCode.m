close all
%%
figure(1)
title('Check walking phase and Time Planning');
plot(TSS_start,ones(1,length(TSS_start)),'r*'); grid on; hold on;
plot(TSS_end,ones(1,length(TSS_end)),'b*'); grid on; hold on;

plot(TUA_start,3*ones(1,length(TUA_start)),'m.');hold on;
plot(TUA_end,3*ones(1,length(TUA_end)),'k.');

plot( T*(1:length(phase_store)) ,phase_store,'g');

legend('TSS start','TSS end','TDS start','TDS end', 'Phase');
%%
figure(2)
title('walking phase and zmp');
subplot(2,1,1);
plot( T*(1:length(phase_store)) ,0.1*phase_store,'g');
hold on;
plot( T*(1:length(zmpx)) ,zmpx,'r');
plot( T*(1:length(zmpx2)) ,zmpx2);
plot( T*(1:length(xh) ) ,xh, 'm');
grid on

subplot(2,1,2);
plot( T*(1:length(phase_store)) ,0.1*phase_store,'g');
hold on;
plot( T*(1:length(zmpy)) ,zmpy,'r'); hold on
plot( T*(1:length(zmpy2)) ,zmpy2);
plot( T*(1:length(yh) ),yh(), 'm');
grid on
%%
figure(3); hold on
plot(xh,yh,'r');
plot(xh2,yh2);
%%
figure(4)
title('Footprint');
plot( XGlobalFootL, YGlobalFootL,'*');
hold on;
plot( XGlobalFootR, YGlobalFootR,'r*');
grid on; axis equal;