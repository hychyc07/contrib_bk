figure(10); 
subplot(2,1,1);hold on;
plot(xfr,'r');plot(xfl);
legend('right','left');

subplot(2,1,2);hold on;
plot(yfr,'r');plot(yfl);
legend('right','left');

%%
figure(11); hold on; axis equal; view(1,0)
subplot(2,1,1);hold on
plot3(xfr,yfr,zfr,'g',xfl,yfl,zfl,'r'); 
subplot(2,1,2);hold
plot(T*(1:length(phase_store)),phase_store,'g');
    
for m=1/T: TotalStepTime/T
    subplot(2,1,1);
    plot3(xfr(m),yfr(m),zfr(m),'r.',xfl(m),yfl(m),zfl(m),'b.');
    subplot(2,1,2);
    plot(time(m),phase_store(m),'b*');
    pause(0.01);
end