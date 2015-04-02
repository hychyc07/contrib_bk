close all;

figure(1); hold on;
title('Orbit angular momentum');
plot(Ltot(1,:),'r');
plot(Ltot(2,:),'g');
plot(Ltot(3,:),'b');

figure(2); hold on
title('Spin angular momentum');
plot(Htot(1,:),'r');
plot(Htot(2,:),'g');
plot(Htot(3,:),'b');

Mtot=Ltot+Htot;
figure(3); hold on
title('Total angular momentum');
plot(Mtot(1,:),'r');
plot(Mtot(2,:),'g');
plot(Mtot(3,:),'b');



figure(6); hold on
title('Phase');
grid on;
dMtot(1,:) = [0 diff(Mtot(1,:))];
dMtot(2,:) = [0 diff(Mtot(2,:))];
dMtot(3,:) = [0 diff(Mtot(3,:))];
    plot(Mtot(1,:),dMtot(1,:),'r');
    plot(Mtot(2,:),dMtot(2,:),'g');
    plot(Mtot(3,:),dMtot(3,:),'b');

close all
figure; hold on; grid on;
quiver (Mtot(1,500:1000), dMtot(1,500:1000), dMtot(1,500:1000), [0 diff(dMtot(1,500:1000))], 0.5); 
figure; hold on; grid on;
quiver (Mtot(2,500:1000), dMtot(2,500:1000), dMtot(2,500:1000), [0 diff(dMtot(2,500:1000))]); 
figure; hold on; grid on;
quiver (Mtot(3,500:1000), dMtot(3,500:1000), dMtot(3,500:1000), [0 diff(dMtot(3,500:1000))]); 


WaistTorque(1,:) = [ 0,diff(Mtot(1,:))];
WaistTorque(2,:) = [ 0,diff(Mtot(2,:))];
WaistTorque(3,:) = [ 0,diff(Mtot(3,:))];
% diff(Mtot(2,:));diff(Mtot(3,:))
figure(4); hold on
title('Total Waist Torque');
plot(WaistTorque(1,:),'r');
plot(WaistTorque(2,:),'g');
plot(WaistTorque(3,:),'b');
%%%%%%%%%%%%%%%%%
% I=[    0.1000         0         0
%          0    0.1000         0
%          0         0    0.1000];
I=IG_body_loc;
Wdd = I^(-1)*WaistTorque;

for i=1:length(Wdd(1,:));
    Wd(1,i) = sum(Wdd(1,i));
    Wd(2,i) = sum(Wdd(2,i));
    Wd(3,i) = sum(Wdd(3,i));
    W(1,i) = sum(Wd(1,i));
    W(2,i) = sum(Wd(2,i));
    W(3,i) = sum(Wd(3,i));
end
Wd = Wd*T;
W = W*T;
figure(5); hold on
title('Waist Angle');
plot(180/pi*W(1,:),'r');
plot(180/pi*W(2,:),'g');
plot(180/pi*W(3,:),'b');