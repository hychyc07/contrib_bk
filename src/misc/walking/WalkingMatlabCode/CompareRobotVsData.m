figure, hold on;
cd /home/jorhabib/iCub/contrib/src/misc/walking/rightLegJnts_00001;
anglesReal = load('data.log');
anglesReal(:,2) = anglesReal(:,2) - min(anglesReal(:,2));
plot(anglesReal(:,2),anglesReal(:,3));
title('ROBOT REAL JNT ANGLES');

figure;
plot(qR2(1,:))
title('COMMANDED JNT ANGLES');