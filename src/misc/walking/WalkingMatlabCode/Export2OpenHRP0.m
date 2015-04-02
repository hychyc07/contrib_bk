% updated 12 April 2010
clc
clear transitionR transitionRend transitionL transitionLend RightHRP LeftHRP Rvel Lvel
%% make sure data is in column format
% chop off the beginning of joint data: 0.2s
checksize = size(qL2);
if checksize(1)<checksize(2) % row vector, so need transpose
	HRP_L = qL2(:,0.2/T:end-round(0.5*(Tprev/T)))';
else
    HRP_L = qL2(:,0.2/T:end-round(0.5*(Tprev/T)));
end
checksize = size(qR2);
if checksize(1)<checksize(2) % row vector, so need transpose
	HRP_R = qR2(:,0.2/T:end-round(0.5*(Tprev/T)))';
else
    HRP_R = qR2(:,0.2/T:end-round(0.5*(Tprev/T)));
end

% interpolation for transition phase
for i=1:6
    x = [0 3]; % transition duration
    xt=min(x) : T: max(x); 
    y= [ 0  HRP_R(1,i)];    
    yy= [HRP_R(end,i) 0];
    
    transitionR(:,i) = spline(x, [0 y 0], xt);
    transitionRend(:,i) = spline([0 1], [0 yy 0], [0:T: 1]);
    
    y = [ 0  HRP_L(1,i)];    
    yy = [HRP_L(end,i)  0];
    transitionL(:,i) = spline(x, [0 y 0], xt);
    transitionLend(:,i) = spline([0 1], [0 yy 0], [0:T: 1]);
end
% connect transition parts
RightHRP = [transitionR; HRP_R ; transitionRend];
LeftHRP = [transitionL; HRP_L ; transitionLend];

Rvel = [ zeros(1,6)	;	( RightHRP(2:end,:)-RightHRP(1:end-1,:) ) / T];
Lvel = [ zeros(1,6)	;	( LeftHRP(2:end,:)-LeftHRP(1:end-1,:) ) / T];

%%
blank = zeros( length(RightHRP(:,1)),1 );
HRP_time = T*( 1:length(RightHRP(:,1)) )';
Ctrl_T = .002; % max control time step in OpenHRP
% position
Angle = [HRP_time, RightHRP(:,1),RightHRP(:,2),RightHRP(:,3),RightHRP(:,4),RightHRP(:,5),RightHRP(:,6),blank,blank,blank,blank-0.5*pi,blank,blank,blank,LeftHRP(:,1),LeftHRP(:,2),LeftHRP(:,3),LeftHRP(:,4),LeftHRP(:,5),LeftHRP(:,6),blank,blank,blank,blank-0.5*pi,blank,blank,blank,blank,blank,blank];
save 'angle.dat' -ASCII -DOUBLE -TABS Angle
% velocity
Velocity = [HRP_time,Rvel(:,1),Rvel(:,2),Rvel(:,3),Rvel(:,4),Rvel(:,5),Rvel(:,6),blank,blank,blank,blank,blank,blank,blank,Lvel(:,1),Lvel(:,2),Lvel(:,3),Lvel(:,4),Lvel(:,5),Lvel(:,6),blank,blank,blank,blank,blank,blank,blank,blank,blank,blank];
save 'vel.dat' -ASCII -DOUBLE -TABS Velocity

%% plot 
figure(91);clf
title('Trajectories of Joint Angular Velocity: rad/s', 'Fontsize',15); hold on
plot(HRP_time,Rvel(:,1),HRP_time,Rvel(:,2),HRP_time,Rvel(:,3),HRP_time,Rvel(:,4),HRP_time,Rvel(:,5),HRP_time,Rvel(:,6));

figure(92);clf
title('Joint Trajectories: rad', 'Fontsize',15); hold on
plot(HRP_time,RightHRP(:,1),HRP_time,RightHRP(:,2),HRP_time,RightHRP(:,3),HRP_time,RightHRP(:,4),HRP_time,RightHRP(:,5),HRP_time,RightHRP(:,6));
legend('Hip Pitch','Hip Rotation','Hip Yaw','Knee','Ankle Pitch','Ankle Rotation');

figure(93);clf
subplot(2,1,1); title('Knee angle'); hold on;
plot(HRP_time,RightHRP(:,4),'r');grid on;
% axis([1 25 0 .8]);
subplot(2,1,2); title('Knee angluar velocity'); hold on;
plot(HRP_time,Rvel(:,4));grid on;