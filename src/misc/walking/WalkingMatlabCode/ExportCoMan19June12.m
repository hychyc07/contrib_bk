% updated 21 Jan 2011
clc
clear ExportAngle ExportTime x xx xt xxt y yy transitionR transitionL transitionRend transitionLend
clear RightHRP LeftHRP blank ExportVelocity Angle qR_exp qL_exp

[row col]=size(qR2);
if row>col
    qR_exp=qR;
    qL_exp=qL;
else
    qR_exp=qR';
    qL_exp=qL';
end
clear row col

for i=1:6
    x = [0, 1]; % transition duration
    xt=min(x):T:max(x); 
    y= [qR_exp(1,i)  qR_exp(1,i)];    
    yy= [qR_exp(end,i) qR_exp(1,i)];
    
    transitionR(:,i) = spline(x, [0 y 0], xt);
    transitionRend(:,i) = spline(x, [0 yy 0], xt);
    
    y = [ qL_exp(1,i)  qL_exp(1,i)];    
    yy = [qL_exp(end,i)  qL_exp(1,i)];
    transitionL(:,i) = spline(x, [0 y 0], xt);
    transitionLend(:,i) = spline(x, [0 yy 0], xt);
end
% connect transition parts
RightHRP = [transitionR; qR_exp ; transitionRend];
LeftHRP = [transitionL; qL_exp ; transitionLend];
blank = zeros( length(RightHRP(:,1)),1 );
%% interpolate joint trajectory for Ccub
HRP_time = T*( 1:length(RightHRP(:,1)) )';
Ctrl_T = .001; % max control time step in OpenHRP
ExportTime = [HRP_time(1):Ctrl_T:HRP_time(end)]';
% position
Angle = [blank,blank,blank, -RightHRP(:,1), -LeftHRP(:,1), RightHRP(:,2), RightHRP(:,3),-RightHRP(:,4),-RightHRP(:,5),RightHRP(:,6), -LeftHRP(:,2),-LeftHRP(:,3),-LeftHRP(:,4),-LeftHRP(:,5),-LeftHRP(:,6)];
[row column] = size(Angle);
for i=1:column
    ExportAngle(:,i) = interp1(HRP_time, Angle(1:row,i), ExportTime);
end

% for i=1:column
%     ExportAngle(:,i)  = lowpass(20, ExportAngle(:,i), Ctrl_T);   
% end

% 
% ExportAngle = [blank,blank,blank, -AngleR(:,1), -AngleL(:,1), AngleR(:,2), AngleR(:,3),-AngleR(:,4),-AngleR(:,5),AngleR(:,6), -AngleL(:,2),-AngleL(:,3),-AngleL(:,4),-AngleL(:,5),-AngleL(:,6)];

clear row column
[row column] = size(ExportAngle);
% set joint angle limit

%       pitch limit        roll limit 26,13degree, roll outward is minus,
limit = [40/180*pi, 0.44;
            -40/180*pi, -0.23];

if max(ExportAngle(:,9))>limit(1,1)||max(ExportAngle(:,14))>limit(1,1)
    for i=1:row
        if ExportAngle(i,9)>limit(1,1); ExportAngle(i,9)=limit(1,1);    end
        if ExportAngle(i,14)>limit(1,1); ExportAngle(i,14)=limit(1,1);  end
    end
end

if min(ExportAngle(:,10))<limit(2,2)||min(ExportAngle(:,15))<limit(2,2)
    for i=1:row
        if ExportAngle(i,10)<limit(2,2); ExportAngle(i,10)=limit(2,2);    end
        if ExportAngle(i,15)<limit(2,2); ExportAngle(i,15)=limit(2,2);  end
    end
end

clear limit 

%%
save 'PreviewCoMan.txt' -ASCII -DOUBLE -TABS ExportAngle

fid=fopen('SpecificationPreview.txt','w');
fprintf(fid, '%6.0f\t%1.0f\n',row,Ctrl_T*1000);
fclose(fid);
%%
ExportVelocity = [zeros(1,column); (ExportAngle(2:end,:)-ExportAngle(1:end-1,:))/Ctrl_T];
%% plot 
figure(90);clf
title('Left Joint Trajectories: rad', 'Fontsize',15); hold on
plot(ExportTime,ExportAngle(:,5),ExportTime,ExportAngle(:,11),ExportTime,ExportAngle(:,12),ExportTime,ExportAngle(:,13),ExportTime,ExportAngle(:,14),ExportTime,ExportAngle(:,15));
legend('Left Hip Pitch','Left Hip Rotation','Left Hip Yaw','Left Knee','Left Ankle Pitch','Left Ankle Rotation');

figure(91);clf
title('Right Joint Trajectories: rad', 'Fontsize',15); hold on
plot(ExportTime,ExportAngle(:,4),ExportTime,ExportAngle(:,6),ExportTime,ExportAngle(:,7),ExportTime,ExportAngle(:,8),ExportTime,ExportAngle(:,9),ExportTime,ExportAngle(:,10));
legend('Right Hip Pitch','Right Hip Rotation','Right Hip Yaw','Right Knee','Right Ankle Pitch','Right Ankle Rotation');


figure(92);clf
title('Joint Velocity Trajectories: rad', 'Fontsize',15); hold on
plot(ExportTime,ExportVelocity(:,2),ExportTime,ExportVelocity(:,3),ExportTime,ExportVelocity(:,4),ExportTime,ExportVelocity(:,5),ExportTime,ExportVelocity(:,6),ExportTime,ExportVelocity(:,7));
legend('Hip Pitch','Hip Rotation','Hip Yaw','Knee','Ankle Pitch','Ankle Rotation');

