%Updated 1st July 2010
%Updated 13 Sept 2010
%double checked on 7 Feb 2013
close all; clc;
figure; view(-30,70);

qL2 = qL2_tmp;
qR2 = qR2_tmp;

global hip_Comy lengthupperleg lengthunderleg footankletotip footankletoside footankletoback PI
global T Time_startwalking TotalStepTime
Lfront=footankletotip;
Lback=footankletoback;
Lside=footankletoside;
% view(0,0);
[row col]=size(qR2);
if row>col
    qR_plot=qR2;
    qL_plot=qL2;
else
    qR_plot=qR2';
    qL_plot=qL2';
end
clear row col
% pause(2)
q0_r = [];
q1_r = [];
q2_r = [];
q3_r = [];
q4_r = [];
q5_r = [];

q0_l = [];
q1_l = [];
q2_l = [];
q3_l = [];
q4_l = [];
q5_l = [];


tic
for n= 1:10:floor( (TotalStepTime)/T)% 0.2/T
    hold off;
    newplot;
    
%     plot3(zmpx,zmpy,zmpz,zmpx_multibody,zmpy_multibody,zmpz,zmpx_multibody2,zmpy_multibody2,zmpz,xh,yh,zh,xh2,yh2,zh,COG_Global(1,:),COG_Global(2,:),COG_Global(3,:))
    plot3(zmpx,zmpy,zmpz,'k')
%     plot3(zmpx_multibody,zmpy_multibody,zmpz,'r')
    plot3(zmpx_multibody2,zmpy_multibody2,zmpz,'r')
%     plot3(xh,yh,zh)
    plot3(xh2,yh2,zh)
%     plot3(COG_Global(1,:),COG_Global(2,:),COG_Global(3,:))
    plot3(COG_Global2(1,:),COG_Global2(2,:),COG_Global2(3,:))
    hold on;
    plot3(Foot_PL(1,:),Foot_PL(2,:),Foot_PL(3,:),Foot_PR(1,:),Foot_PR(2,:),Foot_PR(3,:));
    
    % include ankle joint to foot
    linkanklefootLegL=line('erase','normal');
    linkanklefootLegR=line('erase','normal');
    
    
    linkupperbody=line('erase','normal');
    linkhip      =line('erase','normal');
    linkupperlegL=line('erase','normal');
    linklowerlegL=line('erase','normal');
    linkupperlegR=line('erase','normal');
    linklowerlegR=line('erase','normal');
    
    linkfootfL=line('erase','normal');
    linkfootrL=line('erase','normal');
    linkfootlL=line('erase','normal');
    linkfootbL=line('erase','normal');
    
    linkfootfR=line('erase','normal');
    linkfootrR=line('erase','normal');
    linkfootlR=line('erase','normal');
    linkfootbR=line('erase','normal');
    
    % ZMPDes=line('erase','normal');
    % ZMPmulti1=line('erase','normal');
    % ZMPmulti2=line('erase','normal');
    ZMPDes=line('erase','normal');
    ZMPreal=line('erase','normal');
    
    set(linkupperbody,'Marker','.','MarkerEdgeColor',[240 150 10]/255,'Color',[240 150 10]/255,'Linewidth',[3],'MarkerSize',[40]);
    set(linkhip,'Marker','.','MarkerEdgeColor',[90 100 110]/255,'Color',[240 150 10]/255,'Linewidth',[3],'MarkerSize',[20]);
    set(linkupperlegL,'Marker','.','MarkerEdgeColor',[90 0 110]/255,'Color',[4 118 246]/255,'Linewidth',[3],'MarkerSize',[20]);
    set(linklowerlegL,'Marker','.','MarkerEdgeColor',[90 0 110]/255,'Color',[4 118 246]/255,'Linewidth',[3],'MarkerSize',[20]);
    set(linkupperlegR,'Marker','.','MarkerEdgeColor',[90 0 110]/255,'Color',[250 50 10]/255,'Linewidth',[3],'MarkerSize',[20]);
    set(linklowerlegR,'Marker','.','MarkerEdgeColor',[90 0 110]/255,'Color',[250 50 10]/255,'Linewidth',[3],'MarkerSize',[20]);
    
    set(linkfootfL,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color',[4 118 246]/255,'Linewidth',[2],'MarkerSize',[6]);
    set(linkfootrL,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color',[4 118 246]/255,'Linewidth',[2],'MarkerSize',[6]);
    set(linkfootlL,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color',[4 118 246]/255,'Linewidth',[2],'MarkerSize',[6]);
    set(linkfootbL,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color',[4 118 246]/255,'Linewidth',[2],'MarkerSize',[6]);
    
    set(linkfootfR,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color',[250 50 10]/255,'Linewidth',[2],'MarkerSize',[6]);
    set(linkfootrR,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color',[250 50 10]/255,'Linewidth',[2],'MarkerSize',[6]);
    set(linkfootlR,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color',[250 50 10]/255,'Linewidth',[2],'MarkerSize',[6]);
    set(linkfootbR,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color',[250 50 10]/255,'Linewidth',[2],'MarkerSize',[6]);
    
    % set(ZMPDes,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','b','Linewidth',[2],'MarkerSize',[6]);
    % set(ZMPmulti1,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','g','Linewidth',[2],'MarkerSize',[6]);
    % set(ZMPmulti2,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','r','Linewidth',[2],'MarkerSize',[6]);
    
    set(ZMPDes,'XData',[zmpx(n);zmpx(n)],'YData',[zmpy(n);zmpy(n)],'ZData',[0;-0.02]);
    set(ZMPreal,'XData',[zmpx_multibody2(n);zmpx_multibody2(n)],'YData',[zmpy_multibody2(n);zmpy_multibody2(n)],'ZData',[0;-0.04]);
    
    
    % include ankle joint to foot
    set(linkanklefootLegL,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color',[4 118 246]/255,'Linewidth',[3],'MarkerSize',[6]);
    set(linkanklefootLegR,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color',[250 50 10]/255,'Linewidth',[3],'MarkerSize',[6]);
    
    
    axis('equal');
    % axis([-.1 max(comx)+0.2 -0.2 0.2 0 0.6]); grid on;
    axis([-.1 max(zmpx)+.2 -0.12 0.12]);
    view(90,0);
%     view(0,0);
%     view(3)
    
    %%%%%%%%%%%%%%% here plot the walking robot%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    body_head=Body_P2(:,n)+Body_R(:,:,n)* [0;0;0.1];
    
    hip_left = Body_P2(:,n)+Body_R(:,:,n)* [0.0;hip_Comy;0];
    
    c1=cos(qL_plot(n,1)); %Y
    s1=sin(qL_plot(n,1));
    
    c2=cos(qL_plot(n, 2)); %X
    s2=sin(qL_plot(n,2));
    
    c3=cos(qL_plot(n,3)); %Z
    s3=sin(qL_plot(n,3));
    
    Rupperleg(1,1) = c1*c3;               Rupperleg(1,2) = -c1*s3+s1*s2*c3;     Rupperleg(1,3) = s1*c2;
    Rupperleg(2,1) = c2*s3;               Rupperleg(2,2) = c2*c3;               Rupperleg(2,3) = -s2;
    Rupperleg(3,1) = -s1*c3+c1*s2*s3;     Rupperleg(3,2) = s1*s3+c1*s2*c3;      Rupperleg(3,3) = c1*c2;
    
    knee_left = hip_left + Body_R(:,:,n)*Rupperleg*[0;0;-lengthupperleg];
    
    c4 = cos(qL_plot(n,4)); %Y
    s4 = sin(qL_plot(n,4));
    
    Runderleg(1,1) = c4;     Runderleg(1,2) = 0;   Runderleg(1,3) = s4;
    Runderleg(2,1) = 0.0;    Runderleg(2,2) = 1;   Runderleg(2,3) = 0;
    Runderleg(3,1) = -s4;    Runderleg(3,2) = 0;   Runderleg(3,3) = c4;
    
    ankle_left = knee_left + Body_R(:,:,n)*Rupperleg*Runderleg*[0;0;-lengthunderleg];
    
    c5 = cos(qL_plot(n,5)); %Y
    s5 = sin(qL_plot(n,5));
    c6 = cos(qL_plot(n,6)); %X
    s6 = sin(qL_plot(n,6));
    
    Rfoot(1,1) = c5;    Rfoot(1,2) = s5*s6;     Rfoot(1,3) = s5*c6;
    Rfoot(2,1) = 0;     Rfoot(2,2) = c6;        Rfoot(2,3) = -s6;
    Rfoot(3,1) = -s5;   Rfoot(3,2) = c5*s6;     Rfoot(3,3) = c6*c5;
    
    footfl_left = ankle_left + Body_R(:,:,n)*Rupperleg*Runderleg*Rfoot*[footankletotip;footankletoside; -ankle_height];
    footfr_left = ankle_left + Body_R(:,:,n)*Rupperleg*Runderleg*Rfoot*[footankletotip;-footankletoside;-ankle_height];
    footbl_left = ankle_left + Body_R(:,:,n)*Rupperleg*Runderleg*Rfoot*[-footankletoback;footankletoside;-ankle_height];
    footbr_left = ankle_left + Body_R(:,:,n)*Rupperleg*Runderleg*Rfoot*[-footankletoback;-footankletoside;-ankle_height];
    
    hip_right = Body_P2(:,n) +  Body_R(:,:,n) * [0.0;-hip_Comy;0];
    
    c1=cos(qR_plot(n,1)); %Y
    s1=sin(qR_plot(n,1));
    
    c2=cos(qR_plot(n,2)); %X
    s2=sin(qR_plot(n,2));
    
    c3=cos(qR_plot(n,3)); %Z
    s3=sin(qR_plot(n,3));
    
    Rupperleg(1,1) = c1*c3;               Rupperleg(1,2) = -c1*s3+s1*s2*c3;     Rupperleg(1,3) = s1*c2;
    Rupperleg(2,1) = c2*s3;               Rupperleg(2,2) = c2*c3;               Rupperleg(2,3) = -s2;
    Rupperleg(3,1) = -s1*c3+c1*s2*s3;     Rupperleg(3,2) = s1*s3+c1*s2*c3;      Rupperleg(3,3) = c1*c2;
    
    knee_right= hip_right + Body_R(:,:,n)*Rupperleg*[0;0;-lengthupperleg];
    
    c4 = cos(qR_plot(n,4)); %Y
    s4 = sin(qR_plot(n,4));
    
    Runderleg(1,1) = c4;     Runderleg(1,2) = 0;   Runderleg(1,3) = s4;
    Runderleg(2,1) = 0.0;    Runderleg(2,2) = 1;   Runderleg(2,3) = 0;
    Runderleg(3,1) = -s4;    Runderleg(3,2) = 0;   Runderleg(3,3) = c4;
    
    ankle_right = knee_right + Body_R(:,:,n)*Rupperleg*Runderleg*[0;0;-lengthunderleg];
    
    c5 = cos(qR_plot(n,5)); %Y
    s5 = sin(qR_plot(n,5));
    c6 = cos(qR_plot(n,6)); %X
    s6 = sin(qR_plot(n,6));
    
    Rfoot(1,1) = c5;    Rfoot(1,2) = s5*s6;     Rfoot(1,3) = s5*c6;
    Rfoot(2,1) = 0;     Rfoot(2,2) = c6;        Rfoot(2,3) = -s6;
    Rfoot(3,1) = -s5;   Rfoot(3,2) = c5*s6;     Rfoot(3,3) = c6*c5;
    
    footfl_right = ankle_right + Body_R(:,:,n)*Rupperleg*Runderleg*Rfoot*[footankletotip;footankletoside;-ankle_height];
    footfr_right = ankle_right + Body_R(:,:,n)*Rupperleg*Runderleg*Rfoot*[footankletotip;-footankletoside;-ankle_height];
    footbl_right = ankle_right + Body_R(:,:,n)*Rupperleg*Runderleg*Rfoot*[-footankletoback;footankletoside;-ankle_height];
    footbr_right = ankle_right + Body_R(:,:,n)*Rupperleg*Runderleg*Rfoot*[-footankletoback;-footankletoside;-ankle_height];
    
    %update linepositions
    offset = 0.03;
    
    set(linkupperbody,'XData',[Body_P2(1,n);body_head(1)],'YData',[Body_P2(2,n);body_head(2)],'ZData',[Body_P2(3,n);body_head(3)]);
    set(linkhip,'XData',[hip_left(1);hip_right(1)],'YData',[hip_left(2);hip_right(2)],'ZData',[hip_left(3);hip_right(3)]);
    set(linkupperlegL,'XData',[hip_left(1);knee_left(1)],'YData',[hip_left(2);knee_left(2)],'ZData',[hip_left(3);knee_left(3)]);
    
    
    q0_r = [q0_r; angle_jnt([hip_right(1:2); hip_right(3)-0.05], hip_right, knee_right,'sag')];
    text(hip_right(1)+offset,hip_right(2)+offset,hip_right(3),strcat('q0: ', num2str(q0_r(end))));
    
    %         q1_r = [q1_r; angle_jnt([hip_right(1:2); hip_right(3)-0.05], hip_right, knee_right,'front')];
    %         text(hip_right(1)+offset,hip_right(2)+offset,hip_right(3)-2*offset,strcat('q1: ',num2str(q1_r(end))));
    
    q1_r = [q1_r; angle_jnt([hip_right(1); hip_right(2)+0.05; hip_right(3)], hip_right, knee_right,'front')];
    q1_r(end) = q1_r(end)-90;
    text(hip_right(1)+offset,hip_right(2)+offset,hip_right(3)-2*offset,strcat('q1: ',num2str(q1_r(end))));
    
    q0_l = [q0_l; angle_jnt([hip_left(1:2); hip_left(3)-0.05], hip_left, knee_left,'sag')];
    text(hip_left(1)+offset,hip_left(2)+offset,hip_left(3),strcat('q0: ', num2str(q0_l(end))));
    
    %         q1_l = [q1_l; angle_jnt([hip_left(1:2); hip_left(3)-0.05], hip_left, knee_left,'front')];
    %         text(hip_left(1)+offset,hip_left(2)+offset,hip_left(3)-2*offset,strcat('q1: ',num2str(q1_l(end))));
    %
    q1_l = [q1_l; angle_jnt([hip_left(1); hip_left(2)-0.05; hip_left(3)], hip_left, knee_left,'front')];
    q1_l(end) = q1_l(end) - 90;
    text(hip_left(1)+offset,hip_left(2)+offset,hip_left(3)-2*offset,strcat('q1: ',num2str(q1_l(end))));
    
    
    set(linklowerlegL,'XData',[knee_left(1);ankle_left(1)],'YData',[knee_left(2);ankle_left(2)],'ZData',[knee_left(3);ankle_left(3)]);
    set(linkupperlegR,'XData',[hip_right(1);knee_right(1)],'YData',[hip_right(2);knee_right(2)],'ZData',[hip_right(3);knee_right(3)]);
    
    q3_r = [q3_r; angle_jnt(hip_right, knee_right, ankle_right,'sag')];
    q3_r(end) = -1*(180-q3_r(end));
    text(knee_right(1)+offset,knee_right(2)+offset,knee_right(3),num2str(q3_r(end)));
    
    q4_r = [q4_r; angle_jnt([ankle_right(1:2); ankle_right(3)+0.05], ankle_right, knee_right,'sag')];
    q4_r = -1*abs(q4_r);
    text(ankle_right(1)+offset, ankle_right(2)+offset, ankle_right(3), num2str(q4_r(end)));
    
    q3_l = [q3_l; angle_jnt(hip_left, knee_left, ankle_left,'sag')];
    q3_l(end) = -1*(180-q3_l(end));
    text(knee_left(1)+offset,knee_left(2)+offset,knee_left(3),num2str(q3_l(end)));
    
    q4_l = [q4_l; angle_jnt([ankle_left(1:2); ankle_left(3)+0.05], ankle_left, knee_left,'sag')];
    q4_l = -1*abs(q4_l);
    text(ankle_left(1)+offset, ankle_left(2)+offset, ankle_left(3), num2str(q4_l(end)));
    
    
    q5_r = [q5_r; angle_jnt([ankle_right(1);ankle_right(2)+0.05; ankle_right(3)], ankle_right, knee_right,'front')];
    q5_r(end) = q5_r(end)-90;
    text(ankle_right(1)+offset, ankle_right(2)-offset, ankle_right(3), strcat('q5: ',num2str(q5_r(end))));
    
    q5_l = [q5_l; angle_jnt([ankle_left(1);ankle_left(2)+0.05; ankle_left(3)], ankle_left, knee_left,'front')];
    q5_l(end) = 90-q5_l(end);
    text(ankle_left(1)+offset, ankle_left(2)-offset, ankle_left(3), num2str(q5_l(end)));
    
    
    set(linklowerlegR,'XData',[knee_right(1);ankle_right(1)],'YData',[knee_right(2);ankle_right(2)],'ZData',[knee_right(3);ankle_right(3)]);
    
    set(linkfootfL,'XData',[footfl_left(1);footfr_left(1)],'YData',[footfl_left(2);footfr_left(2)],'ZData',[footfl_left(3);footfr_left(3)]);
    set(linkfootbL,'XData',[footbl_left(1);footbr_left(1)],'YData',[footbl_left(2);footbr_left(2)],'ZData',[footbl_left(3);footbr_left(3)]);
    set(linkfootlL,'XData',[footfl_left(1);footbl_left(1)],'YData',[footfl_left(2);footbl_left(2)],'ZData',[footfl_left(3);footbl_left(3)]);
    set(linkfootrL,'XData',[footfr_left(1);footbr_left(1)],'YData',[footfr_left(2);footbr_left(2)],'ZData',[footfr_left(3);footbr_left(3)]);
    
    set(linkfootfR,'XData',[footfl_right(1);footfr_right(1)],'YData',[footfl_right(2);footfr_right(2)],'ZData',[footfl_right(3);footfr_right(3)]);
    set(linkfootbR,'XData',[footbl_right(1);footbr_right(1)],'YData',[footbl_right(2);footbr_right(2)],'ZData',[footbl_right(3);footbr_right(3)]);
    set(linkfootlR,'XData',[footfl_right(1);footbl_right(1)],'YData',[footfl_right(2);footbl_right(2)],'ZData',[footfl_right(3);footbl_right(3)]);
    set(linkfootrR,'XData',[footfr_right(1);footbr_right(1)],'YData',[footfr_right(2);footbr_right(2)],'ZData',[footfr_right(3);footbr_right(3)]);
    
    % include ankle joint to foot
    set(linkanklefootLegL,'XData',[ankle_left(1);ankle_left(1)],'YData',[ankle_left(2);ankle_left(2)],'ZData',[ankle_left(3);ankle_left(3)-ankle_height]);
    set(linkanklefootLegR,'XData',[ankle_right(1);ankle_right(1)],'YData',[ankle_right(2);ankle_right(2)],'ZData',[ankle_right(3);ankle_right(3)-ankle_height]);
    
    
    %    set(ZMPDes,'XData',[zmpx(n);zmpx(n)],'YData',[zmpy(n);zmpy(n)],'ZData',[0;-0.02]);
    %    set(ZMPmulti1,'XData',[zmpx_multibody(n);zmpx_multibody(n)],'YData',[zmpy_multibody(n);zmpy_multibody(n)],'ZData',[0;-0.03]);
    %    set(ZMPmulti2,'XData',[zmpx_multibody2(n);zmpx_multibody2(n)],'YData',[zmpy_multibody2(n);zmpy_multibody2(n)],'ZData',[0;-0.05]);
    
    %    set the time elapsed watch
    %    text(max(comx), -0.0, 0.55, ['time=',num2str(time(n),'%5.3f')],'EdgeColor','blue','FontSize',12);
    title('Simulation of Walking Robot','FontSize',20)
    xlabel('Distance in x Axis: m','FontSize',20);
    zlabel('Distance in z Axis: m','FontSize',20);
    
    drawnow;
    %     F(j) = getframe;
    pause(0);
end

toc

%%
q2_l = zeros(size(q1_l));
q2_r = zeros(size(q1_l));

qL2 = -qL2*180/pi;
qR2 = -qR2*180/pi;

qL2(2,:) = -qL2(2,:);

% qR2(3,:) = -qR2(3,:);
qL2(3,:) = -qL2(3,:);

qL2(5,:) = -qL2(5,:);
qR2(5,:) = -qR2(5,:);

qL2(6,:) = -qL2(6,:);

%% Plotting angles
figure, hold on
q0_l_new = interp1(0:length(q0_l)-1,q0_l,0:length(q0_l)/length(qL2):length(q0_l)-1);
q0_r_new = interp1(0:length(q0_r)-1,q0_r,0:length(q0_r)/length(qR2):length(q0_r)-1);
% plot(q0_l_new, '-r','LineWidth',3);
% plot(q0_r_new, '-b','LineWidth',3);

plot(qL2(1,:), '.r','LineWidth',4);
plot(qR2(1,:), '.b','LineWidth',4);

legend('Left Leg jnt 0','Right Leg jnt0');

%%
figure, hold on
q1_l_new = interp1(0:length(q1_l)-1,q1_l,0:length(q1_l)/length(qL2):length(q1_l)-1);
q1_r_new = interp1(0:length(q1_r)-1,q1_r,0:length(q1_r)/length(qR2):length(q1_r)-1);
% plot(q1_l_new, '-r','LineWidth',3);
% plot(q1_r_new, '-b','LineWidth',3);

plot(qL2(2,:), '.r','LineWidth',4);
plot(qR2(2,:), '.b','LineWidth',4);

legend('Left Leg jnt 1','Right Leg jnt1');

%%
figure, hold on
q2_l_new = interp1(0:length(q2_l)-1,q2_l,0:length(q2_l)/length(qL2):length(q2_l)-1);
q2_r_new = interp1(0:length(q2_r)-1,q2_r,0:length(q2_r)/length(qR2):length(q2_r)-1);
% plot(q2_l_new, '-r','LineWidth',3);
% plot(q2_r_new, '-b','LineWidth',3);

plot(qL2(3,:), '.r','LineWidth',4);
plot(qR2(3,:), '.b','LineWidth',4);

legend('Left Leg jnt 2','Right Leg jnt2');

%%
figure, hold on
q3_l_new = interp1(0:length(q3_l)-1,q3_l,0:length(q3_l)/length(qL2):length(q3_l)-1);
q3_r_new = interp1(0:length(q3_r)-1,q3_r,0:length(q3_r)/length(qR2):length(q3_r)-1);
% plot(q3_l_new, '-r','LineWidth',3);
% plot(q3_r_new, '-b','LineWidth',3);

plot(qL2(4,:), '.r','LineWidth',4);
plot(qR2(4,:), '.b','LineWidth',4);

legend('Left Leg jnt 3','Right Leg jnt3');

%%
figure, hold on
q4_l_new = interp1(0:length(q4_l)-1,q4_l,0:length(q4_l)/length(qL2):length(q4_l)-1);
q4_r_new = interp1(0:length(q4_r)-1,q4_r,0:length(q4_r)/length(qR2):length(q4_r)-1);
% plot(q4_l_new, '-r','LineWidth',3);
% plot(q4_r_new, '-b','LineWidth',3);

plot(qL2(5,:), '.r','LineWidth',4);
plot(qR2(5,:), '.b','LineWidth',4);


legend('Left Leg jnt 4','Right Leg jnt4');
%%
figure, hold on
q5_l_new = interp1(0:length(q5_l)-1,q5_l,0:length(q5_l)/length(qL2):length(q5_l)-1);
q5_r_new = interp1(0:length(q5_r)-1,q5_r,0:length(q5_r)/length(qR2):length(q5_r)-1);
% plot(q5_l_new, '-r','LineWidth',3);
% plot(q5_r_new, '-b','LineWidth',3);

plot(qL2(6,:), '.r','LineWidth',4);
plot(qR2(6,:), '.b','LineWidth',4);

legend('Left Leg jnt 5','Right Leg jnt5');
%
%% THIRD WAY
cd /home/jorhabib/iCub/contrib/src/balancing/walkPlayer/build
fidr5 = fopen('seq_leg_right.txt','w');
fidl5 = fopen('seq_leg_left.txt','w');
Tsimu = T;
timeoffset = 3;

manual_off5 = 0;
scale=1.15;
i=0;fprintf(fidr5, strcat([num2str(i+1),' ',num2str(Tsimu*i)           ,' ','%2.2f %2.2f %2.2f %2.2f %2.2f %2.2f\n']),qR2(1,i+1), qR2(2,i+1), qR2(3,i+1), qR2(4,i+1), qR2(5,i+1)+manual_off5, scale*qR2(6,i+1));
for (i=0:length(qR2)-1)
    fprintf(fidr5, strcat([num2str(i+1),' ',num2str(Tsimu*i+timeoffset),' ','%2.2f %2.2f %2.2f %2.2f %2.2f %2.2f\n']),qR2(1,i+1), qR2(2,i+1), qR2(3,i+1), qR2(4,i+1), qR2(5,i+1)+manual_off5, scale*qR2(6,i+1));
end

i=0;fprintf(fidl5, strcat([num2str(i+1),' ',num2str(Tsimu*i)           ,' ','%2.2f %2.2f %2.2f %2.2f %2.2f %2.2f\n']),qL2(1,i+1), qL2(2,i+1), qL2(3,i+1), qL2(4,i+1), qL2(5,i+1)+manual_off5, qL2(6,i+1));
for (i=0:length(qL2)-1)
    fprintf(fidl5, strcat([num2str(i+1),' ',num2str(Tsimu*i+timeoffset),' ','%2.2f %2.2f %2.2f %2.2f %2.2f %2.2f\n']),qL2(1,i+1), qL2(2,i+1), qL2(3,i+1), qL2(4,i+1), qL2(5,i+1)+manual_off5, qL2(6,i+1));
end

cd /home/jorhabib/iCub/contrib/src/misc/walking/WalkingMatlabCode
fclose(fidr5);
fclose(fidl5);