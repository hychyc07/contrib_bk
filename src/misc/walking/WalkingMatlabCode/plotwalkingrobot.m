function plotwalkingrobot(Body_P,Body_R, Foot_PL, Foot_RL,Foot_PR, Foot_RR,qL,qR,time,zmpx, zmpy, zmpx_multibody2, zmpy_multibody2)

global hip_Comy lengthupperleg lengthunderleg footankletotip footankletoside footankletoback PI
global T Time_startwalking TotalStepTime


% axis('equal')
% axHndl=gca;		
% 
% set(axHndl,'XGrid','off','YGrid','off','Drawmode','fast')

linkupperbody=line('erase','normal');
linkhip=line('erase','normal');
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

ZMPDes=line('erase','normal');
ZMPreal=line('erase','normal');

set(linkupperbody,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','y','Linewidth',[2],'MarkerSize',[6]);
set(linkhip,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','y','Linewidth',[2],'MarkerSize',[6]);
set(linkupperlegL,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','b','Linewidth',[2],'MarkerSize',[6]);
set(linklowerlegL,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','b','Linewidth',[2],'MarkerSize',[6]);
set(linkupperlegR,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','r','Linewidth',[2],'MarkerSize',[6]);
set(linklowerlegR,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','r','Linewidth',[2],'MarkerSize',[6]);

set(linkfootfL,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','b','Linewidth',[2],'MarkerSize',[6]);
set(linkfootrL,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','b','Linewidth',[2],'MarkerSize',[6]);
set(linkfootlL,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','b','Linewidth',[2],'MarkerSize',[6]);
set(linkfootbL,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','b','Linewidth',[2],'MarkerSize',[6]);

set(linkfootfR,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','r','Linewidth',[2],'MarkerSize',[6]);
set(linkfootrR,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','r','Linewidth',[2],'MarkerSize',[6]);
set(linkfootlR,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','r','Linewidth',[2],'MarkerSize',[6]);
set(linkfootbR,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','r','Linewidth',[2],'MarkerSize',[6]);

set(ZMPDes,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','b','Linewidth',[2],'MarkerSize',[6]);
set(ZMPreal,'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','g','Linewidth',[2],'MarkerSize',[6]);


 axis([-.1 max(zmpx)+.2 -0.12 0.12]);

 grid on;
 view(0,0); 
 
for n= round(0.5*Time_startwalking/T) :1: floor( (TotalStepTime)/T )

body_head=Body_P(:,n)+Body_R(:,:,n) * [0;0;0.3];
    
hip_left = Body_P(:,n) +  Body_R(:,:,n) * [0.0;hip_Comy;0];

c1=cos(qL(1,n)); %Y
s1=sin(qL(1,n));

c2=cos(qL(2,n)); %X
s2=sin(qL(2,n));

c3=cos(qL(3,n)); %Z
s3=sin(qL(3,n));

Rupperleg(1,1) = c1*c3;               Rupperleg(1,2) = -c1*s3+s1*s2*c3;     Rupperleg(1,3) = s1*c2; 
Rupperleg(2,1) = c2*s3;               Rupperleg(2,2) = c2*c3;               Rupperleg(2,3) = -s2; 
Rupperleg(3,1) = -s1*c3+c1*s2*s3;     Rupperleg(3,2) = s1*s3+c1*s2*c3;      Rupperleg(3,3) = c1*c2; 

knee_left = hip_left + Body_R(:,:,n)*Rupperleg*[0;0;-lengthupperleg];

c4 = cos(qL(4,n)); %Y
s4 = sin(qL(4,n));

Runderleg(1,1) = c4;     Runderleg(1,2) = 0;   Runderleg(1,3) = s4; 
Runderleg(2,1) = 0.0;    Runderleg(2,2) = 1;   Runderleg(2,3) = 0; 
Runderleg(3,1) = -s4;    Runderleg(3,2) = 0;   Runderleg(3,3) = c4; 

ankle_left = knee_left + Body_R(:,:,n)*Rupperleg*Runderleg*[0;0;-lengthunderleg];

c5 = cos(qL(5,n)); %Y
s5 = sin(qL(5,n));
c6 = cos(qL(6,n)); %X
s6 = sin(qL(6,n));

Rfoot(1,1) = c5;    Rfoot(1,2) = s5*s6;     Rfoot(1,3) = s5*c6; 
Rfoot(2,1) = 0;     Rfoot(2,2) = c6;        Rfoot(2,3) = -s6; 
Rfoot(3,1) = -s5;   Rfoot(3,2) = c5*s6;     Rfoot(3,3) = c6*c5; 

footfl_left = ankle_left + Body_R(:,:,n)*Rupperleg*Runderleg*Rfoot*[footankletotip;footankletoside;0];
footfr_left = ankle_left + Body_R(:,:,n)*Rupperleg*Runderleg*Rfoot*[footankletotip;-footankletoside;0];
footbl_left = ankle_left + Body_R(:,:,n)*Rupperleg*Runderleg*Rfoot*[-footankletoback;footankletoside;0];
footbr_left = ankle_left + Body_R(:,:,n)*Rupperleg*Runderleg*Rfoot*[-footankletoback;-footankletoside;0];

hip_right = Body_P(:,n) +  Body_R(:,:,n) * [0.0;-hip_Comy;0];

% c1=cos(qR(1,n)); %Z
% s1=sin(qR(1,n));
% 
% c2=cos(qR(2,n)); %X
% s2=sin(qR(2,n));
% 
% c3=cos(qR(3,n)); %Y
% s3=sin(qR(3,n));
% 
% Rupperleg(1,1) = c1*c3-s1*s2*s3;    Rupperleg(1,2) = -s1*c2;    Rupperleg(1,3) = c1*s3+s1*s2*c3; 
% Rupperleg(2,1) = s1*c3+c1*s2*s3;    Rupperleg(2,2) = c1*c2;     Rupperleg(2,3) = s1*s3-c1*s2*c3; 
% Rupperleg(3,1) = -c2*s3;            Rupperleg(3,2) = s2;        Rupperleg(3,3) = c2*c3; 

c1=cos(qR(1,n)); %Y
s1=sin(qR(1,n));

c2=cos(qR(2,n)); %X
s2=sin(qR(2,n));

c3=cos(qR(3,n)); %Z
s3=sin(qR(3,n));

Rupperleg(1,1) = c1*c3;               Rupperleg(1,2) = -c1*s3+s1*s2*c3;     Rupperleg(1,3) = s1*c2; 
Rupperleg(2,1) = c2*s3;               Rupperleg(2,2) = c2*c3;               Rupperleg(2,3) = -s2; 
Rupperleg(3,1) = -s1*c3+c1*s2*s3;     Rupperleg(3,2) = s1*s3+c1*s2*c3;      Rupperleg(3,3) = c1*c2; 

knee_right= hip_right + Body_R(:,:,n)*Rupperleg*[0;0;-lengthupperleg];

c4 = cos(qR(4,n)); %Y
s4 = sin(qR(4,n));

Runderleg(1,1) = c4;     Runderleg(1,2) = 0;   Runderleg(1,3) = s4; 
Runderleg(2,1) = 0.0;    Runderleg(2,2) = 1;   Runderleg(2,3) = 0; 
Runderleg(3,1) = -s4;    Runderleg(3,2) = 0;   Runderleg(3,3) = c4; 

ankle_right = knee_right + Body_R(:,:,n)*Rupperleg*Runderleg*[0;0;-lengthunderleg];

c5 = cos(qR(5,n)); %Y
s5 = sin(qR(5,n));
c6 = cos(qR(6,n)); %X
s6 = sin(qR(6,n));

Rfoot(1,1) = c5;    Rfoot(1,2) = s5*s6;     Rfoot(1,3) = s5*c6; 
Rfoot(2,1) = 0;     Rfoot(2,2) = c6;        Rfoot(2,3) = -s6; 
Rfoot(3,1) = -s5;   Rfoot(3,2) = c5*s6;     Rfoot(3,3) = c6*c5; 

footfl_right = ankle_right + Body_R(:,:,n)*Rupperleg*Runderleg*Rfoot*[footankletotip;footankletoside;0];
footfr_right = ankle_right + Body_R(:,:,n)*Rupperleg*Runderleg*Rfoot*[footankletotip;-footankletoside;0];
footbl_right = ankle_right + Body_R(:,:,n)*Rupperleg*Runderleg*Rfoot*[-footankletoback;footankletoside;0];
footbr_right = ankle_right + Body_R(:,:,n)*Rupperleg*Runderleg*Rfoot*[-footankletoback;-footankletoside;0];

   %update linepositions
   
   set(linkupperbody,'XData',[Body_P(1,n);body_head(1)],'YData',[Body_P(2,n);body_head(2)],'ZData',[Body_P(3,n);body_head(3)]);
   set(linkhip,'XData',[hip_left(1);hip_right(1)],'YData',[hip_left(2);hip_right(2)],'ZData',[hip_left(3);hip_right(3)]);
   set(linkupperlegL,'XData',[hip_left(1);knee_left(1)],'YData',[hip_left(2);knee_left(2)],'ZData',[hip_left(3);knee_left(3)]);
   set(linklowerlegL,'XData',[knee_left(1);ankle_left(1)],'YData',[knee_left(2);ankle_left(2)],'ZData',[knee_left(3);ankle_left(3)]);
   set(linkupperlegR,'XData',[hip_right(1);knee_right(1)],'YData',[hip_right(2);knee_right(2)],'ZData',[hip_right(3);knee_right(3)]);
   set(linklowerlegR,'XData',[knee_right(1);ankle_right(1)],'YData',[knee_right(2);ankle_right(2)],'ZData',[knee_right(3);ankle_right(3)]);

   set(linkfootfL,'XData',[footfl_left(1);footfr_left(1)],'YData',[footfl_left(2);footfr_left(2)],'ZData',[footfl_left(3);footfr_left(3)]);
   set(linkfootbL,'XData',[footbl_left(1);footbr_left(1)],'YData',[footbl_left(2);footbr_left(2)],'ZData',[footbl_left(3);footbr_left(3)]);
   set(linkfootlL,'XData',[footfl_left(1);footbl_left(1)],'YData',[footfl_left(2);footbl_left(2)],'ZData',[footfl_left(3);footbl_left(3)]);
   set(linkfootrL,'XData',[footfr_left(1);footbr_left(1)],'YData',[footfr_left(2);footbr_left(2)],'ZData',[footfr_left(3);footbr_left(3)]);
   
   set(linkfootfR,'XData',[footfl_right(1);footfr_right(1)],'YData',[footfl_right(2);footfr_right(2)],'ZData',[footfl_right(3);footfr_right(3)]);
   set(linkfootbR,'XData',[footbl_right(1);footbr_right(1)],'YData',[footbl_right(2);footbr_right(2)],'ZData',[footbl_right(3);footbr_right(3)]);
   set(linkfootlR,'XData',[footfl_right(1);footbl_right(1)],'YData',[footfl_right(2);footbl_right(2)],'ZData',[footfl_right(3);footbl_right(3)]);
   set(linkfootrR,'XData',[footfr_right(1);footbr_right(1)],'YData',[footfr_right(2);footbr_right(2)],'ZData',[footfr_right(3);footbr_right(3)]);
   
   set(ZMPDes,'XData',[zmpx(n);zmpx(n)],'YData',[zmpy(n);zmpy(n)],'ZData',[0;-0.02]);
   set(ZMPreal,'XData',[zmpx_multibody2(n);zmpx_multibody2(n)],'YData',[zmpy_multibody2(n);zmpy_multibody2(n)],'ZData',[0;-0.04]);

    drawnow;hold on;
%     view(0,0); 
    
%     pause(0.2);
    title('Simulation of Walking Robot','FontSize',20)    
    xlabel('Distance in x Axis: m','FontSize',20);
    zlabel('Distance in z Axis: m','FontSize',20);

    
end