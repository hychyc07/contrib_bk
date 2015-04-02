function [COG,COG_Global]=calcCOG(Body_P,Body_R, Foot_PL, Foot_RL, Foot_PR, Foot_RR,qL,qR)
% bug corrected wrong 'COG(:,6) = hip_right + Body_R*Rupperleg*COG_UpperLeg;'
% should be 'COG(:,6) = knee_right + Body_R*Rupperleg**Runderleg *COG_UpperLeg;'
global hip_Comy lengthupperleg lengthunderleg 
global MassUpperbody MassUpperleg MassUnderleg MassFoot MassTot
global COG_UB COG_UpperLeg COG_UnderLeg COG_foot

COG(:,1) = Body_P + Body_R * COG_UB;

hip_left = Body_P +  Body_R * [0;hip_Comy;0];

% c1=cos(qL(1)); %Z
% s1=sin(qL(1));
% 
% c2=cos(qL(2)); %X
% s2=sin(qL(2));
% 
% c3=cos(qL(3)); %Y
% s3=sin(qL(3));
% 
% Rupperleg(1,1) = c1*c3-s1*s2*s3;    Rupperleg(1,2) = -s1*c2;    Rupperleg(1,3) = c1*s3+s1*s2*c3; 
% Rupperleg(2,1) = s1*c3+c1*s2*s3;    Rupperleg(2,2) = c1*c2;     Rupperleg(2,3) = s1*s3-c1*s2*c3; 
% Rupperleg(3,1) = -c2*s3;            Rupperleg(3,2) = s2;        Rupperleg(3,3) = c2*c3; 

c1=cos(qL(1)); %Y
s1=sin(qL(1));

c2=cos(qL(2)); %X
s2=sin(qL(2));

c3=cos(qL(3)); %Z
s3=sin(qL(3));

Rupperleg(1,1) = c1*c3;               Rupperleg(1,2) = -c1*s3+s1*s2*c3;     Rupperleg(1,3) = s1*c2; 
Rupperleg(2,1) = c2*s3;               Rupperleg(2,2) = c2*c3;               Rupperleg(2,3) = -s2; 
Rupperleg(3,1) = -s1*c3+c1*s2*s3;     Rupperleg(3,2) = s1*s3+c1*s2*c3;      Rupperleg(3,3) = c1*c2; 

COG(:,2) = hip_left + Body_R*Rupperleg*COG_UpperLeg;

knee_left = hip_left + Body_R*Rupperleg*[0;0;-lengthupperleg];

c4 = cos(qL(4)); %Y
s4 = sin(qL(4));

Runderleg(1,1) = c4;     Runderleg(1,2) = 0;   Runderleg(1,3) = s4; 
Runderleg(2,1) = 0.0;    Runderleg(2,2) = 1;   Runderleg(2,3) = 0; 
Runderleg(3,1) = -s4;    Runderleg(3,2) = 0;   Runderleg(3,3) = c4; 

COG(:,3) = knee_left + Body_R*Rupperleg*Runderleg*COG_UnderLeg;

ankle_left = knee_left + Body_R*Rupperleg*Runderleg*[0;0;-lengthunderleg];

c5 = cos(qL(5)); %Y
s5 = sin(qL(5));
c6 = cos(qL(6)); %X
s6 = sin(qL(6));

Rfoot(1,1) = c5;    Rfoot(1,2) = s5*s6;     Rfoot(1,3) = s5*c6; 
Rfoot(2,1) = 0;     Rfoot(2,2) = c6;        Rfoot(2,3) = -s6; 
Rfoot(3,1) = -s5;   Rfoot(3,2) = c5*s6;     Rfoot(3,3) = c6*c5; 

COG(:,4) = ankle_left + Body_R*Rupperleg*Runderleg*Rfoot*COG_foot;

hip_right = Body_P +  Body_R * [0;-hip_Comy;0];

% c1=cos(qR(1)); %Z
% s1=sin(qR(1));
% 
% c2=cos(qR(2)); %X
% s2=sin(qR(2));
% 
% c3=cos(qR(3)); %Y
% s3=sin(qR(3));
% 
% Rupperleg(1,1) = c1*c3-s1*s2*s3;    Rupperleg(1,2) = -s1*c2;    Rupperleg(1,3) = c1*s3+s1*s2*c3; 
% Rupperleg(2,1) = s1*c3+c1*s2*s3;    Rupperleg(2,2) = c1*c2;     Rupperleg(2,3) = s1*s3-c1*s2*c3; 
% Rupperleg(3,1) = -c2*s3;            Rupperleg(3,2) = s2;        Rupperleg(3,3) = c2*c3; 

c1=cos(qR(1)); %Y
s1=sin(qR(1));

c2=cos(qR(2)); %X
s2=sin(qR(2));

c3=cos(qR(3)); %Z
s3=sin(qR(3));

Rupperleg(1,1) = c1*c3;               Rupperleg(1,2) = -c1*s3+s1*s2*c3;     Rupperleg(1,3) = s1*c2; 
Rupperleg(2,1) = c2*s3;               Rupperleg(2,2) = c2*c3;               Rupperleg(2,3) = -s2; 
Rupperleg(3,1) = -s1*c3+c1*s2*s3;     Rupperleg(3,2) = s1*s3+c1*s2*c3;      Rupperleg(3,3) = c1*c2; 

COG(:,5) = hip_right + Body_R*Rupperleg*COG_UpperLeg;

knee_right= hip_right + Body_R*Rupperleg*[0;0;-lengthupperleg];

c4 = cos(qR(4)); %Y
s4 = sin(qR(4));

Runderleg(1,1) = c4;     Runderleg(1,2) = 0;   Runderleg(1,3) = s4; 
Runderleg(2,1) = 0.0;    Runderleg(2,2) = 1;   Runderleg(2,3) = 0; 
Runderleg(3,1) = -s4;    Runderleg(3,2) = 0;   Runderleg(3,3) = c4; 

COG(:,6) = knee_right + Body_R*Rupperleg*Runderleg*COG_UnderLeg;

ankle_right = knee_right + Body_R*Rupperleg*Runderleg*[0;0;-lengthunderleg];

c5 = cos(qR(5)); %Y
s5 = sin(qR(5));
c6 = cos(qR(6)); %X
s6 = sin(qR(6));

Rfoot(1,1) = c5;    Rfoot(1,2) = s5*s6;     Rfoot(1,3) = s5*c6; 
Rfoot(2,1) = 0;     Rfoot(2,2) = c6;        Rfoot(2,3) = -s6; 
Rfoot(3,1) = -s5;   Rfoot(3,2) = c5*s6;     Rfoot(3,3) = c6*c5; 

COG(:,7) = ankle_right + Body_R*Rupperleg*Runderleg*Rfoot*COG_foot;

COG_Global=(MassUpperbody*COG(:,1)+MassUpperleg*(COG(:,2)+COG(:,5))+MassUnderleg*(COG(:,3)+COG(:,6))+MassFoot*(COG(:,4)+COG(:,7)))/(MassTot);


