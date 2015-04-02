function [q]=inversekinematics(Body_P,Body_R,Dt, Foot_P, Foot_R)

global hip_Comy lengthupperleg lengthunderleg footankletotip footankletoside


A=lengthupperleg;
B=lengthunderleg;
 
%D=(0.0;0.035;0);

rT=Foot_R'; % transpose
r = rT * (Body_P +  Body_R * Dt - Foot_P);
C = sqrt(r(1)*r(1)+r(2)*r(2)+r(3)*r(3));

c5 = (C*C-A*A-B*B)/(2.0*A*B);

if (c5>=1.0)
  q(1,4)= 0.0;
elseif (c5<=-1.0)
  q(1,4)= pi;
else 
  q(1,4)= acos(c5);
end
   

q6a = asin((A/C)*sin(pi- q(1,4)));

q(1,6) = atan2(r(2),r(3));
if (q(1,6)>pi/2.0)
  q(1,6) = q(1,6)-pi;
elseif (q(1,6)<-pi/2.0)
  q(1,6)= q(1,6)+pi;
end

q(1,5) = -atan2(r(1), sign(r(3))*sqrt(r(2)*r(2)+r(3)*r(3))) - q6a;

BRt = Body_R'; % transpose

c = cos(-q(1,6));
s = sin(-q(1,6));

Rroll(1,1) = 1.0;   Rroll(1,2) = 0.0;   Rroll(1,3) = 0.0; 
Rroll(2,1) = 0.0;   Rroll(2,2) = c;     Rroll(2,3) = -s; 
Rroll(3,1) = 0.0;   Rroll(3,2) = s;     Rroll(3,3) = c; 

c = cos(-q(1,5)-q(1,4));
s = sin(-q(1,5)-q(1,4));

Rpitch(1,1) = c;     Rpitch(1,2) = 0;   Rpitch(1,3) = s; 
Rpitch(2,1) = 0.0;   Rpitch(2,2) = 1;   Rpitch(2,3) = 0; 
Rpitch(3,1) = -s;    Rpitch(3,2) = 0;   Rpitch(3,3) = c; 

R = BRt * Foot_R * Rroll *Rpitch;
% q(1,1) = atan2(-R(1,2),R(2,2)); Z-Y-X rotatie
% 
% cz = cos(q(1,1));
% sz = sin(q(1,1));
% q(1,2) = atan2(R(3,2), -R(1,2)*sz + R(2,2) *cz);
% q(1,3) = atan2( -R(3,1), R(3,3));

q(1,3) = atan2(R(2,1),R(2,2)); %Z   Y-X-Z rotatie
q(1,1) = atan2( R(1,3), R(3,3)); %Y

cz = cos(q(1,1));
sz = sin(q(1,1));
q(1,2) = atan2(-R(2,3),R(1,3)*sz + R(3,3) *cz);