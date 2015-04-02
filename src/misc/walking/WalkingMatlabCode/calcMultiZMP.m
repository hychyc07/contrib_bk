% this is the complete and right (tested) multibody zmp equation
% calculation. the angular momentum includes the orbit angular momentum
% part and the spin angular momentum part
% the equation is : L=sum(ci*Pi)+sum(RiIRi'*RiWi)
% programed on 5 of July 2010
function [zmpx_multibody,zmpy_multibody,Ltot,Htot, Htot_old,Lx_old,Ly_old]=calcMultiZMP(Body_R,qL,qR,qdL,qdR,qd_abs, COG,COGd, COGdd,COG_Global,COG_Globald, Lx_old, Ly_old, Htot_old)
global T Mass G
global MassUpperbody MassUpperleg MassUnderleg MassFoot 
global IG_body_loc IG_foot_loc IG_underleg_loc IG_upperleg_loc
g=G;

%orbital angular momentum part of the total angular momentum
for i=1:7
    Rc(:,i) = COG(:,i) - COG_Global;
    Vc(:,i) = COGd(:,i) - COG_Globald;
    Lseg(:,i)=cross( Rc(:,i),Mass(i)*Vc(:,i) );
end
Lx = sum(Lseg(1,:));    Ly = sum(Lseg(2,:));    Lz = sum(Lseg(3,:));    Ltot=[Lx;Ly;Lz];
dLx = (Lx - Lx_old)/T;  dLy = (Ly - Ly_old)/T;
Lx_old = Lx;    Ly_old = Ly;
%%% this orbital term is already included in the following multi-mass
%%% equations

item(1)=MassUpperbody*COG(1,1)*(g + COGdd(3,1) ) - MassUpperbody*COG(3,1)*COGdd(1,1);
item(2)=MassUpperleg*COG(1,2)*(g + COGdd(3,2) )- MassUpperleg*COG(3,2)*COGdd(1,2);
item(3)=MassUnderleg*COG(1,3)*(g + COGdd(3,3) )- MassUnderleg*COG(3,3)*COGdd(1,3);
item(4)=MassFoot*COG(1,4)*(g + COGdd(3,4) )- MassFoot*COG(3,4)*COGdd(1,4);
item(5)=MassUpperleg*COG(1,5)*(g + COGdd(3,5) )- MassUpperleg*COG(3,5)*COGdd(1,5);
item(6)=MassUnderleg*COG(1,6)*(g + COGdd(3,6) )- MassUnderleg*COG(3,6)*COGdd(1,6);
item(7)=MassFoot*COG(1,7)*(g + COGdd(3,7) )- MassFoot*COG(3,7)*COGdd(1,7);
XNumerator = sum(item); clear item;

%%%%%%%%%
item(1)=MassUpperbody*COG(2,1)*(g + COGdd(3,1) ) - MassUpperbody*COG(3,1)*COGdd(2,1);
item(2)=MassUpperleg*COG(2,2)*(g + COGdd(3,2) )- MassUpperleg*COG(3,2)*COGdd(2,2);
item(3)=MassUnderleg*COG(2,3)*(g + COGdd(3,3) )- MassUnderleg*COG(3,3)*COGdd(2,3);
item(4)=MassFoot*COG(2,4)*(g + COGdd(3,4) )- MassFoot*COG(3,4)*COGdd(2,4);
item(5)=MassUpperleg*COG(2,5)*(g + COGdd(3,5) )- MassUpperleg*COG(3,5)*COGdd(2,5);
item(6)=MassUnderleg*COG(2,6)*(g + COGdd(3,6) )- MassUnderleg*COG(3,6)*COGdd(2,6);
item(7)=MassFoot*COG(2,7)*(g + COGdd(3,7) )- MassFoot*COG(3,7)*COGdd(2,7);
YNumerator = sum(item); clear item;
%%%%%%%%%
item(1)=MassUpperbody*(g + COGdd(3,1) ) ;
item(2)=MassUpperleg*(g + COGdd(3,2) );
item(3)=MassUnderleg*(g + COGdd(3,3) );
item(4)=MassFoot*(g + COGdd(3,4) );
item(5)=MassUpperleg*(g + COGdd(3,5) );
item(6)=MassUnderleg*(g + COGdd(3,6) );
item(7)=MassFoot*(g + COGdd(3,7) );
Denominator = sum(item); clear item;

%spin angular momentum part of the total angular momentum
% body spin angular momentum
H_abs_body = IG_body_loc*qd_abs;
% left leg
c1=cos(qL(1)); %Y
s1=sin(qL(1));

RY1L=[[c1 0 s1];[0 1 0];[-s1 0 c1]];

c2=cos(qL(2)); %X
s2=sin(qL(2));

RX2L=[[1 0 0];[0 c2 -s2];[0 s2 c2]];

c3=cos(qL(3)); %Z
s3=sin(qL(3));

RZ3L=[[c3 -s3 0];[s3 c3 0];[0 0 1]];

c4 = cos(qL(4)); %Y
s4 = sin(qL(4));

RY4L=[[c4 0 s4];[0 1 0];[-s4 0 c4]];

c5 = cos(qL(5)); %Y
s5 = sin(qL(5));

RY5L=[[c5 0 s5];[0 1 0];[-s5 0 c5]];

c6 = cos(qL(6)); %X
s6 = sin(qL(6));

RX6L=[[1 0 0];[0 c6 -s6];[0 s6 c6]];

qdL_abs(:,1)=Body_R*[0;qdL(1);0];
qdL_abs(:,2)=Body_R*RY1L*[qdL(2);0;0];
qdL_abs(:,3)=Body_R*RY1L*RX2L*[0;0;qdL(3)];
qdL_abs(:,4)=Body_R*RY1L*RX2L*RZ3L*[0;qdL(4);0];
qdL_abs(:,5)=Body_R*RY1L*RX2L*RZ3L*RY4L*[0;qdL(5);0];
qdL_abs(:,6)=Body_R*RY1L*RX2L*RZ3L*RY4L*RY5L*[qdL(6);0;0];

qdL_link(:,1)= qd_abs+qdL_abs(:,1);
qdL_link(:,2)= qdL_link(:,1)+qdL_abs(:,2);
qdL_link(:,3)= qdL_link(:,2)+qdL_abs(:,3); % angular velocity of link thigh in world coordinate
qdL_link(:,4)= qdL_link(:,3)+qdL_abs(:,4); % angular velocity of link calf in world coordinate
qdL_link(:,5)= qdL_link(:,4)+qdL_abs(:,5); 
qdL_link(:,6)= qdL_link(:,5)+qdL_abs(:,6); % angular velocity of link foot in world coordinate

IG_upperleg_newL = Body_R*RY1L*RX2L*RZ3L*IG_upperleg_loc*RZ3L'*RX2L'*RY1L'*Body_R';
IG_underleg_newL = Body_R*RY1L*RX2L*RZ3L*RY4L*IG_underleg_loc*RY4L'*RZ3L'*RX2L'*RY1L'*Body_R';
IG_foot_newL = Body_R*RY1L*RX2L*RZ3L*RY4L*RY5L*RX6L*IG_foot_loc*RX6L'*RY5L'*RY4L'*RZ3L'*RX2L'*RY1L'*Body_R';

H_abs_upperlegL = IG_upperleg_newL*qdL_link(:,3);
H_abs_underlegL = IG_underleg_newL*qdL_link(:,4);
H_abs_footL = IG_foot_newL*qdL_link(:,6);
%%%% right leg
c1=cos(qR(1)); %Y
s1=sin(qR(1));

RY1R=[[c1 0 s1];[0 1 0];[-s1 0 c1]];

c2=cos(qR(2)); %X
s2=sin(qR(2));

RX2R=[[1 0 0];[0 c2 -s2];[0 s2 c2]];

c3=cos(qR(3)); %Z
s3=sin(qR(3));

RZ3R=[[c3 -s3 0];[s3 c3 0];[0 0 1]];

c4 = cos(qR(4)); %Y
s4 = sin(qR(4));

RY4R=[[c4 0 s4];[0 1 0];[-s4 0 c4]];

c5 = cos(qR(5)); %Y
s5 = sin(qR(5));

RY5R=[[c5 0 s5];[0 1 0];[-s5 0 c5]];

c6 = cos(qR(6)); %X
s6 = sin(qR(6));

RX6R=[[1 0 0];[0 c6 -s6];[0 s6 c6]];

qdR_abs(:,1)=Body_R*[0;qdR(1);0];
qdR_abs(:,2)=Body_R*RY1R*[qdR(2);0;0];
qdR_abs(:,3)=Body_R*RY1R*RX2R*[0;0;qdR(3)];
qdR_abs(:,4)=Body_R*RY1R*RX2R*RZ3R*[0;qdR(4);0]; 
qdR_abs(:,5)=Body_R*RY1R*RX2R*RZ3R*RY4R*[0;qdR(5);0];
qdR_abs(:,6)=Body_R*RY1R*RX2R*RZ3R*RY4R*RY5R*[qdR(6);0;0];

qdR_link(:,1)= qd_abs+qdR_abs(:,1);
qdR_link(:,2)= qdR_link(:,1)+qdR_abs(:,2);
qdR_link(:,3)= qdR_link(:,2)+qdR_abs(:,3);% angular velocity of link thigh in world coordinate
qdR_link(:,4)= qdR_link(:,3)+qdR_abs(:,4);% angular velocity of link calf in world coordinate
qdR_link(:,5)= qdR_link(:,4)+qdR_abs(:,5);
qdR_link(:,6)= qdR_link(:,5)+qdR_abs(:,6);% angular velocity of link foot in world coordinate

IG_upperleg_newR = Body_R*RY1R*RX2R*RZ3R*IG_upperleg_loc*RZ3R'*RX2R'*RY1R'*Body_R';
IG_underleg_newR = Body_R*RY1R*RX2R*RZ3R*RY4R*IG_underleg_loc*RY4R'*RZ3R'*RX2R'*RY1R'*Body_R';
IG_foot_newR = Body_R*RY1R*RX2R*RZ3R*RY4R*RY5R*RX6R*IG_foot_loc*RX6R'*RY5R'*RY4R'*RZ3R'*RX2R'*RY1R'*Body_R';

H_abs_upperlegR = IG_upperleg_newR*qdR_link(:,3);
H_abs_underlegR = IG_underleg_newR*qdR_link(:,4);
H_abs_footR = IG_foot_newR*qdR_link(:,6);

Htot = H_abs_body+H_abs_upperlegL+ H_abs_underlegL+H_abs_footL+H_abs_upperlegR+H_abs_underlegR+H_abs_footR;

dH = (Htot-Htot_old)/T;
Htot_old = Htot;

%%%%%%% zmp multibody
%dL is the orbit angular momentum part, dH is the spin angular momentum
%part
zmpx_multibody = ( XNumerator - dH(2) )/Denominator;
zmpy_multibody= (YNumerator + dH(1) )/Denominator; 
% zmpx_multibody = ( XNumerator)/Denominator;
% zmpy_multibody= (YNumerator)/Denominator; 