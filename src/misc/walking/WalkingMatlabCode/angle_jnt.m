function [ang] = angle_jnt(jnt1, jnt2, jnt3, plane)
% link1 = jnt1 - jnt2;
% link2 = jnt2 - jnt3;
link1 = jnt1 - jnt2;
link2 = jnt3 - jnt2;
if strcmp(plane,'sag')
    link1 = [link1(1) link1(3)];
    link2 = [link2(1) link2(3)];
else
    if strcmp(plane,'front')
        link1 = [link1(2) link1(3)];
        link2 = [link2(2) link2(3)];
    end
end

% ang = mod( atan2( det([link1;link2]) , dot(link1,link2) ) , 2*pi );
% ang = atan2(norm(cross(link1,link2)),dot(link1,link2))
% ang = abs((ang>pi/2)*pi-ang)
ang = atan2(abs(det([link1;link2])),dot(link1,link2));
ang = ang*180/pi;

atan2(abs(det([link1;link2])),dot(link1,link2));