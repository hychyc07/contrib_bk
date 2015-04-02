 figure
% plot3(zmpx,zmpy,zmpz,zmpx_multibody,zmpy_multibody,zmpz,zmpx_multibody2,zmpy_multibody2,zmpz,xh,yh,zh,xh2,yh2,zh,xfl,yfl,zfl,xfr,yfr,zfr,COG_Global(1,:),COG_Global(2,:),COG_Global(3,:))


% plot3(zmpx,zmpy,zmpz,zmpx_multibody2,zmpy_multibody2,zmpz,xh2,yh2,zh,xfl,yfl,zfl,xfr,yfr,zfr), legend ('ZMP_{des}','ZMP_{multibody}','Hip','Left foot', 'right foot')
plot3(zmpx,zmpy,zmpz,xh,yh,zh,xfl,yfl,zfl,xfr,yfr,zfr), legend ('ZMP_{des}','ZMP_{multibody}','Hip','Left foot', 'right foot')
plotfootholds
 
 
plotwalkingrobot(Body_P,Body_R, Foot_PL, Foot_RL,Foot_PR, Foot_RR,qL,qR,time, zmpx, zmpy, zmpx_multibody2, zmpy_multibody2)