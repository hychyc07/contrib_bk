figure
plot3(zmpx,zmpy,zmpz,zmpx_multibody3,zmpy_multibody3,zmpz,xh,yh,zh,xh2,yh2,zh,xfl,yfl,zfl,xfr,yfr,zfr,COG_Global(1,:),COG_Global(2,:),COG_Global(3,:))

plotfootholds
plotwalkingrobot(Body_P3,Body_R, Foot_PL, Foot_RL,Foot_PR, Foot_RR,qL3,qR3,time, zmpx, zmpy, zmpx_multibody3, zmpy_multibody3)