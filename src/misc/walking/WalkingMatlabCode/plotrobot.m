figure
% plot3(zmpx,zmpy,zmpz,zmpx_multibody,zmpy_multibody,zmpz,zmpx_multibody2,zmpy_multibody2,zmpz,xh,yh,zh,xh2,yh2,zh,xfl,yfl,zfl,xfr,yfr,zfr,COG_Global(1,:),COG_Global(2,:),COG_Global(3,:))


plot3(zmpx,zmpy,zmpz,zmpx_multibody,zmpy_multibody,zmpz,zmpx_multibody2,zmpy_multibody2,zmpz,xh,yh,zh,xh2,yh2,zh,xfl,yfl,zfl,xfr,yfr,zfr,COG_Global(1,:),COG_Global(2,:),COG_Global(3,:))
plotfootholds
plotwalkingrobot(Body_P2,Body_R, Foot_PL, Foot_RL,Foot_PR, Foot_RR,qL2,qR2,time, zmpx, zmpy, zmpx_multibody2, zmpy_multibody2)