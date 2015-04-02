% plot3(zmpx,zmpy,zmpz,'k-.','LineWidth',1);
% hold on;
% plot3(zmpx_multibody2,zmpy_multibody2,zmpz,'LineWidth',1);
% hold on;
% plot3(COG_Global(1,:),COG_Global(2,:),COG_Global(3,:),'m','LineWidth',2)
figure(31)
subplot(2,1,1)
title('Multi-Body ZMP Without Integrating Strategies','FontSize',15);hold on
plot3(zmpx,zmpy,zmpz,zmpx_multibody,zmpy_multibody,zmpz);
legend('Desired ZMP','ZMP Multi-Body');
plotfootholds;
axis([-0.05 1.05 -0.2 0.2 0 0.47]);
view(0,90);

subplot(2,1,2)
title('Multi-Body ZMP With Integrating Strategies','FontSize',15);hold on
plot3(zmpx,zmpy,zmpz,zmpx_multibody2,zmpy_multibody2,zmpz);
legend('Desired ZMP','ZMP Multi-Body');
plotfootholds;
axis([-0.05 1.05 -0.2 0.2 0 0.47]);
view(0,90);
% axis equal;