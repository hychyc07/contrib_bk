figure;
plot3(COG_Global(1,:),COG_Global(2,:),COG_Global(3,:));
grid on;hold on;
for i=1:length(COG_Global(1,:))
    plot3(COG_Global(1,i),COG_Global(2,i),COG_Global(3,i),'r*');
    pause(.0001);
end

