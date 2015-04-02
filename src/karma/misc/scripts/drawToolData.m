function drawToolData(data)

hfig=figure('Color','white');
hold on; view(3); grid;
xlim([-0.4 0.1]); xlabel('x [m]');
ylim([-0.3 0.3]); ylabel('y [m]');
zlim([0.0  0.4]); zlabel('z [m]');

hax=get(hfig,'CurrentAxes');
set(hax,'DataAspectRatio',[1 1 1]);

lim=axis;
A=max(abs(lim))*0.3;
quiver3(0,0,0,A,0,0,'Color','r','Linewidth',2);
quiver3(0,0,0,0,A,0,'Color','g','Linewidth',2);
quiver3(0,0,0,0,0,A,'Color','b','Linewidth',2);

dim=size(data);
len=dim(1);

p=cell(len,1);
H=cell(len,1);
Prj=cell(len,1);
Ha=cell(len,1);
He=cell(len,1);

B=A*0.5;
for i=1:len
    j=3;  p{i}  =data(i,j:j+1)';
    j=5;  H{i}  =[data(i,j:j+3); data(i,j+4:j+3+4); data(i,j+8:j+3+8)];
    j=17; Prj{i}=[data(i,j:j+3); data(i,j+4:j+3+4); data(i,j+8:j+3+8)];
    j=29; Ha{i} =[data(i,j:j+3); data(i,j+4:j+3+4); data(i,j+8:j+3+8); data(i,j+12:j+3+12)];
    j=45; He{i} =[data(i,j:j+3); data(i,j+4:j+3+4); data(i,j+8:j+3+8); data(i,j+12:j+3+12)];

    plot3(He{i}(1,4),He{i}(2,4),He{i}(3,4),'bo');
    quiver3(He{i}(1,4),He{i}(2,4),He{i}(3,4),B*He{i}(1,1),B*He{i}(2,1),B*He{i}(3,1),'Color','r','Linewidth',0.5);
    quiver3(He{i}(1,4),He{i}(2,4),He{i}(3,4),B*He{i}(1,2),B*He{i}(2,2),B*He{i}(3,2),'Color','g','Linewidth',0.5);
    quiver3(He{i}(1,4),He{i}(2,4),He{i}(3,4),B*He{i}(1,3),B*He{i}(2,3),B*He{i}(3,3),'Color','b','Linewidth',0.5);
    
    quiver3(Ha{i}(1,4),Ha{i}(2,4),Ha{i}(3,4),B*Ha{i}(1,1),B*Ha{i}(2,1),B*Ha{i}(3,1),'Color','r','Linewidth',0.5);
    quiver3(Ha{i}(1,4),Ha{i}(2,4),Ha{i}(3,4),B*Ha{i}(1,2),B*Ha{i}(2,2),B*Ha{i}(3,2),'Color','g','Linewidth',0.5);
    quiver3(Ha{i}(1,4),Ha{i}(2,4),Ha{i}(3,4),B*Ha{i}(1,3),B*Ha{i}(2,3),B*Ha{i}(3,3),'Color','b','Linewidth',0.5);
    
    drawnow;
end


