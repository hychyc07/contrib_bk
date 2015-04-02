global totalnumberofsteps


Lfront=footankletotip;
Lback=footankletoback;

Lside=footankletoside;

axis('equal')
axHndl=gca;		

set(axHndl,'XGrid','off','YGrid','off','Drawmode','fast')

totalnumberofsteps

for i=1:totalnumberofsteps+1
    
    

FOOTLS1x = XGlobalFootL(i) + cos(AlpaGlobalFootL(i))*Lfront-sin(AlpaGlobalFootL(i))*Lside;     
FOOTLS1y = YGlobalFootL(i) + sin(AlpaGlobalFootL(i))*Lfront+cos(AlpaGlobalFootL(i))*Lside;

FOOTLS2x = XGlobalFootL(i) + cos(AlpaGlobalFootL(i))*Lfront+sin(AlpaGlobalFootL(i))*Lside;     
FOOTLS2y = YGlobalFootL(i) + sin(AlpaGlobalFootL(i))*Lfront-cos(AlpaGlobalFootL(i))*Lside;

FOOTLS4x = XGlobalFootL(i) - cos(AlpaGlobalFootL(i))*Lback-sin(AlpaGlobalFootL(i))*Lside;     
FOOTLS4y = YGlobalFootL(i) - sin(AlpaGlobalFootL(i))*Lback+cos(AlpaGlobalFootL(i))*Lside;

FOOTLS3x = XGlobalFootL(i) - cos(AlpaGlobalFootL(i))*Lback+sin(AlpaGlobalFootL(i))*Lside;     
FOOTLS3y = YGlobalFootL(i) - sin(AlpaGlobalFootL(i))*Lback-cos(AlpaGlobalFootL(i))*Lside;

FOOTRS1x = XGlobalFootR(i) + cos(AlpaGlobalFootR(i))*Lfront-sin(AlpaGlobalFootR(i))*Lside;     
FOOTRS1y = YGlobalFootR(i) + sin(AlpaGlobalFootR(i))*Lfront+cos(AlpaGlobalFootR(i))*Lside;

FOOTRS2x = XGlobalFootR(i) + cos(AlpaGlobalFootR(i))*Lfront+sin(AlpaGlobalFootR(i))*Lside;     
FOOTRS2y = YGlobalFootR(i) + sin(AlpaGlobalFootR(i))*Lfront-cos(AlpaGlobalFootR(i))*Lside;

FOOTRS4x = XGlobalFootR(i) - cos(AlpaGlobalFootR(i))*Lback-sin(AlpaGlobalFootR(i))*Lside;     
FOOTRS4y = YGlobalFootR(i) - sin(AlpaGlobalFootR(i))*Lback+cos(AlpaGlobalFootR(i))*Lside;

FOOTRS3x = XGlobalFootR(i) - cos(AlpaGlobalFootR(i))*Lback+sin(AlpaGlobalFootR(i))*Lside;     
FOOTRS3y = YGlobalFootR(i) - sin(AlpaGlobalFootR(i))*Lback-cos(AlpaGlobalFootR(i))*Lside;



FOOTLS1(i)=line('erase','normal');
FOOTLS2(i)=line('erase','normal');
FOOTLS3(i)=line('erase','normal');
FOOTLS4(i)=line('erase','normal');

FOOTRS1(i)=line('erase','normal');
FOOTRS2(i)=line('erase','normal');
FOOTRS3(i)=line('erase','normal');
FOOTRS4(i)=line('erase','normal');

set(FOOTLS1(i),'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','g','Linewidth',[1],'MarkerSize',[6]);
set(FOOTLS2(i),'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','g','Linewidth',[1],'MarkerSize',[6]);
set(FOOTLS3(i),'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','g','Linewidth',[1],'MarkerSize',[6]);
set(FOOTLS4(i),'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','g','Linewidth',[1],'MarkerSize',[6]);


set(FOOTRS1(i),'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','r','Linewidth',[1],'MarkerSize',[6]);
set(FOOTRS2(i),'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','r','Linewidth',[1],'MarkerSize',[6]);
set(FOOTRS3(i),'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','r','Linewidth',[1],'MarkerSize',[6]);
set(FOOTRS4(i),'Marker','.','MarkerEdgeColor',[0.5;0.5;0.5],'Color','r','Linewidth',[1],'MarkerSize',[6]);


set(FOOTLS1(i),'XData',[FOOTLS1x;FOOTLS2x],'YData',[FOOTLS1y;FOOTLS2y]);
set(FOOTLS2(i),'XData',[FOOTLS2x;FOOTLS3x],'YData',[FOOTLS2y;FOOTLS3y]);
set(FOOTLS3(i),'XData',[FOOTLS3x;FOOTLS4x],'YData',[FOOTLS3y;FOOTLS4y]);
set(FOOTLS4(i),'XData',[FOOTLS4x;FOOTLS1x],'YData',[FOOTLS4y;FOOTLS1y]);

set(FOOTRS1(i),'XData',[FOOTRS1x;FOOTRS2x],'YData',[FOOTRS1y;FOOTRS2y]);
set(FOOTRS2(i),'XData',[FOOTRS2x;FOOTRS3x],'YData',[FOOTRS2y;FOOTRS3y]);
set(FOOTRS3(i),'XData',[FOOTRS3x;FOOTRS4x],'YData',[FOOTRS3y;FOOTRS4y]);
set(FOOTRS4(i),'XData',[FOOTRS4x;FOOTRS1x],'YData',[FOOTRS4y;FOOTRS1y]);

    grid on
    drawnow;
     
    pause(0.02);

end