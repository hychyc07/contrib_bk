%% Convert the angles to iCub angles
   qLiCub(1,n)=-qL2(1,n)*180/pi;
    if (qLiCub(1,n)<-30)
        qLiCub(1,n) = -30;
    end
    if (qLiCub(1,n)>90)
        qLiCub(1,n) = 90;
    end
   qLiCub(2,n)=+qL2(2,n)*180/pi;%+PI/2.0;  
    if (qLiCub(2,n)>(+90))
        qLiCub(2,n) = (+90);
    end
    if (qLiCub(2,n)<(-10))
        qLiCub(2,n) = (-10);
    end
   qLiCub(3,n)=qL2(3,n)*180/pi;%-PI/2.0;
    if (qLiCub(3,n)<(-79))
        qLiCub(3,n) = (-79);
    end
    if (qLiCub(3,n)>(79))
        qLiCub(3,n) = (79);
    end   
   qLiCub(4,n)=-qL2(4,n)*180/pi;%+PI/2.0;  
    if (qLiCub(4,n)<(-125))
        qLiCub(4,n) = (-125);
    end
    if (qLiCub(4,n)>(15))
        qLiCub(4,n) = (15);
    end   
   qLiCub(5,n)=qL2(6,n)*180/pi;
    if (qLiCub(5,n)>44)
        qLiCub(5,n) = 44;
    end
    if (qLiCub(5,n)<-20)
        qLiCub(5,n) = -20;
    end
   qLiCub(6,n)=-qL2(5,n)*180/pi;
    if (qLiCub(6,n)>15)
        qLiCub(6,n) = 15;
    end
    if (qLiCub(6,n)<-15)
        qLiCub(6,n) = -15;
    end   
   qRiCub(1,n)=-qR2(1,n)*180/pi;    
    if (qRiCub(1,n)<-30)
        qRiCub(1,n) = -30;
    end
    if (qRiCub(1,n)>90)
        qRiCub(1,n) = 90;
    end   
   qRiCub(2,n)=-qR2(2,n)*180/pi;%+PI/2.0;  
    if (qRiCub(2,n)>(90))
        qRiCub(2,n) = (90);
    end
    if (qRiCub(2,n)<(-10))
        qRiCub(2,n) = (-10);
    end 
   qRiCub(3,n)=-qR2(3,n)*180/pi;%-PI/2.0;  
    if (qRiCub(3,n)<(-79))
        qRiCub(3,n) = (-79);
    end
    if (qRiCub(3,n)>(79))
        qRiCub(3,n) = (79);
    end      
   qRiCub(4,n)=-qR2(4,n)*180/pi;%+PI/2.0;  
    if (qRiCub(4,n)<(-125))
        qRiCub(4,n) = (-125);
    end
    if (qRiCub(4,n)>(15))
        qRiCub(4,n) = (15);
    end    
   qRiCub(5,n)=-qR2(6,n)*180/pi;  
    if (qRiCub(5,n)>44)
        qRiCub(5,n) = 44;
    end
    if (qRiCub(5,n)<-20)
        qRiCub(5,n) = -20;
    end      
   qRiCub(6,n)=-qR2(5,n)*180/pi;    
     if (qRiCub(6,n)>15)
        qRiCub(6,n) = 15;
    end
    if (qRiCub(6,n)<-15)
        qRiCub(6,n) = -15;
    end  