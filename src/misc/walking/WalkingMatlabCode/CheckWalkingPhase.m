function [WalkingPhase,StartWalkingPhase,EndWalkingPhase,stepnumber]=CheckWalkingPhase(WalkingPhase,StartWalkingPhase,EndWalkingPhase,LeftFootMoving, time, stepnumber,TSS_start, TDS_start, TSS_end, TDS_end,  TUA_start, TUA_end,LambdaX)

global XGlobalFootL XGlobalFootR YGlobalFootL YGlobalFootR AlpaGlobalFootL AlpaGlobalFootR Lift totalnumberofsteps
global ClampedCubicSplineFootX ClampedCubicSplineFootY ClampedCubicSplineFootZ ClampedCubicSplineFootRot ClampedCubicSplineFootRotY
global footankletotip footankletoside footankletoback

if (WalkingPhase==2)
    if (time>=EndWalkingPhase)
        WalkingPhase=1;
        StartWalkingPhase = EndWalkingPhase;
        stepnumber = stepnumber + 1;
    
        if stepnumber<=totalnumberofsteps
            EndWalkingPhase =EndWalkingPhase + (TUA_end(stepnumber)-TSS_start(stepnumber));
        else
            WalkingPhase =2;
            EndWalkingPhase =999999999;
            stepnumber=stepnumber-1;
        end 
        footSpeedBound(1)=0.0;
        footSpeedBound(2)=0.0;        

        ZfootPos = [0.0 Lift(stepnumber) 0.0];
        TimeIntervalsZ = [0 ((EndWalkingPhase-StartWalkingPhase)/2.0) (EndWalkingPhase-StartWalkingPhase)];     
        ClampedCubicSplineFootZ = spline(TimeIntervalsZ,[footSpeedBound(1) ZfootPos footSpeedBound(2)]);
        
        if LeftFootMoving(stepnumber)

            x1=XGlobalFootR(stepnumber) + (footankletotip+footankletoback) * cos(AlpaGlobalFootR(stepnumber)) - 2*footankletoside * sin(AlpaGlobalFootR(stepnumber)); % front left
            y1=YGlobalFootR(stepnumber) + (footankletotip+footankletoback) * sin(AlpaGlobalFootR(stepnumber)) + 2*footankletoside * cos(AlpaGlobalFootR(stepnumber));
            
            x2=XGlobalFootR(stepnumber) + (footankletotip+footankletoback) * cos(AlpaGlobalFootR(stepnumber)) + 2*footankletoside * sin(AlpaGlobalFootR(stepnumber)); % front right
            y2=YGlobalFootR(stepnumber) + (footankletotip+footankletoback) * sin(AlpaGlobalFootR(stepnumber)) - 2*footankletoside * cos(AlpaGlobalFootR(stepnumber));
            
            x3=XGlobalFootR(stepnumber) - (footankletotip+footankletoback) * cos(AlpaGlobalFootR(stepnumber)) - 2*footankletoside * sin(AlpaGlobalFootR(stepnumber)); % back left
            y3=YGlobalFootR(stepnumber) - (footankletotip+footankletoback) * sin(AlpaGlobalFootR(stepnumber)) + 2*footankletoside * cos(AlpaGlobalFootR(stepnumber));
            
            x4=XGlobalFootR(stepnumber) - (footankletotip+footankletoback) * cos(AlpaGlobalFootR(stepnumber)) + 2*footankletoside * sin(AlpaGlobalFootR(stepnumber)); % back right
            y4=YGlobalFootR(stepnumber) - (footankletotip+footankletoback) * sin(AlpaGlobalFootR(stepnumber)) - 2*footankletoside * cos(AlpaGlobalFootR(stepnumber));
                                  
            [status1]=collisionCheck([XGlobalFootL(stepnumber) YGlobalFootL(stepnumber)],[XGlobalFootL(stepnumber+1) YGlobalFootL(stepnumber+1)],[x1 y1],[x2 y2]);
            [status2]=collisionCheck([XGlobalFootL(stepnumber) YGlobalFootL(stepnumber)],[XGlobalFootL(stepnumber+1) YGlobalFootL(stepnumber+1)],[x2 y2],[x4 y4]);
            [status3]=collisionCheck([XGlobalFootL(stepnumber) YGlobalFootL(stepnumber)],[XGlobalFootL(stepnumber+1) YGlobalFootL(stepnumber+1)],[x4 y4],[x3 y3]);
            [status4]=collisionCheck([XGlobalFootL(stepnumber) YGlobalFootL(stepnumber)],[XGlobalFootL(stepnumber+1) YGlobalFootL(stepnumber+1)],[x3 y3],[x1 y1]);
            
                status1
                status2
                status3
                status4
             stepnumber
%             XfootPos = [XGlobalFootL(stepnumber) ((XGlobalFootL(stepnumber)+XGlobalFootL(stepnumber+1))/2.0) XGlobalFootL(stepnumber+1)];
%             YfootPos = [YGlobalFootL(stepnumber) ((YGlobalFootL(stepnumber)+YGlobalFootL(stepnumber+1))/2.0) YGlobalFootL(stepnumber+1)];
%             Rotfootpos = [AlpaGlobalFootL(stepnumber) AlpaGlobalFootR(stepnumber+1) AlpaGlobalFootL(stepnumber+1)];
            
            if (status4<0)&&(status3<0)      
                x3=XGlobalFootR(stepnumber) - (footankletotip+footankletoback) * cos(AlpaGlobalFootR(stepnumber)) - 2*footankletoside * sin(AlpaGlobalFootR(stepnumber));
                y3=YGlobalFootR(stepnumber) - (footankletotip+footankletoback) * sin(AlpaGlobalFootR(stepnumber)) + 2*footankletoside * cos(AlpaGlobalFootR(stepnumber));

                XfootPos = [XGlobalFootL(stepnumber) x3 XGlobalFootL(stepnumber+1)]
                YfootPos = [YGlobalFootL(stepnumber) y3 YGlobalFootL(stepnumber+1)]                
                Rotfootpos = [AlpaGlobalFootL(stepnumber) AlpaGlobalFootR(stepnumber) AlpaGlobalFootL(stepnumber+1)];
                TimeIntervalsZ = [0 ((EndWalkingPhase-StartWalkingPhase)/2.0) (EndWalkingPhase-StartWalkingPhase)];  
                
            elseif (status1<0)&&(status4<0) 
                x1=XGlobalFootR(stepnumber) + (footankletotip+footankletoback) * cos(AlpaGlobalFootR(stepnumber)) - 2*footankletoside * sin(AlpaGlobalFootR(stepnumber));
                y1=YGlobalFootR(stepnumber) + (footankletotip+footankletoback) * sin(AlpaGlobalFootR(stepnumber)) + 2*footankletoside * cos(AlpaGlobalFootR(stepnumber));

                XfootPos = [XGlobalFootL(stepnumber) x1 XGlobalFootL(stepnumber+1)]
                YfootPos = [YGlobalFootL(stepnumber) y1 YGlobalFootL(stepnumber+1)]             
                Rotfootpos = [AlpaGlobalFootL(stepnumber) AlpaGlobalFootR(stepnumber) AlpaGlobalFootL(stepnumber+1)];
                TimeIntervalsZ = [0 ((EndWalkingPhase-StartWalkingPhase)/2.0) (EndWalkingPhase-StartWalkingPhase)]; 
                
            elseif (status1<0)&&(status3<0) 
                x1=XGlobalFootR(stepnumber) + (footankletotip+footankletoback) * cos(AlpaGlobalFootR(stepnumber)) - 2*footankletoside * sin(AlpaGlobalFootR(stepnumber));
                y1=YGlobalFootR(stepnumber) + (footankletotip+footankletoback) * sin(AlpaGlobalFootR(stepnumber)) + 2*footankletoside * cos(AlpaGlobalFootR(stepnumber));

                x3=XGlobalFootR(stepnumber) - (footankletotip+footankletoback) * cos(AlpaGlobalFootR(stepnumber)) - 2*footankletoside * sin(AlpaGlobalFootR(stepnumber));
                y3=YGlobalFootR(stepnumber) - (footankletotip+footankletoback) * sin(AlpaGlobalFootR(stepnumber)) + 2*footankletoside * cos(AlpaGlobalFootR(stepnumber));

                XfootPos = [XGlobalFootL(stepnumber) x3 x1 XGlobalFootL(stepnumber+1)]
                YfootPos = [YGlobalFootL(stepnumber) y3 y1 YGlobalFootL(stepnumber+1)]     
                Rotfootpos = [AlpaGlobalFootL(stepnumber) AlpaGlobalFootR(stepnumber) AlpaGlobalFootR(stepnumber) AlpaGlobalFootL(stepnumber+1)];
                TimeIntervalsZ = [0 ((EndWalkingPhase-StartWalkingPhase)/3.0) (2.0*(EndWalkingPhase-StartWalkingPhase)/3.0) (EndWalkingPhase-StartWalkingPhase)];  
                
            else
                XfootPos = [XGlobalFootL(stepnumber) ((XGlobalFootL(stepnumber)+XGlobalFootL(stepnumber+1))/2.0) XGlobalFootL(stepnumber+1)];
                YfootPos = [YGlobalFootL(stepnumber) ((YGlobalFootL(stepnumber)+YGlobalFootL(stepnumber+1))/2.0) YGlobalFootL(stepnumber+1)];            
                Rotfootpos = [AlpaGlobalFootL(stepnumber) AlpaGlobalFootR(stepnumber) AlpaGlobalFootL(stepnumber+1)];
                TimeIntervalsZ = [0 ((EndWalkingPhase-StartWalkingPhase)/2.0) (EndWalkingPhase-StartWalkingPhase)];  
            end

        
        else
            
            x1=XGlobalFootL(stepnumber) + (footankletotip+footankletoback) * cos(AlpaGlobalFootL(stepnumber)) - (2.0*footankletoside) * sin(AlpaGlobalFootL(stepnumber)); % front left
            y1=YGlobalFootL(stepnumber) + (footankletotip+footankletoback) * sin(AlpaGlobalFootL(stepnumber)) + (2.0*footankletoside) * cos(AlpaGlobalFootL(stepnumber));
            
            x2=XGlobalFootL(stepnumber) + (footankletotip+footankletoback) * cos(AlpaGlobalFootL(stepnumber)) + (2.0*footankletoside) * sin(AlpaGlobalFootL(stepnumber)); % front right
            y2=YGlobalFootL(stepnumber) + (footankletotip+footankletoback) * sin(AlpaGlobalFootL(stepnumber)) - (2.0*footankletoside) * cos(AlpaGlobalFootL(stepnumber));
            
            x3=XGlobalFootL(stepnumber) - (footankletotip+footankletoback) * cos(AlpaGlobalFootL(stepnumber)) - (2.0*footankletoside) * sin(AlpaGlobalFootL(stepnumber)); % back left
            y3=YGlobalFootL(stepnumber) - (footankletotip+footankletoback) * sin(AlpaGlobalFootL(stepnumber)) + (2.0*footankletoside) * cos(AlpaGlobalFootL(stepnumber));
            
            x4=XGlobalFootL(stepnumber) - (footankletotip+footankletoback) * cos(AlpaGlobalFootL(stepnumber)) + (2.0*footankletoside) * sin(AlpaGlobalFootL(stepnumber)); % back right
            y4=YGlobalFootL(stepnumber) - (footankletotip+footankletoback) * sin(AlpaGlobalFootL(stepnumber)) - (2.0*footankletoside) * cos(AlpaGlobalFootL(stepnumber));
                                  
            [status1]=collisionCheck([XGlobalFootR(stepnumber) YGlobalFootR(stepnumber)],[XGlobalFootR(stepnumber+1) YGlobalFootR(stepnumber+1)],[x1 y1],[x2 y2]);
            [status2]=collisionCheck([XGlobalFootR(stepnumber) YGlobalFootR(stepnumber)],[XGlobalFootR(stepnumber+1) YGlobalFootR(stepnumber+1)],[x2 y2],[x4 y4]);
            [status3]=collisionCheck([XGlobalFootR(stepnumber) YGlobalFootR(stepnumber)],[XGlobalFootR(stepnumber+1) YGlobalFootR(stepnumber+1)],[x4 y4],[x3 y3]);
            [status4]=collisionCheck([XGlobalFootR(stepnumber) YGlobalFootR(stepnumber)],[XGlobalFootR(stepnumber+1) YGlobalFootR(stepnumber+1)],[x3 y3],[x1 y1]);
            
                status1
                status2
                status3
                status4
                stepnumber
            
            if (status2<0)&&(status3<0)      
                x4=XGlobalFootL(stepnumber) - (footankletotip+footankletoback) * cos(AlpaGlobalFootL(stepnumber)) + 2*footankletoside * sin(AlpaGlobalFootL(stepnumber));
                y4=YGlobalFootL(stepnumber) - (footankletotip+footankletoback) * sin(AlpaGlobalFootL(stepnumber)) - 2*footankletoside * cos(AlpaGlobalFootL(stepnumber));

                XfootPos = [XGlobalFootR(stepnumber) x4 XGlobalFootR(stepnumber+1)]
                YfootPos = [YGlobalFootR(stepnumber) y4 YGlobalFootR(stepnumber+1)]                
                Rotfootpos = [AlpaGlobalFootR(stepnumber) AlpaGlobalFootL(stepnumber) AlpaGlobalFootR(stepnumber+1)];
                TimeIntervalsZ = [0 ((EndWalkingPhase-StartWalkingPhase)/2.0) (EndWalkingPhase-StartWalkingPhase)];  
                
            elseif (status1<0)&&(status2<0) 
                x2=XGlobalFootL(stepnumber) + (footankletotip+footankletoback) * cos(AlpaGlobalFootL(stepnumber)) + 2*footankletoside * sin(AlpaGlobalFootL(stepnumber));
                y2=YGlobalFootL(stepnumber) + (footankletotip+footankletoback) * sin(AlpaGlobalFootL(stepnumber)) - 2*footankletoside * cos(AlpaGlobalFootL(stepnumber));

                XfootPos = [XGlobalFootR(stepnumber) x2 XGlobalFootR(stepnumber+1)]
                YfootPos = [YGlobalFootR(stepnumber) y2 YGlobalFootR(stepnumber+1)]             
                Rotfootpos = [AlpaGlobalFootR(stepnumber) AlpaGlobalFootL(stepnumber) AlpaGlobalFootR(stepnumber+1)];
                TimeIntervalsZ = [0 ((EndWalkingPhase-StartWalkingPhase)/2.0) (EndWalkingPhase-StartWalkingPhase)]; 
                
            elseif (status1<0)&&(status3<0) 
                x2=XGlobalFootL(stepnumber) + (footankletotip+footankletoback) * cos(AlpaGlobalFootL(stepnumber)) + 2*footankletoside * sin(AlpaGlobalFootL(stepnumber));
                y2=YGlobalFootL(stepnumber) + (footankletotip+footankletoback) * sin(AlpaGlobalFootL(stepnumber)) - 2*footankletoside * cos(AlpaGlobalFootL(stepnumber));

                x4=XGlobalFootL(stepnumber) - (footankletotip+footankletoback) * cos(AlpaGlobalFootL(stepnumber)) + 2*footankletoside * sin(AlpaGlobalFootL(stepnumber));
                y4=YGlobalFootL(stepnumber) - (footankletotip+footankletoback) * sin(AlpaGlobalFootL(stepnumber)) - 2*footankletoside * cos(AlpaGlobalFootL(stepnumber));

                XfootPos = [XGlobalFootR(stepnumber) x4 x2 XGlobalFootR(stepnumber+1)]
                YfootPos = [YGlobalFootR(stepnumber) y4 y2 YGlobalFootR(stepnumber+1)]     
                Rotfootpos = [AlpaGlobalFootR(stepnumber) AlpaGlobalFootL(stepnumber) AlpaGlobalFootL(stepnumber) AlpaGlobalFootR(stepnumber+1)];
                TimeIntervalsZ = [0 ((EndWalkingPhase-StartWalkingPhase)/3.0) (2.0*(EndWalkingPhase-StartWalkingPhase)/3.0) (EndWalkingPhase-StartWalkingPhase)];  
                
            else
                XfootPos = [XGlobalFootR(stepnumber) ((XGlobalFootR(stepnumber)+XGlobalFootR(stepnumber+1))/2.0) XGlobalFootR(stepnumber+1)];
                YfootPos = [YGlobalFootR(stepnumber) ((YGlobalFootR(stepnumber)+YGlobalFootR(stepnumber+1))/2.0) YGlobalFootR(stepnumber+1)];            
                Rotfootpos = [AlpaGlobalFootR(stepnumber) AlpaGlobalFootL(stepnumber) AlpaGlobalFootR(stepnumber+1)];
                TimeIntervalsZ = [0 ((EndWalkingPhase-StartWalkingPhase)/2.0) (EndWalkingPhase-StartWalkingPhase)];  
            end
        end
        ClampedCubicSplineFootX = spline(TimeIntervalsZ,[footSpeedBound(1) XfootPos footSpeedBound(2)]);
        ClampedCubicSplineFootY = spline(TimeIntervalsZ,[footSpeedBound(1) YfootPos footSpeedBound(2)]);
        ClampedCubicSplineFootRot = spline(TimeIntervalsZ,[footSpeedBound(1) Rotfootpos footSpeedBound(2)]);   
    end
end


if (WalkingPhase==3)
    if (time>=EndWalkingPhase)
        WalkingPhase=4;
        StartWalkingPhase = EndWalkingPhase;
        stepnumber = stepnumber + 1;
    
        if stepnumber<=totalnumberofsteps
            EndWalkingPhase =EndWalkingPhase + 0.2*(TUA_end(stepnumber)-TSS_start(stepnumber));
        else
            WalkingPhase =2;
            EndWalkingPhase =999999999;
            stepnumber=stepnumber-1;
        end
        
        footSpeedBound(1)=0.0;
        footSpeedBound(2)=0.0;  
        RotYfootpos = [0.0 (10*pi/180)];
        TimeIntervalsZ = [0 (0.2*(TUA_end(stepnumber)-TSS_start(stepnumber)))];  
        ClampedCubicSplineFootRotY = spline(TimeIntervalsZ,[footSpeedBound(1) RotYfootpos footSpeedBound(2)]);
        
    end
end

if (WalkingPhase==4)
    if (time>=EndWalkingPhase)
        WalkingPhase=5;
        StartWalkingPhase = EndWalkingPhase;
    
        if stepnumber<=totalnumberofsteps
            EndWalkingPhase =EndWalkingPhase + 0.8*(TUA_end(stepnumber)-TSS_start(stepnumber));
        else
            WalkingPhase =2;
            EndWalkingPhase =999999999;
            stepnumber=stepnumber-1;
        end 
        footSpeedBound(1)=0.0;
        footSpeedBound(2)=0.0;        

        RotYfootpos = [(10*pi/180) 0.0];
        TimeIntervalsZ = [0 (0.8*(TUA_end(stepnumber)-TSS_start(stepnumber)))];  
        ClampedCubicSplineFootRotY = spline(TimeIntervalsZ,[footSpeedBound(1) RotYfootpos footSpeedBound(2)]);
                 
        if LeftFootMoving(stepnumber)

            x1=XGlobalFootR(stepnumber) + (footankletotip+footankletoback) * cos(AlpaGlobalFootR(stepnumber)) - 2*footankletoside * sin(AlpaGlobalFootR(stepnumber)); % front left
            y1=YGlobalFootR(stepnumber) + (footankletotip+footankletoback) * sin(AlpaGlobalFootR(stepnumber)) + 2*footankletoside * cos(AlpaGlobalFootR(stepnumber));
            
            x2=XGlobalFootR(stepnumber) + (footankletotip+footankletoback) * cos(AlpaGlobalFootR(stepnumber)) + 2*footankletoside * sin(AlpaGlobalFootR(stepnumber)); % front right
            y2=YGlobalFootR(stepnumber) + (footankletotip+footankletoback) * sin(AlpaGlobalFootR(stepnumber)) - 2*footankletoside * cos(AlpaGlobalFootR(stepnumber));
            
            x3=XGlobalFootR(stepnumber) - (footankletotip+footankletoback) * cos(AlpaGlobalFootR(stepnumber)) - 2*footankletoside * sin(AlpaGlobalFootR(stepnumber)); % back left
            y3=YGlobalFootR(stepnumber) - (footankletotip+footankletoback) * sin(AlpaGlobalFootR(stepnumber)) + 2*footankletoside * cos(AlpaGlobalFootR(stepnumber));
            
            x4=XGlobalFootR(stepnumber) - (footankletotip+footankletoback) * cos(AlpaGlobalFootR(stepnumber)) + 2*footankletoside * sin(AlpaGlobalFootR(stepnumber)); % back right
            y4=YGlobalFootR(stepnumber) - (footankletotip+footankletoback) * sin(AlpaGlobalFootR(stepnumber)) - 2*footankletoside * cos(AlpaGlobalFootR(stepnumber));
                                  
            [status1]=collisionCheck([XGlobalFootL(stepnumber) YGlobalFootL(stepnumber)],[XGlobalFootL(stepnumber+1) YGlobalFootL(stepnumber+1)],[x1 y1],[x2 y2]);
            [status2]=collisionCheck([XGlobalFootL(stepnumber) YGlobalFootL(stepnumber)],[XGlobalFootL(stepnumber+1) YGlobalFootL(stepnumber+1)],[x2 y2],[x4 y4]);
            [status3]=collisionCheck([XGlobalFootL(stepnumber) YGlobalFootL(stepnumber)],[XGlobalFootL(stepnumber+1) YGlobalFootL(stepnumber+1)],[x4 y4],[x3 y3]);
            [status4]=collisionCheck([XGlobalFootL(stepnumber) YGlobalFootL(stepnumber)],[XGlobalFootL(stepnumber+1) YGlobalFootL(stepnumber+1)],[x3 y3],[x1 y1]);
            
                status1
                status2
                status3
                status4
             stepnumber

            rotY_fl = (10*pi/180);
            rotZ_fl = AlpaGlobalFootL(stepnumber);

            c1=cos(rotY_fl); %Y
            s1=sin(rotY_fl);

            RY=[[c1 0 s1];[0 1 0];[-s1 0 c1]];

            c3=cos(rotZ_fl); %Z
            s3=sin(rotZ_fl);

            RZ=[[c3 -s3 0];[s3 c3 0];[0 0 1]];
             
            Pos_foot_start=[(XGlobalFootL(stepnumber)+(footankletotip) * cos(AlpaGlobalFootL(stepnumber)));(YGlobalFootL(stepnumber)+ (footankletotip) * sin(AlpaGlobalFootL(stepnumber)));0.0]+RZ*RY*[-footankletotip;0;0]

            ZfootPos = [Pos_foot_start(3) Lift(stepnumber) 0.0];
            TimeIntervalsZ = [0.0 ((EndWalkingPhase-StartWalkingPhase)/2.0) (EndWalkingPhase-StartWalkingPhase)];     
            ClampedCubicSplineFootZ = spline(TimeIntervalsZ,[0.1 ZfootPos footSpeedBound(2)]);

            if (status4<0)&&(status3<0)      
                x3=XGlobalFootR(stepnumber) - (footankletotip+footankletoback) * cos(AlpaGlobalFootR(stepnumber)) - 2*footankletoside * sin(AlpaGlobalFootR(stepnumber));
                y3=YGlobalFootR(stepnumber) - (footankletotip+footankletoback) * sin(AlpaGlobalFootR(stepnumber)) + 2*footankletoside * cos(AlpaGlobalFootR(stepnumber));

                XfootPos = [Pos_foot_start(1) x3 XGlobalFootL(stepnumber+1)]
                YfootPos = [Pos_foot_start(2) y3 YGlobalFootL(stepnumber+1)]                
                Rotfootpos = [AlpaGlobalFootL(stepnumber) AlpaGlobalFootR(stepnumber) AlpaGlobalFootL(stepnumber+1)];
                TimeIntervalsZ = [0 ((EndWalkingPhase-StartWalkingPhase)/2.0) (EndWalkingPhase-StartWalkingPhase)];  
                
            elseif (status1<0)&&(status4<0) 
                x1=XGlobalFootR(stepnumber) + (footankletotip+footankletoback) * cos(AlpaGlobalFootR(stepnumber)) - 2*footankletoside * sin(AlpaGlobalFootR(stepnumber));
                y1=YGlobalFootR(stepnumber) + (footankletotip+footankletoback) * sin(AlpaGlobalFootR(stepnumber)) + 2*footankletoside * cos(AlpaGlobalFootR(stepnumber));

                XfootPos = [Pos_foot_start(1) x1 XGlobalFootL(stepnumber+1)]
                YfootPos = [Pos_foot_start(2) y1 YGlobalFootL(stepnumber+1)]             
                Rotfootpos = [AlpaGlobalFootL(stepnumber) AlpaGlobalFootR(stepnumber) AlpaGlobalFootL(stepnumber+1)];
                TimeIntervalsZ = [0 ((EndWalkingPhase-StartWalkingPhase)/2.0) (EndWalkingPhase-StartWalkingPhase)]; 
                
            elseif (status1<0)&&(status3<0) 
                x1=XGlobalFootR(stepnumber) + (footankletotip+footankletoback) * cos(AlpaGlobalFootR(stepnumber)) - 2*footankletoside * sin(AlpaGlobalFootR(stepnumber));
                y1=YGlobalFootR(stepnumber) + (footankletotip+footankletoback) * sin(AlpaGlobalFootR(stepnumber)) + 2*footankletoside * cos(AlpaGlobalFootR(stepnumber));

                x3=XGlobalFootR(stepnumber) - (footankletotip+footankletoback) * cos(AlpaGlobalFootR(stepnumber)) - 2*footankletoside * sin(AlpaGlobalFootR(stepnumber));
                y3=YGlobalFootR(stepnumber) - (footankletotip+footankletoback) * sin(AlpaGlobalFootR(stepnumber)) + 2*footankletoside * cos(AlpaGlobalFootR(stepnumber));

                XfootPos = [Pos_foot_start(1) x3 x1 XGlobalFootL(stepnumber+1)]
                YfootPos = [Pos_foot_start(2) y3 y1 YGlobalFootL(stepnumber+1)]     
                Rotfootpos = [AlpaGlobalFootL(stepnumber) AlpaGlobalFootR(stepnumber) AlpaGlobalFootR(stepnumber) AlpaGlobalFootL(stepnumber+1)];
                TimeIntervalsZ = [0 ((EndWalkingPhase-StartWalkingPhase)/3.0) (2.0*(EndWalkingPhase-StartWalkingPhase)/3.0) (EndWalkingPhase-StartWalkingPhase)];  
                
            else
                XfootPos = [Pos_foot_start(1) ((XGlobalFootL(stepnumber)+XGlobalFootL(stepnumber+1))/2.0) XGlobalFootL(stepnumber+1)];
                YfootPos = [Pos_foot_start(2) ((YGlobalFootL(stepnumber)+YGlobalFootL(stepnumber+1))/2.0) YGlobalFootL(stepnumber+1)];            
                Rotfootpos = [AlpaGlobalFootL(stepnumber) AlpaGlobalFootR(stepnumber) AlpaGlobalFootL(stepnumber+1)];
                TimeIntervalsZ = [0 ((EndWalkingPhase-StartWalkingPhase)/2.0) (EndWalkingPhase-StartWalkingPhase)];  
            end

        
        else
            
            x1=XGlobalFootL(stepnumber) + (footankletotip+footankletoback) * cos(AlpaGlobalFootL(stepnumber)) - (2.0*footankletoside) * sin(AlpaGlobalFootL(stepnumber)); % front left
            y1=YGlobalFootL(stepnumber) + (footankletotip+footankletoback) * sin(AlpaGlobalFootL(stepnumber)) + (2.0*footankletoside) * cos(AlpaGlobalFootL(stepnumber));
            
            x2=XGlobalFootL(stepnumber) + (footankletotip+footankletoback) * cos(AlpaGlobalFootL(stepnumber)) + (2.0*footankletoside) * sin(AlpaGlobalFootL(stepnumber)); % front right
            y2=YGlobalFootL(stepnumber) + (footankletotip+footankletoback) * sin(AlpaGlobalFootL(stepnumber)) - (2.0*footankletoside) * cos(AlpaGlobalFootL(stepnumber));
            
            x3=XGlobalFootL(stepnumber) - (footankletotip+footankletoback) * cos(AlpaGlobalFootL(stepnumber)) - (2.0*footankletoside) * sin(AlpaGlobalFootL(stepnumber)); % back left
            y3=YGlobalFootL(stepnumber) - (footankletotip+footankletoback) * sin(AlpaGlobalFootL(stepnumber)) + (2.0*footankletoside) * cos(AlpaGlobalFootL(stepnumber));
            
            x4=XGlobalFootL(stepnumber) - (footankletotip+footankletoback) * cos(AlpaGlobalFootL(stepnumber)) + (2.0*footankletoside) * sin(AlpaGlobalFootL(stepnumber)); % back right
            y4=YGlobalFootL(stepnumber) - (footankletotip+footankletoback) * sin(AlpaGlobalFootL(stepnumber)) - (2.0*footankletoside) * cos(AlpaGlobalFootL(stepnumber));
                                  
            [status1]=collisionCheck([XGlobalFootR(stepnumber) YGlobalFootR(stepnumber)],[XGlobalFootR(stepnumber+1) YGlobalFootR(stepnumber+1)],[x1 y1],[x2 y2]);
            [status2]=collisionCheck([XGlobalFootR(stepnumber) YGlobalFootR(stepnumber)],[XGlobalFootR(stepnumber+1) YGlobalFootR(stepnumber+1)],[x2 y2],[x4 y4]);
            [status3]=collisionCheck([XGlobalFootR(stepnumber) YGlobalFootR(stepnumber)],[XGlobalFootR(stepnumber+1) YGlobalFootR(stepnumber+1)],[x4 y4],[x3 y3]);
            [status4]=collisionCheck([XGlobalFootR(stepnumber) YGlobalFootR(stepnumber)],[XGlobalFootR(stepnumber+1) YGlobalFootR(stepnumber+1)],[x3 y3],[x1 y1]);
            
                status1
                status2
                status3
                status4
                stepnumber

            rotY_fr = (10*pi/180);
            rotZ_fr = AlpaGlobalFootR(stepnumber);

            c1=cos(rotY_fr); %Y
            s1=sin(rotY_fr);

            RY=[[c1 0 s1];[0 1 0];[-s1 0 c1]];

            c3=cos(rotZ_fr); %Z
            s3=sin(rotZ_fr);

            RZ=[[c3 -s3 0];[s3 c3 0];[0 0 1]];
            
            Pos_foot_start=[(XGlobalFootR(stepnumber)+(footankletotip) * cos(AlpaGlobalFootR(stepnumber)));(YGlobalFootR(stepnumber)+ (footankletotip) * sin(AlpaGlobalFootR(stepnumber)));0.0]+RZ*RY*[-footankletotip;0;0]

            ZfootPos = [Pos_foot_start(3) Lift(stepnumber) 0.0];
            TimeIntervalsZ = [0.0 ((EndWalkingPhase-StartWalkingPhase)/2.0) (EndWalkingPhase-StartWalkingPhase)];     
            ClampedCubicSplineFootZ = spline(TimeIntervalsZ,[0.1 ZfootPos footSpeedBound(2)]);
                
            if (status2<0)&&(status3<0)      
                x4=XGlobalFootL(stepnumber) - (footankletotip+footankletoback) * cos(AlpaGlobalFootL(stepnumber)) + 2*footankletoside * sin(AlpaGlobalFootL(stepnumber));
                y4=YGlobalFootL(stepnumber) - (footankletotip+footankletoback) * sin(AlpaGlobalFootL(stepnumber)) - 2*footankletoside * cos(AlpaGlobalFootL(stepnumber));

                XfootPos = [Pos_foot_start(1) x4 XGlobalFootR(stepnumber+1)]
                YfootPos = [Pos_foot_start(2) y4 YGlobalFootR(stepnumber+1)]                
                Rotfootpos = [AlpaGlobalFootR(stepnumber) AlpaGlobalFootL(stepnumber) AlpaGlobalFootR(stepnumber+1)];
                TimeIntervalsZ = [0 ((EndWalkingPhase-StartWalkingPhase)/2.0) (EndWalkingPhase-StartWalkingPhase)];  
                
            elseif (status1<0)&&(status2<0) 
                x2=XGlobalFootL(stepnumber) + (footankletotip+footankletoback) * cos(AlpaGlobalFootL(stepnumber)) + 2*footankletoside * sin(AlpaGlobalFootL(stepnumber));
                y2=YGlobalFootL(stepnumber) + (footankletotip+footankletoback) * sin(AlpaGlobalFootL(stepnumber)) - 2*footankletoside * cos(AlpaGlobalFootL(stepnumber));

                XfootPos = [Pos_foot_start(1) x2 XGlobalFootR(stepnumber+1)]
                YfootPos = [Pos_foot_start(2) y2 YGlobalFootR(stepnumber+1)]             
                Rotfootpos = [AlpaGlobalFootR(stepnumber) AlpaGlobalFootL(stepnumber) AlpaGlobalFootR(stepnumber+1)];
                TimeIntervalsZ = [0 ((EndWalkingPhase-StartWalkingPhase)/2.0) (EndWalkingPhase-StartWalkingPhase)]; 
                
            elseif (status1<0)&&(status3<0) 
                x2=XGlobalFootL(stepnumber) + (footankletotip+footankletoback) * cos(AlpaGlobalFootL(stepnumber)) + 2*footankletoside * sin(AlpaGlobalFootL(stepnumber));
                y2=YGlobalFootL(stepnumber) + (footankletotip+footankletoback) * sin(AlpaGlobalFootL(stepnumber)) - 2*footankletoside * cos(AlpaGlobalFootL(stepnumber));

                x4=XGlobalFootL(stepnumber) - (footankletotip+footankletoback) * cos(AlpaGlobalFootL(stepnumber)) + 2*footankletoside * sin(AlpaGlobalFootL(stepnumber));
                y4=YGlobalFootL(stepnumber) - (footankletotip+footankletoback) * sin(AlpaGlobalFootL(stepnumber)) - 2*footankletoside * cos(AlpaGlobalFootL(stepnumber));

                XfootPos = [Pos_foot_start(1) x4 x2 XGlobalFootR(stepnumber+1)]
                YfootPos = [Pos_foot_start(2) y4 y2 YGlobalFootR(stepnumber+1)]     
                Rotfootpos = [AlpaGlobalFootR(stepnumber) AlpaGlobalFootL(stepnumber) AlpaGlobalFootL(stepnumber) AlpaGlobalFootR(stepnumber+1)];
                TimeIntervalsZ = [0 ((EndWalkingPhase-StartWalkingPhase)/3.0) (2.0*(EndWalkingPhase-StartWalkingPhase)/3.0) (EndWalkingPhase-StartWalkingPhase)];  
                
            else
                XfootPos = [Pos_foot_start(1) ((XGlobalFootR(stepnumber)+XGlobalFootR(stepnumber+1))/2.0) XGlobalFootR(stepnumber+1)];
                YfootPos = [Pos_foot_start(2) ((YGlobalFootR(stepnumber)+YGlobalFootR(stepnumber+1))/2.0) YGlobalFootR(stepnumber+1)];            
                Rotfootpos = [AlpaGlobalFootR(stepnumber) AlpaGlobalFootL(stepnumber) AlpaGlobalFootR(stepnumber+1)];
                TimeIntervalsZ = [0 ((EndWalkingPhase-StartWalkingPhase)/2.0) (EndWalkingPhase-StartWalkingPhase)];  
            end
        end
        ClampedCubicSplineFootX = spline(TimeIntervalsZ,[footSpeedBound(1) XfootPos footSpeedBound(2)]);
        ClampedCubicSplineFootY = spline(TimeIntervalsZ,[footSpeedBound(1) YfootPos footSpeedBound(2)]);
        ClampedCubicSplineFootRot = spline(TimeIntervalsZ,[footSpeedBound(1) Rotfootpos footSpeedBound(2)]);   
    end
end


if (WalkingPhase==0)
    if (time>=EndWalkingPhase)
        WalkingPhase=1
        StartWalkingPhase = EndWalkingPhase
        EndWalkingPhase = EndWalkingPhase + (TUA_end(stepnumber)-TSS_start(stepnumber))
        
        footSpeedBound(1)=0.0;
        footSpeedBound(2)=0.0;        
        TimeIntervalsZ = [0 ((EndWalkingPhase-StartWalkingPhase)/2.0) (EndWalkingPhase-StartWalkingPhase)];     
        ZfootPos = [0.0 Lift(stepnumber) 0.0];

        if LeftFootMoving(stepnumber)
            XfootPos = [XGlobalFootL(stepnumber) ((XGlobalFootL(stepnumber)+XGlobalFootL(stepnumber+1))/2.0) XGlobalFootL(stepnumber+1)];
            YfootPos = [YGlobalFootL(stepnumber) ((YGlobalFootL(stepnumber)+YGlobalFootL(stepnumber+1))/2.0) YGlobalFootL(stepnumber+1)];
            Rotfootpos = [AlpaGlobalFootL(stepnumber) ((AlpaGlobalFootL(stepnumber)+AlpaGlobalFootL(stepnumber+1))/2.0) AlpaGlobalFootL(stepnumber+1)];
        else
            XfootPos = [XGlobalFootR(stepnumber) ((XGlobalFootR(stepnumber)+XGlobalFootR(stepnumber+1))/2.0) XGlobalFootR(stepnumber+1)];
            YfootPos = [YGlobalFootR(stepnumber) ((YGlobalFootR(stepnumber)+YGlobalFootR(stepnumber+1))/2.0) YGlobalFootR(stepnumber+1)];
            Rotfootpos = [AlpaGlobalFootR(stepnumber) ((AlpaGlobalFootR(stepnumber)+AlpaGlobalFootR(stepnumber+1))/2.0) AlpaGlobalFootR(stepnumber+1)];
        end
            ClampedCubicSplineFootX = spline(TimeIntervalsZ,[footSpeedBound(1) XfootPos footSpeedBound(2)]);
            ClampedCubicSplineFootY = spline(TimeIntervalsZ,[footSpeedBound(1) YfootPos footSpeedBound(2)]);
            ClampedCubicSplineFootZ = spline(TimeIntervalsZ,[footSpeedBound(1) ZfootPos footSpeedBound(2)]);
            ClampedCubicSplineFootRot = spline(TimeIntervalsZ,[footSpeedBound(1) Rotfootpos footSpeedBound(2)]);
    end
end

if (((WalkingPhase==1)||(WalkingPhase==5))&&(time>=EndWalkingPhase))
    if stepnumber<totalnumberofsteps
        if (LambdaX(stepnumber +1)>0.15)
            WalkingPhase=3;   
        else
            WalkingPhase=2;
        end
    else
        WalkingPhase=2;        
    end
    StartWalkingPhase = EndWalkingPhase
    EndWalkingPhase = EndWalkingPhase + (TDS_end(stepnumber)-TDS_start(stepnumber))
end


