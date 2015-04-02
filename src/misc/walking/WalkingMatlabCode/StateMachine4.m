% updated 12 April 2010 22:51
% at the fist step, two feet have no heel strike and toe off
function [WalkingPhase,StartWalkingPhase,EndWalkingPhase,stepnumber]=StateMachine4(WalkingPhase,StartWalkingPhase,EndWalkingPhase,LeftFootMoving, time, stepnumber,TSS_start, TDS_start, TSS_end, TDS_end,  TUA_start, TUA_end,LambdaX)

global XGlobalFootL XGlobalFootR YGlobalFootL YGlobalFootR AlpaGlobalFootL AlpaGlobalFootR Lift totalnumberofsteps
global ClampedCubicSplineFootX ClampedCubicSplineFootY ClampedCubicSplineFootZ ClampedCubicSplineFootRot ClampedCubicSplineFootRotY
global footankletotip footankletoside footankletoback
global PatternOffset Endsimtime
global Qtoe Qheel 
global SplineHeelStrike
% different speed bounds for interpolation
Speed(1) = 0; % speed boundary at foot flat state

if (WalkingPhase==0)&&(time>=EndWalkingPhase)
        WalkingPhase=1;
        StartWalkingPhase = EndWalkingPhase;
        EndWalkingPhase = EndWalkingPhase + (TSS_end(stepnumber)-TSS_start(stepnumber));
        
        TimeIntervalsZ = [0 0.5 1]*(EndWalkingPhase-StartWalkingPhase);     
        ZfootPos = [0 1 0]*Lift(stepnumber);

        if LeftFootMoving(stepnumber)
            XfootPos = [XGlobalFootL(stepnumber) ((XGlobalFootL(stepnumber)+XGlobalFootL(stepnumber+1))/2.0) XGlobalFootL(stepnumber+1)];
            YfootPos = [YGlobalFootL(stepnumber) ((YGlobalFootL(stepnumber)+YGlobalFootL(stepnumber+1))/2.0) YGlobalFootL(stepnumber+1)];
            Rotfootpos = [AlpaGlobalFootL(stepnumber) ((AlpaGlobalFootL(stepnumber)+AlpaGlobalFootL(stepnumber+1))/2.0) AlpaGlobalFootL(stepnumber+1)];
        else
            XfootPos = [XGlobalFootR(stepnumber) ((XGlobalFootR(stepnumber)+XGlobalFootR(stepnumber+1))/2.0) XGlobalFootR(stepnumber+1)];
            YfootPos = [YGlobalFootR(stepnumber) ((YGlobalFootR(stepnumber)+YGlobalFootR(stepnumber+1))/2.0) YGlobalFootR(stepnumber+1)];
            Rotfootpos = [AlpaGlobalFootR(stepnumber) ((AlpaGlobalFootR(stepnumber)+AlpaGlobalFootR(stepnumber+1))/2.0) AlpaGlobalFootR(stepnumber+1)];
        end
            ClampedCubicSplineFootX = spline(TimeIntervalsZ,[Speed(1) XfootPos Speed(1)]);
            ClampedCubicSplineFootY = spline(TimeIntervalsZ,[Speed(1) YfootPos Speed(1)]);
            ClampedCubicSplineFootZ = spline(TimeIntervalsZ,[Speed(1) ZfootPos Speed(1)]);
            ClampedCubicSplineFootRot = spline(TimeIntervalsZ,[Speed(1) Rotfootpos Speed(1)]);
end


if ( (WalkingPhase==1)||(WalkingPhase==4) )&&(time>=EndWalkingPhase)
% SS ends  -> enter DS 
    if stepnumber>=totalnumberofsteps
        WalkingPhase=0;   
        StartWalkingPhase = EndWalkingPhase;
        EndWalkingPhase = Endsimtime ;
    else
        % SS ends  -> enter DS
        WalkingPhase=3;    
        StartWalkingPhase = EndWalkingPhase;
        EndWalkingPhase = EndWalkingPhase + (TUA_end(stepnumber)-TUA_start(stepnumber));
    end
    
    if (WalkingPhase==3)
		TimeIntervalsZ = [0   TUA_end(stepnumber)-TUA_start(stepnumber) ];  
		% 1st DS toe off is canceled for stabilization reason
		if (stepnumber==1)
			RotYfootpos = [0 0];
		else
			RotYfootpos = [0.0 (Qtoe*pi/180)];  % this is the foot rotation during toe-off motion
		end
		
		ClampedCubicSplineFootRotY = spline(TimeIntervalsZ,[Speed(1) RotYfootpos Speed(1)]);
		
		RotYHeelStrike = [-(Qheel*pi/180)  0.0];  % Heel strike foot rotation
		SplineHeelStrike = spline(TimeIntervalsZ,[Speed(1) RotYHeelStrike Speed(1)]);
    end
end

% DS ends start SS
if (WalkingPhase==3)&&(time>=EndWalkingPhase)
	% enter repetitive SS phase
	WalkingPhase=4;
	stepnumber = stepnumber+1;% everytime when DS phase 3 goes to SS 4, +1 step before next calculation
	StartWalkingPhase = EndWalkingPhase;
	EndWalkingPhase =EndWalkingPhase +(TSS_end(stepnumber)-TSS_start(stepnumber));

	if stepnumber<totalnumberofsteps	
		% 2nd SS swing phase from toe off foot is canceled for stabilization reason
		if (stepnumber==2)
			RotYfootpos = [0  -(Qheel*pi/180)]; 
		else
			 RotYfootpos = [(Qtoe*pi/180)  -(Qheel*pi/180)];  % foot rotates from toe-off angle to heel strike angle to the  ground
		end
        TimeIntervalsZ = [0  TSS_end(stepnumber)-TSS_start(stepnumber)];  
        ClampedCubicSplineFootRotY = spline(TimeIntervalsZ,[Speed(1) RotYfootpos Speed(1)]);
        % this speed calculation comes from the DS duration of  previous
        % step
%         Speed(2) = 0*   (Qtoe*pi/180)/( TUA_end(stepnumber-1)-TUA_start(stepnumber-1) );
                 
        if LeftFootMoving(stepnumber)
			% 2nd SS swing phase from toe off foot is canceled for
			% stabilization reason
			if (stepnumber==2)
				rotY_fl = 0;
			else	rotY_fl = (Qtoe*pi/180);
			end
            rotZ_fl = AlpaGlobalFootL(stepnumber);

            c1=cos(rotY_fl); %Y
            s1=sin(rotY_fl);

            RY=[[c1 0 s1];[0 1 0];[-s1 0 c1]];

            c3=cos(rotZ_fl); %Z
            s3=sin(rotZ_fl);

            RZ=[[c3 -s3 0];[s3 c3 0];[0 0 1]];
             
            % foot position at toe off moment
			Pos_foot_start=[(XGlobalFootL(stepnumber)+(footankletotip) * cos(AlpaGlobalFootL(stepnumber)));(YGlobalFootL(stepnumber)+ (footankletotip) * sin(AlpaGlobalFootL(stepnumber)));0.0]+RZ*RY*[-footankletotip;0;0];
            % foot position at heel strike moment
			Pos_foot_end = [(XGlobalFootL(stepnumber+1)-(footankletoback) * cos(AlpaGlobalFootL(stepnumber+1)));(YGlobalFootL(stepnumber+1)+ (footankletoback) * sin(AlpaGlobalFootL(stepnumber+1)));0.0]+[[cos(-(Qheel*pi/180)) 0 sin(-(Qheel*pi/180))];[0 1 0];[-sin(-(Qheel*pi/180)) 0 cos(-(Qheel*pi/180))]]*[footankletoback;0;0];
            
            ZfootPos = [Pos_foot_start(3) Lift(stepnumber)  Pos_foot_end(3)];
%             TimeIntervalsZ = [0.0 0.5  1]*(EndWalkingPhase-StartWalkingPhase)   ;     
            
            XfootPos = [Pos_foot_start(1) ((XGlobalFootL(stepnumber)+XGlobalFootL(stepnumber+1))/2.0) Pos_foot_end(1)];
            YfootPos = [Pos_foot_start(2) ((YGlobalFootL(stepnumber)+YGlobalFootL(stepnumber+1))/2.0) Pos_foot_end(2)];            
            Rotfootpos = [AlpaGlobalFootL(stepnumber) AlpaGlobalFootR(stepnumber) AlpaGlobalFootL(stepnumber+1)];
            TimeIntervalsZ = [0 0.5 1]*(EndWalkingPhase-StartWalkingPhase);  
            
        
        else % right leg moving
			% 2nd SS swing phase from toe off foot is canceled for
			% stabilization reason
			if (stepnumber==2)
				rotY_fr = 0;
			else	rotY_fr = (Qtoe*pi/180);
			end
            rotZ_fr = AlpaGlobalFootR(stepnumber);

            c1=cos(rotY_fr); %Y
            s1=sin(rotY_fr);

            RY=[[c1 0 s1];[0 1 0];[-s1 0 c1]];

            c3=cos(rotZ_fr); %Z
            s3=sin(rotZ_fr);

            RZ=[[c3 -s3 0];[s3 c3 0];[0 0 1]];
            
            Pos_foot_start=[(XGlobalFootR(stepnumber)+(footankletotip) * cos(AlpaGlobalFootR(stepnumber)));(YGlobalFootR(stepnumber)+ (footankletotip) * sin(AlpaGlobalFootR(stepnumber)));0.0]+RZ*RY*[-footankletotip;0;0];
            Pos_foot_end = [(XGlobalFootR(stepnumber+1)-(footankletoback) * cos(AlpaGlobalFootR(stepnumber+1)));(YGlobalFootR(stepnumber+1)+ (footankletoback) * sin(AlpaGlobalFootR(stepnumber+1)));0.0]+[[cos(-(Qheel*pi/180)) 0 sin(-(Qheel*pi/180))];[0 1 0];[-sin(-(Qheel*pi/180)) 0 cos(-(Qheel*pi/180))]]*[footankletoback;0;0];
            
            ZfootPos = [Pos_foot_start(3) Lift(stepnumber)  Pos_foot_end(3)];
%             TimeIntervalsZ = [0.0 0.5  1]*(EndWalkingPhase-StartWalkingPhase)   ;     
            
            XfootPos = [Pos_foot_start(1) ((XGlobalFootR(stepnumber)+XGlobalFootR(stepnumber+1))/2.0) Pos_foot_end(1)];
            YfootPos = [Pos_foot_start(2) ((YGlobalFootR(stepnumber)+YGlobalFootR(stepnumber+1))/2.0) Pos_foot_end(2)];            
            Rotfootpos = [AlpaGlobalFootR(stepnumber) AlpaGlobalFootL(stepnumber) AlpaGlobalFootR(stepnumber+1)];
            TimeIntervalsZ = [0 0.5 1]*(EndWalkingPhase-StartWalkingPhase);  
            
		end
		
	% this SS is the last step	
	else %stepnumber<totalnumberofsteps
		RotYfootpos = [(Qtoe*pi/180)  0.0];  % foot rotates from toe-off angle to heel strike angle to the  ground
        TimeIntervalsZ = [0  TSS_end(stepnumber)-TSS_start(stepnumber)];  
        ClampedCubicSplineFootRotY = spline(TimeIntervalsZ,[Speed(1) RotYfootpos Speed(1)]);
                 
        if LeftFootMoving(stepnumber)
            rotY_fl = (Qtoe*pi/180);	rotZ_fl = AlpaGlobalFootL(stepnumber);
			
            c1=cos(rotY_fl); %Y
            s1=sin(rotY_fl);
            RY=[[c1 0 s1];[0 1 0];[-s1 0 c1]];
			
            c3=cos(rotZ_fl); %Z
            s3=sin(rotZ_fl);
            RZ=[[c3 -s3 0];[s3 c3 0];[0 0 1]];
             
            % foot position at toe off moment
			Pos_foot_start = [(XGlobalFootL(stepnumber)+(footankletotip) * cos(AlpaGlobalFootL(stepnumber)));(YGlobalFootL(stepnumber)+ (footankletotip) * sin(AlpaGlobalFootL(stepnumber)));0.0]+RZ*RY*[-footankletotip;0;0];
           
            ZfootPos = [Pos_foot_start(3) Lift(stepnumber)  0.0];
            TimeIntervalsZ = [0.0 0.5  1]*(EndWalkingPhase-StartWalkingPhase)   ;     
            
            XfootPos = [Pos_foot_start(1) ((XGlobalFootL(stepnumber)+XGlobalFootL(stepnumber+1))/2.0)  XGlobalFootL(stepnumber+1)];
            YfootPos = [Pos_foot_start(2) ((YGlobalFootL(stepnumber)+YGlobalFootL(stepnumber+1))/2.0)  YGlobalFootL(stepnumber+1)];            
            Rotfootpos = [AlpaGlobalFootL(stepnumber) AlpaGlobalFootR(stepnumber) AlpaGlobalFootL(stepnumber+1)];
            TimeIntervalsZ = [0 0.5 1]*(EndWalkingPhase-StartWalkingPhase);           
        
        else % right leg moving
            rotY_fr = (Qtoe*pi/180);
            rotZ_fr = AlpaGlobalFootR(stepnumber);

            c1=cos(rotY_fr); %Y
            s1=sin(rotY_fr);
            RY=[[c1 0 s1];[0 1 0];[-s1 0 c1]];

            c3=cos(rotZ_fr); %Z
            s3=sin(rotZ_fr);
            RZ=[[c3 -s3 0];[s3 c3 0];[0 0 1]];
            
            Pos_foot_start=[(XGlobalFootR(stepnumber)+(footankletotip) * cos(AlpaGlobalFootR(stepnumber)));(YGlobalFootR(stepnumber)+ (footankletotip) * sin(AlpaGlobalFootR(stepnumber)));0.0]+RZ*RY*[-footankletotip;0;0];
            
            ZfootPos = [Pos_foot_start(3) Lift(stepnumber)  0.0];
            TimeIntervalsZ = [0.0 0.5  1]*(EndWalkingPhase-StartWalkingPhase)   ;     
            
            XfootPos = [Pos_foot_start(1) ((XGlobalFootR(stepnumber)+XGlobalFootR(stepnumber+1))/2.0)  XGlobalFootR(stepnumber+1)];
            YfootPos = [Pos_foot_start(2) ((YGlobalFootR(stepnumber)+YGlobalFootR(stepnumber+1))/2.0)  YGlobalFootR(stepnumber+1)];            
            Rotfootpos = [AlpaGlobalFootR(stepnumber) AlpaGlobalFootL(stepnumber) AlpaGlobalFootR(stepnumber+1)];
            TimeIntervalsZ = [0 0.5 1]*(EndWalkingPhase-StartWalkingPhase);              
		end
		
	end
%         Speed(3) = footankletotip*Speed(2)*cos(Qtoe/180*pi);    % for z 
%         Speed(4) = footankletotip*Speed(2)*sin(Qtoe/180*pi);    % for x
        
        ClampedCubicSplineFootX = spline(TimeIntervalsZ,[Speed(1) XfootPos Speed(1)]);
        ClampedCubicSplineFootY = spline(TimeIntervalsZ,[Speed(1) YfootPos Speed(1)]);
        ClampedCubicSplineFootZ = spline(TimeIntervalsZ,[Speed(1) ZfootPos Speed(1)]);
        
        ClampedCubicSplineFootRot = spline(TimeIntervalsZ,[Speed(1) Rotfootpos Speed(1)]);
end