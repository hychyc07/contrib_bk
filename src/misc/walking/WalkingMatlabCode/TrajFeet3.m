function [xfl,yfl,zfl,rotY_fl,rotZ_fl,xfr,yfr,zfr,rotY_fr,rotZ_fr]= TrajFeet3( WalkingPhase, StartWalkingPhase, time, stepnumber , TUA_end)
global XGlobalFootL YGlobalFootL AlpaGlobalFootL XGlobalFootR YGlobalFootR AlpaGlobalFootR LeftFootMoving
global ClampedCubicSplineFootX ClampedCubicSplineFootY ClampedCubicSplineFootZ ClampedCubicSplineFootRot ClampedCubicSplineFootRotY footankletotip
global totalnumberofsteps footankletoback 
global SplineHeelStrike 

if (WalkingPhase==0) 
    if stepnumber>=totalnumberofsteps % stop at final step
        [xfl,yfl,zfl,rotY_fl,rotZ_fl]=TrajStanceLeg(XGlobalFootL(end),YGlobalFootL(end),AlpaGlobalFootL(end));
        [xfr,yfr,zfr,rotY_fr,rotZ_fr]=TrajStanceLeg(XGlobalFootR(end),YGlobalFootR(end),AlpaGlobalFootR(end));
	else % at the beginning
        [xfl,yfl,zfl,rotY_fl,rotZ_fl]=TrajStanceLeg(XGlobalFootL(stepnumber),YGlobalFootL(stepnumber),AlpaGlobalFootL(stepnumber+1));
        [xfr,yfr,zfr,rotY_fr,rotZ_fr]=TrajStanceLeg(XGlobalFootR(stepnumber),YGlobalFootR(stepnumber),AlpaGlobalFootR(stepnumber+1));
    end

elseif (WalkingPhase==1) % singles support, steunbeen
    if LeftFootMoving(stepnumber)            
        xfl = ppval(ClampedCubicSplineFootX,time-StartWalkingPhase);
        yfl = ppval(ClampedCubicSplineFootY,time-StartWalkingPhase);
        zfl = ppval(ClampedCubicSplineFootZ,time-StartWalkingPhase);
        rotZ_fl = ppval(ClampedCubicSplineFootRot,time-StartWalkingPhase);
        rotY_fl = 0.0;
        [xfr,yfr,zfr,rotY_fr,rotZ_fr]=TrajStanceLeg(XGlobalFootR(stepnumber+1),YGlobalFootR(stepnumber+1),AlpaGlobalFootR(stepnumber+1));
    else
        [xfl,yfl,zfl,rotY_fl,rotZ_fl]=TrajStanceLeg(XGlobalFootL(stepnumber+1),YGlobalFootL(stepnumber+1),AlpaGlobalFootL(stepnumber+1));
        xfr = ppval(ClampedCubicSplineFootX,time-StartWalkingPhase);
        yfr = ppval(ClampedCubicSplineFootY,time-StartWalkingPhase);
        zfr = ppval(ClampedCubicSplineFootZ,time-StartWalkingPhase);
        rotY_fr = 0.0;
        rotZ_fr = ppval(ClampedCubicSplineFootRot,time-StartWalkingPhase);
    end
elseif (WalkingPhase==3)
	% Left toe off,  Right Heel strike 
    if ~LeftFootMoving(stepnumber) 
        % Left toe off foot trajectory
		rotY_fl = ppval(ClampedCubicSplineFootRotY,time-StartWalkingPhase);
        rotZ_fl = AlpaGlobalFootL(stepnumber+1);
        
        c1=cos(rotY_fl); %Y
        s1=sin(rotY_fl);

        RY=[[c1 0 s1];[0 1 0];[-s1 0 c1]];

        c3=cos(rotZ_fl); %Z
        s3=sin(rotZ_fl);

        RZ=[[c3 -s3 0];[s3 c3 0];[0 0 1]];
        
        Pos_foot=[(XGlobalFootL(stepnumber)+(footankletotip) * cos(AlpaGlobalFootL(stepnumber)));(YGlobalFootL(stepnumber)+ (footankletotip) * sin(AlpaGlobalFootL(stepnumber)));0.0]+RZ*RY*[-footankletotip;0;0];
        % in toe off period, foot position xyz totally depends on foot
        % rotation
        xfl=Pos_foot(1);
        yfl=Pos_foot(2);
        zfl=Pos_foot(3);
		
		% Right Heel strike foot trajectory
		if time<=TUA_end(1) 
% 			'1st DS no Heel strike'
			[xfr,yfr,zfr,rotY_fr,rotZ_fr]=TrajStanceLeg(XGlobalFootR(stepnumber+1),YGlobalFootR(stepnumber+1),AlpaGlobalFootR(stepnumber+1));
		else
        % right foot heel strike
			rotY_fr= ppval(SplineHeelStrike,time-StartWalkingPhase);		

			xfr=XGlobalFootR(stepnumber+1)-footankletoback*(1-cos(rotY_fr));
			yfr=YGlobalFootR(stepnumber+1);
			zfr= footankletoback * sin(-rotY_fr);
			rotZ_fr =AlpaGlobalFootR(stepnumber+1);
		end
    
	else % Right toe off  Left Heel strike
		% Left Heel strike Foot Trajectory
		if time<=TUA_end(1) 
% 			'1st DS no Heel strike'
			[xfl,yfl,zfl,rotY_fl,rotZ_fl]=TrajStanceLeg(XGlobalFootL(stepnumber+1),YGlobalFootL(stepnumber+1),AlpaGlobalFootL(stepnumber+1));
		else
			rotY_fl= ppval(SplineHeelStrike,time-StartWalkingPhase);
			xfl=XGlobalFootL(stepnumber+1)-footankletoback*(1-cos(rotY_fl));
			yfl=YGlobalFootL(stepnumber+1);
			zfl= footankletoback * sin(-rotY_fl);
			rotZ_fl =AlpaGlobalFootL(stepnumber+1);
		end
		
		% Right Toe Off Foot Trajectory
		rotY_fr = ppval(ClampedCubicSplineFootRotY,time-StartWalkingPhase);
        rotZ_fr = AlpaGlobalFootR(stepnumber);
        
        c1=cos(rotY_fr); %Y
        s1=sin(rotY_fr);

        RY=[[c1 0 s1];[0 1 0];[-s1 0 c1]];

        c3=cos(rotZ_fr); %Z
        s3=sin(rotZ_fr);

        RZ=[[c3 -s3 0];[s3 c3 0];[0 0 1]];
        
        Pos_foot=[(XGlobalFootR(stepnumber)+(footankletotip) * cos(AlpaGlobalFootR(stepnumber)));(YGlobalFootR(stepnumber)+ (footankletotip) * sin(AlpaGlobalFootR(stepnumber)));0.0]+RZ*RY*[-footankletotip;0;0];
        xfr=Pos_foot(1);
        yfr=Pos_foot(2);
        zfr=Pos_foot(3);
	end
	
elseif (WalkingPhase==4) % singles support
    if LeftFootMoving(stepnumber)            
        xfl = ppval(ClampedCubicSplineFootX,time-StartWalkingPhase);
        yfl = ppval(ClampedCubicSplineFootY,time-StartWalkingPhase);
        zfl = ppval(ClampedCubicSplineFootZ,time-StartWalkingPhase);
        rotZ_fl = ppval(ClampedCubicSplineFootRot,time-StartWalkingPhase);
        rotY_fl = ppval(ClampedCubicSplineFootRotY,time-StartWalkingPhase);
        [xfr,yfr,zfr,rotY_fr,rotZ_fr]=TrajStanceLeg(XGlobalFootR(stepnumber+1),YGlobalFootR(stepnumber+1),AlpaGlobalFootR(stepnumber+1));
    else
        [xfl,yfl,zfl,rotY_fl,rotZ_fl]=TrajStanceLeg(XGlobalFootL(stepnumber+1),YGlobalFootL(stepnumber+1),AlpaGlobalFootL(stepnumber+1));
        xfr = ppval(ClampedCubicSplineFootX,time-StartWalkingPhase);
        yfr = ppval(ClampedCubicSplineFootY,time-StartWalkingPhase);
        zfr = ppval(ClampedCubicSplineFootZ,time-StartWalkingPhase);
        rotY_fr = ppval(ClampedCubicSplineFootRotY,time-StartWalkingPhase);
        rotZ_fr = ppval(ClampedCubicSplineFootRot,time-StartWalkingPhase);
    end    
end