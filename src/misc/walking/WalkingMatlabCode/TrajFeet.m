function [xfl,yfl,zfl,rotY_fl,rotZ_fl,xfr,yfr,zfr,rotY_fr,rotZ_fr]= TrajFeet( WalkingPhase, StartWalkingPhase, time, stepnumber)
global XGlobalFootL YGlobalFootL AlpaGlobalFootL XGlobalFootR YGlobalFootR AlpaGlobalFootR LeftFootMoving
global ClampedCubicSplineFootX ClampedCubicSplineFootY ClampedCubicSplineFootZ ClampedCubicSplineFootRot ClampedCubicSplineFootRotY footankletotip



if (WalkingPhase==1) % singles support, steunbeen
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
elseif ((WalkingPhase==2)||(WalkingPhase==3))
        [xfl,yfl,zfl,rotY_fl,rotZ_fl]=TrajStanceLeg(XGlobalFootL(stepnumber+1),YGlobalFootL(stepnumber+1),AlpaGlobalFootL(stepnumber+1));
        [xfr,yfr,zfr,rotY_fr,rotZ_fr]=TrajStanceLeg(XGlobalFootR(stepnumber+1),YGlobalFootR(stepnumber+1),AlpaGlobalFootR(stepnumber+1));
elseif (WalkingPhase==4)
    if LeftFootMoving(stepnumber)
        rotY_fl = ppval(ClampedCubicSplineFootRotY,time-StartWalkingPhase);
        rotZ_fl = AlpaGlobalFootL(stepnumber);
        
        c1=cos(rotY_fl); %Y
        s1=sin(rotY_fl);

        RY=[[c1 0 s1];[0 1 0];[-s1 0 c1]];

        c3=cos(rotZ_fl); %Z
        s3=sin(rotZ_fl);

        RZ=[[c3 -s3 0];[s3 c3 0];[0 0 1]];
        
        Pos_foot=[(XGlobalFootL(stepnumber)+(footankletotip) * cos(AlpaGlobalFootL(stepnumber)));(YGlobalFootL(stepnumber)+ (footankletotip) * sin(AlpaGlobalFootL(stepnumber)));0.0]+RZ*RY*[-footankletotip;0;0];
        
        xfl=Pos_foot(1);
        yfl=Pos_foot(2);
        zfl=Pos_foot(3);
         
        [xfr,yfr,zfr,rotY_fr,rotZ_fr]=TrajStanceLeg(XGlobalFootR(stepnumber+1),YGlobalFootR(stepnumber+1),AlpaGlobalFootR(stepnumber+1));
    else
        [xfl,yfl,zfl,rotY_fl,rotZ_fl]=TrajStanceLeg(XGlobalFootL(stepnumber+1),YGlobalFootL(stepnumber+1),AlpaGlobalFootL(stepnumber+1));

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
elseif (WalkingPhase==5) % singles support, steunbeen
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
elseif (WalkingPhase==0) 
        [xfl,yfl,zfl,rotY_fl,rotZ_fl]=TrajStanceLeg(XGlobalFootL(stepnumber),YGlobalFootL(stepnumber),AlpaGlobalFootL(stepnumber+1));
        [xfr,yfr,zfr,rotY_fr,rotZ_fr]=TrajStanceLeg(XGlobalFootR(stepnumber),YGlobalFootR(stepnumber),AlpaGlobalFootR(stepnumber+1));
end

