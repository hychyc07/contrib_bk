global PercentUAphase totalnumberofsteps Time_startwalking T TotalStepTime xbeen ybeen zbeen rotZfoot N_L PI
global XGlobalFootL YGlobalFootL AlpaGlobalFootL XGlobalFootR YGlobalFootR AlpaGlobalFootR LeftFootMoving Lift
global PercentUAphase
global MassUpperbody MassUpperleg MassUnderleg MassFoot MassTot G
global COG_UB COG_UpperLeg COG_UnderLeg COG_foot
global IG_body_loc IG_foot_loc IG_underleg_loc IG_upperleg_loc
global ClampedCubicSplineFootX ClampedCubicSplineFootY ClampedCubicSplineFootZ ClampedCubicSplineFootRot ClampedCubicSplineFootRotY
global hip_Comy lengthupperleg lengthunderleg footankletotip footankletoside footankletoback
global time PatternOffset Endsimtime
global Qtoe Qheel
global SplineHeelStrike 

global step_width SS_store phase_store% 4 Feb 2010

% i move some environment parameters here
G  = 9.81; 


PercentUAphase=0.0; %GlobalPositionsFoot function needs it in (1-PercentUAphase)
% previously this PercentUAphase is inluded in SS phase. now I am going to
% use this in DS phase. so PercentUAphase will be about 80%