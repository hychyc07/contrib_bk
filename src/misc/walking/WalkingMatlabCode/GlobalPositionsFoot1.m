% Updated: 25 March 2010
% XGlobalFootR/L and YGlobalFootR/L are the feet coordinates of right and
% left foot w.r.t the global reference frame during walking. They are not
% trajectory points but the coordinates of the points where the feet touch
% the ground (foothold position).

function [XGlobalFootL, YGlobalFootL, AlpaGlobalFootL, XGlobalFootR, YGlobalFootR, AlpaGlobalFootR, TSS_start, TDS_start, TSS_end, TDS_end, TUA_start, TUA_end] = GlobalPositionsFoot(LambdaX,LambdaY,alpha_foot,Nu,Lift,wait, LeftFootMoving)

global PercentUAphase totalnumberofsteps Time_startwalking TotalStepTime hip_Comy
% i add this
global step_width SS_store T

RestLengthBetweenLegs = step_width*hip_Comy*2.0;


PercentSS = 0.4;                                                           % @@@ percentage of single support Original 0.8
PercentDS = 1-PercentSS;                                                   % percentage of double support

XGlobalFootL(1)     =  0.0;
YGlobalFootL(1)     =  RestLengthBetweenLegs/2;
AlpaGlobalFootL(1)  =  0.0;                                                
XGlobalFootR(1)     =  0.0;
YGlobalFootR(1)     = -RestLengthBetweenLegs/2;
AlpaGlobalFootR(1)  =  0.0; 

for i=1:totalnumberofsteps
    if LeftFootMoving(i)                                                   % if left moving, right remains the same
        XGlobalFootR(i+1) = XGlobalFootR(i);
        YGlobalFootR(i+1) = YGlobalFootR(i);
        AlpaGlobalFootR(i+1) = AlpaGlobalFootR(i); 
        
        % this is the way to determine next foothold position
        XGlobalFootL(i+1) = XGlobalFootR(i) + cos(AlpaGlobalFootR(i))*LambdaX(i) - sin(AlpaGlobalFootR(i))*LambdaY(i);
        YGlobalFootL(i+1) = YGlobalFootR(i) + sin(AlpaGlobalFootR(i))*LambdaX(i) + cos(AlpaGlobalFootR(i))*LambdaY(i);
        AlpaGlobalFootL(i+1) = AlpaGlobalFootR(i) + alpha_foot(i);         % global angle, addition of all relative angles
                
        if (i==1)
            TSS_start(1) = Time_startwalking;
        else
            TSS_start(i) = TDS_end(i-1);
            % the end of 1st DS is the start of 2nd SS
        end
        
        T_cycle = sqrt( (XGlobalFootL(i+1)-XGlobalFootL(i))^2 + (YGlobalFootL(i+1)-YGlobalFootL(i))^2)/Nu(i);
        TSS_end(i) = TSS_start(i) + T_cycle*PercentSS*0.5;
        TDS_start(i) = TSS_end(i);
        
        TUA_start(i) = TDS_start(i) + 0.0*T_cycle*PercentDS*0.5;
        
        TUA_end(i) = TDS_start(i) + T_cycle*PercentDS*0.5;
        
        TDS_end(i) = TDS_start(i) + T_cycle*PercentDS*.5 + wait(i);
        clear T_cycle;
    else
        XGlobalFootL(i+1) = XGlobalFootL(i);
        YGlobalFootL(i+1) = YGlobalFootL(i);
        AlpaGlobalFootL(i+1) = AlpaGlobalFootL(i); 
        
        XGlobalFootR(i+1) = XGlobalFootL(i) + cos(AlpaGlobalFootL(i))*LambdaX(i) - sin(AlpaGlobalFootL(i))*LambdaY(i);
        YGlobalFootR(i+1) = YGlobalFootL(i) + sin(AlpaGlobalFootL(i))*LambdaX(i) + cos(AlpaGlobalFootL(i))*LambdaY(i);
        AlpaGlobalFootR(i+1) = AlpaGlobalFootL(i) + alpha_foot(i);        

        if (i==1)
            TSS_start(1) = Time_startwalking;
        else
            TSS_start(i) = TDS_end(i-1);
        end
        
        T_cycle = sqrt( (XGlobalFootR(i+1)-XGlobalFootR(i))^2 + (YGlobalFootR(i+1)-YGlobalFootR(i))^2)/Nu(i) ;
        
        if (i==totalnumberofsteps)
            TSS_end(i) = TSS_start(i) + T_cycle*PercentSS;                 % It's 0.5 since it T_cycle is for one full strike (two steps)
        else
            TSS_end(i) = TSS_start(i) + T_cycle*PercentSS*.5;
        end
            
        TDS_start(i) = TSS_end(i);
        
        TUA_start(i) = TDS_start(i) + 0.0*T_cycle*PercentDS*0.5;
        
        TUA_end(i) = TDS_start(i) + T_cycle*PercentDS*0.5;
        
        TDS_end(i) = TDS_start(i) + T_cycle*PercentDS*.5 + wait(i);
        clear T_cycle;
    
    end % if    
end % for

%%% convert the original TSS TDS timing, make them dividable by sampling
%%% time T
TSS_start = T * round(TSS_start/T);
TSS_end   = T * round(TSS_end/T);
TDS_start = T * round(TDS_start/T);
TDS_end   = T * round(TDS_end/T);
TUA_start = T * round(TUA_start/T);
TUA_end   = T * round(TUA_end/T);
%%%

TotalStepTime = TDS_end(i)  % display the total time
% Feb 4 2010
SS_store = [TSS_start;TSS_end];