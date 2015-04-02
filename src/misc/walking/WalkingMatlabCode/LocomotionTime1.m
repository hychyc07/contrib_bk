Endsimtime =Time_startwalking + TotalStepTime + 1*Tprev;
N_L = round(Tprev/T);
time = [T:T:Endsimtime]'; % time starts at T but not 0
tsize = length(time);


WalkingPhase=0; % initial walking phase
StartWalkingPhase = 0.0;
EndWalkingPhase = Time_startwalking;
stepnumber = 1;