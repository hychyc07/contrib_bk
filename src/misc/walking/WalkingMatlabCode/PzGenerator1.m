% Updated 12 April 2010 14:34

%%%%%%%%%%%%%%%%%%%%%%%
Atilt=0.004; limit = 1e10;   %A=0.5
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if WalkingPhase==0&&n*T<=(Time_startwalking) % initial DS phase
		az = 0;
		Vz(n+1) = 0;
		Pz(n+1) = hip_height;
    
elseif (WalkingPhase==1)||(WalkingPhase==4) %% single support phase
		%scan to find out which SS interval
		[row,col]=size(SS_store);
	for j=1:col
        if ( (n*T)>=SS_store(1,j) )&&( (n*T)<=SS_store(2,j) )
				Period=SS_store(2,j)-SS_store(1,j);
				w = (2*pi/Period); % double the frequency
				t = n*T -  SS_store(1,j);
                
                % the first and last step is much slower, so lower the
                % hip height
                if (stepnumber==1)||(stepnumber==totalnumberofsteps)
                    A=0.5*Atilt;
                else A = Atilt;
                end
                Pz(n+1) = hip_height + A*( -cos(w*t) + 1) ;
                Vz(n+1) = ( Pz(n+1)-Pz(n) )/T;
                az= ( Vz(n+1)-Vz(n) )/T;
        end
	end
      
elseif WalkingPhase==3||( WalkingPhase==0&&n*T>(Time_startwalking) )
		Pz(n+1) = hip_height;
        Vz(n+1) = ( Pz(n+1)-Pz(n) )/T;
        az= ( Vz(n+1)-Vz(n) )/T;
end

az_store(n) = az;

% Euler integral
z=[Pz(n) Vz(n) az]';
%%%%%%%%%%%%