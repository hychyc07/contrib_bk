for j=1:length(FootxStore)-1
        if ( x(1)>=FootxStore(j) )&&( x(1)<=FootxStore(j+1) )
				Period=FootxStore(j+1) - FootxStore(j);
				w = (2*pi/Period); 
				t = x(1) -  FootxStore(j);

				A = 0.5*Atilt;
				Pz(n+1) = hip_height + A*( cos(w*t) - 1) ;
                Vz(n+1) = ( Pz(n+1)-Pz(n) )/T;
                az= ( Vz(n+1)-Vz(n) )/T;
        else
            Pz(n+1) = Pz(n);
            Vz(n+1) = Vz(n);
            az = (Vz(n+1)-Vz(n))/T;
        end
end
% output to z
z=[Pz(n) Vz(n) az]';

% drawback: sine wave is not very general compared to cubic interpolation.
% bcoz interpolation can set continueous arbitary traj