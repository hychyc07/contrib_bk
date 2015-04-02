% redesign a better smooth zmp trajectory
% started on 13 April 2010 09:14. 
% at 1st step, zmp from one ankle to another, max the stability
clear zmpx zmpy
XzmpToToe = 0*footankletotip;
XzmpToHeel = 0*footankletoback;

% XzmpToToe = .5*(footankletotip-footankletoback);
% XzmpToHeel = - .5*(footankletotip-footankletoback);

% 0<t<=Time_startwalking
zmpx = zeros(1,Time_startwalking/T);
zmpy =  zeros(1,0.5*Time_startwalking/T);
zmpy(end+1:Time_startwalking/T) = spline([0.5*Time_startwalking+T, Time_startwalking],  [0 [zmpy(end) YGlobalFootR(1)] 0], [0.5*Time_startwalking+T : T : Time_startwalking]);
% Time_startwalking < t
for i=1:totalnumberofsteps
	% 1st step
	if i==1 % in the first step NO toe off heel strike for stability reason
		if LeftFootMoving(i)% Right foot stance
		% SS
			zmpy( round(TSS_start(i)/T): round(TSS_end(i)/T) ) = interp1( [TSS_start(i) TSS_end(i)],[YGlobalFootR(i+1) YGlobalFootR(i+1)] ,[TSS_start(i) : T: TSS_end(i)] );
            zmpx( round(TSS_start(i)/T) : round(TSS_end(i)/T) ) = interp1( [TSS_start(i) TSS_end(i)],[XGlobalFootR(i+1)  XGlobalFootR(i+1)] ,[TSS_start(i) : T: TSS_end(i)] );
            % DS
            zmpy( round(TDS_start(i)/T): round(TDS_end(i)/T) ) = interp1( [TDS_start(i) TDS_end(i)],[YGlobalFootR(i+1) YGlobalFootL(i+1)] ,[TDS_start(i) : T: TDS_end(i)] );
            zmpx(  round(TDS_start(i)/T): round(TDS_end(i)/T) ) = interp1( [TDS_start(i) TDS_end(i)],  [XGlobalFootR(i+1)  XGlobalFootL(i+1)] ,[TDS_start(i) : T: TDS_end(i)] );
		else % Left Stance
			%SS phase
            zmpy( round(TSS_start(i)/T) : round(TSS_end(i)/T) ) = interp1( [TSS_start(i) TSS_end(i)],  [YGlobalFootL(i+1) YGlobalFootL(i+1)]  , [TSS_start(i) : T: TSS_end(i)] );
            zmpx( round(TSS_start(i)/T) : round(TSS_end(i)/T) ) = interp1( [TSS_start(i) TSS_end(i)],  [XGlobalFootL(i+1) XGlobalFootL(i+1)]  , [TSS_start(i) : T: TSS_end(i)] );
            % DS phase
            zmpy( round(TDS_start(i)/T) : round(TDS_end(i)/T) ) = interp1( [TDS_start(i) TDS_end(i)],  [YGlobalFootL(i+1) YGlobalFootR(i+1)]  ,[TDS_start(i) : T: TDS_end(i)] );
            zmpx( round(TDS_start(i)/T) : round(TDS_end(i)/T) ) = interp1( [TDS_start(i) TDS_end(i)],  [XGlobalFootL(i+1), XGlobalFootR(i+1)]  , [TDS_start(i) : T: TDS_end(i)] );
		end

	elseif  i==2
		if LeftFootMoving(i)% Right foot stance
		% SS
			zmpy( round(TSS_start(i)/T) : round(TSS_end(i)/T) ) = interp1( [TSS_start(i) TSS_end(i)],[YGlobalFootR(i+1) YGlobalFootR(i+1)] ,[TSS_start(i) : T: TSS_end(i)] );
            zmpx( round(TSS_start(i)/T) : round(TSS_end(i)/T) ) = interp1( [TSS_start(i) TSS_end(i)],[XGlobalFootR(i+1)  XGlobalFootR(i+1)+XzmpToToe] ,[TSS_start(i) : T: TSS_end(i)] );
            % DS
            zmpy( round(TDS_start(i)/T) : round(TDS_end(i)/T) ) = interp1( [TDS_start(i) TDS_end(i)],[YGlobalFootR(i+1) YGlobalFootL(i+1)] ,[TDS_start(i) : T: TDS_end(i)] );
            zmpx(  round(TDS_start(i)/T) : round(TDS_end(i)/T) ) = interp1( [TDS_start(i) TDS_end(i)],  [XGlobalFootR(i+1)+XzmpToToe  XGlobalFootL(i+1)-XzmpToHeel] ,[TDS_start(i) : T: TDS_end(i)] );
		else % Left Stance
			%SS phase
            zmpy( round(TSS_start(i)/T): round(TSS_end(i)/T) ) = interp1( [TSS_start(i) TSS_end(i)],  [YGlobalFootL(i+1) YGlobalFootL(i+1)]  , [TSS_start(i) : T: TSS_end(i)] );
            zmpx( round(TSS_start(i)/T) : round(TSS_end(i)/T) ) = interp1( [TSS_start(i) TSS_end(i)],  [XGlobalFootL(i+1) XGlobalFootL(i+1)+XzmpToToe]  , [TSS_start(i) : T: TSS_end(i)] );
            % DS phase
            zmpy( round(TDS_start(i)/T) : round(TDS_end(i)/T) ) = interp1( [TDS_start(i) TDS_end(i)],  [YGlobalFootL(i+1) YGlobalFootR(i+1)]  ,[TDS_start(i) : T: TDS_end(i)] );
            zmpx( round(TDS_start(i)/T) : round(TDS_end(i)/T) ) = interp1( [TDS_start(i) TDS_end(i)],  [XGlobalFootL(i+1)+XzmpToToe, XGlobalFootR(i+1)-XzmpToHeel]  , [TDS_start(i) : T: TDS_end(i)] );
		end
	% repetative toe off heel strike cycle
	elseif  i>2 && i<totalnumberofsteps 
		if LeftFootMoving(i)% Right foot stance
		% SS
			zmpy( round(TSS_start(i)/T) : round(TSS_end(i)/T) ) = interp1( [TSS_start(i) TSS_end(i)],[YGlobalFootR(i+1) YGlobalFootR(i+1)] ,[TSS_start(i) : T: TSS_end(i)] );
            zmpx( round(TSS_start(i)/T) : round(TSS_end(i)/T) ) = interp1( [TSS_start(i) TSS_end(i)],[XGlobalFootR(i+1)-XzmpToHeel  XGlobalFootR(i+1)+XzmpToToe] ,[TSS_start(i) : T: TSS_end(i)] );
            % DS
            zmpy( round(TDS_start(i)/T) : round(TDS_end(i)/T) ) = interp1( [TDS_start(i) TDS_end(i)],[YGlobalFootR(i+1) YGlobalFootL(i+1)] ,[TDS_start(i) : T: TDS_end(i)] );
            zmpx(  round(TDS_start(i)/T) : round(TDS_end(i)/T) ) = interp1( [TDS_start(i) TDS_end(i)],  [XGlobalFootR(i+1)+XzmpToToe  XGlobalFootL(i+1)-XzmpToHeel] ,[TDS_start(i) : T: TDS_end(i)] );
		else % Left Stance
			%SS phase
            zmpy( round(TSS_start(i)/T): round(TSS_end(i)/T) ) = interp1( [TSS_start(i) TSS_end(i)],  [YGlobalFootL(i+1) YGlobalFootL(i+1)]  , [TSS_start(i) : T: TSS_end(i)] );
            zmpx( round(TSS_start(i)/T) : round(TSS_end(i)/T) ) = interp1( [TSS_start(i) TSS_end(i)],  [XGlobalFootL(i+1)-XzmpToHeel XGlobalFootL(i+1)+XzmpToToe]  , [TSS_start(i) : T: TSS_end(i)] );
            % DS phase
            zmpy( round(TDS_start(i)/T) : round(TDS_end(i)/T) ) = interp1( [TDS_start(i) TDS_end(i)],  [YGlobalFootL(i+1) YGlobalFootR(i+1)]  ,[TDS_start(i) : T: TDS_end(i)] );
            zmpx( round(TDS_start(i)/T) : round(TDS_end(i)/T) ) = interp1( [TDS_start(i) TDS_end(i)],  [XGlobalFootL(i+1)+XzmpToToe, XGlobalFootR(i+1)-XzmpToHeel]  , [TDS_start(i) : T: TDS_end(i)] );
		end
	% Last Step especially treated
	elseif i==totalnumberofsteps
		if LeftFootMoving(i)% Right foot stance
              % SS
              zmpy( round(TSS_start(i)/T) : round(TSS_end(i)/T) ) = interp1( [TSS_start(i) TSS_end(i)], [YGlobalFootR(i+1)  YGlobalFootR(i+1)]  ,[TSS_start(i) : T: TSS_end(i)] );
              zmpx( round(TSS_start(i)/T) : round(TSS_end(i)/T) ) = interp1( [TSS_start(i) TSS_end(i)],  [XGlobalFootR(i+1)-XzmpToHeel  XGlobalFootR(i+1)]  ,[TSS_start(i) : T: TSS_end(i)] );
              % DS
              zmpy( round(TDS_start(i)/T) : round(TDS_end(i)/T) ) = interp1( [TDS_start(i) TDS_end(i)],  [ YGlobalFootR(i+1) .5*(YGlobalFootR(end)+YGlobalFootL(end)) ]  ,[TDS_start(i) : T: TDS_end(i)] );
              zmpx(  round(TDS_start(i)/T) : round(TDS_end(i)/T) ) = interp1( [TDS_start(i) TDS_end(i)],  [ XGlobalFootR(i+1), 0.5*(XGlobalFootR(end)+XGlobalFootL(end)) ]  ,[TDS_start(i) : T: TDS_end(i)] );
	      
		else % left foot stance, Right swing
            % SS
            zmpy( round(TSS_start(i)/T) : round(TSS_end(i)/T) ) = interp1( [TSS_start(i) TSS_end(i)], [YGlobalFootL(i+1) YGlobalFootL(i+1)]  ,[TSS_start(i) : T: TSS_end(i)] );
            zmpx( round(TSS_start(i)/T) : round(TSS_end(i)/T) ) = interp1( [TSS_start(i) TSS_end(i)], [XGlobalFootL(i+1)-XzmpToHeel  XGlobalFootL(i+1)]  ,[TSS_start(i) : T: TSS_end(i)] );
            % DS
            zmpy( round(TDS_start(i)/T) : round(TDS_end(i)/T) ) = interp1( [TDS_start(i) TDS_end(i)], [ YGlobalFootL(i+1), 0.5*(YGlobalFootR(end)+YGlobalFootL(end)) ]  ,[TDS_start(i) : T: TDS_end(i)] );
            zmpx(  round(TDS_start(i)/T) : round(TDS_end(i)/T) ) = interp1( [TDS_start(i) TDS_end(i)], [ XGlobalFootL(i+1), 0.5*(XGlobalFootR(end)+XGlobalFootL(end)) ]  ,[TDS_start(i) : T: TDS_end(i)] );
		end
	end
end

%% rest
zmpx(end+1:tsize)=zmpx(end);
zmpy(end+1:tsize)=zmpy(end);

zmpx=zmpx';
zmpy=zmpy';