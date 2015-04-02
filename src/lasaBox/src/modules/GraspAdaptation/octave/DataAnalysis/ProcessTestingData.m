function Stat = ProcessTestingData(name,mode)

filename = sprintf('data/test/%s.txt',name)
A = load(filename);


Pos = A(:,2:8);
Sen = A(:,10:12);
Nrm = A(:,19:27);
Ftp = A(:,28:87);
PosErr = A(:,88:94);
SenErr = A(:,95:97);
Cnf = A(:,98);

Ftp = Ftp(:,[4*12+[1:12] 0*12+[1:12] 1*12+[1:12]]);

% Confidence threshold
if(nargin>1)
    TCnf = sign(max(0,Cnf-exp(-0.5*2.5*2.5)));
else
    TCnf = sign(max(0,Cnf-exp(-0.5*2.0*2.0)));
end


%contact area
%contact threshold
CarThresMin = 16;
CarThresMax = 22;
Car(:,1) = sum(max(0,min(Ftp(:,0*12+[1:12]),CarThresMax)-CarThresMin) / (CarThresMax-CarThresMin),2);
Car(:,2) = sum(max(0,min(Ftp(:,1*12+[1:12]),CarThresMax)-CarThresMin) / (CarThresMax-CarThresMin),2);
Car(:,3) = sum(max(0,min(Ftp(:,2*12+[1:12]),CarThresMax)-CarThresMin) / (CarThresMax-CarThresMin),2);


% In/Out Confidence ids
CnfInIds  = find(TCnf>0);
CnfOutIds = find(TCnf==0);


% 3 finger in contact ids (within conf)
FngOnThres = 18;
FngOnIds  = find((TCnf>0)&(Sen(:,1)>FngOnThres)&(Sen(:,2)>FngOnThres)&(Sen(:,3)>FngOnThres));
Fng2OnIds  = find((TCnf>0)&(Sen(:,1)>FngOnThres)&((Sen(:,2)>FngOnThres)|(Sen(:,3)>FngOnThres)));

if (nargin>1)
    FngOn = zeros(size(A,1),3);
    FngOn(find(Sen(:,1)>FngOnThres),1) = 1;
    FngOn(find(Sen(:,2)>FngOnThres),2) = 1;
    FngOn(find(Sen(:,3)>FngOnThres),3) = 1;
    FngOnIds = find((TCnf>0)&(sum(FngOn,2)>=2));
    CnfInIds = 1:size(A,1);
    FngOnIds = find((TCnf>0)&(sum(FngOn,2)>=2));
    %FngOnIds = find((sum(FngOn,2)>=2));
    %FngOnIds = 1:size(A,1);%find((TCnf>0));%&(sum(FngOn,2)>=2));
end


% Shakiness
SPos = Smooth(Pos',5)';
Vel = diff(SPos);
Vel = [ Vel(1,:);Vel];

off = 20;
SVel = Smooth(Vel',off)';
SVel = [SVel(off/2+1:end,:);Vel(end-off/2+1:end,:)];

cnt = 1;
for alpha=1:0.02:2
    alphas(cnt)   = alpha;
    errAlpha(cnt) = sum(sum(abs(SVel*alpha-Vel).^2,2));
    cnt++;
end
[err id] = min(errAlpha);
alpha = 1;%alphas(id);
SVel = SVel * alpha;
Shak = abs(Vel-SVel)./repmat(max(abs(Vel)),size(Vel,1),1);

%max(Vel)

Stat_InRatio     = length(CnfInIds)/length(TCnf);

Stat_TimeInContact  = length(FngOnIds) / length(CnfInIds);
Stat_Time2FingerInContact  = length(Fng2OnIds) / length(CnfInIds);

Stat_MaxInPosRange(1)  = max(Pos(FngOnIds,1)) - min(Pos(FngOnIds,1));
Stat_MaxInPosRange(2)  = max(sum(Pos(FngOnIds,2:3),2)) - min(sum(Pos(FngOnIds,2:3),2));
Stat_MaxInPosRange(3)  = max(sum(Pos(FngOnIds,4:5),2)) - min(sum(Pos(FngOnIds,4:5),2));
Stat_MaxInPosRange(4)  = max(sum(Pos(FngOnIds,6:7),2)) - min(sum(Pos(FngOnIds,6:7),2));

Stat_MaxPosRange(1)  = max(Pos(:,1)) - min(Pos(:,1));
Stat_MaxPosRange(2)  = max(sum(Pos(:,2:3),2)) - min(sum(Pos(:,2:3),2));
Stat_MaxPosRange(3)  = max(sum(Pos(:,4:5),2)) - min(sum(Pos(:,4:5),2));
Stat_MaxPosRange(4)  = max(sum(Pos(:,6:7),2)) - min(sum(Pos(:,6:7),2));


Stat_MeanInPressure   = mean(mean(Sen(CnfInIds,:)));
Stat_StdInPressure    = std(mean(Sen(CnfInIds,:)));
Stat_MeanInContact    = mean(mean(Car(CnfInIds,:)));
Stat_StdInContact     = std(mean(Car(CnfInIds,:)));

Stat_MeanInPressureError     = mean(mean(abs(SenErr(CnfInIds,:))));
Stat_StdInPressureError      = std(mean(abs(SenErr(CnfInIds,:))));
Stat_MeanOutPressureError    = mean(mean(abs(SenErr(CnfOutIds,:))));
Stat_StdOutPressureError     = std(mean(abs(SenErr(CnfOutIds,:))));

Stat_MeanInPosError(1)     = mean(abs(PosErr(CnfInIds,1))) / Stat_MaxInPosRange(1);
Stat_MeanInPosError(2)     = mean(sum(abs(PosErr(CnfInIds,2:3)),2)) / Stat_MaxInPosRange(2);
Stat_MeanInPosError(3)     = mean(sum(abs(PosErr(CnfInIds,4:5)),2)) / Stat_MaxInPosRange(3);
Stat_MeanInPosError(4)     = mean(sum(abs(PosErr(CnfInIds,6:7)),2)) / Stat_MaxInPosRange(4);

Stat_StdInPosError(1)     = std(abs(PosErr(CnfInIds,1))) / Stat_MaxInPosRange(1);
Stat_StdInPosError(2)     = std(sum(abs(PosErr(CnfInIds,2:3)),2)) / Stat_MaxInPosRange(2);
Stat_StdInPosError(3)     = std(sum(abs(PosErr(CnfInIds,4:5)),2)) / Stat_MaxInPosRange(3);
Stat_StdInPosError(4)     = std(sum(abs(PosErr(CnfInIds,6:7)),2)) / Stat_MaxInPosRange(4);


Stat_MeanOutPosError(1)     = mean(abs(PosErr(CnfOutIds,1))) / Stat_MaxInPosRange(1);
Stat_MeanOutPosError(2)     = mean(sum(abs(PosErr(CnfOutIds,2:3)),2)) / Stat_MaxInPosRange(2);
Stat_MeanOutPosError(3)     = mean(sum(abs(PosErr(CnfOutIds,4:5)),2)) / Stat_MaxInPosRange(3);
Stat_MeanOutPosError(4)     = mean(sum(abs(PosErr(CnfOutIds,6:7)),2)) / Stat_MaxInPosRange(4);


Stat_StdOutPosError(1)     = std(abs(PosErr(CnfOutIds,1))) / Stat_MaxInPosRange(1);
Stat_StdOutPosError(2)     = std(sum(abs(PosErr(CnfOutIds,2:3)),2)) / Stat_MaxInPosRange(2);
Stat_StdOutPosError(3)     = std(sum(abs(PosErr(CnfOutIds,4:5)),2)) / Stat_MaxInPosRange(3);
Stat_StdOutPosError(4)     = std(sum(abs(PosErr(CnfOutIds,6:7)),2)) / Stat_MaxInPosRange(4);


Stat_MeanPosShakiness(1)     = mean(abs(Shak(CnfInIds,1)));
Stat_MeanPosShakiness(2)     = mean(sum(abs(Shak(CnfInIds,2:3)),2));
Stat_MeanPosShakiness(3)     = mean(sum(abs(Shak(CnfInIds,4:5)),2));
Stat_MeanPosShakiness(4)     = mean(sum(abs(Shak(CnfInIds,6:7)),2));

Stat_StdPosShakiness(1)     = std(abs(Shak(CnfInIds,1)));
Stat_StdPosShakiness(2)     = std(sum(abs(Shak(CnfInIds,2:3)),2));
Stat_StdPosShakiness(3)     = std(sum(abs(Shak(CnfInIds,4:5)),2));
Stat_StdPosShakiness(4)     = std(sum(abs(Shak(CnfInIds,6:7)),2));


Stat = [Stat_InRatio Stat_TimeInContact Stat_Time2FingerInContact Stat_MaxInPosRange Stat_MaxPosRange Stat_MeanInPressure Stat_StdInPressure Stat_MeanInContact Stat_StdInContact Stat_MeanInPressureError Stat_StdInPressureError ...
        Stat_MeanOutPressureError Stat_StdOutPressureError Stat_MeanInPosError Stat_StdInPosError Stat_MeanOutPosError Stat_StdOutPosError Stat_MeanPosShakiness Stat_StdPosShakiness];


