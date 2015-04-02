function Stat = ProcessTestingDataExt(name)

filename = sprintf('data/test/%s.txt',name)
A = load(filename);

filename = sprintf('data/test/fc_res/fc_%s.txt',name)
B = load(filename);



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

FC_GraspIds = find((TCnf>0)&(Sen(:,1)>FngOnThres)&(Sen(:,2)>FngOnThres)&(Sen(:,3)>FngOnThres)&B(:,1)>0);
FC_Grasp = zeros(size(A,1),1);
FC_Grasp(FC_GraspIds)=1;

if(length(FC_GraspIds)>0)
    Stat_FCTime = length(FC_GraspIds) / length(CnfInIds);
    Stat_FCQualMean = mean(B(FC_GraspIds,2));
    Stat_FCQualStd  = std(B(FC_GraspIds,2));
    Stat_FCMaxQual  = max(B(FC_GraspIds,2));

    Stat = [Stat_FCTime Stat_FCQualMean Stat_FCQualStd Stat_FCMaxQual];
else
    Stat = [0 0 0 0];
end



