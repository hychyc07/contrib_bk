function LearnModel(name)

global GRASP_ADAPTATION_PATH
if isempty(GRASP_ADAPTATION_PATH)
    GRASP_ADAPTATION_PATH = pwd;
end

A = load(sprintf('%s/%s_p.txt',GRASP_ADAPTATION_PATH,name));

GScale = 100;
Data = [A(:,2:8) A(:,10:12) A(:,19:27)*GScale];

Data = Data(1:3:end,:);

N = size(Data,1)
M = size(Data,2);

Data = Data';

DataCov = zeros(M,M,N);
for i=1:N
    DataCov(:,:,i) = eye(M,M)*1;
    DataCov(11:19,11:19,i) *= (GScale*0.01)^2;
end

nbStates = 3;
[Priors, Mu, Sigma]  = EM_init_kmeans(Data, nbStates);
[Priors, Mu, Sigma]  = ProbEM(Data, DataCov,Priors, Mu, Sigma,30);


% downscaling

K = size(Sigma,3);
D = size(Sigma,1);

for k=1:K

    scale = [ones(7+3,1);ones(9,1)./GScale];
    scaleM = repmat(scale,1,D);

    Mu(:,k) = scale.*Mu(:,k);
    dd = zeros(D,D);
    for i=1:3
        n = Mu(D-9+[1:3]+(i-1)*3,k);
        %[1:3]+(i-1)*3
        %n
        n = n / sqrt(n'*n);
        %D-9+[1:3]+(i-1)*3
        dd(D-9+[1:3]+(i-1)*3,D-9+[1:3]+(i-1)*3) = 0.001*n*n';
    end

    [V Dg]  = eig(Sigma(:,:,k));
    
    V = V.*scaleM;

    Sigma(:,:,k) = V*Dg*V';
end



save(sprintf("%s/%s_p_gmm.mat",GRASP_ADAPTATION_PATH,name),"Priors","Mu","Sigma");
GMMexport(sprintf("%s/%s_p_gmm.mat",GRASP_ADAPTATION_PATH,name),sprintf("%s/%s_p_gmm.txt",GRASP_ADAPTATION_PATH,name));    

