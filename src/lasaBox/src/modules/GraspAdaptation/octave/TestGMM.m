function TestGMM(name)

global GRASP_ADAPTATION_PATH
if isempty(GRASP_ADAPTATION_PATH)
    GRASP_ADAPTATION_PATH = pwd;
end

useAxis = 0;

load(sprintf("%s/%s_p_gmm.mat",GRASP_ADAPTATION_PATH,name));

A = load(sprintf('%s/%s_p.txt',GRASP_ADAPTATION_PATH,name));

Data = [A(:,2:8) A(:,10:12) A(:,19:27)];
N = size(Data,1);
Data = Data';

inC     = 11:19;
outC    =  1:10;



[y, Sigma_y] = GMR(Priors, Mu, Sigma, Data(inC,:), inC, outC);

% Orignial data + new confidence
 
figure(1);
clf;
subplot(2,1,1);
hold on;
plot(Data(1:7,:)','b');
plot(   y(1:7,:)','r');
plot(  (y(1:7,:)+repmat(2*sqrt(diag(Sigma_y(1:7,1:7))),1,size(y,2)))','g');
plot(  (y(1:7,:)-repmat(2*sqrt(diag(Sigma_y(1:7,1:7))),1,size(y,2)))','g');

subplot(2,1,2);
hold on;
plot(Data(8:10,:)','b');
plot(   y(8:10,:)','r');

plot(   (y(8:10,:)+repmat(2*sqrt(diag(Sigma_y(8:10,8:10))),1,size(y,2)))','g');
plot(   (y(8:10,:)-repmat(2*sqrt(diag(Sigma_y(8:10,8:10))),1,size(y,2)))','g');




% 3D plots

figure(2);
clf;
subplot(2,2,1);
hold on;
ids = 11:13;
plotGMM3(Mu(ids,:), Sigma(ids,ids,:));
plot3(Data(ids(1),:),Data(ids(2),:),Data(ids(3),:),'*');
if(useAxis)
    axis([-1 1 -1 1 -1 1]*1.1);
end

subplot(2,2,2);
hold on;
ids = 14:16;
plotGMM3(Mu(ids,:), Sigma(ids,ids,:));
plot3(Data(ids(1),:),Data(ids(2),:),Data(ids(3),:),'*');
if(useAxis)
    axis([-1 1 -1 1 -1 1]*1.1);
end

subplot(2,2,3);
hold on;
ids = 17:19;
plotGMM3(Mu(ids,:), Sigma(ids,ids,:));
plot3(Data(ids(1),:),Data(ids(2),:),Data(ids(3),:),'*');
if(useAxis)
    axis([-1 1 -1 1 -1 1]*1.1);
end

