function [Priors, Mu, Sigma, lik] = ProbEM(Data, DataCov, Priors0, Mu0, Sigma0,maxIter)
%
% This function learns the parameters of a Gaussian Mixture Model 
% (GMM) using a recursive Expectation-Maximization (EM) algorithm, starting 
% from an initial estimation of the parameters.
%
% Inputs -----------------------------------------------------------------
%   o Data:    D x N array representing N datapoints of D dimensions.
%   o DataCov: D x D x N array representing covariance of N datapoints 
%              of D dimensions.
%   o Priors0: 1 x K array representing the initial prior probabilities 
%              of the K GMM components.
%   o Mu0:     D x K array representing the initial centers of the K GMM 
%              components.
%   o Sigma0:  D x D x K array representing the initial covariance matrices 
%              of the K GMM components.
% Outputs ----------------------------------------------------------------
%   o Priors:  1 x K array representing the prior probabilities of the K GMM 
%              components.
%   o Mu:      D x K array representing the centers of the K GMM components.
%   o Sigma:   D x D x K array representing the covariance matrices of the 
%              K GMM components.
%
% Copyright (c) 2006 Sylvain Calinon, LASA Lab, EPFL, CH-1015 Lausanne,
%               Switzerland, http://lasa.epfl.ch
%
% The program is free for non-commercial academic use. 
% Please contact the authors if you are interested in using the 
% software for commercial purposes. The software must not be modified or 
% distributed without prior permission of the authors.
% Please acknowledge the authors in any academic publications that have 
% made use of this code or part of it. Please use this BibTex reference: 
% 
% @article{Calinon06SMC,
%   title="On Learning, Representing and Generalizing a Task in a Humanoid 
%     Robot",
%   author="S. Calinon and F. Guenter and A. Billard",
%   journal="IEEE Transactions on Systems, Man and Cybernetics, Part B. 
%     Special issue on robot learning by observation, demonstration and 
%     imitation",
%   year="2006",
%   volume="36",
%   number="5"
% }

%% Criterion to stop the EM iterative update
loglik_threshold = 1e-10;

%% Initialization of the parameters
[nbVar, nbData] = size(Data);
nbStates = size(Sigma0,3);
loglik_old = -realmax;
nbStep = 0;

Mu = Mu0;
Sigma = Sigma0;
Priors = Priors0;

disp("EM"); fflush(stdout);

dataMC = zeros(nbVar, nbVar, nbData);
for i=1:nbData
    dataMC(:,:,i) = Data(:,i)*Data(:,i)'+DataCov(:,:,i);
end

%% EM fast matrix computation (see the commented code for a version 
%% involving one-by-one computation, which is easier to understand)
while 1
  if(nbStep>0)
      disp(sprintf("  iteration: %02d   - logLik %f",nbStep,loglik_old)); fflush(stdout);
  else
      disp(sprintf("  iteration: %02d",nbStep)); fflush(stdout);
  end
  %% E-step %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  for i=1:nbStates
    %Compute probability p(x|i)
    Pxi(:,i) = gaussPDF(Data, Mu(:,i), Sigma(:,:,i));
  end

  %Compute posterior probability p(i|x)
  Pix_tmp = repmat(Priors,[nbData 1]).*Pxi;
  %Pix = Pix_tmp ./ repmat(sum(Pix_tmp,2),[1 nbStates]);
  Pix = Pix_tmp;

  % Update posterior probability p(i|x) using data covariance
  for i=1:nbStates
    invSigma = inv(Sigma(:,:,i));
    for j=1:nbData
      Pix(j,i) = Pix(j,i)*exp(-0.5*trace(invSigma*DataCov(:,:,j)));
    end
  end
  Pix = Pix_tmp ./ repmat(sum(Pix_tmp,2),[1 nbStates]);
  %size(Pix)
  %return

  %Compute cumulated posterior probability
  E = sum(Pix);
  %% M-step %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  for i=1:nbStates
    %Update the priors
    Priors(i) = E(i) / nbData;
    %Update the centers
    Mu(:,i) = Data*Pix(:,i) / E(i);
    %Update the covariance matrices

    Data_tmp2a = repmat(reshape(Pix(:,i),[1 1 nbData]), [nbVar nbVar 1]);

    Sigma(:,:,i) = (sum(dataMC.*Data_tmp2a, 3) / E(i)) - Mu(:,i)*Mu(:,i)';

  end
  %% Stopping criterion %%%%%%%%%%%%%%%%%%%%
  for i=1:nbStates
    %Compute the new probability p(x|i)
    Pxi(:,i) = gaussPDF(Data, Mu(:,i), Sigma(:,:,i));
  end
  %Compute the log likelihood
  F = Pxi*Priors';
  F(find(F<realmin)) = realmin;
  loglik = mean(log(F));
  %Stop the process depending on the increase of the log likelihood 
  if abs((loglik/loglik_old)-1) < loglik_threshold
    break;
  end
  loglik_old = loglik;
  nbStep = nbStep+1;
  if(nbStep>maxIter)
    break;
  end
end

disp(sprintf("  Final logLik %f",loglik_old)); fflush(stdout);

lik = loglik;

% %% EM slow one-by-one computation (better suited to understand the
% %% algorithm) 
% while 1
%   %% E-step %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   for i=1:nbStates
%     %Compute probability p(x|i)
%     Pxi(:,i) = gaussPDF(Data, Mu(:,i), Sigma(:,:,i));
%   end
%   %Compute posterior probability p(i|x)
%   for j=1:nbData
%     Pix(j,:) = (Priors.*Pxi(j,:))./(sum(Priors.*Pxi(j,:))+realmin);
%   end
%   %Compute cumulated posterior probability
%   E = sum(Pix);
%   %% M-step %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   for i=1:nbStates
%     %Update the priors
%     Priors(i) = E(i) / nbData;
%     %Update the centers
%     Mu(:,i) = Data*Pix(:,i) / E(i);
%     %Update the covariance matrices 
%     covtmp = zeros(nbVar,nbVar);
%     for j=1:nbData
%       covtmp = covtmp + (Data(:,j)-Mu(:,i))*(Data(:,j)-Mu(:,i))'.*Pix(j,i);
%     end
%     Sigma(:,:,i) = covtmp / E(i);
%   end
%   %% Stopping criterion %%%%%%%%%%%%%%%%%%%%
%   for i=1:nbStates
%     %Compute the new probability p(x|i)
%     Pxi(:,i) = gaussPDF(Data, Mu(:,i), Sigma(:,:,i));
%   end
%   %Compute the log likelihood
%   F = Pxi*Priors';
%   F(find(F<realmin)) = realmin;
%   loglik = mean(log(F));
%   %Stop the process depending on the increase of the log likelihood 
%   if abs((loglik/loglik_old)-1) < loglik_threshold
%     break;
%   end
%   loglik_old = loglik;
%   nbStep = nbStep+1;
% end

%% Add a tiny variance to avoid numerical instability
%for i=1:nbStates
%  Sigma(:,:,i) = Sigma(:,:,i) + 1E-5.*diag(ones(nbVar,1));
%end

