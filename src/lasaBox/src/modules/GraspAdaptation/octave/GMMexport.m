function GMMexport(srcFilename,dstFilename)

FileData    = load(srcFilename);
Priors      = FileData.Priors;
Mu          = FileData.Mu;
Sigma       = FileData.Sigma;

nbStates = size(Mu,2);
nbVars   = size(Mu,1);


myfile = fopen (dstFilename, "w");

fprintf(myfile,"%d %d\n",nbVars,nbStates);
fprintf(myfile,"%f\n",Priors);
fprintf(myfile,"%f ",Mu);
fprintf(myfile,"\n")
for i=1:nbStates
    fprintf(myfile,"%f ",Sigma(:,:,i)');
    fprintf(myfile,"\n")
end
fclose(myfile);



