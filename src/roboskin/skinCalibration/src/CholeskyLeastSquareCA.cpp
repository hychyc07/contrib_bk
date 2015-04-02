#include "iCub/skinCalibration/CholeskyLeastSquareCA.h"
#include <math.h>
using namespace iCub::skinCalibration;
using namespace std;

CholeskyLeastSquareCA::~CholeskyLeastSquareCA(){
	SkinCalibrationLogger::writeMsg("[CholeskyLeastSquareCA] Removing algorithm\n");
	if(initialized){
		ofstream file;
		map<long int, Matrix *>::iterator it;
		file.open(filefullpath.c_str());
		for(it = R.begin(); it!=R.end();){
			SkinCalibrationLogger::writeMsg("[CholeskyLeastSquareCA] Writing R matrix for taxel %i\n",it->first);
			if(it->second!='\0'){
				file<<it->first;
				for(int i=0; i<9; i++){
					file<<" "<<(*it->second)(i/3,i-(3*(i/3)));
				}
				delete it->second;
				it++;
				if(it!=R.end())
					file<<endl;
			}
		}
		file.close();
	}
}

bool CholeskyLeastSquareCA::parseFile(string filename){
	ifstream file;
	file.open(filename.c_str());
	if(file.peek() == ifstream::traits_type::eof()) //The file is empty
		return true;
	while(!file.eof()){
		long int id;
		file>>id;
		if(file.fail())
			return false;
		R[id] = new Matrix(3,3);
		for(int i =0;i <9; i++){
			file>>(*R[id])(i/3,i-(3*(i/3)));
			if(file.fail())
				return false;
		}
	}
	return true;
}

void CholeskyLeastSquareCA::skewSymmetric(Vector *v,Matrix *M){
	M->resize(3,3);
	M->zero();
	(*M)(0,1) = -(*v)[2];
	(*M)(0,2) = (*v)[1];
	(*M)(1,0) = (*v)[2];
	(*M)(1,2) = -(*v)[0];
	(*M)(2,0) = -(*v)[1];
	(*M)(2,1) = (*v)[0];
}

Matrix CholeskyLeastSquareCA::computeWeightMatrix(CalibrationSample *cs){
	Matrix K(3,3);
	Vector diag(3);
	double fnorm = norm(*cs->force);
	map<long int, double>::const_iterator it;
	double max = 0;
	for(it = cs->taxels_measures->begin(); it!=cs->taxels_measures->end(); it++){
		if(it->second>max)
			max = it->second;
	}
	diag[0]=sqrt((*cs->taxels_measures)[cs->taxel_id]/max);
	diag[1]=diag[0];
	diag[2]=diag[0];
	return K.diagonal(diag);
}

bool CholeskyLeastSquareCA::init(ResourceFinder *rf, BodyPart robotBodyPart, SkinPart skinPart, map<long int, Vector> *initial_estimate){
	SkinCalibrationLogger::writeMsg("[CholeskyLeastSquareCA] Init\n");
	string filename = "";
	
	this->skinPart = skinPart;
	this->robotBodyPart = robotBodyPart;
	
	yarp::os::Property config;
	filename = "CholeskyLeastSquareCA.ini";
	filefullpath = rf->findFile(filename.c_str());
	if(filefullpath.empty()){
		SkinCalibrationLogger::writeMsg("[CholeskyLeastSquareCA] Config file not found, aborting!\n");
		initialized = false;
		return initialized;
	}

	config.fromConfigFile(filefullpath.c_str());
	if(!config.check("UsePreviousInformation")){
		UsePreviousInformation = false;
	}
	else{
		UsePreviousInformation = (config.find("UsePreviousInformation").asInt() == 1);
	}

	config.fromConfigFile(filefullpath.c_str());
	if(!config.check("UseWeightMatrix")){
		UseWeightMatrix = false;
	}
	else{
		UseWeightMatrix = (config.find("UseWeightMatrix").asInt() == 1);
	}

	
		filename = "CholeskyLeastSquareCA_";
		filename += BodyPart_s[robotBodyPart]+"_"+ SkinPart_s[skinPart];
		filefullpath = rf->findFile(filename.c_str());
		if(filefullpath.empty()){ //if we don't have any previuos information
			SkinCalibrationLogger::writeMsg("[CholeskyLeastSquareCA] Previous information not found!\n");
			ofstream fileo;
			filefullpath = rf->getContextPath();
			filefullpath += "/";
			filefullpath += filename;
			SkinCalibrationLogger::writeMsg("[CholeskyLeastSquareCA] Creating a new file: %s\n",filefullpath.c_str());
			fileo.open(filefullpath.c_str());
			map<long int, Vector>::iterator it;
			for(it = initial_estimate->begin(); it!=initial_estimate->end();it++){
				it->second.zero(); // can the old estimate be updated? to be investigated.
				R[it->first] = new Matrix(3,3);
				R[it->first]->eye();
				fileo<<it->first<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<endl;
			}
			fileo.close();
		}
		else{
			if(UsePreviousInformation){
				SkinCalibrationLogger::writeMsg("[CholeskyLeastSquareCA] Parsing the file %s...\n",filefullpath.c_str());
				if(!parseFile(filefullpath)){ //the file is corrupted
					R.clear();
					SkinCalibrationLogger::writeMsg("[CholeskyLeastSquareCA] ...the file is corrupted!\n");
					ofstream fileo;
					filefullpath = rf->getContextPath();
					filefullpath += "/";
					filefullpath += filename;
					SkinCalibrationLogger::writeMsg("[CholeskyLeastSquareCA] Creating a new file: %s\n",filefullpath.c_str());
					fileo.open(filefullpath.c_str());
					map<long int, Vector>::iterator it;
					for(it = initial_estimate->begin(); it!=initial_estimate->end();it++){
						it->second.zero();
						R[it->first] = new Matrix(3,3);
						R[it->first]->eye();
					}
				}
			}
			else{
				ofstream fileo;
				filefullpath = rf->getContextPath();
				filefullpath += "/";
				filefullpath += filename;
				SkinCalibrationLogger::writeMsg("[CholeskyLeastSquareCA] Creating a new file: %s\n",filefullpath.c_str());
				fileo.open(filefullpath.c_str());
				map<long int, Vector>::iterator it;
				for(it = initial_estimate->begin(); it!=initial_estimate->end();it++){
					it->second.zero();
					R[it->first] = new Matrix(3,3);
					R[it->first]->eye();
				}
			}
		}
	
	SkinCalibrationLogger::writeMsg("[CholeskyLeastSquareCA] Init end\n");
	initialized = true;
	return initialized;
}

void CholeskyLeastSquareCA::CholeskyFactorization(Matrix& R,int size){
	double term=0;
	Matrix temp(size,size);
	temp = (R.submatrix(0,size-1,0,size-1));
	gsl_linalg_cholesky_decomp((gsl_matrix *)temp.getGslMatrix());
	for(int r=0; r<size; r++)
		for(int c=0; c<size; c++)
			if(c>=r)
				R(r,c) = temp(r,c);
			else
				R(r,c) = 0.0;

	//for(int i=0; i<size; i++){
 //       for(int j=0; j<size; j++){
 //           if(j<i){
 //               //if(j==0)
 //               //    R(j,i) = (R(j,i))/R(j,j);
 //               //else{
	//			   term = 0;
	//			   for(int k=0; k<j;k++)
	//				   term += R(k,j)*R(k,i);
 //                  R(j,i) = (R(j,i) - term)/R(j,j);
	//			//}
	//		}
 //           else if(j == i){
 //               //if(j==0)
 //                //   R(j,i) = sqrt(R(j,i));
 //               //else{
	//				term = 0;
	//				for(int k=0; k<i;k++)
	//				   term += R(k,i)*R(k,i);
 //                   R(i,i) = sqrt(R(i,i)-term);
	//			//}
	//		}
 //           else{
 //                R(j,i) = 0.0;
	//		}
	//	}
	//}
}

void CholeskyLeastSquareCA::CholeskyUpdate(Matrix& R, Vector x){
	int size = x.size();
	double r,c,s;
	for(int k=0; k<size; k++){
           r = sqrt((R(k,k)*R(k,k))+(x(k)*x(k)));
           c = r/R(k, k);
           s = x(k)/R(k, k);
           R(k, k) = r;
		   for(int j=k+1; j<size; j++){
				R(k,j) = (R(k,j) + s*x(j))/c;
				x(j) = c*x(j) - s*R(k,j);
		   }
	}
}

void  CholeskyLeastSquareCA::fbSubstitution(Matrix &R, Vector& x){
  
  // L = R' and C = R
  // Ax = d
  // A'Ax = b, b=A'd -> LCx = b. Then y is defined to be Cx
  int i = 0;
  int j = 0;
  Vector y(x.size());
  Vector b(R.getRow(R.rows()-1));
  int n = b.size();
  Matrix C(R.submatrix(0,R.rows()-2,0,R.cols()-1));
  Matrix L(C.transposed());
  // Forward solve Ly = b
  for (i = 0; i < n; i++)
  {
    y[i] = b[i];
    for (j = 0; j < i; j++)
    {
      y[i] -= L(i, j) * y[j];
    }
    y[i] /= L(i, i);
  }
  // Backward solve Ux = y
  for (i = n - 1; i >= 0; i--)
  {
    x[i] = y[i];
    for (j = i + 1; j < n; j++)
    {
      x[i] -= C(i, j) * x[j];
    }
    x[i] /= C(i, i);
  }
 }

bool CholeskyLeastSquareCA::calibrate(CalibrationBuffer *cb, map<long int, Vector> *estimates){
	long int id = cb->getTaxelID();
	int counter=0;
	vector<CalibrationSample *>::const_iterator it;
	Matrix Rn(3,3);
	Matrix A(cb->getSize()*3,3);
	Matrix AtA;
	Vector b(cb->getSize()*3);
	SkinCalibrationLogger::writeMsg("[CholeskyLeastSquareCA] Starting calibration step for taxel %i\n",id);
	if((*estimates)[id].size() == 0){
		(*estimates)[id].resize(3);
		(*estimates)[id].zero();
	}

	for(it = cb->getSamples()->begin(); it!=cb->getSamples()->end(); it++){
		Matrix ForceDotP;
		Vector Force = *(*it)->force;

		skewSymmetric(&Force,&ForceDotP);
		Vector Moment(*(*it)->moment);
		Matrix Rot(*(*it)->Rws);
		//SkinCalibrationLogger::writeMsg("[CholeskyLeastSquareCA] R matrix %f %f %f; %f %f %f; %f %f %f;\n",R(0,0),R(0,1),R(0,2),R(1,0),R(1,1),R(1,2),R(2,0),R(2,1),R(2,2));
		Vector o(*(*it)->ows);
		Moment = (Moment)+(ForceDotP*o);
		Moment = Moment/(-norm(Force));
		ForceDotP = ForceDotP*Rot;
		ForceDotP = ForceDotP /norm(Force);
		if(UseWeightMatrix)
			ForceDotP= computeWeightMatrix(*it)*ForceDotP;
		A.setSubmatrix(ForceDotP,counter,0);
		
		if(UseWeightMatrix)
			Moment= computeWeightMatrix(*it)*Moment;
		b.setSubvector(counter,Moment);
		counter += 3;
	}

	
	SkinCalibrationLogger::writeMsg("[CholeskyLeastSquareCA] Computing R matrix for taxel %i\n",id);
	if(R[id]=='\0'){
		//Compute the Cholesky Factorization of A'A
		AtA = A.transposed() * A;
		R[id] = new Matrix(AtA.rows()+1,AtA.cols()); 
		(*R[id]).zero();
		(*R[id]).setSubmatrix(AtA,0,0);
		CholeskyFactorization(*R[id],AtA.rows());
		//Append the A'b vector to the matrix
		(*R[id]).setRow(AtA.rows(),A.transposed()*b);
		(*estimates)[id].zero();
	}
	else{
		//Update the Cholesky Factorization
		for(int k=0; k < A.rows(); k++){
			CholeskyUpdate(*(R[id]),A.getRow(k));
		}
		//Update the vector A'b
		(*R[id]).setRow((*R[id]).rows()-1,(A.transposed()*b)+(*R[id]).getRow((*R[id]).rows()-1));
	}
	SkinCalibrationLogger::writeMsg("[CholeskyLeastSquareCA] Computing position estimate for taxel %i\n",id);
	//SkinCalibrationLogger::writeMsg("estimates size %i\n",(*estimates)[id].size());
	//SkinCalibrationLogger::writeMsg("A size %i %i\n",A.rows(),A.cols());
	SkinCalibrationLogger::writeMsg("[CholeskyLeastSquareCA] R matrix %f %f %f; %f %f %f; %f %f %f;\n",((*R[id]))(0,0),(*(R[id]))(0,1),(*(R[id]))(0,2),(*R[id])(1,0),(*R[id])(1,1),(*R[id])(1,2),(*R[id])(2,0),(*R[id])(2,1),(*R[id])(2,2));
	SkinCalibrationLogger::writeMsg("[CholeskyLeastSquareCA] Previous Estimate %f %f %f\n",(*estimates)[id][0],(*estimates)[id][1],(*estimates)[id][2]);
	//(*estimates)[id] = (*estimates)[id] + pinv(Hn)*A.transposed()*(b - (A*(*estimates)[id]));
	fbSubstitution(*R[id],(*estimates)[id]);
	SkinCalibrationLogger::writeMsg("[CholeskyLeastSquareCA] Estimate %f %f %f\n",(*estimates)[id][0],(*estimates)[id][1],(*estimates)[id][2]);
	return true;
}
