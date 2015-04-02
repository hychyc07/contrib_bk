#include <iCub/skinCalibration/EnvironmentCA.h>

using namespace iCub::skinCalibration;

EnvironmentCA::EnvironmentCA():CalibrationAlgorithm("Environment"){
	meshBP='\0';
	mean_w.clear();
};

EnvironmentCA::~EnvironmentCA(){
	SkinCalibrationLogger::writeMsg("[EnvironmentCA] removing algorithm\n");
	if(initialized){
		if(meshBP){
			delete meshBP;
			meshBP='\0';
		}
	}
}

bool EnvironmentCA::init(ResourceFinder *rf, BodyPart robotBodyPart, SkinPart skinPart, map<long int, Vector> *initial_estimate){
	SkinCalibrationLogger::writeMsg("[EnvironmentCA] Init\n");
	string filename = "";
	
	this->skinPart = skinPart;
	this->robotBodyPart = robotBodyPart;
	
	yarp::os::Property config;
	filename = "EnvironmentCA.ini";
	filefullpath = rf->findFile(filename.c_str());
	if(filefullpath.empty()){
		SkinCalibrationLogger::writeMsg("[EnvironmentCA] Config file not found, aborting!\n");
		initialized = false;
		return initialized;
	}
	config.fromConfigFile(filefullpath.c_str());

	if(!config.check("scaleFactor")){
		SkinCalibrationLogger::writeMsg("[EnvironmentCA] scaleFactor not found, aborting!\n");
		initialized = false;
		return initialized;
	}
	scale_factor = config.find("scaleFactor").asDouble();

	rotRefFrame.resize(3,3);
	rotRefFrame.zero();
	traslRefFrame.resize(3);
	traslRefFrame.zero();
	if (config.check("meshToRefFrameTrans")){
		Vector angaxis(4);
		angaxis.zero();
		Bottle *temp;
		temp = config.find("meshToRefFrameTrans").asList();
		if(temp && temp->size() == 7){
			for(int i=3; i<7; i++){
				angaxis[i-3] = (temp->get(i).asDouble());
				SkinCalibrationLogger::writeMsg("[EnvironmentCA] meshToRefFrameTrans angaxis %i %f\n",i-3,temp->get(i).asDouble());
			}
			rotRefFrame = axis2dcm(angaxis,1).submatrix(0,2,0,2);
			for(int i=0; i<3; i++)
				traslRefFrame[i] = temp->get(i).asDouble();
		}
		else{
			SkinCalibrationLogger::writeMsg("[EnvironmentCA] meshToRefFrameTrans wrong format...\n");
			return false;
		}
		SkinCalibrationLogger::writeMsg("[EnvironmentCA] meshToRefFrameTrans matrix %f %f %f; %f %f %f; %f %f %f; %f %f %f;\n",rotRefFrame(0,0),rotRefFrame(0,1),rotRefFrame(0,2),rotRefFrame(1,0),rotRefFrame(1,1),rotRefFrame(1,2),rotRefFrame(2,0),rotRefFrame(2,1),rotRefFrame(2,2),traslRefFrame[0],traslRefFrame[1],traslRefFrame[2]);
	}
	else{
		SkinCalibrationLogger::writeMsg("[EnvironmentCA] meshToRefFrameTrans not found, aborting!\n");
		initialized = false;
		return initialized;
	}

	if(!config.check("meshFileName")){
		SkinCalibrationLogger::writeMsg("[EnvironmentCA] meshFileName not found, aborting!\n");
		initialized = false;
		return initialized;
	}
	meshfilename =  config.find("meshFileName").asString();
	meshfilename =  rf->findFile(meshfilename.c_str());
	if(meshfilename.empty()){
		SkinCalibrationLogger::writeMsg("[EnvironmentCA] mesh file not found, aborting!\n");
		initialized = false;
		return initialized;
	}
	SkinCalibrationLogger::writeMsg("[EnvironmentCA] parsing the mesh file...\n");
	meshBP = new MeshBodyPart(meshfilename.c_str(),scale_factor,rotRefFrame,traslRefFrame);
	if(!meshBP->isInitialized()){
		SkinCalibrationLogger::writeMsg("[EnvironmentCA] ...mesh file corrupted, aborting!\n");
		initialized = false;
		return initialized;
	}
	SkinCalibrationLogger::writeMsg("[EnvironmentCA] ...done!\n");

	if(!config.check("sphereRadius")){
		SkinCalibrationLogger::writeMsg("[EnvironmentCA] sphereRadius not found, aborting!\n");
		initialized = false;
		return initialized;
	}
	sphere_radius = config.find("sphereRadius").asDouble();

	if (config.check("sphereCenter")){
		Bottle *temp;
		temp = config.find("sphereCenter").asList();
		if(temp && temp->size() == 3){
			sphere_center.resize(3);
			for(int i=0; i<3; i++){
				sphere_center[i] = (temp->get(i).asDouble());
			}
		}
		else{
			SkinCalibrationLogger::writeMsg("[EnvironmentCA] sphereCenter wrong format, aborting!\n");
			initialized = false;
			return initialized;
		}
	}
	else{
		SkinCalibrationLogger::writeMsg("[EnvironmentCA] sphereCenter not found, aborting!\n");
		initialized = false;
		return initialized;
	}

	if(!config.check("contactCentroidsMinDist")){
		SkinCalibrationLogger::writeMsg("[EnvironmentCA] contactCentroidsMinDist not found, aborting!\n");
		initialized = false;
		return initialized;
	}
	centroid_mindist = config.find("contactCentroidsMinDist").asDouble();


	if(!config.check("outOfContactTaxelResponse")){
		SkinCalibrationLogger::writeMsg("[EnvironmentCA] outOfContactTaxelResponse not found, aborting!\n");
		initialized = false;
		return initialized;
	}
	w = config.find("outOfContactTaxelResponse").asDouble();
	
	SkinCalibrationLogger::writeMsg("[EnvironmentCA] Init end\n");
	initialized = true;
	return initialized;
}


bool EnvironmentCA::circ_intersections(Vector c1, Vector c2, double r1, double r2, Vector &i1, Vector &i2){

	double temp_r;
	Vector temp_v;
	double deltax; 
    double deltay;
	double delta;
	double s,u,cx,cy;

	if(r1>r2){
        temp_r = r1;
        r1=r2;
        r2=temp_r;
        temp_v=c1;
        c1=c2;
        c2=temp_v;
	}

	deltax = c2[0]-c1[0];
	deltay = c2[1]-c1[1];

	delta = sqrt(pow(deltax,2)+pow(deltay,2));
    if(delta > r2+r1 || delta < r2-r1){
           return false;
	}

    s = (pow(delta,2) + pow(r1,2)-pow(r2,2))/(2*delta);
    cx = c1[0] + ((deltax*s)/delta);
    cy = c1[1] + ((deltay*s)/delta);
    u = sqrt(pow(r1,2) - pow(s,2));
	
	i1.resize(2);
    i1[0] = cx - ((deltay*u)/delta);
    i1[1] = cy + ((deltax*u)/delta);
    
	i2.resize(2);
    i2[0] = cx + ((deltay*u)/delta);
    i2[1] = cy - ((deltax*u)/delta);
   
	return true;
}

bool EnvironmentCA::trilateration(long int id, Vector& pos_estimate){
	Vector vx;
	Vector vy;
	Vector i12D,i22D;
	Vector i1,i2,i3,i4;
	Vector c0(2),c1(2);
	double d0,d1,d2;
	c0.zero();
	c1.zero();

	c1[0] = norm(triples[id][1].center-triples[id][0].center);
	if(circ_intersections(c0, c1, triples[id][0].radius, triples[id][1].radius, i12D, i22D)){
		vx=(triples[id][1].center-triples[id][0].center)/c1[0];
		vy=cross(vx,triples[id][0].normal);
		vy=vy/norm(vy);

		i1.resize(3);
		i2.resize(3);
		i1 = triples[id][0].center + vx*i12D[0] + vy*i12D[1];
		i2 = triples[id][0].center + vx*i22D[0] + vy*i22D[1];
	}
	else return false;

	c1[0] = norm(triples[id][2].center-triples[id][1].center);
	if(circ_intersections(c0, c1, triples[id][1].radius, triples[id][2].radius, i12D, i22D)){
		vx=(triples[id][2].center-triples[id][1].center)/c1[0];
		vy=cross(vx,triples[id][1].normal);
		vy=vy/norm(vy);

		i3.resize(3);
		i4.resize(3);
		i3 = triples[id][1].center + vx*i12D[0] + vy*i12D[1];
		i4 = triples[id][1].center + vx*i22D[0] + vy*i22D[1];
	}
	else return false;

	d0 = norm(i1-i3);
	d1 = norm(i1-i4);

	if(d1 > d0){
		d2 = norm(i2-i4);
		if(fabs(d2-d0)<0.0001){
			return false;
		}
		else if(d2<d0){
			pos_estimate = (i2+i4)/2;
		}
		else{
			pos_estimate = (i1+i3)/2;
		}
	}
	else{
		d2 = norm(i2-i3);
		if(fabs(d2-d1)<0.0001){
			return false;
		}
		else if(d2<d1){
			pos_estimate = (i2+i3)/2;
		}
		else{
			pos_estimate = (i1+i4)/2;
		}
	}

	return true;
}


bool EnvironmentCA::calibrate(CalibrationBuffer *cb, map<long int, Vector> *estimates){
	long int id = cb->getTaxelID();
	vector<CalibrationSample *>::const_iterator it;
	map<long int,double>::iterator mit;
	Vector contact_centroid,sphere_center_w,pos_estimate;
	double d,distance,N,u,v,t,a,tmax,sigma;
	trilat_record record;
	int face,contact_face;
	bool result = false;

	if((*estimates)[id].size() == 0){
		(*estimates)[id].resize(3); 
		(*estimates)[id].zero();
	}
	for(it = cb->getSamples()->begin(); it!=cb->getSamples()->end(); it++){
		sphere_center_w = ((*(*it)->Rw).transposed() * sphere_center) - ((*(*it)->Rw).transposed() * (*(*it)->ow));
		if(meshBP->findClosestFace(sphere_center_w,distance,face)){
			//SkinCalibrationLogger::writeMsg("Closest face %i\n",face);
			Vector dir = -1*meshBP->getFaceNormal(face);
			d = distance-sphere_radius;
			contact_face=-1;
			meshBP->findRayMeshIntersectionPoint(sphere_center_w,dir,contact_face,u,v,t,contact_centroid);
			//SkinCalibrationLogger::writeMsg("Intersection face %i\n",face);
			if(contact_centroid.size()!=0 && (triples[id].size()==0 || 
				norm(triples[id][triples[id].size()-1].center-contact_centroid)>=centroid_mindist)){
				SkinCalibrationLogger::writeMsg("Contact Centroid %f %f %f\n",contact_centroid[0],contact_centroid[1],contact_centroid[2]);
				N = norm((*(*it)->force));
				tmax=0; //Maximum taxel response for this sample
				for(mit=(*it)->taxels_measures->begin(); mit!=(*it)->taxels_measures->end();mit++){
					if(mit->second> tmax){
						tmax = mit->second;
					}
				}
				//physical model
				//a = pow((N*R)/K,e_a);
				//heuristics
				a= 0.005*((*it)->taxels_measures->size()/3) + 0.002*((*it)->taxels_measures->size()%3); //heuristic for contact area radius estimate
				//w, taxel response estimate at distance a from the contact centroids
				sigma = -(a*a)/(2*log(w/tmax));
				record.center = contact_centroid;
				record.radius = sqrt(-(2*sigma*log((*(*it)->taxels_measures)[id]/tmax)));
				record.normal = meshBP->getFaceNormal(face);
				triples[id].push_back(record);
			}
			if(triples[id].size() == 3){
				if(trilateration(id,pos_estimate)){
					SkinCalibrationLogger::writeMsg("Pos Estimate %f %f %f\n",pos_estimate[0],pos_estimate[1],pos_estimate[2]);
					result = true;
					(*estimates)[id]= ((mean_w[id]/(mean_w[id]+1))*(*estimates)[id]) + (pos_estimate/(mean_w[id]+1));
					mean_w[id]++;
					triples[id].clear();
				}
				else
					triples[id].clear();
			}
		}
	}
	return result;

}
