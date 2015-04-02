#include "iCub/skinCalibration/ActivationAreaCA.h"

using namespace iCub::skinCalibration;


ActivationAreaCA::ActivationAreaCA():CalibrationAlgorithm("ActivationArea"){
	meshBP='\0';
}

ActivationAreaCA::~ActivationAreaCA(){
	SkinCalibrationLogger::writeMsg("[ActivationAreaCA] removing algorithm\n");
	if(initialized){
		ofstream file;
		vector<long int>::iterator taxel;
		for(taxel = taxel_id.begin(); taxel != taxel_id.end(); taxel++){
			stringstream filename;
			filename << filefullpath<<"ActivationAreaCA_";
			filename << BodyPart_s[robotBodyPart]<<"_"<<SkinPart_s[skinPart]<<"_"<<*taxel;

			vector<Vector>::iterator it;
			file.open((filename.str()+".txt").c_str());
			for(it = activation_points[*taxel].begin(); it!=activation_points[*taxel].end();){
					file<<(*it)[0]<<" "<<(*it)[1]<<" "<<(*it)[2]<<" "<<(*it)[3];
					it++;
					if(it!=activation_points[*taxel].end())
						file<<endl;
			}
			file.close();
		}
		if(meshBP){
			delete meshBP;
			meshBP='\0';
		}
	}
}

//bool ActivationAreaCA::loadMeshFile(){
//	ifstream file;
//	file.open(meshfilename.c_str());
//	if(file.peek() == ifstream::traits_type::eof()) //The file is empty
//		return true;
//	mesh_points.clear();
//	mesh_faces.clear();
//	vertices_normals.clear();
//	faces_normals.clear();
//	while(!file.eof()){
//		string line;
//		getline(file,line);
//		Vector v(3);
//		v.zero();
//		int v1,v2,v3;
//		int n;
//		if(sscanf(line.c_str(),"v %lf %lf %lf",&v.data()[0],&v.data()[1],&v.data()[2])==3){
//			v = v/scale_factor;
//			v=rotRefFrame*v + traslRefFrame;
//			mesh_points.push_back(v);
//		}
//		else if(sscanf(line.c_str(),"vn %lf %lf %lf",&v.data()[0],&v.data()[1],&v.data()[2])==3){
//			v=rotRefFrame*v + traslRefFrame;
//			vertices_normals.push_back(v);
//		}		
//		else if(sscanf(line.c_str(),"f %i//%i %i//%*i %i//%*i",&v1,&n,&v2,&v3)==4){
//			Vector u = mesh_points[v2-1]-mesh_points[v1-1];
//			Vector w = mesh_points[v3-1]-mesh_points[v1-1];
//			Vector nm = cross(u,w);
//			nm = nm/norm(nm);
//			if(dot(vertices_normals[n-1],nm)>0){
//				faces_normals.push_back(nm);
//			}
//			else{
//				faces_normals.push_back(-1*nm);
//			}
//			v[0] = v1-1;
//			v[1] = v2-1;
//			v[2] = v3-1;
//			v.push_back(faces_normals.size()-1);
//			mesh_faces.push_back(v);
//		}
//	}
//	printf("ActivationAreaCA the loaded mesh has %i faces and %i points\n",mesh_faces.size(),mesh_points.size());
//	return true;
//}

//void ActivationAreaCA::rayTriangleIntersection(Vector o, Vector d,  Vector p0, Vector p1, Vector p2, Vector n, Vector &results){
//	Vector e1 = p1-p0;
//    Vector e2 = p2-p0;
//	Vector nt = cross(e1,e2);
// //   if(dot(n,nt) <0){
// //       Vector temp = e1;
// //       e1=e2;
// //       e2=temp;
//	//}
//    Vector q  = cross(d,e2);
//    double a  = dot(e1,q);
//	double epsilon = 0.00001;
//
//    if (a>-epsilon && a<epsilon){
//		results.resize(0);
//        return;
//	}
//    
//    double f = 1/a;
//    Vector s = o-p0;
//    double u = f*dot(s,q);
//    
//    if (u<0.0 || u>1.0){
//		//if(fabs(u)<1)
//			//printf("u = %lf\n",u);
//		results.resize(0);
//        return;      
//	}
//    
//    Vector r = cross(s,e1);
//    double v = f*dot(d,r);
//    
//    if (v<0.0 || u+v>1.0){
//		//printf("u = %lf v = %lf\n",u,v);
//        results.resize(0);
//        return;       
//	}
//    
//    double t = f*dot(e2,r);
//	results.resize(3);
//	Vector p = p0+ (u*e1)+(v*e2);
//	results[0] = p[0];
//	results[1] = p[1];
//	results[2] = p[2];
//	//results[3] = t;
//}



//void ActivationAreaCA::findIntersectionPoint(CalibrationSample*cs, Vector *int_point){
//
//	vector<Vector>::iterator it;
//	//We need the sensor to wrist transformation that is the inverse of the one provided by the calibrationSample
//	Vector o = cross(*cs->force,*cs->moment) / pow(norm(*cs->force),2);
//	o = (*cs->Rws).transposed()*o - (*cs->Rws).transposed()*(*cs->ows);
//	Vector dir=  (*cs->Rws).transposed()*(*cs->force);
//	
//	(*int_point).resize(0);
//	for(it = mesh_faces.begin(); it!= mesh_faces.end(); it++){
//		Vector results;
//		if(dot(faces_normals[(*it)[3]],*cs->force)>0){
//			rayTriangleIntersection(o,dir,mesh_points[(*it)[0]],mesh_points[(*it)[1]],mesh_points[(*it)[2]],faces_normals[(*it)[3]],results);
//			if(results.size() != 0){
//				*int_point=results;
//				return;
//			}
//		}
//	}
//}

bool ActivationAreaCA::init(ResourceFinder *rf, BodyPart robotBodyPart, SkinPart skinPart, map<long int, Vector> *initial_estimate){
	SkinCalibrationLogger::writeMsg("[ActivationAreaCA] init\n");
	yarp::os::Property config;
	string filename = "";
	filename = "ActivationAreaCA.ini";
	filefullpath = rf->findFile(filename.c_str());
	if(filefullpath.empty()){
		SkinCalibrationLogger::writeMsg("[ActivationAreaCA] No config file, aborting!\n");
		initialized = false;
		return initialized;
	}

	config.fromConfigFile(filefullpath.c_str());
	if(!config.check("taxelid")){
		SkinCalibrationLogger::writeMsg("[ActivationAreaCA] No taxel id found, aborting!\n");
		initialized = false;
		return initialized;
	}
	Bottle * taxel_id_list = config.find("taxelid").asList();
	if(taxel_id_list){
		for(int i=0; i<taxel_id_list->size(); i++){
			if(taxel_id_list->get(i).isInt()){
				taxel_id.push_back(taxel_id_list->get(i).asInt());
				SkinCalibrationLogger::writeMsg("[ActivationAreaCA] Added id %i.\n",taxel_id_list->get(i).asInt());
			}
			else if(taxel_id_list->get(i).isString()){
				SkinCalibrationLogger::writeMsg("[ActivationAreaCA] Range id detected.\n");
				int low,high;
				ConstString s = taxel_id_list->get(i).asString();
				if(s.find("#")<0){
					SkinCalibrationLogger::writeMsg("[ActivationAreaCA] wrong format, aborting.\n");
					initialized = false;
					return initialized;
				}
				low = atoi(s.substr(0,s.find("#")-1).c_str());
				high = atoi(s.substr(s.find("#")+1,-1).c_str());
				for(int j=low; j<high; j++){
					taxel_id.push_back(j);
				}
				SkinCalibrationLogger::writeMsg("[ActivationAreaCA] Added id from %i to %i.\n",low,high);
			}
		}
	}

	if(!config.check("scaleFactor")){
		SkinCalibrationLogger::writeMsg("[ActivationAreaCA] scaleFactor not found, aborting!\n");
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
				SkinCalibrationLogger::writeMsg("[ActivationAreaCA] meshToRefFrameTrans angaxis %i %f\n",i-3,temp->get(i).asDouble());
			}
			rotRefFrame = axis2dcm(angaxis,1).submatrix(0,2,0,2);
			for(int i=0; i<3; i++)
				traslRefFrame[i] = temp->get(i).asDouble();
		}
		else{
			SkinCalibrationLogger::writeMsg("[ActivationAreaCA] meshToRefFrameTrans wrong format...\n");
			return false;
		}
		SkinCalibrationLogger::writeMsg("[ActivationAreaCA] meshToRefFrameTrans matrix %f %f %f; %f %f %f; %f %f %f; %f %f %f;\n",rotRefFrame(0,0),rotRefFrame(0,1),rotRefFrame(0,2),rotRefFrame(1,0),rotRefFrame(1,1),rotRefFrame(1,2),rotRefFrame(2,0),rotRefFrame(2,1),rotRefFrame(2,2),traslRefFrame[0],traslRefFrame[1],traslRefFrame[2]);
	}
	else{
		SkinCalibrationLogger::writeMsg("[ActivationAreaCA] meshToRefFrameTrans not found, aborting!\n");
		initialized = false;
		return initialized;
	}

	if(!config.check("meshFileName")){
		SkinCalibrationLogger::writeMsg("[ActivationAreaCA] No meshFileName found in configuration, aborting!\n");
		initialized = false;
		return initialized;
	}
	meshfilename =  config.find("meshFileName").asString();
	meshfilename =  rf->findFile(meshfilename.c_str());
	if(meshfilename.empty()){
		SkinCalibrationLogger::writeMsg("[ActivationAreaCA] Mesh file not found, aborting!\n");
		initialized = false;
		return initialized;
	}
	SkinCalibrationLogger::writeMsg("[ActivationAreaCA] Parsing the mesh file...\n");
	meshBP = new MeshBodyPart(meshfilename.c_str(),scale_factor,rotRefFrame,traslRefFrame);

	if(!meshBP->isInitialized()){
		SkinCalibrationLogger::writeMsg("[ActivationAreaCA] mesh file corrupted, aborting!\n");
		initialized = false;
		return initialized;
	}
	SkinCalibrationLogger::writeMsg("[ActivationAreaCA] ...done\n");

	this->skinPart = skinPart;
	this->robotBodyPart = robotBodyPart;
	filename = "ActivationAreaCA_";
	filename += BodyPart_s[robotBodyPart]+"_"+ SkinPart_s[skinPart];
	filefullpath = rf->getContextPath();
	filefullpath += "/";
	this->robotBodyPart = robotBodyPart;
    this->skinPart = skinPart;
	SkinCalibrationLogger::writeMsg("[ActivationAreaCA] Init end\n");
	initialized = true;
	return initialized;
}


bool ActivationAreaCA::calibrate(CalibrationBuffer *cb, map<long int, Vector> *estimates){
	long int id = cb->getTaxelID();
	if(find(taxel_id.begin(), taxel_id.end(), id)!=taxel_id.end()){
		SkinCalibrationLogger::writeMsg("ActivationAreaCA receiving %i measure set for taxel %i\n",activation_points[id].size()+1,id);
		vector<CalibrationSample *>::const_iterator it;
		for(it = cb->getSamples()->begin(); it!=cb->getSamples()->end(); it++){
			Vector point;
			Vector o = cross(*(*it)->force,*(*it)->moment) / pow(norm(*(*it)->force),2);
			o = (*(*it)->Rws).transposed()*o - (*(*it)->Rws).transposed()*(*(*it)->ows);
			Vector dir=  (*(*it)->Rws).transposed()*(*(*it)->force);
			meshBP->findRayMeshIntersectionPoint(o,dir,point);
			if(point.size()!=0){
				SkinCalibrationLogger::writeMsg("Intersection point %f %f %f\n",point[0],point[1],point[2]);
				point.push_back(((*(*it)->taxels_measures)[(*it)->taxel_id]));
				activation_points[id].push_back(point);
			}
		}
	}
	return false;
}
