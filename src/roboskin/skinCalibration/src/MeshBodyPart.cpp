#include "iCub/skinCalibration/MeshBodyPart.h"

using namespace iCub::skinCalibration;

MeshBodyPart::MeshBodyPart(){
	meshfilename = "";
	initialized=false;
};


MeshBodyPart::MeshBodyPart(const char *mfilename, double scfactor, Matrix& rotationRefFrame, Vector& translationRefFrame){
	meshfilename = mfilename;
	scale_factor = scfactor;
	rotRefFrame = rotationRefFrame;
	translRefFrame =  translationRefFrame;
	initialized = loadMeshFile();
}

MeshBodyPart::~MeshBodyPart(){

	/*if(initialized){
		ofstream file;
		file.open((meshfilename+"prova").c_str());
		for(int i =0; i<faces_normals.size();i++){
				file<<faces_normals[i][0]<<" "<<faces_normals[i][1]<<" "<<faces_normals[i][2]<<endl;
		}
		file.close();

		file.open((meshfilename+"pppprova").c_str());
		for(int i =0; i<vertices_normals.size();i++){
				file<<vertices_normals[i][0]<<" "<<vertices_normals[i][1]<<" "<<vertices_normals[i][2]<<endl;
		}
		file.close();

		file.open((meshfilename+"puntiprova").c_str());
		for(int i =0; i<mesh_points.size();i++){
				file<<mesh_points[i][0]<<" "<<mesh_points[i][1]<<" "<<mesh_points[i][2]<<endl;
		}
		file.close();
	}
*/
	mesh_vertices.clear();
	mesh_vertices2D.clear();
	mesh_faces.clear();
	vertices_normals.clear();
	faces_normals.clear();
	vertex_faces.clear();
	vertex_neighbors.clear();
}

bool MeshBodyPart::loadMeshFile(){
	ifstream file;
	file.open(meshfilename.c_str());
	if(file.peek() == ifstream::traits_type::eof()) //The file is empty
		return true;
	mesh_vertices.clear();
	mesh_faces.clear();
	vertices_normals.clear();
	faces_normals.clear();
	while(!file.eof()){
		string line;
		getline(file,line);
		Vector v(3);
		v.zero();
		int v1,v2,v3;
		int n;
		if(sscanf(line.c_str(),"v %lf %lf %lf",&v.data()[0],&v.data()[1],&v.data()[2])==3){
			v = v/scale_factor;
			v=rotRefFrame*v + translRefFrame;
			mesh_vertices.push_back(v);
		}
		else if(sscanf(line.c_str(),"vn %lf %lf %lf",&v.data()[0],&v.data()[1],&v.data()[2])==3){
			v=rotRefFrame*v;
			vertices_normals.push_back(v);
		}		
		else if(sscanf(line.c_str(),"f %i//%i %i//%*i %i//%*i",&v1,&n,&v2,&v3)==4){
			Vector u = mesh_vertices[v2-1]-mesh_vertices[v1-1];
			Vector w = mesh_vertices[v3-1]-mesh_vertices[v1-1];
			Vector nm = cross(u,w);
			vector<int> data(4);
			nm = nm/norm(nm);
			if(dot(vertices_normals[n-1],nm)>0){
				faces_normals.push_back(nm);
			}
			else{
				faces_normals.push_back(-1*nm);
			}
			data[0] = v1-1;
		    data[1] = v2-1;
			data[2] = v3-1;
			data[3] = faces_normals.size()-1;
			mesh_faces.push_back(data);
		}
	}
	SkinCalibrationLogger::writeMsg("[MeshBodyPart] The loaded mesh has %i faces and %i points\n",mesh_faces.size(),mesh_vertices.size());
	return true;
}


void MeshBodyPart::rayTriangleIntersection(Vector o, Vector d,  Vector p0, Vector p1, Vector p2, Vector n, double &u, double &v, double &t, Vector &results){

	Vector e1 = p1-p0;
    Vector e2 = p2-p0;
	Vector nt = cross(e1,e2);
    if(dot(n,nt) <0){
        Vector temp = e1;
        e1=e2;
        e2=temp;
	}
    Vector q  = cross(d,e2);
    double a  = dot(e1,q);
	double epsilon = 0.00001;

    if (a>-epsilon && a<epsilon){
		results.resize(0);
        return;
	}
    
    double f = 1/a;
    Vector s = o-p0;
    u = f*dot(s,q);
    
    if (u<0.0 || u>1.0){
		//if(fabs(u)<1)
			//printf("u = %lf\n",u);
		results.resize(0);
        return;      
	}
    
    Vector r = cross(s,e1);
    v = f*dot(d,r);
    
    if (v<0.0 || u+v>1.0){
		//printf("u = %lf v = %lf\n",u,v);
        results.resize(0);
        return;       
	}
    
    t = f*dot(e2,r);
	results.resize(3);
	Vector p = p0+ (u*e1)+(v*e2);
	results[0] = p[0];
	results[1] = p[1];
	results[2] = p[2];
	//results[3] = t;
}

void MeshBodyPart::initializeVertexFacesMatrix(){

	vertex_faces.clear();

	for(unsigned int i=0; i < mesh_faces.size(); i++){
		vertex_faces[mesh_faces[i][0]].push_back(i);
		vertex_faces[mesh_faces[i][1]].push_back(i);
		vertex_faces[mesh_faces[i][2]].push_back(i);
	}
}

void MeshBodyPart::computeMeshLaplacian(){

	Matrix diag;
	Vector sumdiag;
	sumdiag.clear();
	
	meshLaplacian.resize(mesh_vertices.size(),mesh_vertices.size());
	meshLaplacian.zero();
	initializeVertexFacesMatrix();

	for(unsigned int i = 0; i < mesh_vertices.size(); i++){
		for(unsigned int j = 0; j < vertex_faces[i].size(); j++){
			int face = vertex_faces[i][j];
			int v0 = i;
			int v1,v2;
			double alpha,beta;

			if(mesh_faces[face][0] == v0){
				v1 = mesh_faces[face][1];
				v2 = mesh_faces[face][2];
			}
			else if(mesh_faces[face][1] == v0){
				v1 = mesh_faces[face][0];
				v2 = mesh_faces[face][2];
			}
			else{
				v1 = mesh_faces[face][0];
				v2 = mesh_faces[face][1];
			}

			alpha = computeAngle(mesh_vertices[v2]-mesh_vertices[v0],mesh_vertices[v2]-mesh_vertices[v1]);
            beta =  computeAngle(mesh_vertices[v1]-mesh_vertices[v0],mesh_vertices[v1]-mesh_vertices[v2]);
			meshLaplacian(v0,v1) = meshLaplacian(v0,v1) + (1/tan(alpha));
            meshLaplacian(v0,v2) = meshLaplacian(v0,v2) + (1/tan(beta));
		}
	}
	sumdiag.size(diag.rows());
	sumdiag.zero();
	for(int c = 0; c < meshLaplacian.cols(); c++)
		for(int r = 0; r< meshLaplacian.rows(); r++)
			sumdiag(c)+=meshLaplacian(r,c);

	meshLaplacian = diag.diagonal(sumdiag) - meshLaplacian;
}

double MeshBodyPart::computeAngle(Vector u, Vector v){

	double du, dv;
	du = sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
	dv = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
	//du = max(du,eps); dv = max(dv,eps);
	return  acos((u[0]*v[0] + u[1]*v[1] + u[2]*v[2]) / (du*dv));
}

void MeshBodyPart::computeMeshBoundary(vector<int> &boundary){
	Matrix edgeCounter(mesh_vertices.size(),mesh_vertices.size());
	edgeCounter.zero();
	for(unsigned int i = 0; i < mesh_faces.size(); i++){
		vector<int> &f = mesh_faces[i];
		edgeCounter(f[0],f[1])++;
	    edgeCounter(f[0],f[2])++;
        edgeCounter(f[2],f[1])++;
		edgeCounter(f[0],f[0])+=3.0; //mark the existance of the vertex in the mesh_faces 
		edgeCounter(f[1],f[1])+=3.0; //mark the existance of the vertex in the mesh_faces
		edgeCounter(f[2],f[2])+=3.0; //mark the existance of the vertex in the mesh_faces
	}

	edgeCounter = edgeCounter + edgeCounter.transposed();
	boundary.clear();

	for(int r = 0; r < edgeCounter.rows(); r++){
		if(edgeCounter(r,r) != 0.0){ //The vertex is not used in mesh_faces so must be skipped
			for(int c = 0; c < edgeCounter.cols(); c++){
				if(edgeCounter(r,c) == 1.0){
					boundary.push_back(r);
					boundary.push_back(c);
					r=edgeCounter.rows();
					break;
				}
			}
		}
	}

	if(boundary.size()>=2){
		int start = boundary[0];
		int previous = boundary[0];
		int current = boundary[1];
		while(current != start){
			for(int c = 0; c < edgeCounter.cols(); c++){
				if(edgeCounter(current,c) == 1.0 && c != previous){
					boundary.push_back(c);
					previous = current;
					current = c;
					break;
				}
			}
		}
	}
}

void MeshBodyPart::computeParameterization(){

	computeMeshLaplacian();
	int n = mesh_vertices.size(); 
	Matrix A(n*2,n*2);
	
	vector<int> boundary;
	A.zero();
	A.setSubmatrix(meshLaplacian,0,0);
	A.setSubmatrix(meshLaplacian,n,n);

	computeMeshBoundary(boundary);

	for(unsigned int s=0; s < boundary.size(); s++){
		int s1,s2;
		if(s==0)
			s1=boundary.size()-1;
		else
			s1 = s-1;

		if(s==boundary.size()-1)
			s2=0;
		else
			s2=s+1;
		
		int i = boundary[s];
		int i1 = boundary[s1];
		int i2 = boundary[s2];
		A(i,i1+n) = 1;
		A(i,i2+n) = -1;
		A(i+n,i1) = -1;
		A(i+n,i2) = 1;
	}

	int i1 = boundary[0];
	int i2 = boundary[boundary.size()/2];
	for(int c=0; c < A.rows(); c++){
		A(i1,c) = 0;
		A(i1+n,c) = 0;
		A(i2,c) = 0;  
		A(i2+n,c) = 0;
		if(c==i1)
			A(i1,i1) = 1;
		if(c==i2)
			A(i2,i2) = 1;
		
		if(c==i1+n)
			A(i1+n,i1+n) = 1;	 
		if(c==i2+n)
			A(i2+n,i2+n) = 1;
	}
	//A(i2,:) = 0;  A(i2+n,:) = 0;
	//A(i2,i2) = 1; A(i2+n,i2+n) = 1;
	//p1 = [0 0]; p2 = [1 0];
	Vector y(2*n,1);
	y[i1] = 0.0; 
	y[i1+n] = 0.0;
	y[i2] = 1.0; 
	y[i2+n] = 0.0;

	y = pinv(A)*y;
	for(int r=0; r<n; r++){
		mesh_vertices2D[r].resize(2);
		mesh_vertices2D[r][0] = y[r];
		mesh_vertices2D[r][1] = y[r+n];
	}
}

void MeshBodyPart::extractSubMesh(vector<int> &vertices_id, MeshBodyPart &newMesh){

	map<int,int> vertices_id_mapping;
	vector<int> face(4);
	newMesh.mesh_vertices.clear();
	newMesh.mesh_vertices2D.clear();
	newMesh.mesh_faces.clear();
	newMesh.vertices_normals.clear();
	newMesh.faces_normals.clear();

	newMesh.meshfilename = "";
	newMesh.scale_factor = scale_factor;
	newMesh.rotRefFrame = rotRefFrame; 
	newMesh.meshLaplacian = meshLaplacian;
	newMesh.translRefFrame = translRefFrame;

	for(unsigned int i=0; i < vertices_id.size(); i++){
		vertices_id_mapping[vertices_id[i]] = i;
		newMesh.mesh_vertices.push_back(mesh_vertices[vertices_id[i]]);
		newMesh.vertices_normals.push_back(vertices_normals[vertices_id[i]]);
	};
	for(unsigned int i=0; i < mesh_faces.size(); i++){
		if(find(vertices_id.begin(),vertices_id.end(),mesh_faces[i][0]) != vertices_id.end()){
			if(find(vertices_id.begin(),vertices_id.end(),mesh_faces[i][1]) != vertices_id.end()){
				if(find(vertices_id.begin(),vertices_id.end(),mesh_faces[i][2]) != vertices_id.end()){
					face[0] = vertices_id_mapping[mesh_faces[i][0]];
					face[1] = vertices_id_mapping[mesh_faces[i][1]];
					face[2] = vertices_id_mapping[mesh_faces[i][2]];
					newMesh.faces_normals.push_back(faces_normals[mesh_faces[i][3]]);
					face[3] = newMesh.faces_normals.size()-1;
					newMesh.mesh_faces.push_back(face);
				}
			}
		}	
	}
	newMesh.initialized = true;
}

void MeshBodyPart::findVertexFaces(int v, vector<int>& result){

}

void MeshBodyPart::findVertexNeighbors(int v, vector<int>& result){

	for(unsigned int i=0; i < mesh_faces.size(); i++){
		if(mesh_faces[i][0] == v){
			result.push_back(mesh_faces[i][1]);
			result.push_back(mesh_faces[i][2]);
		}
		else if(mesh_faces[i][1] == v){
			result.push_back(mesh_faces[i][0]);
			result.push_back(mesh_faces[i][2]);
		}
		else if(mesh_faces[i][2] == v){
			result.push_back(mesh_faces[i][0]);
			result.push_back(mesh_faces[i][1]);
		}
	}
	vector<int>::iterator it = unique(result.begin(),result.end());
	result.resize( it - result.begin() ); 
}

void MeshBodyPart::findRayMeshIntersectionPoint(Vector& o, Vector& dir, Vector &int_point){

	vector<vector<int> >::iterator it;
	double u,v,t;
	Vector minusdir = -1*dir;
	int_point.resize(0);
	for(it = mesh_faces.begin(); it!= mesh_faces.end(); it++){
		Vector results;
		if(dot(faces_normals[(*it)[3]],dir)<0){
			rayTriangleIntersection(o,dir,mesh_vertices[(*it)[0]],mesh_vertices[(*it)[1]],mesh_vertices[(*it)[2]],faces_normals[(*it)[3]],u,v,t,results);
			if(results.size() != 0){
				int_point=results;
				return;
			}
		}
	}
}

void MeshBodyPart::findRayMeshIntersectionPoint(Vector& o, Vector& dir, int &face, double &u, double &v,double &t, Vector &int_point){

	vector<vector<int> >::iterator it;
	int face_size;
	int_point.resize(0);
	u=0;
	v=0;
	t=0;

	if(face<0){ // If face is greter or equals to 0, the intersection will be computed only for face mesh_faces[face]
		face = 0;
		face_size = mesh_faces.size();
	}
	else{
		face_size = face+1;
	}

	for(int i = face; i<face_size; i++){
		Vector results;
		if(dot(faces_normals[mesh_faces[i][3]],dir)<0){
			rayTriangleIntersection(o,dir,mesh_vertices[mesh_faces[i][0]],mesh_vertices[mesh_faces[i][1]],mesh_vertices[mesh_faces[i][2]],faces_normals[mesh_faces[i][3]],u,v,t,results);
			if(results.size() != 0){
				int_point=results;
				face = i;
				return;
			}
		}
	}
}

void MeshBodyPart::findClosestProjectionOnMesh(Vector &o, Vector &dir, int &face, Vector &prj_point){

	vector<vector<int> >::iterator it;
	double u,v,t;
	double tmin = DBL_MAX;
	
	prj_point.resize(0);
	for(it = mesh_faces.begin(); it!= mesh_faces.end(); it++){
		Vector results;
		rayTriangleIntersection(o,dir,mesh_vertices[(*it)[0]],mesh_vertices[(*it)[1]],mesh_vertices[(*it)[2]],faces_normals[(*it)[3]],u,v,t,results);
		if(results.size() != 0){
			if(fabs(t) < tmin){
				prj_point=results;
				tmin = fabs(t);
				face = distance(mesh_faces.begin(), it);
			}
			continue;
		}
		rayTriangleIntersection(o,-1*dir,mesh_vertices[(*it)[0]],mesh_vertices[(*it)[1]],mesh_vertices[(*it)[2]],faces_normals[(*it)[3]],u,v,t,results);
		if(results.size() != 0){
			if(fabs(t) < tmin){
				prj_point=results;
				tmin = fabs(t);
				face = distance(mesh_faces.begin(), it);
			}
			continue;
		}
	}
}

bool MeshBodyPart::findClosestFace(Vector &p, double &distance, int &face){
	Vector diff;
	double mindist=0xFFFFFFFF;
	bool found = false;
	/*for(unsigned int i = 0; i < mesh_faces.size(); i++){
		diff = p-mesh_vertices[mesh_faces[i][0]];
		distance = dot(faces_normals[mesh_faces[i][3]],diff);
		if(distance<0 && mindist>fabs(distance)){
			mindist = fabs(distance);
			face = i;
			found=true;
		}
	}
	distance = mindist;*/
	for(unsigned int i = 0; i < mesh_faces.size(); i++){
		diff = p-((mesh_vertices[mesh_faces[i][0]]+mesh_vertices[mesh_faces[i][1]]+mesh_vertices[mesh_faces[i][2]])/3);
		distance = norm(diff);
		if(dot(faces_normals[mesh_faces[i][3]],diff)>=0 && mindist>fabs(distance)){
			mindist = fabs(distance);
			face = i;
			found=true;
		}
	}
	distance = mindist;
	return found;
}

Vector& MeshBodyPart::getPoint(int i){
	return mesh_vertices[i];
}

vector<int>& MeshBodyPart::getFace(int i){
	return mesh_faces[i];
}

Vector& MeshBodyPart::getFaceNormal(int i){
	return faces_normals[mesh_faces[i][3]];
}

bool MeshBodyPart::isInitialized(){
	return initialized;
}
