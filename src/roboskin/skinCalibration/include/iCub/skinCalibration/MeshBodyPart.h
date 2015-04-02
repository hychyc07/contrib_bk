#ifndef __ICUB_MESHBODYPART_H__
#define __ICUB_MESHBODYPART_H__

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <iCub/ctrl/math.h>

#include <map>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include "iCub/skinCalibration/CalibrationSample.h"
#include "iCub/skinCalibration/SkinCalibrationLogger.h"
#include "yarp/math/Math.h"
#include "yarp/math/SVD.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


namespace iCub{
	namespace skinCalibration{

		class MeshBodyPart{

				vector<Vector> mesh_vertices;
				vector<Vector> mesh_vertices2D;
				vector<vector<int> > mesh_faces;
				vector<Vector> vertices_normals;
				vector<Vector> faces_normals;

				map<int, vector<int> > vertex_faces;
				map<int, vector<int> > vertex_neighbors;

				//string filefullpath;
				string meshfilename;

				double scale_factor;
				Matrix rotRefFrame;
				Matrix meshLaplacian;
				Vector translRefFrame;
				bool initialized;

				bool loadMeshFile();

				void rayTriangleIntersection(Vector o, Vector d,  Vector p0, Vector p1, Vector p2, Vector n, double &u, double &v, double &t, Vector &results);
				
				double computeAngle(Vector u, Vector v);
				void initializeVertexFacesMatrix();

			public:
				MeshBodyPart();
				MeshBodyPart(const char *mfilename, double scfactor, Matrix& rotationRefFrame, Vector& translationRefFrame);
				~MeshBodyPart();

				void findRayMeshIntersectionPoint(Vector &o, Vector &dir, Vector &int_point);
				void findRayMeshIntersectionPoint(Vector &o, Vector &dir, int &face, double &u, double &v, double &t, Vector &int_point);
				void findClosestProjectionOnMesh(Vector &o, Vector &dir, int &face, Vector &prj_point);
				void findVertexFaces(int v, vector<int>& result);
				void findVertexNeighbors(int v, vector<int>& result);
				bool findClosestFace(Vector &p, double &distance, int &face);

				void computeMeshLaplacian();
				void computeMeshBoundary(vector<int> &boundary);
				void computeParameterization();
				void extractSubMesh(vector<int> &vertices_id, MeshBodyPart &newMesh);


				Vector& getPoint(int i);
				vector<int>& getFace(int i);
				Vector& getFaceNormal(int i);

				bool isInitialized();
		};
	}
}

#endif //__ICUB_MESHBODYPART_H__
