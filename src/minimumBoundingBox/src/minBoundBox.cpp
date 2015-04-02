#include <iCub/data3D/minBoundBox.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;
using namespace iCub::data3D;

void buildRotMat2(const double &theta,yarp::sig::Matrix &rot)
{
    rot.resize(2,2);
    rot(0,0)=cos(theta);
    rot(0,1)=sin(theta);
    rot(1,0)=-sin(theta);
    rot(1,1)=cos(theta);
}

void buildRotMat3(const double &alpha,const double &beta,const double &gamma,yarp::sig::Matrix &rot)
{
    rot.resize(3,3);
    rot(0,0)=cos(beta)*cos(gamma);
    rot(0,1)=-cos(beta)*sin(gamma);
    rot(0,2)=sin(beta);
    rot(1,0)=sin(alpha)*sin(beta)*cos(gamma)+cos(alpha)*sin(gamma);
    rot(1,1)=-sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma);
    rot(1,2)=-sin(alpha)*cos(beta);
    rot(2,0)=-cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma);
    rot(2,1)=cos(alpha)*sin(beta)*sin(gamma)+sin(alpha)*cos(gamma);
    rot(2,2)=cos(alpha)*cos(beta);
}

void findRotation2D(const Matrix &xy, Matrix &rot2)
{
    yarp::sig::Vector edgeangles;
    double met=1000000;
    for (int j=0; j<xy.rows()-1; j++)
    {
        double val=atan2(xy(j+1,1)-xy(j,1),xy(j+1,0)-xy(j,0));
        bool contain=false;
        for (int k=0; k<edgeangles.size(); k++)
        {
            if (edgeangles[k]==Helpers::mod(val,M_PI/2))
            {
                contain=true;
                break;
            }
        }
        if (!contain)
        {
            edgeangles.push_back(Helpers::mod(val,M_PI/2));
            Matrix rot_i;
            buildRotMat2(-Helpers::mod(val,M_PI/2),rot_i);
            Matrix xyr=xy*rot_i;
            yarp::sig::Vector minimum;
            Helpers::min(xyr,minimum);
            yarp::sig::Vector maximum;
            Helpers::max(xyr,maximum);
            yarp::sig::Vector diff=maximum-minimum;
            double prod=diff[0]*diff[1];
            if (prod<met)
            {
                met=prod;
                rot2=rot_i;
            }
        }
    }
}

void findRotation(const Matrix &xyz_i, Matrix &rot2)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_on_plane(new pcl::PointCloud<pcl::PointXYZ>);

    for (int j=0; j<xyz_i.rows(); j++)
    {
        pcl::PointXYZ point;
        point.x=xyz_i(j,0);
        point.y=xyz_i(j,1);
        point.y=xyz_i(j,2);
        cloud_on_plane->push_back(point);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull2D (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> chull2D;
    chull2D.setInputCloud (cloud_on_plane);
    chull2D.setDimension(2);
    chull2D.reconstruct (*cloud_hull2D);

    Matrix xy(cloud_hull2D->size(),2);
    for (int j=0; j<cloud_hull2D->size()-1; j++)
    {
        xy(j,0)=cloud_hull2D->at(j).x;
        xy(j,1)=cloud_hull2D->at(j).y;
    }

    findRotation2D(xy,rot2);
}

void assignCorners(const Matrix &minmax, Matrix &cornerpoints)
{
    cornerpoints(0,0)=minmax(0,0);
    cornerpoints(0,1)=minmax(0,1);
    cornerpoints(0,2)=minmax(0,2);
    cornerpoints(1,0)=minmax(1,0);
    cornerpoints(1,1)=minmax(0,1);
    cornerpoints(1,2)=minmax(0,2);
    cornerpoints(2,0)=minmax(1,0);
    cornerpoints(2,1)=minmax(1,1);
    cornerpoints(2,2)=minmax(0,2);
    cornerpoints(3,0)=minmax(0,0);
    cornerpoints(3,1)=minmax(1,1);
    cornerpoints(3,2)=minmax(0,2);
    cornerpoints(4,0)=minmax(0,0);
    cornerpoints(4,1)=minmax(0,1);
    cornerpoints(4,2)=minmax(1,2);
    cornerpoints(5,0)=minmax(1,0);
    cornerpoints(5,1)=minmax(0,1);
    cornerpoints(5,2)=minmax(1,2);
    cornerpoints(6,0)=minmax(1,0);
    cornerpoints(6,1)=minmax(1,1);
    cornerpoints(6,2)=minmax(1,2);
    cornerpoints(7,0)=minmax(0,0);
    cornerpoints(7,1)=minmax(1,1);
    cornerpoints(7,2)=minmax(1,2);
}

void eulerAngles(const std::vector<yarp::sig::Vector> &edges1,const std::vector<yarp::sig::Vector> &edges2,const std::vector<yarp::sig::Vector> &crossProduct,yarp::sig::Vector &alpha,yarp::sig::Vector &beta,yarp::sig::Vector &gamma)
{
    double singamma;
    alpha.resize(crossProduct.size(),0.0);
    beta.resize(crossProduct.size(),0.0);
    gamma.resize(crossProduct.size(),0.0);
    for (int i=0; i<crossProduct.size(); i++)
    {
        double value=crossProduct.at(i)[0];
        beta[i]=asin(std::min(abs(value),1.0)*Helpers::sign(value));
        if (value==1.0)
            alpha[i]=asin(Helpers::sign(edges2.at(i)[2])*std::min(1.0,abs(edges2.at(i)[2])));
        else
        {
            double tmp=crossProduct.at(i)[2]/cos(beta[i]);
            alpha[i]=acos(Helpers::sign(tmp)*std::min(1.0,abs(tmp)));
            if (Helpers::sign(crossProduct.at(i)[1])!=Helpers::sign(-sin(alpha[i])*cos(beta[i])))
                alpha[i]=-alpha[i];
            singamma=-edges2.at(i)[0]/cos(beta[i]);
            if (edges1.at(i)[0]>=0)
                gamma[i]=asin(Helpers::sign(singamma)*std::min(1.0,abs(singamma)));
            else
                gamma[i]=-M_PI-asin(Helpers::sign(singamma)*std::min(1.0,abs(singamma)));
        }
    }
}

void retrieveEdges2(const yarp::sig::Vector &vx, const yarp::sig::Vector &vy, const yarp::sig::Vector &vz, const std::vector<yarp::sig::Vector> &edges1, std::vector<yarp::sig::Vector> &edges)
{
    for (int i=0; i<vx.size(); i++)
    {
        yarp::sig::Vector vect(3);
        vect[0]=vx[i];
        vect[1]=vy[i];
        vect[2]=vz[i];
        double dotprod=dot(edges1.at(i),vect);
        vect=vect-dotprod*edges1.at(i);
        double normvect=norm(vect);
        vect=vect/normvect;
        edges.push_back(vect);
    }
}

void retrieveEdges(const yarp::sig::Vector &vx, const yarp::sig::Vector &vy, const yarp::sig::Vector &vz, std::vector<yarp::sig::Vector> &edges)
{
    yarp::sig::Vector vxSquare=vx*vx;
    yarp::sig::Vector vySquare=vy*vy;
    yarp::sig::Vector vzSquare=vz*vz;
    yarp::sig::Vector sum=vxSquare+vySquare+vzSquare;

    for (int i=0; i<sum.size(); i++)
    {
        yarp::sig::Vector vect(3);
        vect[0]=vx[i]/(sqrt(sum[i]));
        vect[1]=vy[i]/(sqrt(sum[i]));
        vect[2]=vz[i]/(sqrt(sum[i]));
        edges.push_back(vect);
    }
}

BoundingBox MinimumBoundingBox::getMinimumBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    BoundingBox bb;
    std::vector< pcl::Vertices > polygons;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ConvexHull<pcl::PointXYZRGB> chull;
    chull.setInputCloud (cloud);
    chull.setDimension(3);
    chull.reconstruct (*cloud_hull,polygons);

    yarp::sig::Vector x1;
    yarp::sig::Vector x2;
    yarp::sig::Vector x3;
    yarp::sig::Vector y1;
    yarp::sig::Vector y2;
    yarp::sig::Vector y3;
    yarp::sig::Vector z1;
    yarp::sig::Vector z2;
    yarp::sig::Vector z3;

    for (int i=0; i<polygons.size(); i++)
    {
        pcl::Vertices vertex=polygons.at(i);
        x1.push_back(cloud_hull->at(vertex.vertices[0]).x);
        x2.push_back(cloud_hull->at(vertex.vertices[1]).x);
        x3.push_back(cloud_hull->at(vertex.vertices[2]).x);
        y1.push_back(cloud_hull->at(vertex.vertices[0]).y);
        y2.push_back(cloud_hull->at(vertex.vertices[1]).y);
        y3.push_back(cloud_hull->at(vertex.vertices[2]).y);
        z1.push_back(cloud_hull->at(vertex.vertices[0]).z);
        z2.push_back(cloud_hull->at(vertex.vertices[1]).z);
        z3.push_back(cloud_hull->at(vertex.vertices[2]).z);
    }

    Matrix pointCloud(cloud_hull->size(),3);
    for (int i=0; i<cloud_hull->size(); i++)
    {
        pointCloud(i,0)=cloud_hull->at(i).x;
        pointCloud(i,1)=cloud_hull->at(i).y;
        pointCloud(i,2)=cloud_hull->at(i).z;
    }

    yarp::sig::Vector v1x=x2-x1;
    yarp::sig::Vector v1y=y2-y1;
    yarp::sig::Vector v1z=z2-z1;
    std::vector<yarp::sig::Vector> edges1;

    retrieveEdges(v1x,v1y,v1z,edges1);

    yarp::sig::Vector v2x=x3-x1;
    yarp::sig::Vector v2y=y3-y1;
    yarp::sig::Vector v2z=z3-z1;
    std::vector<yarp::sig::Vector> edges2;

    retrieveEdges2(v2x,v2y,v2z,edges1,edges2);

    std::vector<yarp::sig::Vector> crossProduct;
    for (int i=0; i<edges1.size(); i++)
    {
        yarp::sig::Vector vect=cross(edges1.at(i),edges2.at(i));
        crossProduct.push_back(vect);
    }

    yarp::sig::Vector alpha;
    yarp::sig::Vector beta;
    yarp::sig::Vector gamma;
    eulerAngles(edges1, edges2, crossProduct, alpha, beta, gamma);

    Matrix minmax(2,3);
    double minVol=100000;
    Matrix rot2;

    for (int i=0; i<alpha.size(); i++)
    {
        Matrix rot;
        buildRotMat3(alpha[i],beta[i],gamma[i],rot);
        Matrix xyz_i=pointCloud*rot;
        findRotation(xyz_i,rot2);

        Matrix rot3dim=eye(3,3);
        rot3dim.setSubmatrix(rot2,0,0);
        Matrix rotation=rot*rot3dim;

        xyz_i=pointCloud*rotation;
        yarp::sig::Vector minimum;
        Helpers::min(xyz_i,minimum);
        yarp::sig::Vector maximum;
        Helpers::max(xyz_i,maximum);
        yarp::sig::Vector h=maximum-minimum;

        double prod=h[0]*h[1]*h[2];
        if (prod<minVol)
        {
            minVol=prod;
            bb.setOrientation(rotation);
            minmax.setRow(0,minimum);
            minmax.setRow(1,maximum);
        }
    }
    Matrix cornerpointsTmp(8,3);
    assignCorners(minmax,cornerpointsTmp);
    Matrix cornerpoints=cornerpointsTmp*(bb.getOrientation().transposed());

    std::vector<iCub::data3D::PointXYZ> corners;
    for (unsigned int i=0; i<cornerpoints.rows(); i++)
        corners.push_back(PointXYZ(cornerpoints(i,0),cornerpoints(i,1),cornerpoints(i,2)));

    bb.setCorners(corners);
    return bb;
}

