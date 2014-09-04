#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include "opencv2/opencv.hpp"
#include "Caster.h"
#include <vector>
#include "Utils.h"

using namespace std;
using namespace pcl;
using namespace cv;

//#define TRANS_MATRIX
#define QUAT //FFAILED
//#define CLOUD_CAST

void out(vector<PointXYZ> v)
{
	for(int i=0; i<v.size(); i++)
			cout<<v[i].x<<"\t"<<v[i].y<<"\t"<<v[i].z<<endl;
}

int main()
{
	Caster cast = Caster();

	cast.centerSphere = PointXYZ(5, 6, 9);
	cast.radius = 3;
	cast.tangentialPoint = PointXYZ(5, 9, 9);
	cast.calculateTangentialPlaneCoeff();

#ifdef CLOUD_CAST
	PointXYZ A = PointXYZ(4,5,8);
	PointXYZ B = PointXYZ(5,6,6);
	PointXYZ C = PointXYZ(5,4,11);
	PointXYZ D = PointXYZ(6,8,7);
	PointXYZ E = PointXYZ(6,6,12);
	vector<PointXYZ> v;
	v.push_back(A);
	v.push_back(B);
	v.push_back(C);
	v.push_back(D);
	v.push_back(E);
	out(v);
	vector<PointXYZ> a = cast.castCloudPoints(v);
	out(a);
#endif

#ifdef TRANS_MATRIX
	PointXYZ A = PointXYZ(3, 3, 0);
	PointXYZ B = PointXYZ(-3, 3, 0);
	PointXYZ C = PointXYZ(-3, -3, 0);
	PointXYZ D = PointXYZ(3, -3, 0);
#define P2r3 4.242640687
	PointXYZ A2 = PointXYZ(0, P2r3, 0);
	PointXYZ B2 = PointXYZ(-P2r3,0, 0);
	PointXYZ C2 = PointXYZ(0, -P2r3, 0);
	PointXYZ D2 = PointXYZ(P2r3,0, 0);
#undef P2r3
	vector<PointXYZ> v1,v2;
	v1.push_back(A);
	v1.push_back(B);
	v1.push_back(C);
	v1.push_back(D);
	v2.push_back(A2);
	v2.push_back(B2);
	v2.push_back(C2);
	v2.push_back(D2);
	cout << cast.calculateTransformationMatrix(v1, v2)<<endl;
#endif

#ifdef QUAT
	Point2f A = Point2f(0,0);
	Point2f B = Point2f(-6,5);
	Point2f C = Point2f(-11,5);
	Point2f D = Point2f(-7,6);
	Point2f E = Point2f(-12,6);
	vector<Point2f> v;
	v.push_back(A);
	v.push_back(B);
	v.push_back(C);
	v.push_back(D);
	v.push_back(E);
//	out(v);cout<<endl;
	cout<<v<<endl<<endl;

	vector<PointXYZ> v2 = Caster::imageOnPlane(0, 3, 0, -27, PointXYZ(5, 9, 9), v);
out(v2);
//	for (int i = 0; i < v2.size();i++)
//	cout << v2[i] << endl;
#endif

	return 0;
}
