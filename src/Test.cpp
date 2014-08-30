#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include "opencv2/opencv.hpp"
#include "Caster.h"
#include <vector>

using namespace std;
using namespace pcl;
using namespace cv;

//#define TRANS_MATRIX
//#define QUAT
//#define TAN_PLANE
#define CLOUD_CAST

int main()
{
	Caster cast = Caster();

	cast.centerSphere = PointXYZ(5, 6, 9);
	cast.radius = 3;
	cast.tangentialPoint = PointXYZ(5, 9, 9);

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
#ifdef TAN_PLANE
	
	cast.calculateTengentialPlaneCoeff();
	cout<<cast.A<<"\t"<<cast.B<<"\t"<<cast.C<<"\t"<<cast.D<<endl;
#endif

#ifdef QUAT
	PointXYZ A = PointXYZ(-8,4);
	PointXYZ B = PointXYZ(-6,5);
	PointXYZ C = PointXYZ(-11,5);
	PointXYZ D = PointXYZ(-7,6);
	PointXYZ E = PointXYZ(-12,6);
	vector<PointXYZ> v;
	v.push_back(A);
	v.push_back(B);
	v.push_back(C);
	v.push_back(D);
	v.push_back(E);

	vector<PointXYZ> v2 = Caster::imageOnPlane(0, 3, 0, -27, PointXYZ(5, 9, 9), v);

	for (int i = 0; i < v2.size();i++)
	cout << v2[i] << endl;

#endif

	return 0;
}