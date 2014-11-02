#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include "opencv2/opencv.hpp"
#include "Caster.h"
#include <vector>
#include "Utils.h"
#include "Quaternion.h"
#include "TransformationMatrix.h"

using namespace std;
using namespace pcl;
using namespace cv;

//#define TRANS_MATRIX //PASSED
//#define QUAT //FFAILED
//#define CLOUD_CAST //PASSED

//#define POINT_PARALLEL

void out(vector<PointXYZ> v) {
	for (int i = 0; i < v.size(); i++)
		cout << v[i].x << "\t" << v[i].y << "\t" << v[i].z << endl;
}

void out(vector<Point2f> v) {
	for (int i = 0; i < v.size(); i++)
		cout << v[i].x << "\t" << v[i].y<< endl;
}

void toFile(Quaternion q) {
//	ifstream plik.open("/tmp/points.txt");
//	plik.close();
}

int main() {
	cout << "TEST MAIN SOURCE CODE\n";
	Caster cast = Caster();

//	cast.centerSphere = PointXYZ(0, -2, -7);
//	cast.radius = 26;

#ifdef POINT_PARALLEL
	cast.tangentialPoint = PointXYZ(0, -2, 19);
#else
//	cast.tangentialPoint = PointXYZ(0, 24, -7);
#endif

	PointXYZ A = PointXYZ(19, -13, 6);
	PointXYZ B = PointXYZ(-19, 10, -4);
	PointXYZ C = PointXYZ(-5, -20, -11);
	PointXYZ D = PointXYZ(14, 4, -9);
	PointXYZ E = PointXYZ(-9, 9, -17);
//	cast.calculateTangentialPlaneCoeff();
	vector<PointXYZ> v;
	v.push_back(A);
	v.push_back(B);
	v.push_back(C);
	v.push_back(D);
	v.push_back(E);

//#define TEST1
//#define TEST2
//#define TEST3
//#define TEST4
//#define TEST5
//#define TEST6
#define TEST7

#ifdef TEST1
	Point2f A2 = Point2f(19,-13);
	Point2f B2 = Point2f(-19,-3);
	Point2f C2 = Point2f(-5,4);
	Point2f D2 = Point2f(14,2);
	Point2f E2 = Point2f(-9,10);
#endif
#ifdef TEST2
	Point2f A2 = Point2f(39,18);
	Point2f B2 = Point2f(1,28);
	Point2f C2 = Point2f(15,35);
	Point2f D2 = Point2f(34,33);
	Point2f E2 = Point2f(11,41);
#endif
#ifdef TEST3
	Point2f A2 = Point2f(32.3, -15.1);
	Point2f B2 = Point2f(-32.3, 1.9);
	Point2f C2 = Point2f(-8.5, 13.8);
	Point2f D2 = Point2f(23.8, 10.4);
	Point2f E2 = Point2f(-15.3, 24);
#endif
#ifdef TEST4
	Point2f A2 = Point2f(22.95,5.24);
	Point2f B2 = Point2f(-14.95,-5.10);
	Point2f C2 = Point2f(6.33,7.96);
	Point2f D2 = Point2f(11.12,15.73);
	Point2f E2 = Point2f(-12.79,11.16);
//	Point2f A2 = Point2f(19.45,4.3);
//	Point2f B2 = Point2f(-18.45,-6.04);
//	Point2f C2 = Point2f(-9.83,7.03);
//	Point2f D2 = Point2f(7.62,14.79);
//	Point2f E2 = Point2f(-16.29,10.22);
#endif
#ifdef TEST5
//	Point2f A2 = Point2f(39.02,4.01);
//	Point2f B2 = Point2f(-25.42,-13.57);
//	Point2f C2 = Point2f(-10.76,8.64);
//	Point2f D2 = Point2f(18.91,21.84);
//	Point2f E2 = Point2f(-21.75,14.07);
	Point2f A2 = Point2f(20.34,29.18);
	Point2f B2 = Point2f(-27.02,-17.79);
	Point2f C2 = Point2f(-.46,-16.2);
	Point2f D2 = Point2f(25.81,2.89);
	Point2f E2 = Point2f(1.25,-28.43);
#endif
#ifdef TEST6
	Point2f A2 = Point2f(9.95448,20.7583);
	Point2f B2 = Point2f(-17.9545,-6.90192);
	Point2f C2 = Point2f(-2.33013,-5.9641);
	Point2f D2 = Point2f(13.1244,5.26795);
	Point2f E2 = Point2f(-2.79423,-13.1603);
#endif
#ifdef TEST7
//TUTAJ MUSIALEM ZMIENIC ZNAK. PYTANIE: DLACZEGO KURTFA?
	Point2f A2 = Point2f(12.94082, 26.98579);
	Point2f B2 = Point2f(-23.34085,  -8.97250);
	Point2f C2 = Point2f(-3.02917,  -7.75333);
	Point2f D2 = Point2f(17.06172, 6.84834);
	Point2f E2 = Point2f(-3.63250, -17.10839);
#endif
	vector<Point2f> v3;
	v3.push_back(A2);
	v3.push_back(B2);
	v3.push_back(C2);
	v3.push_back(D2);
	v3.push_back(E2);

	Mat_<double> matrix = cast.cloudToImage(v, v3);
	cout <<"\nTHE RESULT:\n"<< matrix << endl;
	cout << "Cloud:\n";out(v);
	cout << "Transformed image:\n";
	Utils::out(Utils::transformPoints(v3, matrix));

//	Quaternion q = Quaternion(DEG2RAD(30), PointXYZ(0, 1, 0));
//	q.out();
//	cout << Quaternion::rotate(PointXYZ(32.3, 24, 15.1), q,PointXYZ(0,24,-7)) << endl;
//	cout << Quaternion::rotate(PointXYZ(-32.3, 24,-1.9), q,PointXYZ(0,24,-7) )<< endl;
//	cout << Quaternion::rotate(PointXYZ(-8.5, 24, -13.8), q,PointXYZ(0,24,-7)) << endl;
//	cout << Quaternion::rotate(PointXYZ(23.8,24, -10.4), q, PointXYZ(0,24,-7))<< endl;
//	cout << Quaternion::rotate(PointXYZ(-15.3, 24,-24), q,PointXYZ(0,24,-7)) << endl;

//	PointXYZ Ar = PointXYZ(19, -13, 6);
//	PointXYZ Br = PointXYZ(-19, 10, -4);
//	PointXYZ Cr = PointXYZ(-5, -20, -11);
//	PointXYZ Dr = PointXYZ(14, 4, -9);
//	PointXYZ Er = PointXYZ(-9, 9, -17);

//	PointXYZ Ar = PointXYZ(19,-13,0);
//	PointXYZ Br = PointXYZ(-19,-3,0);
//	PointXYZ Cr = PointXYZ(-5,4,0);
//	PointXYZ Dr = PointXYZ(14, 2, 0);
//	PointXYZ Er = PointXYZ(-9,10,0);

//	PointXYZ Ar = PointXYZ(18.6427,24,-15.511);
//	PointXYZ Br = PointXYZ(-9.27182,24,12.1315);
//	PointXYZ Cr = PointXYZ(2.83382,24,-9.70549);
//	PointXYZ Dr = PointXYZ(3.18318,24,-18.8266);
//	PointXYZ Er = PointXYZ(-15.3878,24,-3.08844);

//	PointXYZ Ar = PointXYZ(19, 0, -13);
//	PointXYZ Br = PointXYZ(-19, 0, -3);
//	PointXYZ Cr = PointXYZ(-5, 0, 4);
//	PointXYZ Dr = PointXYZ(14, 0, 2);
//	PointXYZ Er = PointXYZ(-9, 0, 10);

	PointXYZ Ar = PointXYZ(9.95448,0,-20.7583);
	PointXYZ Br = PointXYZ(-17.9545,0,6.90192);
	PointXYZ Cr = PointXYZ(-2.33013,0,5.9641);
	PointXYZ Dr = PointXYZ(13.1244,0,-5.26795);
	PointXYZ Er = PointXYZ(-2.79423,0,13.1603);


	vector<PointXYZ> v2;
	v2.push_back(Ar);
	v2.push_back(Br);
	v2.push_back(Cr);
	v2.push_back(Dr);
	v2.push_back(Er);

	cout<<"QUATERNION ROTATION:\n";
	Quaternion q = Quaternion(DEG2RAD(-30),PointXYZ(0,1,0));
	q.out();
	out(Quaternion::rotate(v2,q,PointXYZ(0,0,0)));
	cout<<q.toTransformationMatrix()<<endl;

//	PointXYZ p = PointXYZ(0,0,0);
//	PointXYZ p = PointXYZ(2,2,0);
//	PointXYZ s = PointXYZ(7,9,5);
//	Quaternion v = Quaternion(0, s.x, s.y, s.z);

//	PointXYZ I=p, C=p2;
//			I=Utils::normalizeVector(I);
//			C=Utils::normalizeVector(C);
//			cout<<I<<"\t"<<C<<endl;
//			double li = sqrt(I.x*I.x+I.y*I.y+I.z*I.z);
//			double lc = sqrt(C.x*C.x+C.y*C.y+C.z*C.z);
//			double dot = Utils::dotProduct(I,C);
//			double angle = acos(Utils::dotProduct(I,C));///li/lc);
//			cout<<angle*180/3.14<<endl;

//	Quaternion q = Quaternion(0.70711,0,-0.70711,0);
//	Quaternion q = Quaternion(1.57,PointXYZ(0,0,1));
//	Quaternion q=Quaternion(2,4,6,4);
//	Quaternion temp2 = q.inv();
//	q.normalize();
//	Quaternion temp = (q*v);
//	temp.out();
//	PointXYZ P=Quaternion::rotate(p2,q);
//	PointXYZ scale = cast.determineScale(P,p2);
//	double scale = p2.x/P.x;

//	cout<<P<<"\t"<<p2;//<<"\t"<<scale<<endl;
//	Mat_<double> translate = Mat::zeros(4,4,CV_64F);
//	cout<<translate(3,3)<<endl;
//	Mat matrix = TransformationMatrix::combineMatrix(p,q,PointXYZ(2,2,2));
//	cout<<matrix<<endl<<cast.transformPoint(p2,matrix)<<endl;

//	q.out();

#ifdef CLOUD_CAST
	vector<PointXYZ> a = cast.castCloudPoints(v);
#endif

#ifdef TRANS_MATRIX
#ifdef POINT_PARALLEL
	PointXYZ A2 = PointXYZ(2.15,7-2,19);
	PointXYZ B2 = PointXYZ(-2.47,-9.07,19);
	PointXYZ C2 = PointXYZ(6.22,-2.27,19);
	PointXYZ D2 = PointXYZ(-3,2.1,19);
	PointXYZ E2 = PointXYZ(-2.9,-5.75,19);
#else
	PointXYZ A2 = PointXYZ(-3.52, 24, 45.91 - 7);
	PointXYZ B2 = PointXYZ(-24.2, 24, -29.91 - 7);
	PointXYZ C2 = PointXYZ(1.93, 24, -12.66 - 7);
	PointXYZ D2 = PointXYZ(17.46, 24, 22.25 - 7);
	PointXYZ E2 = PointXYZ(8.32, 24, -25.59 - 7);
#endif
	vector<PointXYZ> v1, v2;
	v2.push_back(A2);
	v2.push_back(B2);
	v2.push_back(C2);
	v2.push_back(D2);
	v2.push_back(E2);
	out(v2);
	out(a);
	cout << cast.calculateTransformationMatrix(v2, a) << endl;
#endif

#ifdef QUAT
#ifdef POINT_PARALLEL
	Point2f A2 = Point2f(2.15,7-2);
	Point2f B2 = Point2f(-2.47,-9.07);
	Point2f C2 = Point2f(6.22,-2.27);
	Point2f D2 = Point2f(-3,2.1);
	Point2f E2 = Point2f(-2.9,-5.75);
#else
//	Point2f A2 = Point2f(-3.52, 45.91);
//	Point2f B2 = Point2f(-24.2, -29.91);
//	Point2f C2 = Point2f(1.93, -12.66);
//	Point2f D2 = Point2f(17.46, 22.25);
//	Point2f E2 = Point2f(8.32, -25.59);
	Point2f A2 = Point2f(6, 9);
	Point2f B2 = Point2f(3, 1);
	Point2f C2 = Point2f(3, 5);
	Point2f D2 = Point2f(8, 3);
	Point2f E2 = Point2f(0, 8);
#endif
	vector<Point2f> v3;
	v3.push_back(A2);
	v3.push_back(B2);
	v3.push_back(C2);
	v3.push_back(D2);
	v3.push_back(E2);
//	out(v);cout<<endl;
	cout << v3 << endl << endl;

//	vector<PointXYZ> v2 = Caster::imageOnPlane(0, 3, 0, -27, PointXYZ(5, 9, 9), v);
	vector<PointXYZ> outq = Caster::imageOnPlane(cast.A, cast.B, cast.C, cast.D,
			cast.tangentialPoint, v3);
	out(outq);
//	for (int i = 0; i < v2.size();i++)
//	cout << v2[i] << endl;
#endif

	return 0;
}
