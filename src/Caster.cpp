/*
 * Caster.cpp
 *
 *  Created on: Aug 26, 2014
 *      Author: michal
 */

#include "Caster.h"

#define SQ(a) (a*a)

Caster::Caster() {
	A = 1;
	B = 3;
	C = 3;
	D = 7;

}

Caster::~Caster() {

}

std::pair<vector<double>, PointXYZ> Caster::cloudToImage(vector<PointXYZ> cloudPoints,
		vector<Point2f> imagePoints) {
	cloudCasted=false;imageCasted=false;
	while(true)
	{
// TODO: wybierz punkt P
	PointXYZ P; //TEMP
	tangentialPoint = P;

	calculateTangentialPlaneCoeff();

	vector<PointXYZ> castedImage = imageOnPlane(A, B, C, D, tangentialPoint, imagePoints);
	vector<PointXYZ> castedCloud = castCloudPoints(cloudPoints);

// TODO: transformuj (SVD)

// oblicz stddev??

//zacznij od poczatku lub skoncz
	if(true) break;//TODO zrobic warunek
	}

	std::pair<vector<double>, PointXYZ> output;
	output.first.push_back(A);
	output.first.push_back(B);
	output.first.push_back(C);
	output.first.push_back(D);
	output.second=tangentialPoint;
	return output;
}

vector<PointXYZ> Caster::imageOnPlane(double A, double B, double C, double D, PointXYZ castedCentroid, vector<Point2f> points){
	if (A == 1 && B == 3 && C == 3 && D == 7) {
		std::cerr
				<< "ABCD = 1337: Lack of initialization probable \n FUNCTION RETURN";
	}
	vector<PointXYZ> pointsIn3d = vector<PointXYZ>();

	//TODO: code!

	//Quaternion:
	// <cos phi/2 ; dx sin phi/2 ; dy sin phi/2 ; dz sin phi/2>
	// phi = arcos( C / sqrt(AA+BB+CC) )
	// alpha: Ax + By + D = 0
	// d = [-B, A, 0] ?TODO: napewno?

	// Translation: (x,y) += tangential - centroid'
	//	vector<PointXYZ> pointsIn3d(points.size(), 0);

	return pointsIn3d;
}

void Caster::calculateTangentialPlaneCoeff() {
	/*
	 * x0, y0, z0	- center of sphere
	 * a, b, c		- tangential point (point on sphere)
	 */
	double a = tangentialPoint.x;
	double b = tangentialPoint.y;
	double c = tangentialPoint.z;
	double x0 = centerSphere.x;
	double y0 = centerSphere.y;
	double z0 = centerSphere.z;

	this->A = a - x0;
	this->B = b - y0;
	this->C = c - z0;
	this->D = -SQ(a) - SQ(b) - SQ(c) + a * x0 + b * y0 + c * z0;
}

vector<PointXYZ> Caster::castCloudPoints(vector<PointXYZ> points) {

	vector<PointXYZ> output = vector<PointXYZ>();
	for (int i = 0; i < points.size(); i++) {
		double cx = points[i].x;
		double cy = points[i].y;
		double cz = points[i].z;
		double t = -1 * (A * cx + B * cy + C * cz + D)
				/ (SQ(A) + SQ(B) + SQ(C));
		output[i].x = cx + A * t;
		output[i].y = cy + B * t;
		output[i].z = cz + C * t;
	}
	cloudCasted = true;
	return output;
}



vector<PointXYZ> Caster::transformPoints(vector<PointXYZ> imagePoints, vector<PointXYZ> cloudPoints)
{
	if (!cloudCasted || !imageCasted)
	{
		cerr<<"Cast error";
		return vector<PointXYZ>();
	}

	//Here point have to be in the same centroid (casted)

	//Calculating centroids
	PointXYZ C_centroid, I_centroid;
	for (int i=0; i<imagePoints.size(); i++)
	{
		C_centroid.x += cloudPoints[i].x;
		C_centroid.y += cloudPoints[i].y;
		C_centroid.z += cloudPoints[i].z;
		I_centroid.x += imagePoints[i].x;
		I_centroid.y += imagePoints[i].y;
		I_centroid.z += imagePoints[i].z;
	}
	C_centroid /= cloudPoints.size();
	I_centroid /= imagePoints.size();

	//Covariance matrix
	Mat_<float> covariance = Mat::zeros(3,3,CV_32F);
	for (int i=0; i<imagePoints.size();i++)
	{
		Mat_<float> I = Mat::zeros(1,3,CV_32F);
		Mat_<float> C = Mat::zeros(3,1,CV_32F);
		I(0) = imagePoints[i].x - I_centroid.x;
		I(1) = imagePoints[i].y - I_centroid.y;
		I(2) = imagePoints[i].z - I_centroid.z;
		C(0) = cloudPoints[i].x - C_centroid.x;
		C(1) = cloudPoints[i].y - C_centroid.y;
		C(2) = cloudPoints[i].z - C_centroid.z;
		covariance += C*I;
	}

	//SVD
	Mat_<float> U, S, Vt;
	SVD::compute(covariance, S, U, Vt);
	Mat_<float> R = Vt.t()*U.t();


}

//double Caster::MSE(vector<PointXYZ> set1, vector<PointXYZ> set2) {
//	double MSE = 0;
//	for (int i = 0; i < set1.size(); i++) {
//		MSE += SQ(set1[i] - set2[i]);
//	}
//	return MSE / set1.size();
//}
