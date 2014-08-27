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
	// TODO Auto-generated destructor stub
}

vector<double> Caster::cloudToImage(vector<PointXYZ> cloudPoints,
		vector<Point2f> imagePoints) {
	cloudCasted=false;imageCasted=false;
// wybierz punkt P

//oblicz plaszczyzne styczna

// rzutuj punkty chmury i obrazu

// transformuj (SVD)

// oblicz stddev

//zacznij od poczatku lub skoncz
	return vector<double>();
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

//	vector<PointXYZ> output(points.size(), 0);
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

vector<PointXYZ> Caster::castImagePoints(vector<Point2f> points) {
	if (A == 1 && B == 3 && C == 3 && D == 7) {
		std::cerr
				<< "ABCD = 1337: Lack of initialization probable \n FUNCTION RETURN";
	}
//	vector<PointXYZ> pointsIn3d(points.size(), 0);
	vector<PointXYZ> pointsIn3d = vector<PointXYZ>();
	//Ax + By + Cz + D = 0
	//z = (-Ax -By -D) / C
	for (int i = 0; i < points.size(); i++) {
		pointsIn3d[i].x = points[i].x + tangentialPoint.x;
		pointsIn3d[i].y = points[i].y + tangentialPoint.y;
		pointsIn3d[i].z = (-A * pointsIn3d[i].x - B * pointsIn3d[i].y - D) / C;
	}
	imageCasted = true;
	return pointsIn3d;
}

vector<PointXYZ> Caster::transformPoints(vector<PointXYZ> imagePoints, vector<PointXYZ> cloudPoints)
{
	if (!cloudCasted || imageCasted)
	{
		cerr<<"Cast error";
		return vector<PointXYZ>();
	}

	//Here point have to be in the same centroid (casted)

}

//double Caster::MSE(vector<PointXYZ> set1, vector<PointXYZ> set2) {
//	double MSE = 0;
//	for (int i = 0; i < set1.size(); i++) {
//		MSE += SQ(set1[i] - set2[i]);
//	}
//	return MSE / set1.size();
//}
