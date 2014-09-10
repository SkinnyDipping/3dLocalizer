/*
 * Sphere.cpp
 *
 *  Created on: Sep 7, 2014
 *      Author: michal
 */

#include "Sphere.h"

#define DEG2RAD 0.0174532925f
#define RAD2DEG 57.2957795f

Sphere::Sphere(PointXYZ center, double radius) {
	this->center = center;
	this->radius=radius;

}

Sphere::~Sphere() {
	// TODO Auto-generated destructor stub
}

std::vector<PointXYZ>& Sphere::generateSphere(double resolution)
{
	for (int i=-180; i<180; i+=resolution)
		for(int j=-180; j<180; j+=resolution)
		{
			PointXYZ p = Coors::sphericalToCartesian(PointXYZ(radius,i*DEG2RAD, j*DEG2RAD));
			p.x+=this->center.x;p.y+=this->center.y;p.z+=this->center.z;
			this->points.push_back(p);
		}
	return this->points;
}

std::vector<PointXYZ> Sphere::getSpherePoints(double R, PointXYZ center, double resolution) {
	std::vector<PointXYZ> spherePoints;
//TODO add resolution
	for (int i=-90; i<90; i++)
		for(int j=-90; j<90; j++)
		{
			PointXYZ p;
			p.x = R*cos(i*DEG2RAD)*cos(j*DEG2RAD)+center.x;
			p.y = R*cos(i*DEG2RAD)*sin(j*DEG2RAD)+center.y;
			p.z = R*sin(i*DEG2RAD)+center.z;
			spherePoints.push_back(p);
		}

	return spherePoints;
}

std::vector<double> Sphere::Coors::cartesianToSpherical(double x, double y,
		double z) {
	std::vector<double> coors;
	coors.push_back(sqrt(x * x + y * y + z * z));
	coors.push_back(atan(y / x));
	coors.push_back(asin(z / coors[0]));
	return coors;
}

std::vector<double> Sphere::Coors::sphericalToCartesian(double r, double theta,
		double phi) {
	std::vector<double> coors;
	coors.push_back(r * cos(theta) * cos(phi));
	coors.push_back(r * cos(theta) * sin(phi));
	coors.push_back(r * sin(theta));
	return coors;
}

PointXYZ Sphere::Coors::cartesianToSpherical(PointXYZ p) {
	PointXYZ out;
	out.x = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
	out.y = atan(p.y / p.x);
	out.z = asin(p.z / out.x);
	return out;
}

PointXYZ Sphere::Coors::sphericalToCartesian(PointXYZ p) {
	PointXYZ out;
	out.x = p.x * cos(p.y) * cos(p.z);
	out.y = p.x * cos(p.y) * sin(p.z);
	out.z = p.x * sin(p.y);
	return out;
}
