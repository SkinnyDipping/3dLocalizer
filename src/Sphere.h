/*
 * Sphere.h
 *
 *  Created on: Sep 7, 2014
 *      Author: michal
 */

#ifndef SPHERE_H_
#define SPHERE_H_

#include "pcl/point_types.h"

using namespace pcl;

class Sphere {
private:
	double radius;
	PointXYZ center;
	std::vector<PointXYZ> points;

public:
	Sphere(PointXYZ center, double radius);
	virtual ~Sphere();

	std::vector<PointXYZ>& generateSphere(double resolution=1);
	std::vector<PointXYZ>& getSpherePoints();

	static std::vector<PointXYZ> getSpherePoints(double Radius, PointXYZ center=PointXYZ(0,0,0), double resolution=1);

	class Coors {
	public:
		static std::vector<double> cartesianToSpherical(double x, double y,
				double z);
		static std::vector<double> sphericalToCartesian(double r, double theta,
				double phi);
		static PointXYZ cartesianToSpherical(PointXYZ p);
		static PointXYZ sphericalToCartesian(PointXYZ p);
	};
};

#endif /* SPHERE_H_ */
