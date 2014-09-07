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
public:
	Sphere();
	virtual ~Sphere();
	static std::vector<PointXYZ> getSpherePoints(double R, double resolution);

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
