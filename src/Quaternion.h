/*
 * Quaternion.h
 *
 *  Created on: Sep 4, 2014
 *      Author: michal
 */

#ifndef QUATERNION_H_
#define QUATERNION_H_

#include "pcl/point_types.h"
#include "opencv2/opencv.hpp"

using namespace pcl;
using namespace cv;

class Quaternion {

private:
	double w,x,y,z;

public:
	Quaternion();
	virtual ~Quaternion();
	//	Quaternion(double w, double x, double y, double z) {
	//		this->w = w;
	//		this->x = x;
	//		this->y = y;
	//		this->z = z;
	//	}

		Quaternion(double angle, PointXYZ vector);

	//	Quaternion(double angle, vector<double> vector) {
	//		PointXYZ Pvector = PointXYZ(vector[0], vector[1], vector[2]);
	//		Quaternion(angle, Pvector);
	//	}

		void normalize();

		Mat_<double> toTransformationMatrix();

		static PointXYZ rotate(PointXYZ p, Quaternion q);
};

#endif /* QUATERNION_H_ */
