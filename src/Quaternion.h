/*
 * Quaternion.h
 *
 *  Created on: Sep 4, 2014
 *      Author: michal
 */

#ifndef QUATERNION_H_
#define QUATERNION_H_

#include <ostream>

#include "pcl/point_types.h"
#include "opencv2/opencv.hpp"
#include "Utils.h"

using namespace pcl;
using namespace cv;

class Quaternion {

public:
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

		Quaternion(double angle, PointXYZ vector=PointXYZ(0,0,1));
		Quaternion(double w,double x,double y,double z);

	//	Quaternion(double angle, vector<double> vector) {
	//		PointXYZ Pvector = PointXYZ(vector[0], vector[1], vector[2]);
	//		Quaternion(angle, Pvector);
	//	}

		void normalize();
		Quaternion operator*(Quaternion& q);

		Quaternion inv();
		Quaternion conj();
		double norm();
		void out(){
			cout<<"<"<<w<<", "<<x<<", "<<y<<", "<<z<<">"<<endl;
		}


		Mat_<double> toTransformationMatrix();

		static PointXYZ rotate(PointXYZ point, Quaternion q, PointXYZ rotationPoint=PointXYZ(0,0,0));
		static Point2f rotate(Point2f point, Quaternion q, Point2f rotationPoint=Point2f(0,0));
		static vector<PointXYZ> rotate(vector<PointXYZ> point, Quaternion q, PointXYZ rotationCenter=PointXYZ(0,0,0));
		static vector<Point2f> rotate(vector<Point2f> points, Quaternion q, Point2f rotationCenter=Point2f(0,0));

};

#endif /* QUATERNION_H_ */
