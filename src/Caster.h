/*
 * Caster.h
 *
 *  Created on: Aug 26, 2014
 *      Author: michal
 */

#ifndef CASTER_H_
#define CASTER_H_

#include <vector>
#include <iostream>
#include "pcl/point_types.h"
#include "pcl/io/io.h"
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;
using namespace pcl;

class Caster {

private:
	//Tangential plane coefficients:
	//Ax + By + Cz + D = 0
	double A, B, C, D;

	//Virtual sphere properties: center and radius
	PointXYZ centerSphere;
	double radius;

	PointXYZ tangentialPoint;

	bool cloudCasted, imageCasted;

public:
	Caster();
	virtual ~Caster();

	/*
	 * Casts cloud to image. Points in arguments are corresponding
	 *
	 * @return vector of plane coefficients
	 */
	vector<double> cloudToImage(vector<PointXYZ> cloudPoints,
			vector<Point2f> imagePoints);

	/*
	 * Casts 2d points on 3d plane
	 *
	 * Function assumes, that specified center is corresponding
	 * to basic point of image's coordinate system
	 *
	 * @param center Assumed center
	 * @return vector of castes coordinates
	 */
	vector<PointXYZ> imageOnPlane(PointXYZ center, vector<Point2f> points);
	vector<PointXYZ> imageOnPlane(double A, double B, double C, double D,
			PointXYZ center, vector<Point2f> points);

private:
	void calculateTangentialPlaneCoeff();
	vector<PointXYZ> castCloudPoints(vector<PointXYZ> points);
	vector<PointXYZ> castImagePoints(vector<Point2f> points);
	vector<PointXYZ> transformPoints(vector<PointXYZ> imagePoints,
			vector<PointXYZ> cloudPoints);
	double MSE(vector<PointXYZ> set1, vector<PointXYZ> set2);

};

#endif /* CASTER_H_ */
