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
#include "Utils.h"
#include "Quaternion.h"
#include "TransformationMatrix.h"

using namespace cv;
using namespace std;
using namespace pcl;

class Caster {

//private:
public:
	//Tangential plane coefficients:
	//Ax + By + Cz + D = 0
	double A, B, C, D;

	//Virtual sphere properties: center and radius
	PointXYZ centerSphere;
	double radius;

	PointXYZ tangentialPoint;
	Quaternion quaternion;
	PointXYZ scaleFactor;
	double minMSE;

	bool cloudCasted, imageCasted;

public:
	Caster();
	virtual ~Caster();

	/*
	 * Casts cloud to image. Points in arguments are corresponding
	 *
	 * @return (vector of plane coefficients, centroid of image keypoints)
	 */
	Mat cloudToImage(vector<PointXYZ> cloudPoints, vector<Point2f> imagePoints);

	/*
	 * Casts 2d points on 3d plane
	 *
	 * Function casts points on 3d plane given by A, B, C, D.
	 * It does not guarantee proper rotation after cast,
	 * it needs to be done manually.
	 *
	 * @param castedCentroid centroid on points after cast
	 * @return vector of castes coordinates
	 */
	vector<PointXYZ> imageOnPlane(PointXYZ center, vector<Point2f> points);
	vector<PointXYZ> imageOnPlane(double A, double B, double C, double D,
			PointXYZ castedCentroid, vector<Point2f> points);

public:
	void inline calculateTangentialPlaneCoeff();
	vector<PointXYZ> castCloudPoints(vector<PointXYZ> points);
	vector<PointXYZ> castImagePoints(vector<Point2f> points);
	Mat_<float> calculateTransformationMatrix(vector<PointXYZ> imagePoints,
			vector<PointXYZ> cloudPoints);
	double MSE(vector<PointXYZ> set1, vector<PointXYZ> set2);

	double distance(PointXYZ p1, PointXYZ p2);

	static PointXYZ transformPoint(PointXYZ point, Mat_<float> matrix);
	PointXYZ determineScale(PointXYZ initialPoint, PointXYZ finalPoint);
	void out(vector<PointXYZ> v);

};

#endif /* CASTER_H_ */
