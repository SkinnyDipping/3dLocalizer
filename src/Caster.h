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
#include "Sphere.h"

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
	Mat scaleMatrix;
	double minMSE;

	bool cloudCasted, imageCasted;

public:
	Caster();
	virtual ~Caster();

	/*
	 * Casts cloud to image. Points in arguments are corresponding
	 *
	 * @return Transformation Matrix image->cloud. Image's centroid shall be at origin
	 */
	Mat cloudToImage(vector<PointXYZ> cloudPoints, vector<Point2f> imagePoints);

private:
	/*
	 * Casts 2d points on 3d plane
	 *
	 * Function casts points on 3d plane given by A, B, C, D.
	 * It does not guarantee proper rotation after cast,
	 * it needs to be done manually.
	 *
	 * @param castedCentroid centroid on points after cast
	 * @return vector of casted coordinates
	 */
	vector<PointXYZ> imageOnPlane(PointXYZ center, vector<Point2f> points);
	vector<PointXYZ> imageOnPlane(double A, double B, double C, double D,
			PointXYZ castedCentroid, vector<Point2f> points);

	/*
	 * Calculates and assigns A,B,C,D coefficients of tangential plane in tangentialPoint
	 * (which lies on sphere)
	 */
	void inline calculateTangentialPlaneCoeff();

	/*
	 * Casts cloud points on a ABCD plane
	 */
	vector<PointXYZ> castCloudPoints(vector<PointXYZ> points);

	/*
	 * Casts image points on ABCD plane
	 *
	 * @param Points shall have z coordinate = 0
	 */
	vector<PointXYZ> castImagePoints(vector<Point2f> points);
	Mat_<float> calculateTransformationMatrix(vector<PointXYZ> imagePoints,
			vector<PointXYZ> cloudPoints);

	/*
	 * Computes Mean Square Error between two sets of points
	 */
	double MSE(vector<PointXYZ> set1, vector<PointXYZ> set2);

	/*
	 * Carthesian distance between two points
	 */
	double distance(PointXYZ p1, PointXYZ p2);


	Mat determineScale(PointXYZ initialPoint, PointXYZ finalPoint, PointXYZ centroid = PointXYZ(0,0,0));
	void out(vector<PointXYZ> v);	//TEMP

};

#endif /* CASTER_H_ */
