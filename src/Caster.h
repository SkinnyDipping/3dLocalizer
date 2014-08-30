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
	class Quaternion
	{
	public:
		double w, x, y, z;

		Quaternion(double w, double x, double y, double z){
			this->w = w;
			this->x = x;
			this->y = y;
			this->z = z;
		}

		Quaternion(double angle, PointXYZ vector)
		{

			this->w = cos(angle / 2);
			double sinus = sin(angle / 2);
			this->x = vector.x*sinus;
			this->y = vector.y*sinus;
			this->z = vector.z*sinus;
		}

		Quaternion(double angle, vector<double> vector)
		{
			PointXYZ vector = PointXYZ(vector[0], vector[1], vector[2]);
			Quaternion(angle, vector);
		}

		Mat_<double> toTransformationMatrix()
		{
			float xx = x*x, yy = y*y, zz = z*z, xy = x*y, xz = x*z, yz = y*z, wx = w*x, wy = w*y, wz = w*z;
			Mat_<double> output = Mat(4, 4, CV_64F);

			output(0, 0) = 1 - 2 * (yy + zz);
			output(0, 1) = 2 * (xy + wz);
			output(0, 2) = 2 * (xz - wy);
			output(0, 3) = 0;

			output(1, 0) = 2 * (xy - wz);
			output(1, 1) = 1 - 2 * (xx + zz);
			output(1, 2) = 2 * (yz + wx);
			output(1, 3) = 0;

			output(2, 0) = 2 * (xz + wy);
			output(2, 1) = 2 * (yz - wx);
			output(2, 2) = 1 - 2 * (xx + yy);
			output(2, 3) = 0;

			output(3, 0) = 0;
			output(3, 1) = 0;
			output(3, 2) = 0;
			output(3, 3) = 1;

			return output;
		}
	};

public:
	//Tangential plane coefficients:
	//Ax + By + Cz + D = 0
	double A, B, C, D;
	vector<double> finestCoeff;

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
	 * @return (vector of plane coefficients, centroid of image keypoints)
	 */
	std::pair<vector<double>, PointXYZ> cloudToImage(vector<PointXYZ> cloudPoints,
			vector<Point2f> imagePoints);

	/*
	 * Casts 2d points on 3d plane
	 *
	 * Function casts points on 3d plane given by A, B, C, D.
	 * It not guarantees proper rotation after cast,
	 * it needs to be done manually.
	 *
	 * @param castedCentroid centroid on points after cast
	 * @return vector of castes coordinates
	 */
	vector<PointXYZ> imageOnPlane(PointXYZ center, vector<Point2f> points);
	static vector<PointXYZ> imageOnPlane(double A, double B, double C, double D,
			PointXYZ castedCentroid, vector<Point2f> points);

public:
	void calculateTangentialPlaneCoeff();
	vector<PointXYZ> castCloudPoints(vector<PointXYZ> points);
	vector<PointXYZ> castImagePoints(vector<Point2f> points);
	Mat_<float> calculateTransformationMatrix(vector<PointXYZ> imagePoints,
			vector<PointXYZ> cloudPoints);
	double MSE(vector<PointXYZ> set1, vector<PointXYZ> set2);

	//TODO implement
	double distance(PointXYZ p1, PointXYZ p1);
	double distance(Point2f p1, Point2f p2);

	PointXYZ transformPoint(PointXYZ point, Mat_<float> matrix);

};

#endif /* CASTER_H_ */

//TODO SCALE IS ALSO NEEDED
