/*
 * Utils.cpp
 *
 *  Created on: Sep 4, 2014
 *      Author: michal
 */

#include "Utils.h"

Utils::Utils() {
	// TODO Auto-generated constructor stub

}

Utils::~Utils() {
	// TODO Auto-generated destructor stub
}

PointXYZ Utils::crossProduct(PointXYZ p1, PointXYZ p2) {
	PointXYZ output;
	output.x = p1.y * p2.z - p1.z * p2.y;
	output.y = p1.z * p2.x - p1.x * p2.z;
	output.z = p1.x * p2.y - p1.y * p2.x;
	return output;
}

double Utils::dotProduct(PointXYZ p1, PointXYZ p2)
{
	return p1.x*p2.x+p1.y*p2.y+ p1.z*p2.z;
}

PointXYZ Utils::add(PointXYZ p1, PointXYZ p2)
{
	return PointXYZ(p1.x+p2.x,p1.y+p2.y,p1.z+p2.z);
}

PointXYZ Utils::normalizeVector(PointXYZ vector)
{
	double N = sqrt(vector.x*vector.x+vector.y*vector.y+vector.z*vector.z);
	return PointXYZ(vector.x/N, vector.y/N, vector.z/N);
}

PointXYZ Utils::calculateCentroid(vector<PointXYZ> points)
{
	PointXYZ centroid = PointXYZ(0,0,0);
	for (int i=0; i<points.size();i++)
	{
		centroid.x += points[i].x;
		centroid.y += points[i].y;
		centroid.z += points[i].z;
	}
	centroid.x /= points.size();
	centroid.y /= points.size();
	centroid.z /= points.size();

	return centroid;
}

Point2f Utils::calculateCentroid(vector<Point2f> points)
{
	Point2f centroid = Point2f(0,0);
	for (int i=0; i<points.size();i++)
	{
		centroid.x += points[i].x;
		centroid.y += points[i].y;
	}
	centroid.x /= points.size();
	centroid.y /= points.size();

	return centroid;
}

PointXYZ Utils::transformPoint(PointXYZ point, Mat_<float> matrix) {
	PointXYZ output = PointXYZ(0, 0, 0);
	Mat_<float> pointMat = Mat(4, 1, CV_32F);
	pointMat(0) = point.x;
	pointMat(1) = point.y;
	pointMat(2) = point.z;
	pointMat(3) = 1;
	Mat_<float> outputMat = matrix * pointMat;
	output.x = outputMat(0);
	output.y = outputMat(1);
	output.z = outputMat(2);

	return output;
}

PointXYZRGB Utils::transformPoint(PointXYZRGB point, Mat_<float> matrix) {
	PointXYZRGB output = PointXYZRGB(0, 0, 0);
	Mat_<float> pointMat = Mat(4, 1, CV_32F);
	pointMat(0) = point.x;
	pointMat(1) = point.y;
	pointMat(2) = point.z;
	pointMat(3) = 1;
	Mat_<float> outputMat = matrix * pointMat;
	output.x = outputMat(0);
	output.y = outputMat(1);
	output.z = outputMat(2);
	output.r = point.r;
	output.g = point.g;
	output.b = point.b;

	return output;
}
