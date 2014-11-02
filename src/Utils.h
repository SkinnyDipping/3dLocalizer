/*
 * Utils.h
 *
 *  Created on: Sep 4, 2014
 *      Author: michal
 */

#ifndef UTILS_H_
#define UTILS_H_

#include "pcl/point_types.h"
#include "opencv2/opencv.hpp"

using namespace pcl;
using namespace std;
using namespace cv;

class Utils {
public:
	Utils();
	virtual ~Utils();
	static PointXYZ crossProduct(PointXYZ p1, PointXYZ p2);
	static double dotProduct(PointXYZ p1, PointXYZ p2);
	static PointXYZ add(PointXYZ p1, PointXYZ p2);
	static PointXYZ normalizeVector(PointXYZ vector);

	static PointXYZ calculateCentroid(vector<PointXYZ> points);
	static Point2f calculateCentroid(vector<Point2f> points);

	static PointXYZ transformPoint(PointXYZ point, Mat_<float> matrix);
	static PointXYZRGB transformPoint(PointXYZRGB point, Mat_<float> matrix);
	static PointXYZ transformPoint(Point2f point, Mat matrix, Point2f centroid);
	static vector<PointXYZ> transformPoints(vector<Point2f>, Mat matrix);

	static void out(vector<PointXYZ> vector);

};

#endif /* UTILS_H_ */
