/*
 * TransformationMatrix.h
 *
 *  Created on: Sep 6, 2014
 *      Author: michal
 */

#ifndef TRANSFORMATIONMATRIX_H_
#define TRANSFORMATIONMATRIX_H_

#include "pcl/point_types.h"
#include "opencv2/opencv.hpp"
#include "Quaternion.h"

using namespace pcl;
using namespace cv;
using namespace std;

class TransformationMatrix {
public:
	TransformationMatrix();
	virtual ~TransformationMatrix();

	static Mat combineMatrix(PointXYZ targetPoint, Quaternion qtrn,
			Mat_<double> scaleMatrix = Mat::eye(4,4,CV_64F));
	Mat calculateTransformationMatrix(vector<PointXYZ> imagePoints,
			vector<PointXYZ> cloudPoints);
};

#endif /* TRANSFORMATIONMATRIX_H_ */
