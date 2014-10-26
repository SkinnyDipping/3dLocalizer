/*
 * TransformationMatrix.cpp
 *
 *  Created on: Sep 6, 2014
 *      Author: michal
 */

#include "TransformationMatrix.h"

TransformationMatrix::TransformationMatrix() {
	// TODO Auto-generated constructor stub

}

TransformationMatrix::~TransformationMatrix() {
	// TODO Auto-generated destructor stub
}

Mat TransformationMatrix::combineMatrix(PointXYZ targetPoint, Quaternion qtrn,
		Mat_<double> scaleMatrix) {
	Mat_<double> rotation = qtrn.toTransformationMatrix();
//	Mat_<double> scale = Mat::eye(4, 4, CV_64F);
	Mat_<double> translate = Mat::eye(4, 4, CV_64F);
	Mat_<double> qtrnMat = qtrn.toTransformationMatrix();

//	scale(0, 0) = scaleFactor.x;
//	scale(1, 1) = scaleFactor.y;
//	scale(2, 2) = scaleFactor.z;

	translate(0, 3) = targetPoint.x;
	translate(1, 3) = targetPoint.y;
	translate(2, 3) = targetPoint.z;

	Mat temp = translate * scaleMatrix;
	return (temp * rotation);
}

Mat TransformationMatrix::calculateTransformationMatrix(
		vector<PointXYZ> imagePoints, vector<PointXYZ> cloudPoints) {
	//TODO uncomment
//	if (!cloudCasted || !imageCasted)
//	{
//		cerr<<"Cast error";
//		return Mat_<float>(1,1,CV_32F);
//	}

//Here point have to be in the same centroid (casted)

	//Calculating centroids
	PointXYZ C_centroid = Utils::calculateCentroid(cloudPoints);
	PointXYZ I_centroid = Utils::calculateCentroid(imagePoints);

	for (int i = 0; i < imagePoints.size(); i++) {
		imagePoints[i].x -= I_centroid.x;
		imagePoints[i].y -= I_centroid.y;
		imagePoints[i].z -= I_centroid.z;
		cloudPoints[i].y -= C_centroid.y;
		cloudPoints[i].x -= C_centroid.x;
		cloudPoints[i].z -= C_centroid.z;
	}

	//Covariance matrix
	Mat_<float> covariance = Mat::zeros(3, 3, CV_32F);
	for (int i = 0; i < imagePoints.size(); i++) {
		Mat_<float> I = Mat::zeros(3, 1, CV_32F);
		Mat_<float> C = Mat::zeros(1, 3, CV_32F);
		I(0) = imagePoints[i].x - I_centroid.x;
		I(1) = imagePoints[i].y - I_centroid.y;
		I(2) = imagePoints[i].z - I_centroid.z;
		C(0) = cloudPoints[i].x - C_centroid.x;
		C(1) = cloudPoints[i].y - C_centroid.y;
		C(2) = cloudPoints[i].z - C_centroid.z;
		covariance += I * C;
	}

	//SVD
	Mat_<float> U, S, Vt;
	SVD::compute(covariance, S, U, Vt);
	Mat_<float> R = Vt.t() * U.t();
	if (determinant(R) < 0) {
		R(0, 2) *= -1;
		R(1, 2) *= -1;
		R(2, 2) *= -1;
	}

	Mat_<float> centroidI = Mat_<float>(3, 1, CV_32F);
	centroidI(0) = I_centroid.x;
	centroidI(1) = I_centroid.y;
	centroidI(2) = I_centroid.z;
	Mat_<float> centroidC = Mat_<float>(3, 1, CV_32F);
	centroidC(0) = C_centroid.x;
	centroidC(1) = C_centroid.y;
	centroidC(2) = C_centroid.z;

	Mat_<float> T = -1 * R * centroidI + centroidC;

	Mat_<float> transformationMatrix = Mat_<float>(4, 4, CV_32F);

	transformationMatrix(0, 0) = R(0, 0);
	transformationMatrix(0, 1) = R(0, 1);
	transformationMatrix(0, 2) = R(0, 2);
	transformationMatrix(1, 0) = R(1, 0);
	transformationMatrix(1, 1) = R(1, 1);
	transformationMatrix(1, 2) = R(1, 2);
	transformationMatrix(2, 0) = R(2, 0);
	transformationMatrix(2, 1) = R(2, 1);
	transformationMatrix(2, 2) = R(2, 2);
	transformationMatrix(0, 3) = T(0);
	transformationMatrix(1, 3) = T(1);
	transformationMatrix(2, 3) = T(2);
	transformationMatrix(3, 3) = 1;
	transformationMatrix(3, 2) = 0;
	transformationMatrix(3, 1) = 0;
	transformationMatrix(3, 0) = 0;

	return transformationMatrix;
}
