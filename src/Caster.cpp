/*
 * Caster.cpp
 *
 *  Created on: Aug 26, 2014
 *      Author: michal
 */

#include "Caster.h"

#define SQ(a) (a*a)

Caster::Caster() {
	A = 1;
	B = 3;
	C = 3;
	D = 7;
	vector<double> finest = vector<double>();

}

Caster::~Caster() {

}

std::pair<vector<double>, PointXYZ> Caster::cloudToImage(vector<PointXYZ> cloudPoints,
		vector<Point2f> imagePoints) {
	cloudCasted=false;imageCasted=false;

	//Finding centroid of cloud
	PointXYZ cloudCentroid = PointXYZ(0, 0, 0); //TODO constructor ok?
	for (int i = 0; i < cloudPoints.size(); i++)
	{
		cloudCentroid.x += cloudPoints[i].x;
		cloudCentroid.y += cloudPoints[i].y;
		cloudCentroid.z += cloudPoints[i].z;
	}
	cloudCentroid.x /= cloudPoints.size();
	cloudCentroid.y /= cloudPoints.size();
	cloudCentroid.z /= cloudPoints.size();

	//Updating sphere parameters
	this->centerSphere = cloudCentroid;
	this->radius = 0;
	for (int i = 0; i < cloudPoints.size(); i++)
	{
		float temp = distance(this->centerSphere, cloudPoints[i]);
		if (temp>this->radius) this->radius = temp;
	}

	//Finding image centroid and transforming to origin
	Point2f imageCentroid = Point2f(0, 0, CV_32F);
	for (int i = 0; i < imagePoints.size(); i++)
	{
		imageCentroid.x += imagePoints[i].x;
		imageCentroid.y += imagePoints[i].y;
		imageCentroid.z += imagePoints[i].z;
	}
	imageCentroid /= imagePoints.size();
	for (int i = 0; i < imagePoints.size(); i++)
	{
		imagePoints[i].x -= imageCentroid.x;
		imagePoints[i].y -= imageCentroid.y;
		imagePoints[i].z -= imageCentroid.z;
	}



	while(true)
	{
// TODO: wybierz punkt P
	PointXYZ P; //TEMP
	this->tangentialPoint = P;

	calculateTangentialPlaneCoeff();

	vector<PointXYZ> castedImage = imageOnPlane(this->A, this->B, this->C, this->D, this->tangentialPoint, imagePoints);
	vector<PointXYZ> castedCloud = castCloudPoints(cloudPoints);

	Mat_<float> transformationMatrix = calculateTransformationMatrix(castedImage, castedCloud);

//zacznij od poczatku lub skoncz
	if(true) break;//TODO zrobic warunek
	}

	std::pair<vector<double>, PointXYZ> output;
	output.first = finestCoeff;
	output.second=tangentialPoint;
	return output;
}

vector<PointXYZ> Caster::imageOnPlane(double A, double B, double C, double D, PointXYZ castedCentroid, vector<Point2f> points){
	if (A == 1 && B == 3 && C == 3 && D == 7) {
		std::cerr
				<< "ABCD = 1337: Lack of initialization probable \n FUNCTION RETURN";
	}
	vector<PointXYZ> pointsIn3d = vector<PointXYZ>();


	//Quaternion:
	// <cos phi/2 ; dx sin phi/2 ; dy sin phi/2 ; dz sin phi/2>
	// phi = arcos( C / sqrt(AA+BB+CC) )
	// alpha: Ax + By + D = 0
	// d = [-B, A, 0] ?TODO: napewno?

	double phi = acos(C / sqrt(A*A + B*B + C*C));
	PointXYZ vector = PointXYZ(-B, A, 0);

	Quaternion q = Quaternion(angle, vector);

	Mat_<float> rotationMatrix = q.toTransformationMatrix();

	for (int i = 0; i < points.size(); i++)
	{
		pointsIn3d[i].x = points[i].x;
		pointsIn3d[i].y = points[i].y;
		pointsIn3d[i].z = 0;
		pointsIn3d[i] = transformPoint(pointsIn3d[i], rotationMatrix);
		
	}
	//TODO
	// Translation: (x,y) += tangential - centroid'
	//	vector<PointXYZ> pointsIn3d(points.size(), 0);

	return pointsIn3d;
}

void Caster::calculateTangentialPlaneCoeff() {
	/*
	 * x0, y0, z0	- center of sphere
	 * a, b, c		- tangential point (point on sphere)
	 */
	double a = tangentialPoint.x;
	double b = tangentialPoint.y;
	double c = tangentialPoint.z;
	double x0 = centerSphere.x;
	double y0 = centerSphere.y;
	double z0 = centerSphere.z;

	this->A = a - x0;
	this->B = b - y0;
	this->C = c - z0;
	this->D = -SQ(a) - SQ(b) - SQ(c) + a * x0 + b * y0 + c * z0;
}

vector<PointXYZ> Caster::castCloudPoints(vector<PointXYZ> points) {

	vector<PointXYZ> output = vector<PointXYZ>();
	for (int i = 0; i < points.size(); i++) {
		double cx = points[i].x;
		double cy = points[i].y;
		double cz = points[i].z;
		double t = -1 * (A * cx + B * cy + C * cz + D)
				/ (SQ(A) + SQ(B) + SQ(C));
		output[i].x = cx + A * t;
		output[i].y = cy + B * t;
		output[i].z = cz + C * t;
	}
	cloudCasted = true;
	return output;
}



Mat_<float> Caster::calculateTransformationMatrix(vector<PointXYZ> imagePoints, vector<PointXYZ> cloudPoints)
{
	if (!cloudCasted || !imageCasted)
	{
		cerr<<"Cast error";
		return vector<PointXYZ>();
	}

	//Here point have to be in the same centroid (casted)

	//Calculating centroids
	PointXYZ C_centroid, I_centroid;
	for (int i=0; i<imagePoints.size(); i++)
	{
		C_centroid.x += cloudPoints[i].x;
		C_centroid.y += cloudPoints[i].y;
		C_centroid.z += cloudPoints[i].z;
		I_centroid.x += imagePoints[i].x;
		I_centroid.y += imagePoints[i].y;
		I_centroid.z += imagePoints[i].z;
	}
	C_centroid /= cloudPoints.size();
	I_centroid /= imagePoints.size();

	//Covariance matrix
	Mat_<float> covariance = Mat::zeros(3,3,CV_32F);
	for (int i=0; i<imagePoints.size();i++)
	{
		Mat_<float> I = Mat::zeros(3,1,CV_32F);
		Mat_<float> C = Mat::zeros(1,3,CV_32F);
		I(0) = imagePoints[i].x - I_centroid.x;
		I(1) = imagePoints[i].y - I_centroid.y;
		I(2) = imagePoints[i].z - I_centroid.z;
		C(0) = cloudPoints[i].x - C_centroid.x;
		C(1) = cloudPoints[i].y - C_centroid.y;
		C(2) = cloudPoints[i].z - C_centroid.z;
		covariance += I*C;
	}

	//SVD
	Mat_<float> U, S, Vt;
	SVD::compute(covariance, S, U, Vt);
	Mat_<float> R = Vt.t()*U.t();
	if (determinant(R) < 0)
	{
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

	Mat_<float> T = -1 * R*centroidI + centroidC;

	Mat_<float> transfomationMatrix = Mat_<float>(4, 4, C_32F);

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

//double Caster::MSE(vector<PointXYZ> set1, vector<PointXYZ> set2) {
//	double MSE = 0;
//	for (int i = 0; i < set1.size(); i++) {
//		MSE += SQ(set1[i] - set2[i]);
//	}
//	return MSE / set1.size();
//}

PointXYZ Caster::transformPoint(PointXYZ point, Mat_<float> matrix)
{
	PointXYZ output = PointXYZ(0, 0, 0);
	Mat_<float> pointMat = Mat(4, 1, CV_32F);
	pointMat(0) = point.x;
	pointMat(1) = point.y;
	pointMat(2) = point.z;
	pointMat(3) = 1;
	Mat_<float> outputMat = matrix*pointMat;
	output.x = outputMat(0);
	output.y = outputMat(1);
	output.z = outputMat(2);

	return output;
}