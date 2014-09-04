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

std::pair<vector<double>, PointXYZ> Caster::cloudToImage(
		vector<PointXYZ> cloudPoints, vector<Point2f> imagePoints) {
	cloudCasted = false;
	imageCasted = false;

	//Finding centroid of cloud
	PointXYZ cloudCentroid = PointXYZ(0, 0, 0);
	for (int i = 0; i < cloudPoints.size(); i++) {
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
	for (int i = 0; i < cloudPoints.size(); i++) {
		double temp = distance(this->centerSphere, cloudPoints[i]);
		if (temp > this->radius)
			this->radius = temp;
	}

	//Finding image centroid and transforming to origin
	Point2f imageCentroid = Point2f(0, 0);
	for (int i = 0; i < imagePoints.size(); i++) {
		imageCentroid.x += imagePoints[i].x;
		imageCentroid.y += imagePoints[i].y;
	}
	imageCentroid.x /= imagePoints.size();
	imageCentroid.y /= imagePoints.size();
	for (int i = 0; i < imagePoints.size(); i++) {
		imagePoints[i].x -= imageCentroid.x;
		imagePoints[i].y -= imageCentroid.y;
	}

	while (true) {
// TODO: wybierz punkt P
		PointXYZ P; //TEMP
		this->tangentialPoint = P;

		calculateTangentialPlaneCoeff();

		vector<PointXYZ> castedImage = imageOnPlane(this->A, this->B, this->C,
				this->D, this->tangentialPoint, imagePoints);
		vector<PointXYZ> castedCloud = castCloudPoints(cloudPoints);

		Mat_<float> transformationMatrix = calculateTransformationMatrix(
				castedImage, castedCloud);

//zacznij od poczatku lub skoncz
		if (true)
			break; //TODO zrobic warunek
	}

	std::pair<vector<double>, PointXYZ> output;
	output.first = finestCoeff;
	output.second = tangentialPoint;
	return output;
}

//FIXME
vector<PointXYZ> Caster::imageOnPlane(double A, double B, double C, double D,
		PointXYZ castedCentroid, vector<Point2f> points) {
	if (A == 1 && B == 3 && C == 3 && D == 7) {
		std::cerr
				<< "ABCD = 1337: Lack of initialization probable \n FUNCTION RETURN";
	}
	vector<PointXYZ> pointsIn3d;

	//Quaternion:
	// <cos phi/2 ; dx sin phi/2 ; dy sin phi/2 ; dz sin phi/2>
	// phi = arcos( C / sqrt(AA+BB+CC) )
	// alpha: Ax + By + D = 0
	// d = [-B, A, 0]

	double phi = acos(C / sqrt(A * A + B * B + C * C));
	double normVec = sqrt(B * B + A * A);
	PointXYZ vector = PointXYZ(B / normVec, -A / normVec, 0);

	cout << phi << endl << normVec << endl << vector << endl
			<< sqrt(
					vector.x * vector.x + vector.y * vector.y
							+ vector.z * vector.z) << endl;

	Quaternion q = Quaternion(phi, vector);

	Mat_<float> rotationMatrix = q.toTransformationMatrix();
//	cout<<rotationMatrix<<endl;
	for (int i = 0; i < points.size(); i++) {
		pointsIn3d.push_back(PointXYZ(points[i].x, points[i].y, 0));
		pointsIn3d[i] = transformPoint(pointsIn3d[i], rotationMatrix);
//		pointsIn3d[i] = Quaternion::rotate(pointsIn3d[i], q);
		pointsIn3d[i].x += castedCentroid.x;
		pointsIn3d[i].y += castedCentroid.y;
		pointsIn3d[i].z += castedCentroid.z;

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

	vector<PointXYZ> output;
	for (int i = 0; i < points.size(); i++) {
		double cx = points[i].x;
		double cy = points[i].y;
		double cz = points[i].z;
		double t = -1 * (A * cx + B * cy + C * cz + D)
				/ (SQ(A) + SQ(B) + SQ(C));
		output.push_back(PointXYZ(cx + A * t, cy + B * t, cz + C * t));
	}
	cloudCasted = true;
	return output;
}

Mat_<float> Caster::calculateTransformationMatrix(vector<PointXYZ> imagePoints,
		vector<PointXYZ> cloudPoints) {
	//TODO uncomment
//	if (!cloudCasted || !imageCasted)
//	{
//		cerr<<"Cast error";
//		return Mat_<float>(1,1,CV_32F);
//	}

	//Here point have to be in the same centroid (casted)

	//Calculating centroids
	PointXYZ C_centroid, I_centroid;
	for (int i = 0; i < imagePoints.size(); i++) {
		C_centroid.x += cloudPoints[i].x;
		C_centroid.y += cloudPoints[i].y;
		C_centroid.z += cloudPoints[i].z;
		I_centroid.x += imagePoints[i].x;
		I_centroid.y += imagePoints[i].y;
		I_centroid.z += imagePoints[i].z;
	}
	C_centroid.x /= cloudPoints.size();
	C_centroid.y /= cloudPoints.size();
	C_centroid.z /= cloudPoints.size();
	I_centroid.x /= imagePoints.size();
	I_centroid.y /= imagePoints.size();
	I_centroid.z /= imagePoints.size();

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

//double Caster::MSE(vector<PointXYZ> set1, vector<PointXYZ> set2) {
//	double MSE = 0;
//	for (int i = 0; i < set1.size(); i++) {
//		MSE += SQ(set1[i] - set2[i]);
//	}
//	return MSE / set1.size();
//}

PointXYZ Caster::transformPoint(PointXYZ point, Mat_<float> matrix) {
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

double Caster::distance(PointXYZ p1, PointXYZ p2) {
	return 5;	//TODO implement
}
