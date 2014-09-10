/*
 * Caster.cpp
 *
 *  Created on: Aug 26, 2014
 *      Author: michal
 */

#include "Caster.h"

#define SQ(a) (a*a)

//TEMP
void Caster::out(vector<PointXYZ> v) {
	for (int i = 0; i < v.size(); i++)
		cout << v[i].x << "\t" << v[i].y << "\t" << v[i].z << endl;
}

Caster::Caster() {
	A = 1;
	B = 3;
	C = 3;
	D = 7;
	minMSE = 9999999;
	radius = 0;
}

Caster::~Caster() {
}

Mat Caster::cloudToImage(vector<PointXYZ> cloudPoints,
		vector<Point2f> imagePoints) {
	cloudCasted = false;
	imageCasted = false;

	//Finding centroid of cloud
	PointXYZ cloudCentroid = Utils::calculateCentroid(cloudPoints);

	//Updating sphere parameters
	this->centerSphere = cloudCentroid;
	this->radius = 0;
	for (int i = 0; i < cloudPoints.size(); i++) {
		double temp = distance(this->centerSphere, cloudPoints[i]);
		if (temp > this->radius)
			this->radius = ceil(temp);
	}

	//Finding image centroid and transforming to origin
	Point2f imageCentroid = Utils::calculateCentroid(imagePoints);
	for (int i = 0; i < imagePoints.size(); i++) {
		imagePoints[i].x -= imageCentroid.x;
		imagePoints[i].y -= imageCentroid.y;
	}

	//Getting vector of sphere point
	vector<PointXYZ> spherePoints = Sphere::getSpherePoints(this->radius, this->centerSphere);
//	spherePoints.push_back(PointXYZ(0,24,-7)); //TEMP
//cout<<imagePoints<<endl;
	for (int spherePointIndex = 0; spherePointIndex < spherePoints.size(); spherePointIndex++) {

//TODO SPRAWDZIĆ TRANSLACJE DO ŚRODKA DO CHUJA JEBAKI WAŁA
//		PointXYZ P=PointXYZ(0,24,-7); //TEMP
		PointXYZ P = spherePoints[spherePointIndex];

		this->tangentialPoint = P;

		calculateTangentialPlaneCoeff();

		vector<PointXYZ> castedImage = imageOnPlane(this->A, this->B, this->C,
				this->D, this->tangentialPoint, imagePoints);
		vector<PointXYZ> castedCloud = castCloudPoints(cloudPoints);

		//This is the index of reference point.
		//TODO some intelligent choosing
		int referenceIdx = 0;

		//Calculating rotation angle (using dot product)
		PointXYZ I = castedImage[referenceIdx], C = castedCloud[referenceIdx];

		I = Utils::normalizeVector(I);
		C = Utils::normalizeVector(C);
		double angle = acos(Utils::dotProduct(I, C));

		//Rotating castedCloud points
		vector<PointXYZ> rotatedPoints;
		PointXYZ vector = Utils::normalizeVector(
				PointXYZ(this->A, this->B, this->C));

		Quaternion qtrn = Quaternion(angle, vector);


		//Actual rotation
		rotatedPoints = Quaternion::rotate(castedCloud, qtrn, tangentialPoint);

		//Scale and scaling
		PointXYZ scale = determineScale(castedImage[referenceIdx],
				rotatedPoints[referenceIdx]);
		for (int i = 0; i < castedImage.size(); i++) {
			castedImage[i].x *= scale.x;
			castedImage[i].y *= scale.y;
			castedImage[i].z *= scale.z;
		}

		//Calculating MSE
		double mse = MSE(castedCloud, castedImage);
		if (mse < minMSE) {
			this->quaternion = qtrn;
			this->scaleFactor = scale;
			this->minMSE = mse;
		}
#define THRESHOLD 20
		if (minMSE - THRESHOLD <= 0)
			break;
#undef THRESHOLD
	}

	return TransformationMatrix::combineMatrix(this->tangentialPoint,
			this->quaternion, this->scaleFactor);
}

//OK
vector<PointXYZ> Caster::imageOnPlane(double A, double B, double C, double D,
		PointXYZ castedCentroid, vector<Point2f> points) {
	if (A == 1 && B == 3 && C == 3 && D == 7) {
		std::cerr
				<< "ABCD = 1337: Lack of initialization probable \n FUNCTION RETURN";
	}
	vector<PointXYZ> pointsIn3d, points2;

	//Calculating centroid and translating to origin
//	Point2f I_centroid = Utils::calculateCentroid(points);
//	for (int i = 0; i < points.size(); i++) {
//		points[i].x -= I_centroid.x;
//		points[i].y -= I_centroid.y;
//	}

	//Quaternion:
	// <cos phi/2 ; dx sin phi/2 ; dy sin phi/2 ; dz sin phi/2>
	// phi = arcos( C / sqrt(AA+BB+CC) )
	// alpha: Ax + By + D = 0
	// d = [-B, A, 0]

	double phi = acos(C / sqrt(A * A + B * B + C * C));
	double normVec = sqrt(B * B + A * A + C * C);
	PointXYZ vector = PointXYZ(B / normVec, -A / normVec, 0);
	Quaternion q = Quaternion(-phi, vector);

	for (int i = 0; i < points.size(); i++) {
		pointsIn3d.push_back(PointXYZ(points[i].x, points[i].y, 0));
	}
	points2 = Quaternion::rotate(pointsIn3d, q, tangentialPoint);
	return points2;
}

//OK
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

//OK
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

//OK
double Caster::MSE(vector<PointXYZ> set1, vector<PointXYZ> set2) {
	double MSE = 0;
	for (int i = 0; i < set1.size(); i++)
		MSE += distance(set1[i], set2[i]) * distance(set1[i], set2[i]);
	MSE /= set1.size();
	return MSE;
}



//OK
PointXYZ Caster::determineScale(PointXYZ initialPoint, PointXYZ finalPoint) {
#define INF (1.0/0.0)
	double sx = finalPoint.x / initialPoint.x;
	double sy = finalPoint.y / initialPoint.y;
	double sz = finalPoint.z / initialPoint.z;
//	if (!(sx != sx || sx == INF || sx == -INF || sx==0)) {
//		sy = sx;
//		sz = sx;
//	} else if (!(sy != sy || sy == INF || sy == -INF||sy==0)) {
//		sx = sy;
//		sz = sy;
//	} else if (!(sz != sz || sz == INF || sz == -INF||sz==0)) {
//		sx = sz;
//		sy = sz;
//	} else {
//		sx = 1;
//		sy = 1;
//		sz = 1;
//	}
#undef INF
	return PointXYZ(sx, sy, sz);
}

//OK
double Caster::distance(PointXYZ p1, PointXYZ p2) {
	return sqrt(
			(p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y)
					+ (p1.z - p2.z) * (p1.z - p2.z));
}
