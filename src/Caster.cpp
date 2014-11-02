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

std::ostream& operator<<(std::ostream &out, const Quaternion &q) {
	return out << "<" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << ">";
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

	cout << "imagePoints:\n" << imagePoints << endl;
	cout << "cloudPoints:\n";
	out(cloudPoints);

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
	cout << "SHPERE:\ncenter: " << this->centerSphere << "\nradius: "
			<< this->radius << endl;

	//Finding image centroid and transforming to origin
	Point2f imageCentroid = Utils::calculateCentroid(imagePoints);
	for (int i = 0; i < imagePoints.size(); i++) {
		imagePoints[i].x -= imageCentroid.x;
		imagePoints[i].y -= imageCentroid.y;
	}

	//Getting vector of sphere point
	vector<PointXYZ> spherePoints = Sphere::getSpherePoints(this->radius,
			this->centerSphere);
	int ii = 0;
//#define ALL_POINTS
#ifdef ALL_POINTS
	for (int spherePointIndex = 0; spherePointIndex < spherePoints.size(); spherePointIndex++) {
		PointXYZ P = spherePoints[spherePointIndex];
#else
	PointXYZ P = PointXYZ(this->centerSphere.x,
			this->centerSphere.y + this->radius, this->centerSphere.z); //TEMP
#endif

	this->tangentialPoint = P;
//	cout<<"tangential Point: "<<this->tangentialPoint<<endl;

	calculateTangentialPlaneCoeff();

	vector<PointXYZ> castedImage = imageOnPlane(this->A, this->B, this->C,
			this->D, this->tangentialPoint, imagePoints);
	vector<PointXYZ> castedCloud = castCloudPoints(cloudPoints);

	//This is the index of reference point.
	//TODO some intelligent choosing
	int referenceIdx = 0;

	//Calculating rotation angle (using dot product)
	PointXYZ I = castedImage[referenceIdx], C = castedCloud[referenceIdx];
	I.x -= tangentialPoint.x;
	I.y -= tangentialPoint.y;
	I.z -= tangentialPoint.z;
	C.x -= tangentialPoint.x;
	C.y -= tangentialPoint.y;
	C.z -= tangentialPoint.z;

	I = Utils::normalizeVector(I);
	C = Utils::normalizeVector(C);
	double angle = acos(Utils::dotProduct(I, C));

//	cout<<"Angle: "<<RAD2DEG(angle)<<endl;

	//Rotating castedImage points
	vector<PointXYZ> rotatedPoints;
	PointXYZ vector = Utils::normalizeVector(
			PointXYZ(this->A, this->B, this->C));

	Quaternion qtrn = Quaternion(angle, vector);
	this->quaternion = this->quaternion * qtrn;

	//Actual rotation
	rotatedPoints = Quaternion::rotate(castedImage, qtrn,
			this->tangentialPoint);

	//Scale and scaling
	Mat_<double> scaleMatrix = determineScale(castedCloud[referenceIdx],
			rotatedPoints[referenceIdx], tangentialPoint);
	for (int i = 0; i < castedImage.size(); i++) {
		castedImage[i] = Utils::add(castedImage[i],
				PointXYZ(-tangentialPoint.x, -tangentialPoint.y,
						-tangentialPoint.z));
		castedImage[i] = Utils::transformPoint(castedImage[i], scaleMatrix);
		castedImage[i] = Utils::add(castedImage[i], tangentialPoint);
	}
//	cout << "\nSCALE: " << scaleMatrix<<endl;
	this->scaleMatrix = scaleMatrix;
#ifdef ALL_POINTS
	//Calculating MSE
	double mse = MSE(castedCloud, castedImage);
	/**TEMP**/

	/****/
	if (mse < minMSE) {
		this->quaternion = qtrn;
		this->scaleMatrix = scaleMatrix;
		this->minMSE = mse;
		ii++;
		cout<<"\nMSE: "<<mse<<"\nPoint: "<<this->tangentialPoint<<"\niteration: "<<ii<<endl<<endl;
	}

#define THRESHOLD 2
//	if (minMSE - THRESHOLD <= 0)
//	break;
#undef THRESHOLD
}
#endif
//cout<<spherePoints.size();
	cout << "\nRESULT:\nTangential point: " << this->tangentialPoint
			<< "\nquaternion: " << this->quaternion << "\nscale matrix:\n"
			<< this->scaleMatrix << endl;
	return TransformationMatrix::combineMatrix(this->tangentialPoint,
			this->quaternion, this->scaleMatrix);
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
//	PointXYZ vector = PointXYZ(B, -A, 0);
	Quaternion q = Quaternion(-phi, vector);
	//cout<<q<<endl;

	for (int i = 0; i < points.size(); i++) {
		pointsIn3d.push_back(PointXYZ(points[i].x, points[i].y, 0));
	}
	this->quaternion = q;
	points2 = Quaternion::rotate(pointsIn3d, q);
	PointXYZ centroidAfterRotation = Utils::calculateCentroid(points2);
	PointXYZ translationVector;
	translationVector.x = this->tangentialPoint.x - centroidAfterRotation.x;
	translationVector.y = this->tangentialPoint.y - centroidAfterRotation.y;
	translationVector.z = this->tangentialPoint.z - centroidAfterRotation.z;
	for (int i = 0; i < points2.size(); i++) {
		points2[i].x += this->tangentialPoint.x;
		points2[i].y += this->tangentialPoint.y;
		points2[i].z += this->tangentialPoint.z;
	}
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
Mat Caster::determineScale(PointXYZ initialPoint, PointXYZ finalPoint,
		PointXYZ centroid) {
#define INF (1.0/0.0)
	double sx = (initialPoint.x - centroid.x) / (finalPoint.x - centroid.x);
	double sy = (initialPoint.y - centroid.y) / (finalPoint.y - centroid.y);
	double sz = (initialPoint.z - centroid.z) / (finalPoint.z - centroid.z);
	if (sx != sx || sx == INF)
		sx = 1;
	if (sy != sy || sy == INF)
		sy = 1;
	if (sz != sz || sz == INF)
		sz = 1;
//	if(sx!=sx || sy!= sy || sz!=sz)
//		cerr<<"Failed determining scale: scale coefficient is nan\n";
//	if (sx == INF || sy == INF || sz == INF)
//		cerr << "Failed determining scale: scale coefficient is INF\n";
#undef INF
//	return PointXYZ(sx, sy, sz);
	Mat_<double> matrix = Mat::eye(4, 4, CV_64F);
	matrix(0, 0) = sx;
	matrix(1, 1) = sy;
	matrix(2, 2) = sz;
	return matrix;
}

//OK
double Caster::distance(PointXYZ p1, PointXYZ p2) {
	return sqrt(
			(p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y)
					+ (p1.z - p2.z) * (p1.z - p2.z));
}
