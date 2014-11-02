/*
 * Quaternion.cpp
 *
 *  Created on: Sep 4, 2014
 *      Author: michal
 */

#include "Quaternion.h"

Quaternion::Quaternion() {
	this->w = 1;
	this->x = 0;
	this->y = 0;
	this->z = 0;
}

Quaternion::~Quaternion() {
	// TODO Auto-generated destructor stub
}

Quaternion::Quaternion(double angle, PointXYZ vector) {

	this->w = cos(angle / 2);
	double sinus = sin(angle / 2);
	this->x = vector.x * sinus;
	this->y = vector.y * sinus;
	this->z = vector.z * sinus;
}

Quaternion::Quaternion(double w, double x, double y, double z) {
	this->w = w;
	this->x = x;
	this->y = y;
	this->z = z;
}

void Quaternion::normalize() {
	double M = sqrt(w * w + x * x + y * y + z * z);
	this->x = x / M;
	this->y = y / M;
	this->z = z / M;
	this->w = w / M;
}

Quaternion Quaternion::conj() {
	return Quaternion(w, -x, -y, -z);
}

Quaternion Quaternion::inv() {
	Quaternion temp = this->conj();
	double N = temp.norm();
	temp.w = temp.w / N;
	temp.x = temp.x / N;
	temp.y = temp.y / N;
	temp.z = temp.z / N;
	return temp;
}

double Quaternion::norm() {
	return sqrt(w * w + x * x + y * y + z * z);
}

Quaternion Quaternion::operator*(Quaternion& q) {
//	Quaternion newQ = Quaternion();
//	newQ.w = w * q.w
//			- Utils::dotProduct(PointXYZ(x, y, z), PointXYZ(q.x, q.y, q.z));
//	PointXYZ v21 = Utils::crossProduct(PointXYZ(x, y, z),
//			PointXYZ(q.x, q.y, q.z));
//	PointXYZ v22 = PointXYZ(q.x * w, q.y * w, q.z * w);
//	PointXYZ v23 = PointXYZ(x * q.w, y * q.w, z * q.w);
//	PointXYZ v = Utils::add(v21, Utils::add(v22, v23));
//	newQ.x = v.x;
//	newQ.y = v.y;
//	newQ.z = v.z;
//	return newQ;
	double newW = w * q.w - x * q.x - y * q.y - z * q.z;
	double newX = w * q.x + x * q.w + y * q.z - z * q.y;
	double newY = w * q.y - x * q.z + y * q.w + z * q.x;
	double newZ = w * q.z + x * q.y - y * q.x + z * q.w;
	return Quaternion(newW, newX, newY, newZ);
}

Mat_<double> Quaternion::toTransformationMatrix() {
	float xx = x * x, yy = y * y, zz = z * z, xy = x * y, xz = x * z, yz = y
			* z, wx = w * x, wy = w * y, wz = w * z;
	Mat_<double> output = Mat(4, 4, CV_64F);

	output(0, 0) = 1 - 2 * (yy + zz);
	output(0, 1) = 2 * (xy - wz);
	output(0, 2) = 2 * (xz + wy);
	output(0, 3) = 0;

	output(1, 0) = 2 * (xy + wz);
	output(1, 1) = 1 - 2 * (xx + zz);
	output(1, 2) = 2 * (yz - wx);
	output(1, 3) = 0;

	output(2, 0) = 2 * (xz - wy);
	output(2, 1) = 2 * (yz + wx);
	output(2, 2) = 1 - 2 * (xx + yy);
	output(2, 3) = 0;

	output(3, 0) = 0;
	output(3, 1) = 0;
	output(3, 2) = 0;
	output(3, 3) = 1;

//	output(0, 0) = w*w+xx-yy-zz;
//	output(0, 1) = 2 * (xy - wz);
//	output(0, 2) = 2 * (xz + wy);
//	output(0, 3) = 0;
//
//	output(1, 0) = 2 * (xy + wz);
//	output(1, 1) = w*w-xx+yy-zz;
//	output(1, 2) = 2 * (yz - wx);
//	output(1, 3) = 0;
//
//	output(2, 0) = 2 * (xz - wy);
//	output(2, 1) = 2 * (yz + wx);
//	output(2, 2) = w*w-xx-yy+zz;
//	output(2, 3) = 0;
//
//	output(3, 0) = 0;
//	output(3, 1) = 0;
//	output(3, 2) = 0;
//	output(3, 3) = 1;

	return output;
}

PointXYZ Quaternion::rotate(PointXYZ point, Quaternion q,
		PointXYZ rotationCenter) {

	//Translation to center
	point.x -= rotationCenter.x;
	point.y -= rotationCenter.y;
	point.z -= rotationCenter.z;

	//Rotation
	Quaternion v = Quaternion(0, point.x, point.y, point.z);
	Quaternion temp = q.inv();
	Quaternion rotated = (q * v) * temp;

	//Reverse translation
	return PointXYZ(rotated.x + rotationCenter.x, rotated.y + rotationCenter.y,
			rotated.z + rotationCenter.z);
}

//vector<PointXYZ> Quaternion::rotate(vector<PointXYZ> points, Quaternion q,
		//PointXYZ rotationCenter) {
//
	////cout << "\nQUATERNION ROTATE\n";
	//for (int i = 0; i < points.size(); i++) {
////cout<<"\nPoint: "<<points[i]<<endl;
		////Translation to origin
		//points[i].x -= rotationCenter.x;
		//points[i].y -= rotationCenter.y;
		//points[i].z -= rotationCenter.z;
////cout<<"Translated to origin: "<<points[i]<<endl;
////cout<<"Points "<<points[i]<<endl;
		////Rotation
		//Quaternion v = Quaternion(0, points[i].x, points[i].y, points[i].z);
		//Quaternion temp = q.inv();
		//Quaternion rotated = (q * v) * temp;
////v.out();(q*v).out();temp.out();
////rotated.out();
		//points[i].x = rotated.x+rotationCenter.x;
		//points[i].y = rotated.y+rotationCenter.y19,;
		//points[i].z = rotated.z+rotationCenter.z;
////cout<<"Rotated and back to origin: "<<points[i]<<endl;
////cout<<points[i]<<endl;
	//}
//
	////Reverse translation
	//return points;
//}

vector<PointXYZ> Quaternion::rotate(vector<PointXYZ> points, Quaternion q, PointXYZ rotationCenter) {
	
	Mat_<double> point = Mat::zeros(4,1,CV_64F);
	Mat_<double> center = Mat::zeros(4,1,CV_64F);
	Mat rotationMatrix = q.toTransformationMatrix();

	center(0) = rotationCenter.x;
	center(1) = rotationCenter.y;
	center(2) = rotationCenter.z;
	center(3) = 1;

	//cout << "\nQUATERNION ROTATE\n";
	//cout<<"rotation center:\n"<<center<<endl;
//cout<<"rotation matrix:\n"<<rotationMatrix<<endl;

	for (int i=0; i<points.size(); i++) {
		point(0) = points[i].x;
		point(1) = points[i].y;
		point(2) = points[i].z;

		point -= center;
		point(3) = 1;
//cout<<"point:\n"<<point<<endl;
		point = rotationMatrix * point;
//cout << "rotated point:\n"<<point<<endl;
		point += center;

		points[i].x = point(0);
		points[i].y = point(1);
		points[i].z = point(2);
	}
	//cout<<"QUATERNION END"<<endl<<endl;
	return points;
}	

vector<PointXYZ> Quaternion::rotate(vector<Point2f> points, Quaternion q,
		PointXYZ rotationCenter) {
	vector<PointXYZ> dPoints;
	for (int i=0; i<points.size(); i++) {
		dPoints.push_back(PointXYZ(points[i].x, points[i].y, 0));	
	}
	return rotate(dPoints, q, rotationCenter);
}
