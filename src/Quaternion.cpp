/*
 * Quaternion.cpp
 *
 *  Created on: Sep 4, 2014
 *      Author: michal
 */

#include "Quaternion.h"

Quaternion::Quaternion() {
	this->w = 0;
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
	x = x / M;
	y = y / M;
	z = z / M;
	w = w / M;
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
	output(1, 1) = 1 - 2 * (xx - zz);
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
	Quaternion temp = q.conj();//.inv();
	Quaternion rotated = (q * v) * temp;

	//Reverse translation
	return PointXYZ(rotated.x+rotationCenter.x, rotated.y+rotationCenter.y, rotated.z+rotationCenter.z);
}
