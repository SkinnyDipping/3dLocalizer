/*
 * Quaternion.cpp
 *
 *  Created on: Sep 4, 2014
 *      Author: michal
 */

#include "Quaternion.h"

Quaternion::Quaternion() {
	// TODO Auto-generated constructor stub

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

void Quaternion::normalize() {
	double M = sqrt(w * w + x * x + y * y + z * z);
	x = x / M;
	y = y / M;
	z = z / M;
	w = w / M;
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

//PointXYZ Quaternion::rotate(PointXYZ p, Quaternion q) {
//	//Vnew = v + 2r x (r x v + wv)
//	PointXYZ r = PointXYZ(q.x, q.y, q.z);
//	PointXYZ r2 = PointXYZ(q.x * 2, q.y * 2, q.z * 2);
//	PointXYZ a = PointXYZ(p.x * q.w, p.y * q.w, p.z * q.w);
//	PointXYZ c = Utils::crossProduct(r, p);
//	PointXYZ b = Utils::add(a, c);
//	PointXYZ d = Utils::crossProduct(r2, b);
//	return Utils::add(p, d);
//}
