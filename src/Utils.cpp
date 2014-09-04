/*
 * Utils.cpp
 *
 *  Created on: Sep 4, 2014
 *      Author: michal
 */

#include "Utils.h"

Utils::Utils() {
	// TODO Auto-generated constructor stub

}

Utils::~Utils() {
	// TODO Auto-generated destructor stub
}

PointXYZ Utils::crossProduct(PointXYZ& p1, PointXYZ& p2) {
	PointXYZ output;
	output.x = p1.y * p2.z - p1.z * p2.y;
	output.y = p1.z * p2.x - p1.x * p2.z;
	output.z = p1.x * p2.y - p1.y * p2.x;
	return output;
}

PointXYZ Utils::dotProduct(PointXYZ& p1, PointXYZ& p2)
{
	return PointXYZ(p1.x*p2.x,p1.y*p2.y, p1.z*p2.z);
}

PointXYZ Utils::add(PointXYZ& p1, PointXYZ& p2)
{
	return PointXYZ(p1.x+p2.x,p1.y+p2.y,p1.z+p2.z);
}
