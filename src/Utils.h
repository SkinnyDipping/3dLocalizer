/*
 * Utils.h
 *
 *  Created on: Sep 4, 2014
 *      Author: michal
 */

#ifndef UTILS_H_
#define UTILS_H_

#include "pcl/point_types.h"

using namespace pcl;

class Utils {
public:
	Utils();
	virtual ~Utils();
	static PointXYZ crossProduct(PointXYZ& p1, PointXYZ& p2);
	static PointXYZ dotProduct(PointXYZ& p1, PointXYZ& p2);
	static PointXYZ add(PointXYZ& p1, PointXYZ& p2);
};

#endif /* UTILS_H_ */
