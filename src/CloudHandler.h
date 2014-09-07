/*
 * CloudHandler.h
 *
 *  Created on: Sep 7, 2014
 *      Author: michal
 */

#ifndef CLOUDHANDLER_H_
#define CLOUDHANDLER_H_

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

using namespace pcl;
using namespace std;

class CloudHandler {

private:
	PointCloud<PointXYZ>::Ptr cloud;
//	vector<PointXYZ>& keypoints;

private:
	CloudHandler();

public:

	CloudHandler(string pathToCloud);
	virtual ~CloudHandler();

	PointCloud<PointXYZ>::Ptr getCloud();
	vector<PointXYZ>& getKeypoints();

};

#endif /* CLOUDHANDLER_H_ */
