/*
 * CloudHandler.cpp
 *
 *  Created on: Sep 7, 2014
 *      Author: michal
 */

#include "CloudHandler.h"

CloudHandler::CloudHandler(string pathToCloud) {

	if (io::loadPCDFile<PointXYZ>(pathToCloud, *cloud) == -1) {
		cerr << "Could not load cloud data" << endl;
//					return -1;
	}
}

CloudHandler::~CloudHandler() {
	// TODO Auto-generated destructor stub
}

PointCloud<PointXYZ>::Ptr CloudHandler::getCloud()
{
	return cloud;
}
