/*
 * ImageHandler.cpp
 *
 *  Created on: Sep 7, 2014
 *      Author: michal
 */

#include "ImageHandler.h"

ImageHandler::ImageHandler(std::string pathToImage) {
	// TODO load image

	Mat_<float> imageMat = image;

}

ImageHandler::~ImageHandler() {
	// TODO Auto-generated destructor stub
}

std::vector<pcl::PointXYZ>& ImageHandler::getImage3d()
{
	return image3d;
}
