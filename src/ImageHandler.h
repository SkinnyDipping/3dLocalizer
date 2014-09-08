/*
 * ImageHandler.h
 *
 *  Created on: Sep 7, 2014
 *      Author: michal
 */

#ifndef IMAGEHANDLER_H_
#define IMAGEHANDLER_H_

#include "opencv2/opencv.hpp"
#include "pcl/point_types.h"

using namespace cv;

class ImageHandler {

private:
	Mat image;
	vector<pcl::PointXYZRGB> image3d;

private:
	ImageHandler();
public:
	ImageHandler(std::string pathToImage);
	virtual ~ImageHandler();

	Mat& getImage();
	vector<pcl::PointXYZRGB>& getImage3d();
};

#endif /* IMAGEHANDLER_H_ */
