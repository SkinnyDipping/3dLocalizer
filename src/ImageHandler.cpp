/*
 * ImageHandler.cpp
 *
 *  Created on: Sep 7, 2014
 *      Author: michal
 */

#include "ImageHandler.h"

using namespace pcl;

ImageHandler::ImageHandler(std::string pathToImage) {

	this->image = imread(pathToImage);
//	namedWindow("A");
//	imshow("A",image);
	Mat_<Vec3b> imageMat = image;
	for (int i = 0; i < imageMat.rows; i++)
		for (int j = 0; j < imageMat.cols; j++) {
			Vec3b color = imageMat(i, j);
			PointXYZRGB p;
			p.z = 0;
			p.x = j;
			p.y = i;
			p.b = color.val[0];
			p.g = color.val[1];
			p.r = color.val[2];
			image3d.push_back(p);
		}
//	waitKey(0);
}

ImageHandler::~ImageHandler() {
}

std::vector<pcl::PointXYZRGB>& ImageHandler::getImage3d() {
	return image3d;
}

Mat& ImageHandler::getImage(){
	return image;
}
