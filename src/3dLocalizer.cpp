/*
 * 3dLocalizer.cpp
 *
 *  Created on: Aug 27, 2014
 *      Author: michal
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include "opencv2/opencv.hpp"
#include "Caster.h"
#include "CloudHandler.h"
#include "ImageHandler.h"

using namespace std;
using namespace pcl;
using namespace cv;

//#define CLOUD_FILE "/home/michal/PDM/3dLocalizer/PointClouds/pcd/sample.pcd"
#define CLOUD_FILE "/home/michal/PDM/3dLocalizer/PointClouds/pcd/scan_062.pcd"
#define VIDEO_FILE "UZULNIC"
#define IMAGE_FILE "/home/michal/Downloads/image.jpg"

#define VISUALIZATION
//#define VIEWER_TEST

vector<PointXYZ> assignCloudPoints();
vector<Point2f> assignImagePoints();

int main() {

	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

	if (io::loadPCDFile<PointXYZ>(CLOUD_FILE, *cloud) == -1) {
		cerr << "Could not load cloud data" << endl;
		return -1;
	}

	PointCloud<PointXYZRGB>::Ptr image(new PointCloud<PointXYZRGB>);

	Mat_<Vec3b> imageMat = imread(IMAGE_FILE);
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
			image->points.push_back(p);
		}

		visualization::CloudViewer viewer("Simple Cloud Viewer");
		viewer.showCloud(cloud, "C");
		viewer.showCloud(image, "I");
		while (!viewer.wasStopped()) {
		}
//Mat image = imread(IMAGE_FILE);

//	vector<PointXYZ> cloudPoints = assignCloudPoints();
//	vector<Point2f> imagePoints = assignImagePoints();

//	Caster cast = Caster();
//	vector<double> planeCoefs = cast.cloudToImage(cloudPoints, imagePoints).first;

	return 0;
}

vector<PointXYZ> assignCloudPoints() {
	cout << "Assign cloud points" << endl;
	return vector<PointXYZ>();
}
vector<Point2f> assignImagePoints() {
	cout << "assign imag points" << endl;
	return vector<Point2f>();
}
