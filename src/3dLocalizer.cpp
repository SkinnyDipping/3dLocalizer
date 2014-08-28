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

using namespace std;
using namespace pcl;
using namespace cv;

#define CLOUD_FILE "/home/michal/PDM/3dLocalizer/PointClouds/pcd/sample2.pcd"
#define VIDEO_FILE "UZULNIC"

//#define VISUALIZATION

vector<PointXYZ> assignCloudPoints();
vector<Point2f> assignImagePoints();

int main() {

#ifdef VISUALIZATION
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

	if (io::loadPCDFile<PointXYZ>(CLOUD_FILE, *cloud) == -1) {
		cerr << "Could not load cloud data" << endl;
		return -1;
	}

	VideoCapture videoSource(VIDEO_FILE);
	Mat calibrationFrame;
	videoSource >> calibrationFrame;
#endif

//		for (int i = 0; i < cloud->points.size(); i++)
//			cout << cloud->points[i] << endl;

//		visualization::CloudViewer viewer("Simple Cloud Viewer");
//		viewer.showCloud(cloud, "C");
//		while (!viewer.wasStopped()) {
//		}

	vector<PointXYZ> cloudPoints = assignCloudPoints();
	vector<Point2f> imagePoints = assignImagePoints();

	Caster cast = Caster();
	vector<double> planeCoefs = cast.cloudToImage(cloudPoints, imagePoints);

#ifdef VISUALIZATION
	//VISUALIZATION
#endif

	return 0;
}

vector<PointXYZ> assignCloudPoints() {
	return vector<PointXYZ>();
}
vector<Point2f> assignImagePoints() {
	return vector<Point2f>();
}
