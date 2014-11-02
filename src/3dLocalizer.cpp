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
#define CLOUD_FILE "/home/michal/PDM/PointClouds/pcd/chmura.pcd"
#define IMAGE_FILE "/home/michal/Downloads/image.jpg"

#define VISUALIZATION
#define CLOUD_DATA

PointXYZRGB XYZ2XYZRGB(PointXYZ p, int r, int g, int b);
PointXYZRGBA makeXYZRGBA(double x, double y, double z, double r, double g,
		double b, double a);

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
			new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(
		pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
			new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
			cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}

int main() {
cout<<"Prehello\n";
	//Getting PointCloud
	PointCloud<PointXYZ/*RGB*/>::Ptr cloud(new PointCloud<PointXYZ/*RGB*/>);
	cout<<"Loading pointcloud... ";
#ifdef CLOUD_DATA

	if (io::loadPCDFile<PointXYZ/*RGB*/>(CLOUD_FILE, *cloud) == -1) {
		cerr << "Could not load cloud data" << endl;
		return -1;
	}
	cout<<"Done.\n";
#endif

	//Getting image PointCloud
	PointCloud<PointXYZ/*RGB*/>::Ptr image(new PointCloud<PointXYZ/*RGB*/>);

//	vector<PointXYZ> cloudKeypoints;
//	vector<Point2f> imageKeypoints;

//	for (int i = 0; i < 20; i++) {
//		int x = (int) rand() % 70;
//		int y = (int) rand() % 70;
//		int z = (int) rand() % 70;
//		cloud->points.push_back(XYZ2XYZRGB(PointXYZ(x, y, z), 255, 0, 0));
//		cloudKeypoints.push_back(PointXYZ(x, y, z));
//		imageKeypoints.push_back(Point2f(x, y));
//		image->points.push_back(XYZ2XYZRGB(PointXYZ(2*x, 2*y, 0), 0, 255, 0));
//	}

//	Caster cast = Caster();
//	Mat transformationMatrix = cast.cloudToImage(cloudKeypoints,
//			imageKeypoints);
//	cout << transformationMatrix << endl;

	//TODO image centroid to origin
	//TODO transform image points

//	Point2f centroid = Utils::calculateCentroid(imageKeypoints);
//	for (int i = 0; i < image->points.size(); i++) {
//		image->points[i].x -= centroid.x;
//		image->points[i].y -= centroid.y;
//		image->points[i].z = 0;
//		image->points[i] = Utils::transformPoint(image->points[i],
//				transformationMatrix);
//	}
//
//	PointCloud<PointXYZRGBA>::Ptr sphere(new PointCloud<PointXYZRGBA>);
//
//	Sphere takao = Sphere(cast.centerSphere, cast.radius);
//	vector<PointXYZ> &pts = takao.generateSphere();
//	for (int i = 0; i < pts.size(); i++) {
////		sphere->points.push_back(XYZ2XYZRGB(pts[i], 10, 10, 10));
//		sphere->points.push_back(makeXYZRGBA(pts[i].x,pts[i].y,pts[i].z,255,255,255,200));
//	}

//#ifdef VISUALIZATION
//	visualization::CloudViewer viewer("Simple Cloud Viewer");
//	viewer.showCloud(cloud, "Cloud");
////	viewer.showCloud(image, "Image");
////	viewer.showCloud(sphere, "Sphere");
//	while (!viewer.wasStopped()) {
//	}
//#endif

	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(
			new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(
			new pcl::PointCloud<pcl::PointXYZRGB>);
cout<<"HEllo\n";
	basic_cloud_ptr = cloud;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = simpleVis(basic_cloud_ptr);
cout<<"get ready...\n";
	//--------------------
	// -----Main loop-----
	//--------------------
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}

PointXYZRGB XYZ2XYZRGB(PointXYZ p, int r, int g, int b) {
	PointXYZRGB p2 = PointXYZRGB(r, g, b);
	p2.x = p.x;
	p2.y = p.y;
	p2.z = p.z;
	return p2;
}

PointXYZRGBA makeXYZRGBA(double x, double y, double z, double r, double g,
		double b, double a) {
	PointXYZRGBA p;
	p.x = x;
	p.y = y;
	p.z = z;
	p.r = r;
	p.g = g;
	p.b = b;
	p.a = a;
	return p;
}
