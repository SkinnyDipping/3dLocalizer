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
#define VIDEO_FILE "UZULNIC"
#define IMAGE_FILE "/home/michal/Downloads/image.jpg"

#define VISUALIZATION
//#define TEST_DATA
//#define CASTING
//#define VIEWER_TEST
//#define PKT_W_K
//#define SPHERE

PointXYZRGB XYZ2XYZRGB(PointXYZ p, int r, int g, int b);

int main() {

	//Getting PointCloud
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

#ifndef TEST_DATA
	if (io::loadPCDFile<PointXYZ>(CLOUD_FILE, *cloud) == -1) {
		cerr << "Could not load cloud data" << endl;
		return -1;
	}
#endif

	//Getting image PointCloud
	PointCloud<PointXYZRGB>::Ptr image(new PointCloud<PointXYZRGB>);

//	Mat_<Vec3b> imageMat = imread(IMAGE_FILE);
//	Mat imageMatrix = imread(IMAGE_FILE);
//	Mat_<Vec3b> imageMat;
//	for (int i = 0; i < imageMat.rows; i++)
//		for (int j = 0; j < imageMat.cols; j++) {
//			Vec3b color = imageMat(i, j);
//			PointXYZRGB p;
//			p.z = 0;
//			p.x = j;
//			p.y = i;
//			p.b = color.val[0];
//			p.g = color.val[1];
//			p.r = color.val[2];
//			image->points.push_back(p);
//		}

	vector<PointXYZ> cloudKeypoints;
	vector<Point2f> imageKeypoints;

#ifdef TEST_DATA
	PointXYZ A = PointXYZ(19, -13, 6);
	PointXYZ B = PointXYZ(-19, 10, -4);
	PointXYZ C = PointXYZ(-5, -20, -11);
	PointXYZ D = PointXYZ(14, 4, -9);
	PointXYZ E = PointXYZ(-9, 9, -17);
	cloudKeypoints.push_back(A);
	cloudKeypoints.push_back(B);
	cloudKeypoints.push_back(C);
	cloudKeypoints.push_back(D);
	cloudKeypoints.push_back(E);
	Point2f A2 = Point2f(19, 18);
	Point2f B2 = Point2f(-19, 28);
	Point2f C2 = Point2f(-5, 35);
	Point2f D2 = Point2f(14, 33);
	Point2f E2 = Point2f(-9, 41);
	imageKeypoints.push_back(A2);
	imageKeypoints.push_back(B2);
	imageKeypoints.push_back(C2);
	imageKeypoints.push_back(D2);
	imageKeypoints.push_back(E2);

	PointXYZRGB Ac = PointXYZRGB(255, 0, 0);
	PointXYZRGB Bc = PointXYZRGB(255, 0, 0);
	PointXYZRGB Cc = PointXYZRGB(255, 0, 0);
	PointXYZRGB Dc = PointXYZRGB(255, 0, 0);
	PointXYZRGB Ec = PointXYZRGB(255, 0, 0);
	Ac.x = 19;
	Ac.y = -13;
	Ac.z = 6;
	Bc.x = -19;
	Bc.y = 10;
	Bc.z = -4;
	Cc.x = -5;
	Cc.y = -20;
	Cc.z = -11;
	Dc.x = 14;
	Dc.y = 4;
	Dc.z = -9;
	Ec.x = -9;
	Ec.y = 9;
	Ec.z = -17;

	cloud->width = 5;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.push_back(Ac);
	cloud->points.push_back(Bc);
	cloud->points.push_back(Cc);
	cloud->points.push_back(Dc);
	cloud->points.push_back(Ec);

	PointXYZRGB A2c = PointXYZRGB(0, 255, 0);
	PointXYZRGB B2c = PointXYZRGB(0, 255, 0);
	PointXYZRGB C2c = PointXYZRGB(0, 255, 0);
	PointXYZRGB D2c = PointXYZRGB(0, 255, 0);
	PointXYZRGB E2c = PointXYZRGB(0, 255, 0);
	A2c.x = 19;
	A2c.y = 18;
	A2c.z = 0;
	B2c.x = -19;
	B2c.y = 28;
	B2c.z = 0;
	C2c.x = -5;
	C2c.y = 35;
	C2c.z = 0;
	D2c.x = 14;
	D2c.y = 33;
	D2c.z = 0;
	E2c.x = -9;
	E2c.y = 41;
	E2c.z = 0;

	image->width=5;
	image->height=1;
	image->is_dense=false;
	image->points.push_back(A2c);
	image->points.push_back(B2c);
	image->points.push_back(C2c);
	image->points.push_back(D2c);
	image->points.push_back(E2c);
#endif

#ifdef PKT_W_K
for(int i=0; i<20; i++)
{
	int x = (int)rand() %70;
	int y = (int)rand() %70;
	int z = (int)rand() %70;
	cloud->points.push_back(XYZ2XYZRGB(PointXYZ(x,y,z),255,0,0));
	cloudKeypoints.push_back(PointXYZ(x,y,z));
	imageKeypoints.push_back(Point2f(x,y));
	image->points.push_back(XYZ2XYZRGB(PointXYZ(x,y,0),0,255,0));
}

#endif

#ifdef CASTING
	Caster cast = Caster();
	Mat transformationMatrix = cast.cloudToImage(cloudKeypoints,
			imageKeypoints);
	cout<<transformationMatrix<<endl;
//#endif

	//TODO image centroid to origin
	//TODO transform image points

	Point2f centroid = Utils::calculateCentroid(imageKeypoints);
	for (int i=0; i<image->points.size(); i++)
	{
		image->points[i].x -= centroid.x;
		image->points[i].y -= centroid.y;
		image->points[i].z = 0;
		image->points[i] = Caster::transformPoint(image->points[i],transformationMatrix);
//		cout<<image->points[i]<<endl;
	}
#endif

	PointCloud<PointXYZ>::Ptr sphere(new PointCloud<PointXYZ>);
#ifdef SPHERE
	Sphere takao = Sphere(cast.centerSphere, cast.radius);
	vector<PointXYZ> &pts = takao.generateSphere();
//	sphere->width=pts.size();
//	sphere->height=1;
	for (int i=0; i<pts.size(); i++)
	{
		sphere->points.push_back(pts[i]);
	}
#endif

#ifdef VISUALIZATION
	visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(cloud, "Cloud");
//	viewer.showCloud(image, "Image");
//	viewer.showCloud(sphere, "Sphere");
	while (!viewer.wasStopped()) {}
#endif

	return 0;
}

PointXYZRGB XYZ2XYZRGB(PointXYZ p, int r, int g, int b)
{
	PointXYZRGB p2 = PointXYZRGB(r,g,b);
	p2.x=p.x;p2.y=p.y;p2.z=p.z;
	return p2;
}
