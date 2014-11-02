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
#define IMAGE_FILE "/home/michal/Desktop/sekwencje/123_frame.png"

#define VISUALIZATION
#define CLOUD_DATA

PointXYZRGB XYZ2XYZRGB(PointXYZ p, int r, int g, int b);
PointXYZRGB XYZ2XYZRGB(PointXYZ p, uchar *val);
PointXYZRGBA makeXYZRGBA(double x, double y, double z, double r, double g,
		double b, double a);

inline vector<PointXYZ>& setCloudKeypoints();
inline vector<Point2f>& setImageKeypoints();

int main() {

	//Getting PointCloud
	PointCloud<PointXYZ/*RGB*/>::Ptr cloud(new PointCloud<PointXYZ/*RGB*/>);

#ifdef CLOUD_DATA
	if (io::loadPCDFile<PointXYZ/*RGB*/>(CLOUD_FILE, *cloud) == -1) {
		cerr << "Could not load cloud data" << endl;
		return -1;
	}
#endif

	vector<PointXYZ> &cloudKeypoints = setCloudKeypoints();
	vector<Point2f> &imageKeypoints = setImageKeypoints();

	Caster cast = Caster();
	Mat transformationMatrix = cast.cloudToImage(cloudKeypoints,
			imageKeypoints);
	cout << transformationMatrix << endl;

	//Getting image PointCloud
	PointCloud<PointXYZRGB>::Ptr image(new PointCloud<PointXYZRGB>);
	Mat cvImage = imread(IMAGE_FILE);
	cout << (int) cvImage.at<Vec3b>(0, 0).val[2] << endl;
//	vector<Point2f> coors 
	for (int y = 0; y < cvImage.rows; y++)
		for (int x = 0; x < cvImage.cols; x++) {
			PointXYZ newCoordinates = Utils::transformPoint(Point2f(x, y),
					transformationMatrix,Point2f(cvImage.cols / 2, cvImage.rows / 2));
			image->push_back(
					XYZ2XYZRGB(newCoordinates,
							(int) cvImage.at<Vec3b>(y, x).val[2],
							(int) cvImage.at<Vec3b>(y, x).val[1],
							(int) cvImage.at<Vec3b>(y, x).val[0]));
		}

#ifdef VISUALIZATION
	visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(cloud, "Cloud");
//	viewer.showCloud(image, "Image");
	while (!viewer.wasStopped()) {
	}
#endif

	return 0;
}

vector<PointXYZ>& setCloudKeypoints() {
	PointXYZ A = PointXYZ(0.5614, 6.7578, 132.139);
	PointXYZ B = PointXYZ(-0.0762, 6.4742, 132.4927);
	PointXYZ C = PointXYZ(-0.8509, 6.1495, 133.216);
	PointXYZ D = PointXYZ(0.057, 6.5415, 133.2182);
	PointXYZ E = PointXYZ(-0.7907, 6.1947, 131.0959);
	PointXYZ F = PointXYZ(-0.0135, 6.5007, 131.095);
	PointXYZ G = PointXYZ(-1.7732, 5.7418, 133.107);
	PointXYZ H = PointXYZ(-1.7754, 5.7539, 132.413);
	vector<PointXYZ> *v = new vector<PointXYZ>();
	v->push_back(A);
	v->push_back(B);
	v->push_back(C);
	v->push_back(D);
	v->push_back(E);
	v->push_back(F);
	v->push_back(G);
	v->push_back(H);
	return *v;
}

vector<Point2f>& setImageKeypoints() {
	Point2f A = Point2f(497, 230);
	Point2f B = Point2f(440, 196);
	Point2f C = Point2f(367, 127);
	Point2f D = Point2f(454, 131);
	Point2f E = Point2f(368, 323);
	Point2f F = Point2f(443, 322);
	Point2f G = Point2f(271, 131);
	Point2f H = Point2f(270, 198);
	vector<Point2f> *v = new vector<Point2f>;
	v->push_back(A);
	v->push_back(B);
	v->push_back(C);
	v->push_back(D);
	v->push_back(E);
	v->push_back(F);
	v->push_back(G);
	v->push_back(H);
	return *v;
}

PointXYZRGB XYZ2XYZRGB(PointXYZ p, int r, int g, int b) {
	PointXYZRGB p2 = PointXYZRGB(r, g, b);
	p2.x = p.x;
	p2.y = p.y;
	p2.z = p.z;
	return p2;
}

PointXYZRGB XYZ2XYZRGB(PointXYZ p, uchar *val) {
	return XYZ2XYZRGB(p, val[0], val[1], val[2]);
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
