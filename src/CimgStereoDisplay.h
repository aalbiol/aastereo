/*
 * CimgStereoDisplay.h
 *
 *  Created on: Sep 20, 2012
 *      Author: jmmossi
 */

#ifndef CIMGSTEREODISPLAY_H_
#define CIMGSTEREODISPLAY_H_

#define FONT_SIZE_DEFAULT 20
#define FONT_SIZE_SEPARATION 4
#include <deque>
#ifdef HAVE_PCL

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "pcl/visualization/pcl_visualizer.h"

#endif



#include <CImg.h>

#ifdef HAVE_PCL
#include "cimg2pcl.h"
#endif

#include "CimgStereo.h"

struct StereoMosaic
{
	cimg_library::CImg<unsigned char> Left;
	cimg_library::CImg<unsigned char> Center;
	cimg_library::CImg<unsigned char> Right;
	cimg_library::CImg<unsigned char> DepthColor;
	cimg_library::CImg<float> disparity;
	cimg_library::CImg<disp_type> disparity_type;
	cimg_library::CImg<float> xyz;
	std::string texto;
};

class CimgStereoDisplay {

private:
	int _playing;
	int _initialised;
	int _exist_Center;

	int font_size;
	int font_interline;

	cimg_library::CImg<unsigned char> *rectLeft,*rectRight, *rectCenter, *depthColor;
	cimg_library::CImg<float> *disparity, *xyz;
	cimg_library::CImg<disp_type> *disparity_type;
	std::string *texto;

	cimg_library::CImgDisplay disp;


	std::deque<StereoMosaic> buffer;
	int pos;
	int _bufsize;

	int go_backward();
	int go_forward();
	void to_buffer(StereoMosaic &StM);

	void bufsize(int a){ _bufsize=a;buffer.clear();};
	int bufsize(){return _bufsize;};

	CimgStereo *CimgSt_p;


#ifdef HAVE_PCL
		//point cloud 3D

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;// (new pcl::visualization::PCLVisualizer ("3D Viewer"));

	pcl::PointXYZ posSphere;
	float Sphere_radio;

	int drawSpherePCL;

	pcl::PlanarPolygon<pcl::PointXYZ> ground_polygon;
	pcl::PlanarPolygon<pcl::PointXYZ> Ycero_polygon;
	pcl::PlanarPolygon<pcl::PointXYZ> Cristal_polygon;

	int viewerPCL2;

	void MyPointPicking_callback (const pcl::visualization::PointPickingEvent& event, void* viewer_void);
	cimg_library::CImg<float> Q_inv;
	cimg_library::CImg<float> rotMatrix_inv;   //inverse of rotation matrix to ground coordinates

#endif

public:	
	CimgStereoDisplay(CimgStereo *CimgSt);

	//void display(CimgStereo &CImgSt, std::string &str);
	void display(std::string &str);
	void display_mosaico(int x, int y);
	void displayPCL(int x, int y);

#ifdef HAVE_PCL
	void init_viewerPCL2();
	void refresh_mosaico(int x, int y);
#endif
};


#endif /* CIMGSTEREODISPLAY_H_ */
