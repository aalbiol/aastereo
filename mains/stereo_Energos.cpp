
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <math.h>
#include <time.h>
#include <string>
#include <iostream>
#include <sstream>

#include "cv.h"
#include "cxmisc.h"
//#undef HAVE_PCL
#ifdef HAVE_PCL
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#endif

#include <demonio_comun.h>
#include <midemonio.h>

#define cimg_plugin "cimg_gpiv.h"
//#define cimg_plugin1 "cimgcvMat.h"
#include "CImg.h"

using namespace std;
using namespace cimg_library;



#include "smartdisplay3.h"
#include "CimgStereo.h"
#include "stereo_src_Energos.h"

#ifdef HAVE_PCL
	// PCL
	#include <boost/thread/thread.hpp>
	#include "pcl/common/common_headers.h"
	#include "pcl/visualization/pcl_visualizer.h"
	#include <pcl/io/pcd_io.h>

	// Custom PCL auxiliary methods
	//#include "ViewerMethods.h"

	// Conversion cimg to pcl
	#include <cimg2pcl.h>
#endif

//Stereo Pairs are received in single images where upper half is left and lowe half is right.
smartdisplay3 <unsigned char> disp_rectified;
smartdisplay3 <float> disp_disparity;
smartdisplay3 <float> disp_depth;
smartdisplay3 <float> disp_X;
smartdisplay3 <float> disp_Y;
smartdisplay3 <unsigned char> disp_rectified_hor;
smartdisplay3 <unsigned char> disp_colorDepth;
smartdisplay3 <float> disp_range;
smartdisplay3 <unsigned char> disp_rangeAlarm;


#ifdef HAVE_PCL
	boost::shared_ptr<pcl::visualization::PCLVisualizer> PCLviewer;

	#define VIEWERNAME 		"PCLviewer"
	#define POINTCLOUDNAME 	"pointcloud"
#endif


int main(int argc, char **argv)
{
	int nf;
	char  texto[SHARED_TEXT_LENGTH];
	const char *file_token = cimg_option("-t", (char*)0, "Input video Daemon Token Filename.");

	bool showDisparity = cimg_option("-showDisp", false, "Show disparity window if 1");
	bool showX = cimg_option("-showX", false, "Show X window if 1");
	bool showY = cimg_option("-showY", false, "Show Y window if 1");
	bool showDepth = cimg_option("-showDepth", true, "Show depth window if 1");
	bool showColorAlign = cimg_option("-showColorAlign", false, "Show color alinged rectied images (Cyan Red) if 1");
	bool showHorizAlign = cimg_option("-showHorizAlign", false, "Show  alinged rectied images side by side if 1");
	bool showColorDepth= cimg_option("-showColorDepth", false, "Show  depth as color overlayed on rectified image");

	bool saveRectifiedUpDown = cimg_option("-saveRectifiedUpDown", false,"Save rectified images in numbered files with upper half (left) and lower half(right).");
	bool saveRectifiedLeftRight = cimg_option("-saveRectifiedLeftRight", false,"Save rectified images in numbered files left-right.");
#ifdef HAVE_PCL
	bool savexyz = cimg_option("-saveXYZ", false,"Save point clouds and left images in numbered files.");
#endif
	bool showPCL = cimg_option("-showPCL", false,"Show pointcloud using pcl visulizer");

	bool showRange = cimg_option("-showRange", false, "Show Range window if 1");
	float rangeAlarm = cimg_option("-showRangeAlarm", (float) -1.0, "Show Range Alarm with threshold thres");
	int rangeAlarmSizeFilter = cimg_option("-rangeAlarmSizeFilter", 0, "Remove if size is less than value (default:0)");
	
	// Check parameters
#ifndef HAVE_PCL
	if( showPCL )
	{
		std::cerr << "Bad input parameter. You need to install PCL for using the pointcloud viewer" << std::endl;
		exit(1);
	}
#endif

	if( 0 == file_token )
	{
		std::cerr<< "A token must be specified in command line using -t option\n" ;
		exit(0);
	}


	StereoDetect cimgstereo;
	cimgstereo.read_config("stereo_config.xml");
	if(rangeAlarm > 0.0)
		cimgstereo.threshold(rangeAlarm);
		
	cimgstereo.sizeFilter(rangeAlarmSizeFilter);

#ifdef HAVE_PCL
	if( showPCL )
	{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> PCLviewerAux (new pcl::visualization::PCLVisualizer (VIEWERNAME));
		PCLviewer = PCLviewerAux;

		/******************* Inicializacion PCL Viewer ***********************/
		PCLviewer->setBackgroundColor (0, 0, 0);
		PCLviewer->addCoordinateSystem (1.5);
		PCLviewer->setRepresentationToSurfaceForAllActors();
		PCLviewer->initCameraParameters ();

		// Set camera position
		PCLviewer->camera_.pos[0] 	= 0;  // Camera position
		PCLviewer->camera_.pos[1] 	= 0;
		PCLviewer->camera_.pos[2] 	= -4;

		PCLviewer->camera_.view[0] = 0;  // Camera vector that indicates vertical axis of the camera
		PCLviewer->camera_.view[1] = 1;
		PCLviewer->camera_.view[2] = 0;

		PCLviewer->camera_.focal[0] = 0; // A point where the camera is looking at
		PCLviewer->camera_.focal[1] = 0;
		PCLviewer->camera_.focal[2] = 0;

		PCLviewer->updateCamera();
		PCLviewer->spinOnce (100);
	}
#endif


	/******************* Inicializacion Video Daemon ***********************/
	void * handle = initDemonio( file_token, MODE_CLIENTE );
	if( handle == NULL )
		exit(-1);

	CImg<unsigned char> left;
	CImg<unsigned char> right;
	for ( nf = 1 ; 1 ; nf++)
	{
		int rr;
		CImg<unsigned char>composite(handle, texto, &rr);

		if(rr)
		{
			if(rr == A_TERMINAR)
				std::cout << "Terminate Message Received.Smoothly Ending\n";
			break;
		}
		composite.togray();

		//Compute point cloud from stereo pair
		CImg<float> xyz = cimgstereo.composite2xyz( composite ).get_shared();
		CImg<float> Range;
		cimgstereo.Range(Range);
		//Access some of the internal information used WITHOUT copy (for free)
		CImg<unsigned char> rectLeft, rectRight;
		CImg<float> disparity;
		CImg<float> depth;
		cimgstereo.getLeftRectifiedGray(rectLeft);
		cimgstereo.getRightRectifiedGray(rectRight);
		cimgstereo.getDisparity(disparity);
		depth = xyz.get_shared_plane(2);

		if(saveRectifiedUpDown)
		{
			char m[200];
			CImg<unsigned char>rectified = rectLeft.get_append(rectRight, 'y');
			sprintf(m,"rectified_ud_%06d.jpg", nf);
			rectified.save(m);
		}

		if(saveRectifiedLeftRight)
		{
			char m[200];
			CImg<unsigned char>rectified = rectLeft.get_append(rectRight, 'x');
			sprintf(m,"rectified_lr_%06d.jpg", nf);
			rectified.save(m);
		}
#ifdef HAVE_PCL

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc (new pcl::PointCloud<pcl::PointXYZRGB>);
		CImg<unsigned char> color;

		if( savexyz || showPCL )
		{
			color = rectLeft.get_tocolor();
			pc = upvsoft::pointcloud::cimg2pcl( color, xyz);
		}

		if(savexyz)
		{
			char m[200];
			sprintf(m,"xyzrgb_%06d.pcd", nf);
			pcl::io::savePCDFileBinary( m, *pc);
			printf("%s saved\n",m);
		}

		if( showPCL )
		{
			// Draw pointcloud
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pc);
			PCLviewer->removePointCloud( POINTCLOUDNAME );
			PCLviewer->addPointCloud<pcl::PointXYZRGB> ( pc, rgb, POINTCLOUDNAME );

			if (!PCLviewer->wasStopped ())
			{
				PCLviewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, POINTCLOUDNAME );
				PCLviewer->spinOnce (100);
				boost::this_thread::sleep (boost::posix_time::microseconds (100000)); //100000
			}
		}
#endif
		if(showColorAlign){
			CImg<unsigned char>rectified = rectLeft.get_append(rectRight, 'c').append(rectRight, 'c');
			std::string title = "Rectified";
			disp_rectified.display(rectified,title);
		}

		if(showHorizAlign){
			CImg<unsigned char>rectified = rectLeft.get_append(rectRight, 'x');
			int w_1 = rectified.width()-1;
			int h_10 = rectified.height() / 10;
			unsigned char black[] = { 0, 0, 0};
			for ( int k= 1; k < 9; k++)
			{
				rectified.draw_line( 0 , k * h_10, w_1, k*h_10,black, 0.7);
			}

			std::string title = "Rectified";
			disp_rectified_hor.display(rectified,title);
		}

		if(showDisparity) {
			std::string title2 = "Disparity";
			disp_disparity.display(disparity,title2);
		}

		if(showColorDepth) {
			std::string title2 = "Color Depth";
			CImg<unsigned char> tmp = rectLeft.get_tocolor();
			cimgstereo.depthAsColor(tmp);
			disp_colorDepth.display(tmp,title2);
		}

		if(showDepth){
			std::string title3 = "Depth";
			cimg_foroff(depth,o)
			{
				if( isnan( depth[o]))
					depth[o] = 0.0;
			}
			disp_depth.display(depth,title3);
		}

		if(showX){
			CImg<float> X = xyz.get_shared_plane(0);
			std::string title3 = "X-coord";
			cimg_foroff(X,o)
			{
				if( isnan( X[o]))
					X[o] = 0.0;
			}
			disp_X.display(X,title3);
		}
		if(showY){
			CImg<float> Y = xyz.get_shared_plane(1);
			std::string title3 = "Y-coord";
			cimg_foroff(Y,o)
			{
				if( isnan( Y[o]))
					Y[o] = 0.0;
			}
			disp_Y.display(Y,title3);
		}
		
		if(showRange){
			std::string title4 = "Range";
			cimg_foroff(Range,o)
			{
				if( isnan( Range[o]))
					Range[o] = 0.0;
			}
			disp_range.display(Range,title4);
		}

		if(rangeAlarm >= 0.0) {
			std::string title4 = "RangeAlarm";
			CImg<unsigned char> tmp = rectLeft.get_tocolor();
			cimgstereo.rangeAlarm2(tmp);
			disp_rangeAlarm.display(tmp,title4);
		}

	}

	// Remove PCL viewer
#ifdef HAVE_PCL
	if( showPCL )
		PCLviewer->removePointCloud( POINTCLOUDNAME );
#endif

}   
