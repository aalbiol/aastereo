
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
	cimg_usage("Computes disparity statistics of static video streams");

	if( 0 == file_token )
	{
		std::cerr<< "A token must be specified in command line using -t option\n" ;
		exit(0);
	}


	CimgStereo cimgstereo;
	cimgstereo.read_config("stereo_config.xml");


	/******************* Inicializacion Video Daemon ***********************/
	void * handle = initDemonio( file_token, MODE_CLIENTE );
	if( handle == NULL )
		exit(-1);

	CImg<unsigned char> left;
	CImg<unsigned char> right;
	clock_t total_time = 0;
	CImg<float> disparityAcum, disparityAcum2;
	CImg<float> countImg;
	CImg<float> acumRectLeft;
	CImg<float> acumRectRight;

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
		clock_t t1 = clock();
		CImg<float> xyz = cimgstereo.composite2xyz( composite ).get_shared();
		clock_t t2 = clock();
		total_time +=(t2-t1);

		//Access some of the internal information used WITHOUT copy (for free)
		CImg<unsigned char> rectLeft, rectRight;
		CImg<float> disparity;
		CImg<float> depth;
		cimgstereo.getLeftRectifiedGray(rectLeft);
		cimgstereo.getRightRectifiedGray(rectRight);
		cimgstereo.getDisparity(disparity);
		depth = xyz.get_shared_plane(2);

		if( ! disparityAcum.is_sameXY(disparity) ) {
			disparityAcum.assign( disparity ).fill( 0.0 );
			disparityAcum2 = disparityAcum;
			countImg = disparityAcum;
			acumRectLeft = rectLeft;
			acumRectRight = rectRight;
		}
		else {
			acumRectLeft += rectLeft;
			acumRectRight += rectRight;
		}

		cimg_foroff(disparity,o) {
			if ( !isnan( depth[o]) ) {
				disparityAcum[o] += disparity [o];
				disparityAcum2[o] += (disparity [o] * disparity[o]);
				countImg [o] ++;
			}
		}
	}

	cimg_foroff(disparityAcum,o) {
		if ( countImg[o] ) {
			disparityAcum[o] /= countImg[o];
			disparityAcum2[o] /= countImg[o];
			disparityAcum2[o] -= (	disparityAcum[o] *	disparityAcum[o]);
		}
		else {
			disparityAcum[o] = disparityAcum2[o] = 0.0;
		}
	}
	disparityAcum.save_cimg("disparity_mean.cimg");
	disparityAcum2.save_cimg("disparity_variance.cimg");
	countImg.save_cimg("disparity_count.cimg");
	left = acumRectLeft /nf ;
	right = acumRectRight /nf ;
	left.save("left_rect.jpg");
	right.save("right_rect.jpg");

	std::cout << "INFO. SAVED FILES: disparity_mean.cimg, disparity_variance.cimg,  disparity_count.cimg,  left_rect.jpg, right_rect.jpg \n\n";
	std::cout << "Mean Time per Image: " << float(total_time)/CLOCKS_PER_SEC/float(nf) <<"\n";
	std::cout << "IPS: " << 1.0/(float(total_time)/CLOCKS_PER_SEC/float(nf))<<"\n";


}   
