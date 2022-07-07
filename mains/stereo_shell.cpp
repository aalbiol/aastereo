
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <math.h>
#include <time.h>
#include <string>
#include <iostream>
#include <sstream>




#include "opencv2/core.hpp"
//#include "cxmisc.h"

#include <demonio_comun.h>
#include <midemonio.h>

#define cimg_plugin "cimg_gpiv.h"
//#define cimg_plugin1 "cimgcvMat.h"
#include "CImg.h"

using namespace std;
using namespace cimg_library;



#include "smartdisplay3.h"
#include "CimgStereo.h"
#include "CimgStereoDisplay.h"


//Stereo Pairs are received in single images where upper half is left and lowe half is right.
smartdisplay3 <unsigned char> disp_rectified;
smartdisplay3 <float> disp_disparity;
smartdisplay3 <float> disp_depth;
smartdisplay3 <float> disp_X;
smartdisplay3 <float> disp_Y;
smartdisplay3 <unsigned char> disp_rectified_hor;
smartdisplay3 <unsigned char> disp_colorDepth;



int main(int argc, char **argv)
{
	int nf;

	const char *filename = cimg_option("-i", (char*)0, "Input  Filename.");

	bool showDisparity = cimg_option("-showDisp", false, "Show disparity window if 1");
	bool showX = cimg_option("-showX", false, "Show X window if 1");
	bool showY = cimg_option("-showY", false, "Show Y window if 1");
	bool showDepth = cimg_option("-showDepth", true, "Show depth window if 1");
	bool showColorAlign = cimg_option("-showColorAlign", false, "Show color alinged rectied images (Cyan Red) if 1");
	bool showHorizAlign = cimg_option("-showHorizAlign", false, "Show  alinged rectied images side by side if 1");
	bool showColorDepth= cimg_option("-showColorDepth", false, "Show  depth as color overlayed on rectified image");
	bool showStereoDisplay= cimg_option("-showStereoDisplay", false, "Show a mosaic with rectified Left and Right and below the Left image with depth as color overlayed");

	bool saveRectifiedUpDown = cimg_option("-saveRectifiedUpDown", false,"Save rectified images in numbered files with upper half (left) and lower half(right).");
	bool saveRectifiedLeftRight = cimg_option("-saveRectifiedLeftRight", false,"Save rectified images in numbered files left-right.");
	bool saveRectifiedSeparate = cimg_option("-saveRectifiedSeparate", false,"Save rectified images in numbered files left-right.");

	bool savexyz = cimg_option("-saveXYZ", false,"Save point clouds and left images in numbered files.");

	CimgStereo cimgstereo;
	cimgstereo.read_config("stereo_config.xml");

	CimgStereoDisplay stereoDisplay(&cimgstereo);

	CImg<unsigned char> composite (filename);
	composite.display("Input");

	CImg<float> xyz = cimgstereo.composite2xyz( composite ).get_shared();

	//Access some of the internal information used WITHOUT copy (for free)
	CImg<unsigned char> rectLeft, rectRight;
	CImg<float> disparity;
	CImg<float> depth;
	cimgstereo.getLeftRectifiedGray(rectLeft);
	cimgstereo.getRightRectifiedGray(rectRight);
	cimgstereo.getDisparity(disparity);
	depth = xyz.get_shared_slice(2);

	if(saveRectifiedUpDown){
		CImg<unsigned char>rectified = rectLeft.get_append(rectRight, 'y');
		rectified.save("rectifiedUD.png");
	}

	if(saveRectifiedLeftRight) {
		CImg<unsigned char>rectified = rectLeft.get_append(rectRight, 'x');
		rectified.save("rectifiedLR.png");
	}

	if(saveRectifiedSeparate)
	{
		rectLeft.save("rectifiedL.png");
		rectRight.save("rectifiedR.png");
	}


	if( savexyz  )
	{
		CImg<unsigned char> color2;
		cimgstereo.getLeftRectified(color2);
		CImg<float> color2f(color2);
		color2f.append(xyz,'c');


		color2f.save_cimg("xyzrgb.cimg");
		std::cout << "xyzrgb.cimg saved\n";
	}

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
		disparity.display("Disparity",false);
	}

	if(showColorDepth) {
		std::string title2 = "Color Depth";
		CImg<unsigned char> tmp = rectLeft.get_tocolor();
		cimgstereo.depthAsColor(tmp);
		tmp.display(title2.c_str(),false);
	}

	if(showDepth){
		std::string title3 = "Depth";
		cimg_foroff(depth,o)
		{
			if( isnan( depth[o]))
				depth[o] = 0.0;
		}
		depth.display(title3.c_str(), false);
	}

	if(showX){
		CImg<float> X = xyz.get_shared_slice(0);
		std::string title3 = "X-coord";
		cimg_foroff(X,o)
		{
			if( isnan( X[o]))
				X[o] = 0.0;
		}
		disp_X.display(X,title3);
	}
	if(showY){
		CImg<float> Y = xyz.get_shared_slice(1);
		std::string title3 = "Y-coord";
		cimg_foroff(Y,o)
		{
			if( isnan( Y[o]))
				Y[o] = 0.0;
		}
		disp_Y.display(Y,title3);
	}

}   
