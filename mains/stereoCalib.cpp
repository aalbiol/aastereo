#pragma warning( disable: 4996 )

#include "opencv2/core.hpp"
//#include "cxmisc.h"
//#include "highgui.h"
//#include "cvaux.h"
//#include <vector>
#include <string>
//#include <algorithm>
//#include <stdio.h>
//#include <ctype.h>

//#include <demonio_comun.h>
//#include <midemonio.h>

//#define cimg_plugin "cimg_gpiv.h"
#include <iostream>
#include <CImg.h>

#include "CimgStereo.h"
#include "CimgStereoCalib.h"
using namespace std;
using namespace cimg_library;

int main(int argc, char **argv)
{
	int opt;

	int metodo=0; //0: metodo Bouguet , 1: metodo Hartley, OPCION POR DEFECTO
	int diezmado = 1;
	float tamanyo_cuadro = 22.2f; // en mm

	cimg_usage("This program is used to callibrate a stereo pair from a list of chess squares images");
	float squareSize = cimg_option("-size", tamanyo_cuadro, "Size of the square in mm.");
	const char *listfilenameSeparate = cimg_option("-listSeparateLR", (char *)0, "Name of the file containing the names of the images with the squares. Left1 ; Right1; Left2; Right2;...");
	const char *listfilenameCompound = cimg_option("-listCompoundLR", (char *)0, "Name of the file containing the names of the compound images with the squares. stereo_config.xml must exist.");
	bool nullDistortion = cimg_option("-nullDistortion", false, "If set to 1 it indicates that we assume null radial and tangential distortion.");
	bool sameFocal = cimg_option ("-sameFocal", false, "If set to 1 it indicates that both cameras are assumed to have the same focal.");
	bool squarePixels = cimg_option("-squarePixels", false, "If set to 0 it indicates that pixels may not be square and focals are different fx!= fy");
	int nx= cimg_option("-nx", 12, "Number of inner corners in the chess board in x direction");
	int ny= cimg_option("-ny", 8, "Number of inner corners in the chess board in y direction");
	bool interactive = cimg_option("-interactive", false, "Require pressing key when showing corners");
	bool subPixel = cimg_option("-subpixel", false, "Refine corner detection at subpixel level if 1");
	bool autoScale = cimg_option("-autoScale", true, "Auto Scale small images to about VGA size if 1");
	bool preComputeIntrinsics = cimg_option("-preComputeIntrinsics", true, "Precompute separately camera intrinscs before stereo Calib  if 1");


	StereoCalib stereoCalib;


	stereoCalib.autoScale(autoScale);
	stereoCalib.subPixelCorners(subPixel);
	stereoCalib.preComputeIntrinsics(preComputeIntrinsics);

	//Pattern design
	stereoCalib.squareSize(squareSize);
	stereoCalib.numberOfCorners(nx,ny);

	if(interactive)
		stereoCalib.interactive();

	if(nullDistortion)
		stereoCalib.noRadialDistortion();

	if(sameFocal)
		stereoCalib.sameFocalLength();

	if(squarePixels)
		stereoCalib.fixAspectRatio();

	stereoCalib.printConfig();

	if(listfilenameSeparate && listfilenameCompound)
	{
		std::cerr << "Only one of options 'listSeparateLR'  'listCompoundLR'  can be specified\n";
		exit(0);
	}
	if(listfilenameSeparate)
	{
		std::string listname = listfilenameSeparate;
		stereoCalib.calibrate(listname);
	}
	else if(listfilenameCompound)
	{
		std::string listname = listfilenameCompound;
		int t = stereoCalib.read_input_config( "stereo_config.xml" );
		if( t < 0 )
		{
			std::cerr << "Error in stereoCalib.read_input_config\n";
			exit(0);
		}
		stereoCalib.calibrateFromComposite(listname);
	}
	else
	{
		std::cerr << "Option -listSeparateLR and -listCompoundLR missing\n";
		exit(0);
	}

	stereoCalib.saveCVCompact();
//	stereoCalib.saveCV();
	stereoCalib.saveCImg();


	//Create Rectified Calibration Images
	CimgStereo cimgstereo;

	std::string stereoCalibrationDir = ".";
	cimgstereo.read_input_config( "stereo_config.xml" );
	cimgstereo.loadStereoCalibration(stereoCalibrationDir);




	if ( listfilenameSeparate && cimgstereo.getNumCameras() == 2) {
		int numPairs = stereoCalib.getListOfFiles().size() / 2;
		for(int p = 0; p < 1; p++)
		{
			std::string leftname = stereoCalib.getListOfFiles()[2 * p];
			std::string rightname = stereoCalib.getListOfFiles()[2 * p +1];
			CImg<unsigned char> left(leftname.c_str());
			CImg<unsigned char> right(rightname.c_str());

			CImg<unsigned char> rectLeft;
			CImg<unsigned char> rectRight;
			left.resize(-100,-100,-100,1,2);
			right.resize(-100,-100,-100,1,2);
			int rr= cimgstereo.rectify( left , right, rectLeft, rectRight);

			if(rr)
			{
				std::cerr<<"Error en img_stereo.rectify\n";
				break;
			}

			CImg<unsigned char>rectified = rectLeft.get_append(rectRight, 'x');
			int w_1 = rectified.width()-1;
			int h_20 = rectified.height() / 20;
			unsigned char black[] = { 0, 0, 0};
			for ( int k= 1; k < 19; k++)
			{
				rectified.draw_line( 0 , k * h_20, w_1, k*h_20,black, 0.7);
			}
			std::string wintitle = "Rectified: ";
			wintitle += leftname;
			wintitle += " <-> ";
			wintitle += rightname;
			rectified.display(wintitle.c_str(),false);
		}
	}



	if ( listfilenameCompound) {
		CImg<unsigned char>rectified ;
		int numPairs = stereoCalib.getListOfFiles().size() / 2;
		for(int p = 0; p < 1; p++)
		{
			std::string name = stereoCalib.getListOfFiles()[p];
			CImg<unsigned char> compound(name.c_str() );
			if( cimgstereo.getNumCameras() == 2 ) {
				CImg<unsigned char> left;
				CImg<unsigned char> right;
				cimgstereo.splitLR(compound, left, right);

				CImg<unsigned char> rectLeft;
				CImg<unsigned char> rectRight;

				left.resize(-100,-100,-100,1,2);
				right.resize(-100,-100,-100,1,2);
				int rr= cimgstereo.rectifyOnly( left , right);

				if(rr)
				{
					std::cerr<<"Error en img_stereo.rectify\n";
					break;
				}
				cimgstereo.getLeftRectifiedGray( rectLeft );
				cimgstereo.getRightRectifiedGray( rectRight );
				rectified = rectLeft.get_append(rectRight, 'x');

			}
			if( cimgstereo.getNumCameras() == 3 ) {
				std::cerr<<"3 cams display \n";
				CImg<unsigned char> left;
				CImg<unsigned char> right;
				CImg<unsigned char> center;
				cimgstereo.splitLCR(compound, left, center, right);
				std::cerr<<"split done \n";
				rectified = left;
				rectified.append(center, 'x').append(right, 'x');
				rectified.display("Unrectified LCR",false);

				CImg<unsigned char> rectLeft;
				CImg<unsigned char> rectRight;
				CImg<unsigned char> rectCenter;
				left.resize(-100,-100,-100,1,2);
				right.resize(-100,-100,-100,1,2);
				center.resize(-100,-100,-100,1,2);
				int rr= cimgstereo.rectifyOnly( left , center, right);

				if(rr)
				{
					std::cerr<<"Error en img_stereo.rectify\n";
					break;
				}
				std::cerr<<"rectification	 done \n";
				cimgstereo.getLeftRectifiedGray( rectLeft );
				cimgstereo.getRightRectifiedGray( rectRight );
				cimgstereo.getCenterRectifiedGray( rectCenter);
				rectified = rectLeft;
				rectified.append(rectCenter, 'x').append(rectRight, 'x');
			}

			int w_1 = rectified.width()-1;
			int h_20 = rectified.height() / 20;
			unsigned char black[] = { 0, 0, 0};
			for ( int k= 1; k < 19; k++)
			{
				rectified.draw_line( 0 , k * h_20, w_1, k*h_20,black, 0.7);
			}
			std::string wintitle = "Rectified: ";
			wintitle += name;
			rectified.display(wintitle.c_str(),false);
		}
	}


	return 0;
}

