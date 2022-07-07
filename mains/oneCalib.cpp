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



//#define cimg_plugin "cimg_gpiv.h"
#include <iostream>
#include <CImg.h>

#include "CimgOneCalib.h"
using namespace std;
using namespace cimg_library;

int main(int argc, char **argv)
{
	int opt;

	int metodo=0; //0: metodo Bouguet , 1: metodo Hartley, OPCION POR DEFECTO
	int diezmado = 1;
	float tamanyo_cuadro = 28.28f; // en mm 

	cimg_usage("This program is used to callibrate a stereo pair from a list of chess squares images");
	float squareSize = cimg_option("-size", 76, "Size of the square in mm.");
	const char *listfilename = cimg_option("-list", (char *)0, "Name of the file containing the names of the images with the squares. ");

	bool nullDistortion = cimg_option("-nullDistortion", false, "If set to 1 it indicates that we assume null radial and tangential distortion.");
	bool squarePixels = cimg_option("-squarePixels", false, "If set to 0 it indicates that pixels may not be square and focals are different fx!= fy");
	int nx= cimg_option("-nx", 6, "Number of inner corners in the chess board in x direction");
	int ny= cimg_option("-ny", 9, "Number of inner corners in the chess board in y direction");
	bool interactive = cimg_option("-interactive", false, "Require pressing key when showing corners");
	bool subPixel = cimg_option("-subpixel", false, "Refine corner detection at subpixel level if 1");
	bool autoScale = cimg_option("-autoScale", true, "Auto Scale small images to about VGA size if 1");


	OneCalib calib;
	//Pattern design
	calib.squareSize(squareSize);
	calib.numberOfCorners(nx,ny);

	if(interactive)
		calib.interactive();

	if(nullDistortion)
		calib.noRadialDistortion();


	if(squarePixels)
		calib.fixAspectRatio();

	calib.printConfig();


	if(listfilename ==0) {
		std::cerr << "No listfilename given. Use -list option\n";
		exit(0);
	}

	std::string listname = listfilename;
	calib.calibrate(listname);

	calib.saveCVCompact();
	calib.saveCV();
	calib.saveCImg();

	return 0;
}

