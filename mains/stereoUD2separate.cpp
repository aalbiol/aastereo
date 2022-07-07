/*
 * stereoUD2separate.cpp
 *
 *  Created on: Jun 25, 2017
 *      Author: aalbiol
 */

#include <iostream>
#include <string>
#include <CImg.h>

int main(int argc, char **argv) {
	for(int n=1; n< argc; n++){
		std::string fname = argv[n];
		std::cout << "Separando " << fname << "\n";
		std::string Rfname = "R-"+ fname;
		std::string Lfname = "L-"+ fname;
		cimg_library::CImg<unsigned char> img(fname.c_str());
		img.get_rows(0, img.height()/2-1).save(Lfname.c_str());
		img.get_rows(img.height()/2, img.height()-1).save(Rfname.c_str());
	}
}

