/*
 * stereo_src_detect.cpp
 * jmmossi
 * may-2012
 *  
 */


#include <string>
#include <iostream>
#include <math.h>
#include "cv.h"
#include <cmath>
#include <demonio_comun.h>
#include <opencv2/contrib/contrib.hpp> //stereovar

#define cimg_plugin "cimg_gpiv.h"
//#define cimg_plugin2 "cimgIPL.h"
#define cimg_plugin1 "cimgcvMat.h"
#include <CImg.h>
#include "cimg_gpivfunctions.h"
using namespace cimg_library;
using namespace std;
using namespace cv;


#include "stereo_src_Energos.h"

StereoDetect::StereoDetect() {
	CimgStereo();
	_threshold=-1.0;   // less than cero means not initialized
	_sizeFilter = 0;    //default, to not filter
} 

void StereoDetect::binariza_range()
{
	binariza_range(c_Range, _threshold);
}


void StereoDetect::binariza_range(cimg_library::CImg<float> & range)
{
	binariza_range(range, _threshold);
}



void StereoDetect::binariza_range(cimg_library::CImg<float> & range, float thr)
{
	range_bin.assign(range.width(), range.height());

	cimg_forX(range_bin, x)
	cimg_forY(range_bin, y){
		float ff = c_Range(x,y);
		if ( isnan( ff ) || ff > thr)
			range_bin(x,y) = 0;
		else
			range_bin(x,y) = 255;
	} 
}



void StereoDetect::filtra_range()
{
	CImg<int> seg;
	int numObj;
	CImg<int> areas;
	CImg<unsigned char> filtro;
	
	int min_area = _sizeFilter;
	
	Segmentbin8_cimg(range_bin, seg, numObj);		
	Areas_cimg(seg, areas, numObj);
	
	filtro.assign(numObj+1);
	
	filtro(0)=0; //para que el fondo no lo modifique
	for(int ii=1; ii<=numObj; ii++)
	{
		if(areas(ii) < min_area) filtro(ii) = 0; //para eliminar
		else filtro(ii) = 255;  
	}

	range_filtrada.assign(range_bin.width(),range_bin.height());
	cimg_foroff(range_filtrada,ii)
	{
		range_filtrada[ii]=filtro[seg[ii]];
	}

}

void StereoDetect::rangeAlarm2(cimg_library::CImg<unsigned char> & rectified)
{
	binariza_range();
	filtra_range();
	cout << "comprobar tamanyos y memoria retorno en color\n";
	
	cimg_forX(rectified, x)
	cimg_forY(rectified, y) 
		if(range_filtrada(x,y)){
			rectified(x,y,0) = 255;
			rectified(x,y,1) >>= 1;//>> 3;  // shift 1 bit: divide by 2 G,B. To change white pixels by soft red
			rectified(x,y,2) >>= 1;// >> 3;
		}
}

