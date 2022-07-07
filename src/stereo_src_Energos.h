/*
 * stereo_detect.cpp
 * jmmossi
 * may-2012
 *  
 */
#ifndef STEREOSRCDETECT_H_
#define STEREOSRCDETECT_H_

#include "CimgStereo.h"

class StereoDetect : public CimgStereo {

public:
	StereoDetect();

	
	void binariza_range(cimg_library::CImg<float> & range, float threshold);
	void binariza_range(cimg_library::CImg<float> & range);
	void binariza_range();
	
	void filtra_range();
	void rangeAlarm2(cimg_library::CImg<unsigned char> & rectified);
	
	void threshold(float tt){_threshold = tt;}
	float threshold(){return _threshold;}

	void sizeFilter(int tt){_sizeFilter = tt;}
	float sizeFilter(){return _sizeFilter;}
	
private:

	cimg_library::CImg<unsigned char> range_bin;
	cimg_library::CImg<unsigned char> range_filtrada;
	cimg_library::CImg<unsigned char> range_alarm2;
	float _threshold;
	int _sizeFilter;
	

}; // class StereoDetect : CimgStereo 




#endif // STEREOSRCDETECT_H_