#ifndef D_CIMGSTEREOCALIB_H
#define D_CIMGSTEREOCALIB_H

#define BOUGUET_METHOD 0 //Default
#define HARTLEY_METHOD 1

#define MAX_BASELINE 150 //mm
#define MAX_BASELINE_ROTATION 15.0 //deg
#define MAX_PIXEL_RECTANGULARITY 1.1 //ratio
#define MAX_CENTER_DEVIATION 15 // in %
#define MAX_REPROJECTION_ERROR 1.0 // in pixels
#define MAX_RMS_ERROR 1.0 // In pixels
#include "CimgStereo.h"

class OneCalib
{
public:

	OneCalib();
	void numberOfCorners(int nx, int ny) { _nx = nx; _ny = ny;}
	void squareSize(float s) { _squareSize_x = _squareSize_y = s; }
	void method(int m) { _method = m ;};


	void calibrate (const std::string & listFilename);

	void interactive(){delay = 0;}
	void autoScale(bool b){_auto_scale = b;}
	void subPixelCorners(bool b) {_subpixel = b;}

	void noRadialDistortion();
	void fixAspectRatio();
	void saveCVCompact();
	void saveCV();
	void saveCImg();

	void printConfig();
	const std::vector<std::string> & getListOfFiles() const {return listOfFiles;}
private:
	void calibrateFromNames (const std::vector< std::string> & imagelist);

	//Number of inner corners in each dimension
	int _nx;
	int _ny;
	std::vector<std::string>  listOfFiles;
	int imageWidth;
	int imageHeight;
	float _squareSize_x; //in mm.
	float _squareSize_y; //in mm.
	int nimages;

	int _method;
	int scale;
	bool _auto_scale;
	bool _subpixel;

	double rmsError;
	double reprojectionError;

	cv::Mat cameraMatrix, distCoeffs;

	int  _flagsCalibrationIntrinsic ;
	int delay;

	cv::Size imageSize;

	std::vector<std::vector<cv::Point2f> > imagePoints; //Each element contains a vector with the points of one image.
	std::vector<std::vector<cv::Point3f> > objectPoints; //Each element contains a vector with the points of one image.


	void detectPoints(const std::vector< std::string> & imagelist);
	void detectPointsComposite(const std::vector< std::string> & imagelist);
	void calib();
	int  checkCalibration();

};
#endif
