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

class StereoCalib
{
public:
	int read_input_config( const char * filename) { return cimgstereo.read_input_config( filename ); }
	StereoCalib();
	void numberOfCorners(int nx, int ny) { _nx = nx; _ny = ny;}
	void squareSize(float s) { _squareSize_x = _squareSize_y = s; }
	void method(int m) { _method = m ;};
	void calibrateFromNames (const std::vector< std::string> & imagelist);
	void calibrateFromCompositeNames (const std::vector< std::string> & imagelist);
	void calibrate (const std::string & listFilename);
	void calibrateFromComposite (const std::string & listFilename);
	void interactive(){delay = 0;}
	void autoScale(bool b){_auto_scale = b;}
	void subPixelCorners(bool b) {_subpixel = b;}
	void preComputeIntrinsics(bool b){ _preComputeIntrinsics = b;}

	void sameFocalLength();
	void noRadialDistortion();
	void fixAspectRatio();
	void saveCVCompact();
	void saveCV();
	void saveCImg();

	void printConfig();
	const std::vector<std::string> & getListOfFiles() const {return listOfFiles;}
private:


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
	bool _preComputeIntrinsics;

	double rmsError;
	double rmsError12;
	double rmsError13;
	double reprojectionError;

	CimgStereo cimgstereo; //used to split input images

	cv::Mat cameraMatrix[3], distCoeffs[3];
	cv::Mat R, T, E, F;
	cv::Mat R12, T12, R13,T13,E12,F12,E13, F13;
	cv::Mat R1, R2, R3, P1, P2, P3, Q;



	int  _flagsCalibration ;
	int  _flagsCalibrationIntrinsic ;
	int delay;
	int _num_cameras;


	/*std::vector<std::vector<cv::Point2f> > imagePoints[2]; //Two vectors (One for left and one for right) of vectors of points
	std::vector<std::vector<cv::Point3f> > objectPoints; //Each element contains a vector with the points of one image.
	*/
	cv::Size imageSize;

	//ele
	std::vector<std::vector<cv::Point2f> > imagePointsRight; //Two vectors (One for left and one for right) of vectors of points
	std::vector<std::vector<cv::Point3f> > objectPointsRight; //Each element contains a vector with the points of one image.

	std::vector<std::vector<cv::Point2f> > imagePointsLeft; //Two vectors (One for left and one for right) of vectors of points
	std::vector<std::vector<cv::Point3f> > objectPointsLeft; //Each element contains a vector with the points of one image.

	std::vector<std::vector<cv::Point2f> > imagePointsCenter; //Two vectors (One for left and one for right) of vectors of points
	std::vector<std::vector<cv::Point3f> > objectPointsCenter; //Each element contains a vector with the points of one image.

	std::vector<std::vector<cv::Point2f> > imagePointsStereo[2]; //Two vectors (One for left and one for right) of vectors of points
	std::vector<std::vector<cv::Point3f> > objectPointsStereo; //Each element contains a vector with the points of one image.

	std::vector<std::vector<cv::Point2f> > imagePointsStereo12[2]; //Two vectors (One for left and one for right) of vectors of points
	std::vector<std::vector<cv::Point3f> > objectPointsStereo12; //Each element contains a vector with the points of one image.

	std::vector<std::vector<cv::Point2f> > imagePointsStereo13[2]; //Two vectors (One for left and one for right) of vectors of points
	std::vector<std::vector<cv::Point3f> > objectPointsStereo13; //Each element contains a vector with the points of one image.
	//ele//



	//	R The rotation matrix between the 1st and the 2nd cameras� coordinate systems.
	//	T The translation vector between the cameras� coordinate systems.
	//	R1, R2 The output 3 x 3 rectification transforms (rotation matrices) for the first and the second cameras, respectively.
	//	P1, P2 The output 3 x 4 projection matrices in the new (rectified) coordinate systems.
	//	Q The output 4 x 4 disparity-to-depth mapping matrix,
	void detectPoints(const std::vector< std::string> & imagelist);
	void detectPointsComposite(const std::vector< std::string> & imagelist);
	void calib();
	int  checkCalibration();

};
#endif
