#pragma warning( disable: 4996 )
/* *************** License:**************************
 Oct. 3, 2008
 Right to use this code in any way you want without warrenty, support or any guarentee of it working.

 BOOK: It would be nice if you cited it:
 Learning OpenCV: Computer Vision with the OpenCV Library
 by Gary Bradski and Adrian Kaehler
 Published by O'Reilly Media, October 3, 2008

 AVAILABLE AT: 
 http://www.amazon.com/Learning-OpenCV-Computer-Vision-Library/dp/0596516134
 Or: http://oreilly.com/catalog/9780596516130/
 ISBN-10: 0596516134 or: ISBN-13: 978-0596516130    

 OTHER OPENCV SITES:
 * The source code is on sourceforge at:
 http://sourceforge.net/projects/opencvlibrary/
 * The OpenCV wiki page (As of Oct 1, 2008 this is down for changing over servers, but should come back):
 http://opencvlibrary.sourceforge.net/
 * An active user group is at:
 http://tech.groups.yahoo.com/group/OpenCV/
 * The minutes of weekly OpenCV development meetings are at:
 http://pr.willowgarage.com/wiki/OpenCV
 ************************************************** */





#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

//#define cimg_plugin "cimg_gpiv.h"
#include <CImg.h>
#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <ctype.h>

#include "CimgOneCalib.h"
#include <iostream>

using namespace cv;
using namespace std;
using namespace cimg_library;



void maxMinRadialDistortion( double rmax , double k1 , double k2, double k3, double & minRadialSlope, double &maxRadialSlope)
{
	int nptos = 400;
	minRadialSlope = maxRadialSlope = 1.0;
	double slope = 1.0 /nptos;

	for(int i = 0; ; i++)
	{
		double r = i * slope;
		if(r > rmax)
			break;
		double s = 1.0 + 3.0 * k1 * std::pow( r, 2.0 ) + 5.0 * k2 * std::pow(r, 4.0) + 7.0* k3 * std::pow(r,6.0);
		if(s> maxRadialSlope)
			maxRadialSlope = s;
		if(s<minRadialSlope)
			minRadialSlope = s;
	}


}

OneCalib::OneCalib() //Constructor that sets up defaults
{
	_method = BOUGUET_METHOD;
	//CV_CALIB_ZERO_TANGENT_DIST  +

	_flagsCalibrationIntrinsic =  cv::CALIB_ZERO_TANGENT_DIST  + cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5 + cv::CALIB_FIX_K6;

	_nx = 6;
	_ny = 9;
	delay = 50; //ms to show corners
	_auto_scale = true;
	_subpixel = false;
}




void OneCalib::noRadialDistortion()
{
	_flagsCalibrationIntrinsic += cv::CALIB_FIX_K1 + cv::CALIB_FIX_K2 ;
}

void OneCalib::fixAspectRatio() //It fixes pixel aspect ratio to 1
{
	_flagsCalibrationIntrinsic += cv::CALIB_FIX_ASPECT_RATIO;
}

//
// Given a list of chessboard images, the number of corners (nx, ny)
// on the chessboards, and a flag: useCalibrated for calibrated (0) or
// uncalibrated (1: use cvStereoCalibrate(), 2: compute fundamental
// matrix separately) stereo. Calibrate the cameras and display the
// rectified results along with the computed disparity images.
//



void
OneCalib::calibrateFromNames(const vector<string>& imagelist) //Image List
{

	detectPoints(imagelist);
	calib();
	imageWidth = imageSize.width;
	imageHeight = imageSize.height;
	checkCalibration();
}



//Makes some tests about the calibration results to see if they are reasonable
// Intrinsic parameters check of both cameras
//   * See if fx/fy approx 1
//   * See if (cx,cy) is near (width/2, height/2)
//   * See if radial distortion parameters may produce non monotonous distortion correction
// Stereo parameters check
// See if both cam focals are similar:TODO
//   * See if Rotation Matrix is not very different from diag([1,1,1])
//   * See if base line is longer than 30 mm. and shorter than MAX_BASELINE  mm
//   * See if Translation vector has an angle with image horizontal axis smaller than MAX_BASELINE_ROTATION degrees
int  OneCalib::checkCalibration()
{

	return 0;
}


void OneCalib::calibrate (const string & listFilename)
{
	//Read file names
	FILE* f = fopen(listFilename.c_str(), "rt");
	if (0 == f)
	{
		std::cerr << "Can't read list file " << listFilename <<". Exiting\n";
		exit(0);
	}


	char buf[1024];
	while (1)
	{
		if( !fgets( buf, sizeof(buf)-3, f ))
			break;
		size_t len = strlen(buf);
		while( len > 0 && isspace(buf[len-1])) //Remove trailing whitespace
			buf[--len] = '\0';
		if( buf[0] == '#') //Ignore lines starting with #
			continue;
		string name = buf;
		listOfFiles.push_back(name);

	}
	fclose(f);

	//Process
	calibrateFromNames(listOfFiles);
}



void OneCalib::saveCVCompact()
{

	FileStorage fs("intrinsics.yml", FileStorage::WRITE);
	if( fs.isOpened() )
	{
		fs << "M" << cameraMatrix << "D" << distCoeffs;
		fs.release();
	}
	else
		cout << "Error: can not save the intrinsic parameters\n";


	double _imgSize [2];
	_imgSize[0] = imageWidth;
	_imgSize[1] = imageHeight;

	cv::Mat imgSize =  cv::Mat(1, 2, CV_64F, _imgSize);

	FileStorage fs2("mat_imagesize.xml", FileStorage::WRITE);
	if( fs2.isOpened() )
	{
		fs2 << "imagesize" << imgSize;
		fs2.release();
	}
	else
		cout << "Error: can not save the imagesize\n";
	//cvSave("mat_imagesize.xml", & imgSize);
}


void OneCalib::saveCV()
{
	FileStorage fs;
	fs.open("mat_cam.xml", FileStorage::WRITE);
	if( fs.isOpened() )
	{
		fs << "mat_cam" << cameraMatrix;
		fs.release();
	}

	fs.open("mat_distortion.xml", FileStorage::WRITE);
	if( fs.isOpened() )
	{
		fs << "mat_distortion" << distCoeffs;
		fs.release();
	}

	double _imgSize [2];
	_imgSize[0] = imageWidth;
	_imgSize[1] = imageHeight;

	cv::Mat imgSize =  cv::Mat(1, 2, CV_64F, _imgSize);
	fs.open("mat_imagesize.xml", FileStorage::WRITE);
	if( fs.isOpened() )
	{
		fs << "imagesize" << imgSize;
		fs.release();
	}
	//cvSave("mat_imagesize.xml", imgSize);

}

void OneCalib::saveCImg()
{

	cimg_library::CImg<double> C_dist1((double *)distCoeffs.data, distCoeffs.cols , distCoeffs.rows, 1, 1, true);
	C_dist1.save_dlm("mat_distortion.dlm");

	cimg_library::CImg<double> C_mat1((double *)cameraMatrix.data, cameraMatrix.cols , cameraMatrix.rows, 1, 1, true);
	C_mat1.save_dlm("mat_cam.dlm");

	cimg_library::CImg<int> C_imagesize(2);
	C_imagesize[0] = imageWidth;
	C_imagesize[1] = imageHeight;
	C_imagesize.save_dlm("mat_imagesize.dlm");

}



void OneCalib::printConfig()
{
	std::cout<< "Square Size = " << _squareSize_x << " mm.\n";
	std::cout<< "Nx = " << _nx << ".\n";
	std::cout<< "Ny = " << _ny << ".\n";


	if(_flagsCalibrationIntrinsic & cv::CALIB_FIX_ASPECT_RATIO)
		std::cout << "Considering that pixels are square\n";
	else
		std::cout << "Considering that  pixels  may  be  rectangular\n";
	std::cout <<"\n";

	if(_flagsCalibrationIntrinsic & cv::CALIB_FIX_K1)
		std::cout << "Considering no radial distortion\n";
	else
		std::cout << "Considering that  cameras have radial distortion (straight lines appear bended)\n";

}

void
OneCalib::detectPoints(const vector<string>& imagelist) //Image List
{

	int i, j, k;
	nimages = (int)imagelist.size();

	Size boardSize;
	boardSize.width = _nx;
	boardSize.height = _ny;

	imagePoints.resize(nimages);

	for( i = j = 0; i < nimages; i++ ) // j counts valid
	{
		const string& filename = imagelist[i];
		Mat img = imread(filename, 0);
		cout << filename << endl;
		if(img.empty())
			continue;
		cout << " : file found" << endl;
		//Check Sizes
		if( imageSize == Size() ) //Size() es el constructor por defecto (0,0)
			imageSize = img.size();
		else if( img.size() != imageSize )
		{
			cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE);
			if( img.size() != imageSize ){
				cout << "The image " << filename << " has the size different from the first image size. Skipping the image\n";
				continue;
			}
		}

		bool found = false;
		vector<cv::Point2f>& corners = imagePoints[j];
		scale = 1;
		if (_auto_scale){
			double fscale = 720.0/imageSize.width;
			if(fscale < 1)	scale = 1;
			else scale = int (fscale + 0.5);
		}
		else
			scale = 1;

		Mat timg;
		if( scale == 1 )
			timg = img;
		else
			resize(img, timg, Size(), scale, scale);

		//		found = findChessboardCorners(timg, boardSize, corners/*,
		//				CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE*/);
		//
		//		//If corners are not found ---> skip image
		//		if( !found )
		//			continue;
		found = cv::findChessboardCorners(timg, boardSize, corners);
		if( !found )
			continue;
		if (_subpixel){
			cornerSubPix(timg, corners, Size(1,1), Size(-1,-1),
					TermCriteria(cv::TermCriteria::MAX_ITER+cv::TermCriteria::EPS,
							30, 0.01));
		}

		//Show corners
		if(found){
			cout << filename << endl;
			Mat cimg;
			cvtColor(timg, cimg, cv::COLOR_GRAY2BGR);
			double sf = 640./MAX(img.rows, img.cols);
			resize(cimg, cimg, Size(), sf, sf);


			vector<cv::Point2f> corners_escalados = corners;
			for(int kk=0; kk< corners_escalados.size(); kk++){
				corners_escalados[kk] *= sf;
			}
			drawChessboardCorners(cimg, boardSize, corners_escalados, found);
			imshow("corners", cimg);
			char c = (char)waitKey(delay);
			if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
				exit(-1);
			for (int s= 0; s < corners.size(); s++)	{
				corners[s] *= (1.0/scale) ;
			}
			imagePoints[j] = corners;
			j++;
		}
	}

	cout << j << " images have been successfully detected.\n";
	nimages = j;
	if( nimages < 2 ){
		cout << "Error: too few images to run the calibration\n";
		exit(0);
	}

	//Here nimages is the number of valid pairs that contained corners.
	imagePoints.resize(nimages);
	objectPoints.resize(nimages);

	//	int numPointperImage = boardSize.height * boardSize.width;
	//	CImg<double> c_objectPoints(3, nimages * numPointperImage);
	//Generate object points in a regularly spaced grid with z=0 for every point.
	for( i = 0; i < nimages; i++ ){
		for( j = 0; j < boardSize.height; j++ )
			for( k = 0; k < boardSize.width; k++ ){
				cv::Point3f P3f(k*_squareSize_x, j*_squareSize_y, 0.0);
				objectPoints[i].push_back(P3f);
			}
	}
}




void OneCalib::calib()
{
	cameraMatrix = Mat::eye(3, 3, CV_64F);

	//int flags = _flagsCalibrationIntrinsic;

	cout << "Computing  camera intrinsics ...\n";
	std::vector<cv::Mat> rvecs, tvecs;
	rmsError = cv::calibrateCamera(objectPoints, imagePoints,imageSize,
			cameraMatrix, distCoeffs , rvecs, tvecs, _flagsCalibrationIntrinsic);
	std::cout << "RMS-Error= " << rmsError << std::endl;


}
