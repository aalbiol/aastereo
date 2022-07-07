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
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/core/persistence.hpp>


#include <cmath>
#include <cstdlib>
#include <cstring>



// jmmmosi
//#include <demonio_comun.h>


//#define cimg_plugin "cimg_gpiv.h"
#include <CImg.h>
#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <ctype.h>

#include "CimgStereoCalib.h"
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

StereoCalib::StereoCalib() //Constructor that sets up defaults
{
	_method = BOUGUET_METHOD;
	//cv::CALIB_ZERO_TANGENT_DIST  +

	_flagsCalibration =  cv::CALIB_ZERO_TANGENT_DIST  + cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5 + cv::CALIB_FIX_K6;

	_flagsCalibrationIntrinsic = _flagsCalibration;
	_nx = 6;
	_ny = 9;
	delay = 50; //ms to show corners
	_auto_scale = true;
	_subpixel = false;
	_preComputeIntrinsics = true;
	_num_cameras = 2; //stereo
}


void StereoCalib::sameFocalLength()
{
	_flagsCalibration += cv::CALIB_SAME_FOCAL_LENGTH;
}

void StereoCalib::noRadialDistortion()
{
	_flagsCalibration += cv::CALIB_FIX_K1 + cv::CALIB_FIX_K2 ;
	_flagsCalibrationIntrinsic += cv::CALIB_FIX_K1 + cv::CALIB_FIX_K2 ;
}

void StereoCalib::fixAspectRatio() //It fixes pixel aspect ratio to 1
{
	_flagsCalibration += cv::CALIB_FIX_ASPECT_RATIO;
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
StereoCalib::calibrateFromNames(const vector<string>& imagelist) //Image List
{

	detectPoints(imagelist);
	calib();
	imageWidth = imageSize.width;
	imageHeight = imageSize.height;
	checkCalibration();
}


void
StereoCalib::calibrateFromCompositeNames(const vector<string>& imagelist) //Image List
{
	_num_cameras = cimgstereo.getNumCameras();
	detectPointsComposite(imagelist);
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
int  StereoCalib::checkCalibration()
{

	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0
	if(_num_cameras == 3)
		return 0;
	int i,j,k;
	double err = 0;
	int npoints = 0;
	vector<Vec3f> lines[2];
	for( i = 0; i < nimages; i++ )
	{
		int npt = (int)imagePointsStereo[0][i].size();
		Mat imgpt[2];
		for( k = 0; k < 2; k++ )
		{
			imgpt[k] = Mat(imagePointsStereo[k][i]);
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
			computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
		}
		for( j = 0; j < npt; j++ )
		{
			double errij = fabs(imagePointsStereo[0][i][j].x*lines[1][j][0] +
					imagePointsStereo[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
					fabs(imagePointsStereo[1][i][j].x*lines[0][j][0] +
							imagePointsStereo[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	reprojectionError = err/npoints;
	cout << "average reprojection err = " <<  reprojectionError << endl;

	std::cout << "========================================================\n";
	std::cout << "Calibration checking starts...\n";
	std::cout << "========================================================\n\n";
	int failsCount = 0;
	//Converto to CImg to work more easily
	cimg_library::CImg<double> C_R((double *)R.data, R.cols , R.rows, 1, 1);
	cimg_library::CImg<double> C_T((double *)T.data, T.cols , T.rows, 1, 1, true);
	cimg_library::CImg<double> C_dist1((double *)distCoeffs[0].data, distCoeffs[0].cols , distCoeffs[0].rows, 1, 1);
	cimg_library::CImg<double> C_dist2((double *)distCoeffs[1].data, distCoeffs[1].cols , distCoeffs[1].rows, 1, 1);
	cimg_library::CImg<double> C_mat1((double *)cameraMatrix[0].data, cameraMatrix[0].cols , cameraMatrix[0].rows, 1, 1);
	cimg_library::CImg<double> C_mat2((double *)cameraMatrix[1].data, cameraMatrix[1].cols , cameraMatrix[1].rows, 1, 1);
	double r1 ;
	float dx , dy;

	// Checking left cam matrix
	r1 = C_mat1(0,0)/C_mat1(1,1);
	if ( r1 < 1.0 / MAX_PIXEL_RECTANGULARITY || r1 > MAX_PIXEL_RECTANGULARITY)
	{
		failsCount ++;
		std::cout << "Suspicious Left (1) Cam focal ratio fx/fy = " << r1 << std::endl;
	}
	//	dx = C_mat1(2,0) - imageWidth / 2.0;
	//	dy = C_mat1(2,1) - imageHeight / 2.0;
	//	dist_center = std::sqrt( dx * dx + dy * dy);
	//	if ( dist_center > imageWidth * MAX_CENTER_DEVIATION / 100.0)
	//	{
	//		failsCount ++;
	//		std::cout << "Suspicious Left (1) Cam principal point. Dist to  center = " << dist_center << std::endl;
	//	}


	// Checking right cam matrix
	r1 = C_mat2(0,0)/C_mat2(1,1);
	if ( r1 < 1.0 / MAX_PIXEL_RECTANGULARITY || r1 > MAX_PIXEL_RECTANGULARITY)
	{
		failsCount ++;
		std::cout << "Suspicious Right (2) Cam focal ratio fx/fy = " << r1 << std::endl;
	}
	//	dx = C_mat2(2,0) - imageWidth / 2.0;
	//	dy = C_mat2(2,1) - imageHeight / 2.0;
	//	dist_center = std::sqrt( dx * dx + dy * dy);
	//	if ( dist_center > imageWidth * MAX_CENTER_DEVIATION / 100.0)
	//	{
	//		failsCount ++;
	//		std::cout << "Suspicious Right (2) Cam principal point. Dist to  center = " << dist_center << std::endl;
	//	}


	double k1, k2, k3;

	//Checking Left Radial Distortion
	k1 = C_dist1(0);
	k2 = C_dist1(1);
	k3 = C_dist1(4);
	if(k1 != 0.0 || k2 != 0.0  || k3 != 0.0)
	{
		dx = fabs(double (imageWidth -1 - C_mat1(2,0) ) );
		if( fabs( double (C_mat1(2,0) ) ) > dx)
			dx = fabs( double( C_mat1(2,0) ) );
		dy = fabs(double (imageHeight - 1 - C_mat1(2,1) ) );
		if( fabs( double ( C_mat1(2,1) ) ) > dy)
			dy = fabs( double (C_mat1(2,1) ) );

		dx = dx / C_mat1(0,0);
		dy = dy / C_mat1(1,1);

		double rmax = std::sqrt( dx * dx + dy * dy);

		double maxRadialSlope, minRadialSlope;
		maxMinRadialDistortion( rmax , k1 , k2, k3, minRadialSlope, maxRadialSlope);

		std::cout <<  " rmax = " << rmax << " MinSlope = " << minRadialSlope << " MaxSlope = " << maxRadialSlope <<std::endl;

	}

	// Checking Rotation Matrix
	double _pi = 4.0 * std::atan (float(1.0) );
	float deg2rad = _pi / 180.0;
	double maxCosine = std::cos( deg2rad * float(MAX_BASELINE_ROTATION) );
	if ( C_R(0,0) < maxCosine || C_R(1,1) < maxCosine || C_R(2,2) < maxCosine )
	{
		failsCount ++;
		std::cout << "Suspicious Rotation Matrix. Should be close to Identity matrix" << std::endl;
		C_R.print("Rotation Matrix");
		std::cout << std::endl;
	}

	//Baseline length
	double baseline = C_T.magnitude();
	if (baseline < 30 || baseline > MAX_BASELINE )
	{
		failsCount ++;
		std::cout << "Suspicious baseline Length. Baseline (mm) = " << baseline << std::endl;
		std::cout << "Measure intercam distance" << std::endl;
	}
	//Relative Cam position Right cam must be on the right This means a Tranlation vector with first component negative
	if( - C_T(0) / baseline < maxCosine )
	{
		failsCount ++;
		std::cout << "Suspicious Relative position of cameras. Right camera should be on the right" << std::endl;
		C_T.print("Translation Matrix");
		std::cout << std::endl;
	}

	// Check rms square error and reprojection Error

	if( rmsError > MAX_RMS_ERROR)
	{
		failsCount ++;
		std::cout << "RMS Error too large. rmsError = " << rmsError << std::endl;
	}

	if( reprojectionError > MAX_REPROJECTION_ERROR)
	{
		failsCount ++;
		std::cout << "Reprojection Error too large. Reprojection Error = " << reprojectionError << std::endl;
	}

	if(failsCount)
	{
		std::cout << "\n========================================================\n";
		std::cout << "Calibration checking has reported " << failsCount << " suspicious situations\n";
		std::cout << "========================================================\n";
	}
	else
	{
		std::cout << "\n========================================================\n";
		std::cout << "Calibration checking has passed alld the tests\n";
		std::cout << "========================================================\n";
	}
	return failsCount;
}


void StereoCalib::calibrate (const string & listFilename)
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



void StereoCalib::calibrateFromComposite( const string & listFilename)
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
	calibrateFromCompositeNames(listOfFiles);
}

void StereoCalib::saveCVCompact()
{
	double _imgSize [2];
	_imgSize[0] = imageWidth;
	_imgSize[1] = imageHeight;

	cv::Mat imgSize (1, 2, CV_64F, _imgSize);
	if(_num_cameras == 2) {
		// save intrinsic parameters
		cv::FileStorage fs("intrinsics.yml", FileStorage::WRITE);
		if( fs.isOpened() )
		{
			fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
					"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
			fs <<"ImageSize" << imgSize;
			fs.release();
		}
		else
			cout << "Error: can not save the intrinsic parameters\n";

		fs.open("extrinsics.yml", FileStorage::WRITE);
		if( fs.isOpened() )
		{
			fs << "R" << R << "T" << T;
			//fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
			//fs << "E" << E << "F" << F;
			fs.release();
		}
		else
			cout << "Error: can not save the extrinsic parameters\n";
	}
//	else 	if(_num_cameras == 3) {
//		// save intrinsic parameters
//		FileStorage fs("intrinsics.yml", FileStorage::WRITE);
//		if( fs.isOpened() )
//		{
//			fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] << //left
//					"M2" << cameraMatrix[2] << "D2" << distCoeffs[2] << //center
//					"M3" << cameraMatrix[1] << "D3" << distCoeffs[1]; //right
//			fs.release();
//		}
//		else
//			cout << "Error: can not save the intrinsic parameters\n";
//
//		fs.open("extrinsics.yml", FileStorage::WRITE);
//		if( fs.isOpened() )
//		{
//			fs << "R12" << R12 << "T12" << T12;
//			fs << "R13" << R13 << "T13" << T13;
//			fs.release();
//		}
//		else
//			cout << "Error: can not save the extrinsic parameters\n";
//
//		int nimags = imagePointsStereo13[0].size();
//		int npoints = imagePointsStereo13[0][0].size();
//		CImg<float> points1(nimags, npoints, 2);
//		CImg<float> points3(nimags, npoints, 2);
//		for (int i = 0; i < nimags; i ++) {
//			for (int p = 0; p < npoints ; p++){
//				points1( i, p,  0) = imagePointsStereo13[0][i][p].x;
//				points1( i, p,  1) = imagePointsStereo13[0][i][p].y;
//				points3( i, p,  0) = imagePointsStereo13[1][i][p].x;
//				points3( i, p,  1) = imagePointsStereo13[1][i][p].y;
//			}
//		}
//		std::cout << "saving points1.cimg and points3.cimg\n";
//		points1.save_cimg("points1.cimg");
//		points3.save_cimg("points3.cimg");
//	}

//
//	double _imgSize [2];
//	_imgSize[0] = imageWidth;
//	_imgSize[1] = imageHeight;
//
//	cv::Mat imgSize (1, 2, CV_64F, _imgSize);
//	cv::save("mat_imagesize.xml", & imgSize);
}

//
//void StereoCalib::saveCV()
//{
//
//	if(_num_cameras == 2) {
//		FileStorage fs("mat_T.xml", FileStorage::WRITE);
//		if( fs.isOpened() )
//		{
//			fs << "T" << T;
//			fs.release();
//		}
//		fs.open("mat_R.xml", FileStorage::WRITE);
//		if( fs.isOpened() )
//		{
//			fs << "R" << R;
//			fs.release();
//		}
//
//		fs.open("mat_cam1.xml", FileStorage::WRITE);
//		if( fs.isOpened() )
//		{
//			fs << "mat_cam1" << cameraMatrix[0];
//			fs.release();
//		}
//		fs.open("mat_cam2.xml", FileStorage::WRITE);
//		if( fs.isOpened() )
//		{
//			fs << "mat_cam2" << cameraMatrix[1];
//			fs.release();
//		}
//
//		fs.open("mat_distortion1.xml", FileStorage::WRITE);
//		if( fs.isOpened() )
//		{
//			fs << "mat_distortion1" << distCoeffs [0];
//			fs.release();
//		}
//
//		fs.open("mat_distortion2.xml", FileStorage::WRITE);
//		if( fs.isOpened() )
//		{
//			fs << "mat_distortion2" << distCoeffs [1];
//			fs.release();
//		}
//	}
////
////	if(_num_cameras == 3) {
////		FileStorage fs("mat_T12.xml", CV_STORAGE_WRITE);
////		if( fs.isOpened() )
////		{
////			fs << "T12" << T12;
////			fs.release();
////		}
////		fs.open("mat_R12.xml", CV_STORAGE_WRITE);
////		if( fs.isOpened() )
////		{
////			fs << "R12" << R12;
////			fs.release();
////		}
////
////		fs.open("mat_cam1.xml", CV_STORAGE_WRITE); //Left
////		if( fs.isOpened() )
////		{
////			fs << "mat_cam1" << cameraMatrix[0];
////			fs.release();
////		}
////		fs.open("mat_cam2.xml", CV_STORAGE_WRITE); //center
////		if( fs.isOpened() )
////		{
////			fs << "mat_cam2" << cameraMatrix[2];
////			fs.release();
////		}
////
////		fs.open("mat_cam3.xml", CV_STORAGE_WRITE);//right
////		if( fs.isOpened() )
////		{
////			fs << "mat_cam3" << cameraMatrix[1];
////			fs.release();
////		}
////
////
////		fs.open("mat_distortion1.xml", CV_STORAGE_WRITE); //left
////		if( fs.isOpened() )
////		{
////			fs << "mat_distortion1" << distCoeffs [0];
////			fs.release();
////		}
////
////		fs.open("mat_distortion2.xml", CV_STORAGE_WRITE); //center
////		if( fs.isOpened() )
////		{
////			fs << "mat_distortion2" << distCoeffs [2];
////			fs.release();
////		}
////
////		fs.open("mat_distortion3.xml", CV_STORAGE_WRITE); //right
////		if( fs.isOpened() )
////		{
////			fs << "mat_distortion3" << distCoeffs [1];
////			fs.release();
////		}
////	}
//	double _imgSize [2];
//	_imgSize[0] = imageWidth;
//	_imgSize[1] = imageHeight;
//
//	CvMat imgSize =  cvMat(1, 2, CV_64F, _imgSize);
//	cvSave("mat_imagesize.xml", & imgSize);
//	//P1 P2 R1 R2 and Q not saved because they derive from the above
//}

void StereoCalib::saveCImg()
{
	if(_num_cameras == 2) {
		cimg_library::CImg<double> C_R((double *)R.data, R.cols , R.rows, 1, 1, true);
		C_R.save_dlm("mat_R.dlm");

		cimg_library::CImg<double> C_T((double *)T.data, T.cols , T.rows, 1, 1, true);
		C_T.save_dlm("mat_T.dlm");

		cimg_library::CImg<double> C_dist1((double *)distCoeffs[0].data, distCoeffs[0].cols , distCoeffs[0].rows, 1, 1, true);
		C_dist1.save_dlm("mat_distortion1.dlm");
		cimg_library::CImg<double> C_dist2((double *)distCoeffs[1].data, distCoeffs[1].cols , distCoeffs[1].rows, 1, 1, true);
		C_dist2.save_dlm("mat_distortion2.dlm");

		cimg_library::CImg<double> C_mat1((double *)cameraMatrix[0].data, cameraMatrix[0].cols , cameraMatrix[0].rows, 1, 1, true);
		C_mat1.save_dlm("mat_cam1.dlm");

		cimg_library::CImg<double> C_mat2((double *)cameraMatrix[1].data, cameraMatrix[1].cols , cameraMatrix[1].rows, 1, 1, true);
		C_mat2.save_dlm("mat_cam2.dlm");

		cimg_library::CImg<int> C_imagesize(2);
		C_imagesize[0] = imageWidth;
		C_imagesize[1] = imageHeight;
		C_imagesize.save_dlm("mat_imagesize.dlm");
	}
	else 	if(_num_cameras == 3) {
		cimg_library::CImg<double> C_R12((double *)R12.data, R12.cols , R12.rows, 1, 1, true);
		C_R12.save_dlm("mat_R12.dlm");
		cimg_library::CImg<double> C_R13((double *)R13.data, R13.cols , R13.rows, 1, 1, true);
		C_R12.save_dlm("mat_R13.dlm");

		cimg_library::CImg<double> C_T12((double *)T12.data, T12.cols , T12.rows, 1, 1, true);
		C_T12.save_dlm("mat_T12.dlm");
		cimg_library::CImg<double> C_T13((double *)T13.data, T13.cols , T13.rows, 1, 1, true);
		C_T13.save_dlm("mat_T13.dlm");

		cimg_library::CImg<double> C_dist1((double *)distCoeffs[0].data, distCoeffs[0].cols , distCoeffs[0].rows, 1, 1, true); //left
		C_dist1.save_dlm("mat_distortion1.dlm");
		cimg_library::CImg<double> C_dist2((double *)distCoeffs[2].data, distCoeffs[2].cols , distCoeffs[2].rows, 1, 1, true);  //center
		C_dist2.save_dlm("mat_distortion2.dlm");
		cimg_library::CImg<double> C_dist3((double *)distCoeffs[1].data, distCoeffs[1].cols , distCoeffs[1].rows, 1, 1, true);  //right
		C_dist3.save_dlm("mat_distortion3.dlm");

		cimg_library::CImg<double> C_mat1((double *)cameraMatrix[0].data, cameraMatrix[0].cols , cameraMatrix[0].rows, 1, 1, true); //left
		C_mat1.save_dlm("mat_cam1.dlm");
		cimg_library::CImg<double> C_mat2((double *)cameraMatrix[2].data, cameraMatrix[2].cols , cameraMatrix[2].rows, 1, 1, true); //right
		C_mat2.save_dlm("mat_cam2.dlm");
		cimg_library::CImg<double> C_mat3((double *)cameraMatrix[1].data, cameraMatrix[1].cols , cameraMatrix[1].rows, 1, 1, true); //center
		C_mat3.save_dlm("mat_cam3.dlm");

		cimg_library::CImg<int> C_imagesize(2);
		C_imagesize[0] = imageWidth;
		C_imagesize[1] = imageHeight;
		C_imagesize.save_dlm("mat_imagesize.dlm");
	}


	//P1 P2 R1 R2 and Q not saved because they derive from the above
}



void StereoCalib::printConfig()
{
	std::cout<< "Square Size = " << _squareSize_x << " mm.\n";
	std::cout<< "Nx = " << _nx << ".\n";
	std::cout<< "Ny = " << _ny << ".\n";

	if(_flagsCalibration & cv::CALIB_SAME_FOCAL_LENGTH)
		std::cout << "Considering that both cameras are identical\n";
	else
		std::cout << "Considering that  cameras may be different\n";
	std::cout <<"\n";


	if(_flagsCalibration & cv::CALIB_FIX_ASPECT_RATIO)
		std::cout << "Considering that pixels are square\n";
	else
		std::cout << "Considering that  pixels  may  be  rectangular\n";
	std::cout <<"\n";

	if(_flagsCalibration & cv::CALIB_FIX_K1)
		std::cout << "Considering no radial distortion\n";
	else
		std::cout << "Considering that  cameras have radial distortion (straight lines appear bended)\n";

}

void
StereoCalib::detectPoints(const vector<string>& imagelist) //Image List
{
	if( imagelist.size() % 2 != 0 )
	{
		cout << "Error: the image list contains odd (non-even) number of elements\n";
		return;
	}



	int i, j, k;
	nimages = (int)imagelist.size()/2;

	Size boardSize;
	boardSize.width = _nx;
	boardSize.height = _ny;

	imagePointsLeft.resize(nimages);
	imagePointsRight.resize(nimages);
	imagePointsStereo[0].resize(nimages);
	imagePointsStereo[1].resize(nimages);


	for( i = j = 0; i < nimages; i++ ) // j counts valid pairs
	{
		for( k = 0; k < 2; k++ )//k=0--> left k=1--->Right
		{
			const string& filename = imagelist[i*2+k];
			Mat img = imread(filename, 0);
			if(img.empty())
				break;

			//Check Sizes
			if( imageSize == Size() ) //Size() es el constructor por defecto (0,0)
				imageSize = img.size();
			else if( img.size() != imageSize )
			{
				cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
				break;
			}


			bool found = false;
			vector<cv::Point2f>& corners = imagePointsStereo[k][j];

			if (_auto_scale)
			{
				double fscale = 720.0/imageSize.width;
				if(fscale < 1)
					scale = 1;
				else
					scale = int (fscale + 0.5);
			}
			else
				scale = 1;

			//Corners are searched in the original resolution first and if not found interpolation by maxScale is done.
			Mat timg;
			if( scale == 1 )
				timg = img;
			else
				resize(img, timg, Size(), scale, scale);

			found = findChessboardCorners(timg, boardSize, corners,
					cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);


			//If corners are not found ---> skip pair
			if( !found )
				break;
			if (_subpixel)
			{
				cornerSubPix(timg, corners, Size(1,1), Size(-1,-1),
						TermCriteria(cv::TermCriteria::MAX_ITER+cv::TermCriteria::EPS,
								30, 0.01));
			}


			//Show corners
			if(found)
			{
				std::cout << "Found Interactive Delay =" << delay<< "\n";
				imagePointsLeft[j] = imagePointsStereo[0][j];
				imagePointsRight[j] = imagePointsStereo[1][j];
				cout << filename << endl;

				Mat cimg, cimg1;
				cvtColor(timg, cimg, cv::COLOR_GRAY2BGR);
				drawChessboardCorners(cimg, boardSize, corners, found);
				double sf =  1.0; //640./MAX(img.rows, img.cols);
				resize(cimg, cimg1, Size(), sf, sf);
				imshow("corners", cimg1);
				char c = (char)waitKey(delay);
				if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
					exit(-1);

			}

			for (int s= 0; s < corners.size(); s++)
			{
				corners[s] *= (1.0/scale) ;
			}

		}
		if( k == 2 ) //We are here because we have found corners on a pair of LeftRight Images.
		{
			//			std::cout<<"Found pair\n";

			j++;
		}
	}
	cout << j << " pairs have been successfully detected.\n";
	nimages = j;
	if( nimages < 2 )
	{
		cout << "Error: too little pairs to run the calibration\n";
		exit(0);
	}


	//Here nimages is the number of valid pairs that contained corners.
	imagePointsStereo[0].resize(nimages);
	imagePointsStereo[1].resize(nimages);
	imagePointsLeft.resize(nimages);
	imagePointsRight.resize(nimages);
	objectPointsLeft.resize(nimages);
	objectPointsRight.resize(nimages);
	objectPointsStereo.resize(nimages);

	//	int numPointperImage = boardSize.height * boardSize.width;
	//	CImg<double> c_objectPoints(3, nimages * numPointperImage);
	//Generate object points in a regularly spaced grid with z=0 for every point.
	for( i = 0; i < nimages; i++ )
	{
		for( j = 0; j < boardSize.height; j++ )
			for( k = 0; k < boardSize.width; k++ )
			{
				cv::Point3f P3f(k*_squareSize_x, j*_squareSize_y, 0.0);
				objectPointsStereo[i].push_back(P3f);
				objectPointsLeft[i] = objectPointsStereo[i];
				objectPointsRight[i] = objectPointsStereo[i];
				//				c_objectPoints( 0 , numPointperImage * i +  j * boardSize.width + k);
			}
	}

}




void
StereoCalib::detectPointsComposite(const vector<string>& imagelist) //Image List
{

	int i, j, jR, jL, jC, k, j12, j13;
	//nimages = (int)imagelist.size()/2;
	nimages = (int)imagelist.size();

	Size boardSize;
	boardSize.width = _nx;
	boardSize.height = _ny;

	/*	imagePoints[0].resize(nimages);
	imagePoints[1].resize(nimages);*/
	//ele
	imagePointsRight.resize(nimages);
	imagePointsLeft.resize(nimages);
	if(_num_cameras > 2)
		imagePointsCenter.resize(nimages);
	if(_num_cameras == 2) {
		imagePointsStereo[0].resize(nimages);
		imagePointsStereo[1].resize(nimages);
	}
	else
	{
		imagePointsStereo12[0].resize(nimages);
		imagePointsStereo12[1].resize(nimages);
		imagePointsStereo13[0].resize(nimages);
		imagePointsStereo13[1].resize(nimages);
	}
	//ele//


	int foundR, foundL ,foundC;

	for( i = j = j12 = j13 = jR = jL = jC = 0; i < nimages; i++ ) // j counts valid pairs
	{
		foundR = 0;
		foundL = 0;
		foundC = 0;
		const string& filename = imagelist[i];
		CImg<unsigned char> composite( filename.c_str() );
		composite.resize(-100,-100,-100,1,2);

		CImg<unsigned char> left, right, center;

		if(_num_cameras == 2)
			cimgstereo.splitLR(composite, left, right);
		else
			cimgstereo.splitLCR(composite, left, center, right);

		if(left.size() == 0 || right.size() == 0)
		{
			std::cerr << "Error in cimgstereo.splitLR()\n";
			exit(0);
		}

		if(_num_cameras ==3 && center.size() == 0)
		{
			std::cerr << "Error in cimgstereo.splitLCR()\n";
			exit(0);
		}


		// ****left *****
		{
			cv::Mat img(left.height(), left.width(), CV_8U, left.data(), left.width() );
			if(img.empty())
				break;

			//Check Sizes
			if( imageSize == Size() )
				imageSize = img.size();
			else if( img.size() != imageSize )
			{
				cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
				continue;
			}


			bool found = false;
			//vector<Point2f>& corners = imagePoints[0][j];

			//ele
			vector<cv::Point2f>& corners = imagePointsLeft[jL];
			//ele//

			if (_auto_scale)
			{
				double fscale = 720.0/imageSize.width;
				if(fscale < 1)
					scale = 1;
				else
					scale = int (fscale + 0.5);
			}
			else
				scale = 1;

			//Corners are searched in the original resolution first and if not found interpolation by maxScale is done.
			Mat timg;
			if( scale == 1 )
				timg = img;
			else
				resize(img, timg, Size(), scale, scale);

			found = findChessboardCorners(timg, boardSize, corners,
					cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);


			//If corners are not found ---> skip pair
			/*			if( !found )
				continue;*/

			//Show corners
			if(found)
			{
				jL ++;
				foundL = 1;

				if (_subpixel)
				{
					cornerSubPix(timg, corners, Size(1,1), Size(-1,-1),
							TermCriteria(cv::TermCriteria::MAX_ITER+cv::TermCriteria::EPS,
									30, 0.01));
				}


				cout << filename << endl;

				Mat cimg, cimg1;
				cvtColor(timg, cimg, cv::COLOR_GRAY2BGR);

				drawChessboardCorners(cimg, boardSize, corners, found);
				double sf =  1.0; //640./MAX(img.rows, img.cols);
				resize(cimg, cimg1, Size(), sf, sf);
				imshow("corners", cimg1);
				char c = (char)waitKey(delay);
				if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
					exit(-1);

			}

			for (int s= 0; s < corners.size(); s++)
			{
				corners[s] *= (1.0/scale) ;
			}
		}

		//Right
		{
			cv::Mat img(right.height(), right.width(), CV_8U, right.data(), right.width() );
			if(img.empty())
				break;

			bool found = false;
			vector<cv::Point2f>& corners = imagePointsRight[jR];

			if (_auto_scale)
			{
				double fscale = 720.0/imageSize.width;
				if(fscale < 1)
					scale = 1;
				else
					scale = int (fscale + 0.5);
			}
			else
				scale = 1;

			//Corners are searched in the original resolution first and if not found interpolation by maxScale is done.
			Mat timg;
			if( scale == 1 )
				timg = img;
			else
				resize(img, timg, Size(), scale, scale);

			found = findChessboardCorners(timg, boardSize, corners,
					cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);


			//If corners are not found ---> skip pair
			/*			if( !found )
				continue;*/



			//Show corners
			if(found)
			{
				jR++;
				foundR = 1;

				if (_subpixel)
				{
					cornerSubPix(timg, corners, Size(1,1), Size(-1,-1),
							TermCriteria(cv::TermCriteria::MAX_ITER+cv::TermCriteria::EPS,
									30, 0.01));
				}


				cout << filename << endl;

				Mat cimg, cimg1;
				cvtColor(timg, cimg, cv::COLOR_GRAY2BGR);
				drawChessboardCorners(cimg, boardSize, corners, found);
				double sf =  1.0; //640./MAX(img.rows, img.cols);
				resize(cimg, cimg1, Size(), sf, sf);
				imshow("corners", cimg1);
				char c = (char)waitKey(delay);
				if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
					exit(-1);

			}

			for (int s= 0; s < corners.size(); s++)
			{
				corners[s] *= (1.0/scale) ;
			}
		}

		//Center
		if(_num_cameras >= 3)
		{
			cv::Mat img(center.height(), center.width(), CV_8U, center.data(), center.width() );
			if(img.empty())
				break;

			bool found = false;
			vector<cv::Point2f>& corners = imagePointsCenter[jC];

			if (_auto_scale)
			{
				double fscale = 720.0/imageSize.width;
				if(fscale < 1)
					scale = 1;
				else
					scale = int (fscale + 0.5);
			}
			else
				scale = 1;

			//Corners are searched in the original resolution first and if not found interpolation by maxScale is done.
			Mat timg;
			if( scale == 1 )
				timg = img;
			else
				resize(img, timg, Size(), scale, scale);

			found = findChessboardCorners(timg, boardSize, corners,
					cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);


			//If corners are not found ---> skip pair
			/*			if( !found )
				continue;*/



			//Show corners
			if(found)
			{
				jC++;
				foundC = 1;

				if (_subpixel)
				{
					cornerSubPix(img, corners, Size(1,1), Size(-1,-1),
							TermCriteria(cv::TermCriteria::MAX_ITER+cv::TermCriteria::EPS,
									30, 0.01));
				}


				cout << filename << endl;

				Mat cimg, cimg1;
				cvtColor(timg, cimg, cv::COLOR_GRAY2BGR);
				drawChessboardCorners(cimg, boardSize, corners, found);
				double sf =  1.0; //640./MAX(img.rows, img.cols);
				resize(cimg, cimg1, Size(), sf, sf);
				imshow("corners", cimg1);
				char c = (char)waitKey(delay);
				if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
					exit(-1);

			}

			for (int s= 0; s < corners.size(); s++)
			{
				corners[s] *= (1.0/scale) ;
			}
		}

		if( _num_cameras == 2 ) {
			if( foundR && foundL ){
				imagePointsStereo[0][j] = imagePointsLeft[jL -1 ];
				imagePointsStereo[1][j] = imagePointsRight[jR -1 ];
				j++;
			}
		}
		else if( _num_cameras == 3 ){
			if( foundL && foundC ){
				imagePointsStereo12[0][j12] = imagePointsLeft[jL -1 ];
				imagePointsStereo12[1][j12] = imagePointsCenter[jC -1 ];
				j12++;
			}
			if( foundL && foundR ){
				imagePointsStereo13[0][j13] = imagePointsLeft[jL -1 ];
				imagePointsStereo13[1][j13] = imagePointsRight[jR -1 ];
				j13++;
			}

		}


	}

	cout << jL << " left images have been successfully detected.\n";
	cout << jR << " right images have been successfully detected.\n";
	if(_num_cameras == 2)
		cout << j << " pairs have been successfully detected.\n";
	else if (_num_cameras ==3 ){
		cout << j12 << " pairs LC have been successfully detected.\n";
		cout << j13 << " pairs LR have been successfully detected.\n";
	}

	nimages = j;
	if( _num_cameras == 2 && j < 2 )
	{
		cout << "Error: too little pairs to run the calibration\n";
		exit(0);
	}
	if( _num_cameras == 3 && (j12 < 2 || j13 < 2) )
	{
		cout << "Error: too little pairs to run the calibration 3\n";
		exit(0);
	}

	//Here nimages is the number of valid pairs that contained corners.
	/*	imagePoints[0].resize(nimages);
	imagePoints[1].resize(nimages);
	objectPoints.resize(nimages);*/

	imagePointsRight.resize(jR);
	objectPointsRight.resize(jR);
	imagePointsLeft.resize(jL);
	objectPointsLeft.resize(jL);
	if(_num_cameras >= 3)
	{
		imagePointsCenter.resize( jC );
		objectPointsCenter.resize( jC );
	}

	if(_num_cameras == 2) {
		imagePointsStereo[0].resize(nimages);
		imagePointsStereo[1].resize(nimages);
		objectPointsStereo.resize(nimages);
	}

	if(_num_cameras == 3) {
		imagePointsStereo12[0].resize( j12 );
		imagePointsStereo12[1].resize( j12 );
		objectPointsStereo12.resize( j12 );

		imagePointsStereo13[0].resize( j13 );
		imagePointsStereo13[1].resize( j13 );
		objectPointsStereo13.resize( j13 );
	}


	// Fill object points for  intrinsics calibration
	for( i = 0; i < jR; i++ )
	{
		for( j = 0; j < boardSize.height; j++ )
			for( k = 0; k < boardSize.width; k++ )
			{
				cv::Point3f P3f(k*_squareSize_x, j*_squareSize_y, 0.0);
				objectPointsRight[i].push_back(P3f);
				//				c_objectPoints( 0 , numPointperImage * i +  j * boardSize.width + k);
			}
	}
	for( i = 0; i < jL; i++ )
	{
		for( j = 0; j < boardSize.height; j++ )
			for( k = 0; k < boardSize.width; k++ )
			{
				cv::Point3f P3f(k*_squareSize_x, j*_squareSize_y, 0.0);
				objectPointsLeft[i].push_back(P3f);
				//				c_objectPoints( 0 , numPointperImage * i +  j * boardSize.width + k);
			}
	}

	if(_num_cameras >= 3)
	{
		for( i = 0; i < jC; i++ )
		{
			for( j = 0; j < boardSize.height; j++ )
				for( k = 0; k < boardSize.width; k++ )
				{
					cv::Point3f P3f(k*_squareSize_x, j*_squareSize_y, 0.0);
					objectPointsCenter[i].push_back(P3f);
				}
		}
	}

	// Fill object points for stereo
	if(_num_cameras == 2)
	{
		for( i = 0; i < nimages; i++ )
		{
			for( j = 0; j < boardSize.height; j++ )
				for( k = 0; k < boardSize.width; k++ )
				{
					cv::Point3f P3f(k*_squareSize_x, j*_squareSize_y, 0.0);
					objectPointsStereo[i].push_back(P3f);
				}
		}
	}

	if(_num_cameras == 3)
	{
		for( i = 0; i < j12; i++ )
		{
			for( j = 0; j < boardSize.height; j++ )
				for( k = 0; k < boardSize.width; k++ )
				{
					cv::Point3f P3f(k*_squareSize_x, j*_squareSize_y, 0.0);
					objectPointsStereo12[i].push_back(P3f);
				}
		}


		for( i = 0; i < j13; i++ )
		{
			for( j = 0; j < boardSize.height; j++ )
				for( k = 0; k < boardSize.width; k++ )
				{
					cv::Point3f P3f(k*_squareSize_x, j*_squareSize_y, 0.0);
					objectPointsStereo13[i].push_back(P3f);
				}
		}

	}

}

void StereoCalib::calib()
{
	cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
	cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
	int flags = _flagsCalibration;

	if(_preComputeIntrinsics)
	{
		cout << "Computing left camera intrinsics ...\n";
		std::vector<cv::Mat> rvecs_left, tvecs_left;
		cv::calibrateCamera(objectPointsLeft, imagePointsLeft,imageSize,
				cameraMatrix[0], distCoeffs[0] , rvecs_left, tvecs_left, _flagsCalibrationIntrinsic);


		cout << "Computing right camera intrinsics ...\n";
		std::vector<cv::Mat> rvecs_right, tvecs_right;
		cv::calibrateCamera(objectPointsRight, imagePointsRight,imageSize,
				cameraMatrix[1], distCoeffs[1] , rvecs_right, tvecs_right, _flagsCalibrationIntrinsic);

		if(_num_cameras >= 3) {
			cout << "Computing center camera intrinsics ...\n";
			std::vector<cv::Mat> rvecs_center, tvecs_center;
			cv::calibrateCamera(objectPointsCenter, imagePointsCenter,imageSize,
					cameraMatrix[2], distCoeffs[2] , rvecs_center, tvecs_center, _flagsCalibrationIntrinsic);
		}

		flags += + cv::CALIB_FIX_INTRINSIC;
	}

	if(_num_cameras == 2 ) {
		cout << "Running stereo calibration ...\n";
		rmsError = stereoCalibrate(objectPointsStereo, imagePointsStereo[0], imagePointsStereo[1],
				cameraMatrix[0], distCoeffs[0],
				cameraMatrix[1], distCoeffs[1],
				imageSize, R, T, E, F,flags,
				TermCriteria(cv::TermCriteria::MAX_ITER+cv::TermCriteria::EPS, 100, 1e-5));
		cout << "done with RMS error=" << rmsError << endl;
	}
	else if(_num_cameras == 3 ){
		cout << "Running LC (12) calibration ...\n";
		rmsError12 = stereoCalibrate(objectPointsStereo12, imagePointsStereo12[0], imagePointsStereo12[1],
				cameraMatrix[0], distCoeffs[0],
				cameraMatrix[2], distCoeffs[2],
				imageSize, R12, T12, E, F,flags,
				TermCriteria(cv::TermCriteria::MAX_ITER+cv::TermCriteria::EPS, 100,1e-5));
		cout << "done with RMS error=" << rmsError12 << endl;

		cout << "Running LR (13) calibration ...\n";
		rmsError13 = stereoCalibrate(objectPointsStereo13, imagePointsStereo13[0], imagePointsStereo13[1],
				cameraMatrix[0], distCoeffs[0],
				cameraMatrix[1], distCoeffs[1],
				imageSize, R13, T13, E, F,flags,
				TermCriteria(cv::TermCriteria::MAX_ITER+cv::TermCriteria::EPS, 100, 1e-5));
		cout << "done with RMS error=" << rmsError13 << endl;


	}
}
