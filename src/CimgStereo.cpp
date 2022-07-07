/*
 * CimgStereo.cpp
 *
 *  Created on: Sep 7, 2011
 *      Author: aalbiol
 */
#include <vector>
#include <string>
#include <iostream>
//#define cimg_plugin "cimg_gpiv.h"

#include <math.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>


#include "opencv2/core/types_c.h"
#include <cmath>
//#include <demonio_comun.h>



//#define cimg_plugin2 "cimgIPL.h"
#define cimg_plugin1 "cvMat.h"
#include <CImg.h>
using namespace cimg_library;
using namespace std;
using namespace cv;

#include "CimgStereo.h"

CimgStereo::CimgStereo() {
	bm = StereoBM::create();
	bm2 =StereoBM::create();
	//	cv::StereoBM bm;
	//	cv::StereoBM bm2; //Decimation 2

	bm_12_2 = StereoBM::create();
	bm_12=StereoBM::create();
	bm_13=StereoBM::create();

	sgbm = StereoSGBM::create(0,16,11);
	sgbm2 = StereoSGBM::create(0,16,11);
	sgbm_12= StereoSGBM::create(0,16,11);
	sgbm_12_2 = StereoSGBM::create(0,16,11);
	sgbm_13 = StereoSGBM::create(0,16,11);
	//Default Values
	decimate = 1;
	scale_ = 1;
	inputImageSize.width = inputImageSize.height = calibImageSize.width = calibImageSize.height = 0;
	_nf = 0;

	minimumDistance = 0.500; //m.
	maximumDistance = 4.000; //m.

	camHeight_ = 0.0;
	layoutLeft_ = -1;
	layoutRight_ = -1;
	layoutCenter_ = -1;
	SGBM_SmoothFactor_ = 1.0;
	multiscaleMode_ = MODE_CLASSIC;
	method_ = METHOD_BM; //Default
	setBMDefaults();
	warning_bad_depthasColor=0;

	SADWindowSizeRel(-1.0);
	vertBlur_ = 0.0;
	alphaRectify_ = 0.0;
}


CimgStereo::CimgStereo(const CimgStereo & cs) {

	_nf = 0; //frame counter;
	decimate = cs.decimate;
	scale_ = cs.scale_;
	_textureThreshold = cs._textureThreshold;

	method_ =cs.method_; //For disparity estimation
	//Input images: no need to copy


	//Recified input images: 3cam no need to copy

	//Disparity and depth no need to copy

	//Stereo params

	SADWindowSizeRel_ = cs.SADWindowSizeRel_;
	vertBlur_ = cs.vertBlur_; // To compensate synchronization errors




	points_image1_calib_ = cs.points_image1_calib_;
	points_image3_calib_ = cs.points_image3_calib_;
	points1_at_input_scale_ = cs.points1_at_input_scale_;
	points3_at_input_scale_ = cs.points3_at_input_scale_;

	ratio12_13 = cs.ratio12_13;
	calibImageSize = cs.calibImageSize;
	inputImageSize = cs.inputImageSize;
	imageSize_rect = cs.imageSize_rect;

	rectifiedCamFocal = cs.rectifiedCamFocal ;
	baseLine = cs.baseLine ;
	minimumDistance = cs.minimumDistance ;
	maximumDistance = cs.maximumDistance ;
	meanDistance_ = cs.meanDistance_ ;

	minDisparity = cs.minDisparity ;
	maxDisparity = cs.maxDisparity ;

	//Multiscale
	multiscaleMode_  = cs.multiscaleMode_;
	minDisparity1 = cs.minDisparity1;
	maxDisparity1 = cs.maxDisparity1;
	minDisparity2 = cs.minDisparity2;
	maxDisparity2 = cs.maxDisparity2;
	//For 3 cams
	minDisparity12 = cs.minDisparity12;
	maxDisparity12 = cs.maxDisparity12;
	minDisparity12_2 = cs.minDisparity12_2;
	maxDisparity12_2 = cs.maxDisparity12_2;
	minDisparity13 = cs.minDisparity13;
	maxDisparity13 = cs.maxDisparity13;
	//Stereo processor


	SGBM_SmoothFactor_ = cs.SGBM_SmoothFactor_;
	rotationMatrix_ = cs.rotationMatrix_ ;

	camHeight_ = cs.camHeight_;
	_stereoCalibDirectory = cs._stereoCalibDirectory;

	//Input Format
	layoutLeft_ = cs.layoutLeft_;
	layoutRight_ = cs.layoutRight_;
	layoutCenter_ = cs.layoutCenter_; //If -1: two cam stereo

	warning_bad_depthasColor = cs.layoutCenter_;
	alphaRectify_ = 1.0;
}




//If files in current directory stereoConfigDirectory = "."
void CimgStereo::loadStereoCalibration(const std::string & stereoCalibDirectory)
{
	if(stereoCalibDirectory.size() == 0)
	{
		std::cerr << "Invalid Empty stereo Config Directory. Current directory should be '.'\n";
	}

	if (layoutCenter_ < 0) {
		std::string filename;
		filename = stereoCalibDirectory;
		filename += "/intrinsics.yml";
		std::cout<<"Importing stereo calibration file: "<< filename << "\n";
		// reading intrinsic parameters
		cv::FileStorage fs(filename.c_str(), FileStorage::READ);
		if(!fs.isOpened())
		{
			printf("Failed to open file %s\n", filename.c_str());
			exit(0);
		}

		fs["M1"] >> M1;
		fs["D1"] >> D1;
		fs["M2"] >> M2;
		fs["D2"] >> D2;

		// reading extrinsic parameters
		filename = stereoCalibDirectory;
		filename += "/extrinsics.yml";
		std::cout<<"Importing stereo calibration file: "<< filename << "\n";
		cv::FileStorage fs2(filename.c_str(), FileStorage::READ);
		if(!fs2.isOpened())
		{
			printf("Failed to open file %s\n", filename.c_str());
			exit(0);
		}

		fs2["R"] >> R;
		fs2["T"] >> T;
	}
	else { //3-cams
		std::string filename;
		filename = stereoCalibDirectory;
		filename += "/intrinsics.yml";
		std::cout<<"Importing stereo calibration file: "<< filename << "\n";
		// reading intrinsic parameters
		cv::FileStorage fs(filename.c_str(), FileStorage::READ);
		if(!fs.isOpened())
		{
			printf("Failed to open file %s\n", filename.c_str());
			exit(0);
		}

		fs["M1"] >> M1;
		fs["D1"] >> D1;
		fs["M2"] >> M2;
		fs["D2"] >> D2;
		fs["M3"] >> M3;
		fs["D3"] >> D3;

		// reading extrinsic parameters
		filename = stereoCalibDirectory;
		filename += "/extrinsics.yml";
		std::cout<<"Importing stereo calibration file: "<< filename << "\n";
		cv::FileStorage fs2(filename.c_str(), FileStorage::READ);
		if(!fs2.isOpened())
		{
			printf("Failed to open file %s\n", filename.c_str());
			exit(0);
		}

		fs2["R12"] >> R12;
		fs2["T12"] >> T12;

		fs2["R13"] >> R13;
		fs2["T13"] >> T13;

		filename = stereoCalibDirectory;
		filename += "/points1.cimg";
		points_image1_calib_.load(filename.c_str());

		filename = stereoCalibDirectory;
		filename += "/points3.cimg";
		points_image3_calib_.load(filename.c_str());
	}
	//Getting the size for which images were calibrated
	std::string filename;
	filename = stereoCalibDirectory;
	filename += "/mat_imagesize.dlm";
	std::cout<<"Importing stereo calibration file: "<< filename << "\n";
	CImg<int> tmp;
	tmp.load_dlm(filename.c_str());

	calibImageSize.width = tmp(0);
	calibImageSize.height = tmp(1);

	fprintf(stderr, "Calibration imageSize: %d %d\n", calibImageSize.width, calibImageSize.height);
	_stereoCalibDirectory = stereoCalibDirectory;
}




cimg_library::CImg<float> CimgStereo::getLeftProjectionMatrix() const {
	cimg_library::CImg<float> P1cimg(P1);
	return P1cimg;
}



int CimgStereo::start() {

	if (_stereoCalibDirectory.size() == 0)
	{
		std::cerr<<"Stereo Calibration not loaded. Exiting.\n";
		exit(0);
	}

	if(decimate <0 || decimate > 2)
	{
		std::cerr << "Invalid decimate Factor: " << decimate <<"\n";
		exit(0);
	}

	if(SADWindowSize() % 2 == 0 && SADWindowSizeRel_ < 0)
	{
		std::cerr << "SADWindowSize = " << SADWindowSize() <<" should be an odd number.\n";
		exit(0);
	}


	if( method_ == METHOD_OFLOW)
		decimate = 1;

	printf("Computing  cvStereo Rectify matrices...");

	cv::Mat M1e = M1.clone();
	cv::Mat M2e = M2.clone();
	cv::Mat M3e = M3.clone();

	float factorx = float(inputImageSize.width) / float(calibImageSize.width);
	float factory = float(inputImageSize.height) / float(calibImageSize.height);



	M1e.at<double>(0 , 0) *= factorx;
	M1e.at<double>( 0, 2 ) *=  factorx;
	M2e.at<double>(0 , 0) *=  factorx;
	M2e.at<double>(0, 2) *= factorx;


	M1e.at<double>(1,1) *= factory;
	M1e.at<double>(1, 2) *= factory;
	M2e.at<double>(1,1) *= factory;
	M2e.at<double>(1,2) *= factory;

	if (layoutCenter_ > 0) {
		M3e.at<double>(0 , 0) *=  factorx;
		M3e.at<double>(0, 2) *= factorx;
		M3e.at<double>(1,1) *= factory;
		M3e.at<double>(1, 2) *= factory;

	}



	//Here it is possible to scale the images
	//	imageSize_rect.width = inputImageSize.width * factory /factormin * scale_;
	//	imageSize_rect.height = inputImageSize.height * factorx /factormin * scale_;
	float centerx= M1.at<double>( 0, 2 );
	float focalx = M1.at<double>( 0, 0 );
	float sizex = calibImageSize.width;

	float centery= M1.at<double>( 1, 2 );
	float focaly = M1.at<double>( 1, 1 );
	float sizey = calibImageSize.height;

	float k1 =  D1.at<double>(0);
	float k2 =  D1.at<double>(1);
	float k3 =  D1.at<double>(2);


	float realwidth = determineRealSize(sizex, centerx, focalx, k1, k2 , k3);
	float realheight = determineRealSize(sizey, centery, focaly, k1, k2 , k3);
	imageSize_rect.width = realwidth * scale_;
	imageSize_rect.height = realheight * scale_;

	//	imageSize_rect.width = calibImageSize.width * scale_;
	//	imageSize_rect.height = calibImageSize.height * scale_;
	//0.9 is set because otherwise it crops the images too much in the horizontal direction if there is radial distortion
	if( factorx != 1.0 || factory != 1)
	{
		std::cout << "INFO: Calib Size != input size. Adjusting intrinsics\n";
		std::cout << "Input Size: " << inputImageSize.width << " x " << inputImageSize.height << "\n";
		std::cout << "Disparity Size: " << imageSize_rect.width << " x " << imageSize_rect.height << "\n";
		std::cout << "fx = " << factorx << "  fy = " << factory << "\n";

		std::cout << "M1: \n";
		std::cout << M1 << "\n";
		std::cout << M1e << "\n";
		std::cout << "M2: \n";
		std::cout << M2 << "\n";
		std::cout << M2e << "\n";

	}


	if (SADWindowSizeRel_ > 0) {
		int s = imageSize_rect.height * SADWindowSizeRel_ / 100.0;
		if (s%2 == 0) //Force to be odd
			s++;
		SADWindowSize(s);
	}
	//This is to make the threshold independent of window size
	bm->setTextureThreshold(_textureThreshold * bm->getBlockSize() * bm->getBlockSize());

	if(layoutCenter_ > 0) {
		int nimags = points_image1_calib_.width();
		int npoints = points_image1_calib_.height();
		points1_at_input_scale_.resize(nimags);
		points3_at_input_scale_.resize(nimags);

		for( int i = 0; i < nimags ; i++) {
			for( int p = 0; p < npoints; p++) {
				cv::Point2f P1( points_image1_calib_(i,p,0) * factorx , points_image1_calib_(i,p,1) * factory );
				cv::Point2f P3( points_image3_calib_(i,p,0) * factorx , points_image3_calib_(i,p,1) * factory );
				points1_at_input_scale_[i].push_back( P1 );
				points3_at_input_scale_[i].push_back( P3 );
			}
		}
	}


	if (layoutCenter_ < 0) { // 2 cams
		cv::stereoRectify( M1e, D1, M2e, D2, inputImageSize, R, T,//Input params
				R1, R2, P1, P2, Q, //Output params
				0 /*CALIB_ZERO_DISPARITY*/, alphaRectify_, imageSize_rect );//,& roi1, & roi2 ); //flags and options
	}
	//CALIB_ZERO_DISPARITY: Si las cámaras están aproximadamente paralelas y mirando perpendicular al baseline.
	// En caso de cámaras convergente mejor poner 0

	else
	{
		//		ratio12_13 = upvsoft::rectify3Collinear(M1e, D1, M2e, D2,
		//				M3e, D3,
		//				points1_at_input_scale_, points3_at_input_scale_, //scaled
		//				inputImageSize, R12, T12, R13, T13,
		//				R1, R2, R3, P1, P2, P3, Q, double(0.0),
		//				imageSize_rect, (Rect*) 0, (Rect*)0, int (CV_CALIB_ZERO_DISPARITY) );
	}



	std::cout << "done\nBuilding Rectification maps..." ;

	cv::initUndistortRectifyMap(M1e, D1, R1, P1, imageSize_rect, CV_16SC2, map11, map12);
	cv::initUndistortRectifyMap(M2e, D2, R2, P2, imageSize_rect, CV_16SC2, map21, map22);
	if (layoutCenter_ > 0) { // 3 cams
		cv::initUndistortRectifyMap(M3e, D3, R3, P3, imageSize_rect, CV_16SC2, map31, map32);
	}

	printf("done\n");

	//fabs by Antonio to be compatible with versions of opencv >=2.4 that patched the error discovered by Mossi
	Q.at<double>(3,2) = fabs ( Q.at<double>(3,2) );

	cimg_Q.assign( Q );
	cimg_Q.save_dlm("mat_Q.dlm");

	rectifiedCamFocal = cimg_Q(3,2);
	rectifiedCamFocal = std::fabs( rectifiedCamFocal );
	baseLine = cimg_Q(2,3);
	baseLine = std::fabs(1.0 / baseLine);



	if (layoutCenter_ < 0) { // 2 cams
		sgbm->setPreFilterCap(bm->getPreFilterCap());
		sgbm2->setPreFilterCap(bm->getPreFilterCap());
		bm2->setPreFilterCap(bm->getPreFilterCap());


		sgbm->setBlockSize(bm->getBlockSize());
		sgbm2->setBlockSize(bm->getBlockSize());
		bm2->setBlockSize(bm->getBlockSize());

		sgbm->setSpeckleWindowSize( DEFAULT_SPECKLE_WINSIZE ); // the maximum area of speckles to remove
		// (set to 0 to disable speckle filtering)


		//	sgbm.setfullDP = sgbm2.fullDP = false;
		sgbm->setMode(StereoSGBM::MODE_SGBM);
		sgbm->setP1( 8 * sgbm->getBlockSize() * sgbm->getBlockSize() * SGBM_SmoothFactor_);
		//sgbm.P1 = 8 * sgbm.SADWindowSize * sgbm.SADWindowSize * SGBM_SmoothFactor_;
		//		sgbm.P2 = 32 * sgbm.SADWindowSize * sgbm.SADWindowSize * SGBM_SmoothFactor_;
		sgbm->setP2( 32 * sgbm->getBlockSize() * sgbm->getBlockSize() * SGBM_SmoothFactor_);

		//		sgbm2.P1 = 8 * sgbm2.SADWindowSize * sgbm2.SADWindowSize * SGBM_SmoothFactor_;
		//		sgbm2.P2 = 32 * sgbm2.SADWindowSize * sgbm2.SADWindowSize * SGBM_SmoothFactor_;
		sgbm2->setP2( 32 * sgbm2->getBlockSize() * sgbm2->getBlockSize() * SGBM_SmoothFactor_);
		sgbm2->setP1( 8 * sgbm2->getBlockSize() * sgbm2->getBlockSize() * SGBM_SmoothFactor_);

		//sgbm.speckleWindowSize = bm.state->speckleWindowSize;
		//sgbm2.speckleWindowSize = bm2.state->speckleWindowSize = bm.state->speckleWindowSize / 4;
		sgbm->setSpeckleWindowSize( bm->getSpeckleWindowSize() );
		sgbm2->setSpeckleWindowSize( bm->getSpeckleWindowSize() / 4);

		//sgbm.speckleRange = bm.state->speckleRange ;
		//bm2.state->speckleRange = bm.state->speckleRange ;
		//sgbm2.speckleRange = bm2.state->speckleRange ;
		bm2->setSpeckleRange( bm->getSpeckleRange());
		sgbm->setSpeckleRange( bm->getSpeckleRange());
		sgbm2->setSpeckleRange( bm->getSpeckleRange());


		//		sgbm2.uniquenessRatio = sgbm.uniquenessRatio = bm2.state->uniquenessRatio = bm.state->uniquenessRatio;
		//		sgbm2.disp12MaxDiff = sgbm.disp12MaxDiff = bm2.state->disp12MaxDiff = bm.state->disp12MaxDiff;
		bm2->setUniquenessRatio( bm->getUniquenessRatio());
		sgbm->setUniquenessRatio( bm->getUniquenessRatio());
		sgbm2->setUniquenessRatio( bm->getUniquenessRatio());
		bm2->setDisp12MaxDiff( bm->getDisp12MaxDiff());
		sgbm->setDisp12MaxDiff( bm->getDisp12MaxDiff());
		sgbm2->setDisp12MaxDiff( bm->getDisp12MaxDiff());

		//bm.state->textureThreshold = _textureThreshold * bm.state->SADWindowSize * bm.state->SADWindowSize;
		//bm2.state->textureThreshold = _textureThreshold * bm2.state->SADWindowSize * bm2.state->SADWindowSize;
		bm->setTextureThreshold( _textureThreshold /** bm->getBlockSize() * bm->getBlockSize()*/);
		bm2->setTextureThreshold( _textureThreshold /** bm2->getBlockSize() * bm2->getBlockSize()*/);

		if(decimate > 0)
		{
			minDisparity = 0;
			maxDisparity = ceil( rectifiedCamFocal * baseLine / (minimumDistance * 1000.0) );

			int disparities = ceil( ( maxDisparity - minDisparity) / 16.0) * 16.0;
			int disparities2 = ceil( ( maxDisparity - minDisparity) / 32.0) * 16.0;
			std::cout << "Min.Disparity =  "<< minDisparity;
			std::cout <<"MaxDisparity=" << maxDisparity <<  " Number of disparities: " << disparities <<  "   Number of disparities2: " << disparities2<< "\n";

			//bm.state->minDisparity = minDisparity;
			//bm.state->numberOfDisparities = disparities;
			//sgbm.minDisparity = minDisparity;
			//sgbm.numberOfDisparities = disparities;
			bm->setMinDisparity(minDisparity);
			sgbm->setMinDisparity(minDisparity);
			bm->setNumDisparities(disparities);
			sgbm->setNumDisparities(disparities);

			//sgbm2.minDisparity =  bm2.state->minDisparity = bm.state->minDisparity / 2 ;
			bm2->setMinDisparity(minDisparity / 2);
			sgbm2->setMinDisparity(minDisparity / 2);
			//sgbm2.numberOfDisparities = bm2.state->numberOfDisparities = disparities2;
			bm2->setNumDisparities(disparities2);
			sgbm2->setNumDisparities(disparities2);




		}
		else //decimate == 0 Multiscale 1: far 2 near
		{
			//meanDistance_ = (minimumDistance + maximumDistance) / 2.0;

			if(multiscaleMode_ == MODE_CLASSIC) {
				//Distance range is split between both resolutions
				meanDistance_ = std::sqrt(float(minimumDistance) * float(maximumDistance) );
				minDisparity = minDisparity1 = 0;
				maxDisparity1 = ceil( rectifiedCamFocal * baseLine / (meanDistance_ * 0.9 * 1000.0));

				minDisparity2 = floor( rectifiedCamFocal/2.0 * baseLine / (meanDistance_ * 1.1 * 1000.0));
				maxDisparity2 = ceil( rectifiedCamFocal/ 2.0 * baseLine / (minimumDistance * 1000.0));
			}

			if (multiscaleMode_ == MODE_ROBUST) {
				//Both resolutions cover the same range. If both disparities are consistent-->measure. Else --->missing
				minDisparity = minDisparity1 = minDisparity2 = 0;
				maxDisparity1 = ceil( rectifiedCamFocal * baseLine / (minimumDistance* 1000.0));
				maxDisparity2 = ceil( rectifiedCamFocal/ 2.0 * baseLine / (minimumDistance * 1000.0));
			}

			int disparities1 = ceil( ( maxDisparity1 - minDisparity1) / 16.0) * 16.0;
			int disparities2 = ceil( ( maxDisparity2 - minDisparity2) / 16.0) * 16.0;
			std::cout << "Min.Disparity =  "<< minDisparity;
			std::cout <<" MaxDisparity=" << maxDisparity1 <<  " Number of disparities: " << disparities1 <<  "\n";

			std::cout <<"MinDistance1= " <<   (float) rectifiedCamFocal *baseLine / (float)(minDisparity+disparities1)   << "\n";

			std::cout << " Min.Disparity2 =  "<< minDisparity2;
			std::cout <<" MaxDisparity2=" << maxDisparity2 <<  "   Number of disparities2: " << disparities2<< "\n";

			std::cout <<"MinDistance2= " <<   (float) rectifiedCamFocal / 2.0 *baseLine / (float)(minDisparity2 + disparities2)   << "\n";
			std::cout <<"MaxDistance2= " <<   (float) rectifiedCamFocal / 2.0 *baseLine / (float)(minDisparity2 )   << "\n";


			//	std::cout << "Min.Disparity =  "<< minDisparity <<  " Number of disparities: " << disparities <<  "   Number of disparities2: " << disparities2<< "\n";

			//			sgbm.minDisparity = bm.state->minDisparity = minDisparity1;
			//			sgbm.numberOfDisparities = bm.state->numberOfDisparities = disparities1;
			//			sgbm2.minDisparity = bm2.state->minDisparity = minDisparity2 ;
			//			sgbm2.numberOfDisparities = bm2.state->numberOfDisparities = disparities2;

			bm->setMinDisparity(minDisparity1);
			sgbm->setMinDisparity(minDisparity1);
			bm->setNumDisparities(disparities1);
			sgbm->setNumDisparities(disparities1);

			bm2->setMinDisparity(minDisparity2);
			sgbm2->setMinDisparity(minDisparity2);
			bm2->setNumDisparities(disparities2);
			sgbm2->setNumDisparities(disparities2);


		}


	}
	else { // 3 cams
		//		if (ratio12_13 < 1.9){
		//			std::cerr << "Using three cameras with baselines very similar\n";
		//			exit(0);
		//		}
		//
		//		if(multiscaleMode_ == MODE_CLASSIC) {
		//			double logdmin = std::log ( minimumDistance);
		//
		//
		//			//	double centerDistance = std::exp( logdcenter); //for pairs 12-13
		//			//	double centerDistance2 = std::sqrt ( minimumDistance * centerDistance) ; //for multiresolution 12
		//
		//			double centerDistance =  2* minimumDistance * ratio12_13; //for pairs 12-13
		//			double centerDistance2 = 2* minimumDistance ; //for multiresolution 12
		//
		//
		//
		//			//baseline: is the one of pair 12
		//			minDisparity13 = 0;
		//			maxDisparity13 = ceil( rectifiedCamFocal * baseLine * ratio12_13/ (centerDistance * 0.9 * 1000.0));
		//
		//			minDisparity12 = floor( rectifiedCamFocal * baseLine / (centerDistance * 1.1 * 1000.0));
		//			maxDisparity12 = ceil( rectifiedCamFocal * baseLine / (centerDistance2 * 0.9  * 1000.0));
		//
		//
		//			minDisparity12_2 = floor( rectifiedCamFocal /2.0 * baseLine / (centerDistance2 * 1.1 * 1000.0));
		//			maxDisparity12_2 = ceil( rectifiedCamFocal/ 2.0 * baseLine / (minimumDistance * 1000.0));
		//
		//			int disparities13 = ceil( ( maxDisparity13 - minDisparity13) / 16.0) * 16.0;
		//			int disparities12 = ceil( ( maxDisparity12 - minDisparity12) / 16.0) * 16.0;
		//			int disparities12_2 = ceil( ( maxDisparity12_2 - minDisparity12_2) / 16.0) * 16.0;
		//			std::cout << "Min.Disparity13 =  "<< minDisparity13;
		//			std::cout << "  Max.Disparity13 =  "<< maxDisparity13;
		//			std::cout << "  Num Disparities_13 =  "<< disparities13;
		//
		//			std::cout << "\nMin.Disparity12 =  "<< minDisparity12;
		//			std::cout << "Max.Disparity12 =  "<< maxDisparity12;
		//			std::cout << "Num Disparities_12 =  "<< disparities12;
		//
		//			std::cout << "\nMin.Disparity12_2 =  "<< minDisparity12_2;
		//			std::cout << "Max.Disparity12_2 =  "<< maxDisparity12_2;
		//			std::cout << "Num Disparities_12_2 =  "<< disparities12_2 <<"\n";
		//			//	std::cout << "Min.Disparity =  "<< minDisparity <<  " Number of disparities: " << disparities <<  "   Number of disparities2: " << disparities2<< "\n";
		//
		//			sgbm_13.minDisparity = bm_13.state->minDisparity = minDisparity13;
		//			sgbm_13.numberOfDisparities = bm_13.state->numberOfDisparities = disparities13;
		//
		//			sgbm_12.minDisparity = bm_12.state->minDisparity = minDisparity12;
		//			sgbm_12.numberOfDisparities = bm_12.state->numberOfDisparities = disparities12;
		//
		//			sgbm_12_2.minDisparity = bm_12_2.state->minDisparity = minDisparity12_2;
		//			sgbm_12_2.numberOfDisparities = bm_12_2.state->numberOfDisparities = disparities12_2;
		//
		//
		//			sgbm_13.preFilterCap = sgbm_12.preFilterCap = sgbm_12_2.preFilterCap  = bm_13.state-> preFilterCap = bm_12.state-> preFilterCap = bm_12_2.state-> preFilterCap = bm.state-> preFilterCap;;
		//
		//			bm_13.state->SADWindowSize = bm_12.state->SADWindowSize = bm_12_2.state->SADWindowSize = bm.state->SADWindowSize;
		//			sgbm_13.SADWindowSize = sgbm_12.SADWindowSize = sgbm_12_2.SADWindowSize = bm.state->SADWindowSize;
		//
		//		}
		//
		//
		//		if(multiscaleMode_ == MODE_ROBUST) {
		//
		//
		//			double centerDistance =  2* minimumDistance * ratio12_13; //for pairs 12-13
		//
		//
		//
		//
		//			//baseline: is the one of pair 12
		//			minDisparity13 = 0;
		//			maxDisparity13 = ceil( rectifiedCamFocal * baseLine * ratio12_13/ (centerDistance * 0.9 * 1000.0));
		//
		//			minDisparity12 = 0;
		//			maxDisparity12 = ceil( rectifiedCamFocal * baseLine / (minimumDistance * 1000.0));
		//
		//
		//			minDisparity12_2 = floor( rectifiedCamFocal /2.0 * baseLine / (centerDistance * 1.1 * 1000.0));
		//			maxDisparity12_2 = ceil( rectifiedCamFocal/ 2.0 * baseLine / (minimumDistance * 1000.0));
		//
		//			int disparities13 = ceil( ( maxDisparity13 - minDisparity13) / 16.0) * 16.0;
		//			int disparities12 = ceil( ( maxDisparity12 - minDisparity12) / 16.0) * 16.0;
		//			int disparities12_2 = ceil( ( maxDisparity12_2 - minDisparity12_2) / 16.0) * 16.0;
		//			std::cout << "Min.Disparity13 =  "<< minDisparity13;
		//			std::cout << "  Max.Disparity13 =  "<< maxDisparity13;
		//			std::cout << "  Num Disparities_13 =  "<< disparities13;
		//
		//			std::cout << "\nMin.Disparity12 =  "<< minDisparity12;
		//			std::cout << "Max.Disparity12 =  "<< maxDisparity12;
		//			std::cout << "Num Disparities_12 =  "<< disparities12;
		//
		//			std::cout << "\nMin.Disparity12_2 =  "<< minDisparity12_2;
		//			std::cout << "Max.Disparity12_2 =  "<< maxDisparity12_2;
		//			std::cout << "Num Disparities_12_2 =  "<< disparities12_2 <<"\n";
		//			//	std::cout << "Min.Disparity =  "<< minDisparity <<  " Number of disparities: " << disparities <<  "   Number of disparities2: " << disparities2<< "\n";
		//
		//			sgbm_13.minDisparity = bm_13.state->minDisparity = minDisparity13;
		//			sgbm_13.numberOfDisparities = bm_13.state->numberOfDisparities = disparities13;
		//
		//			sgbm_12.minDisparity = bm_12.state->minDisparity = minDisparity12;
		//			sgbm_12.numberOfDisparities = bm_12.state->numberOfDisparities = disparities12;
		//
		//			sgbm_12_2.minDisparity = bm_12_2.state->minDisparity = minDisparity12_2;
		//			sgbm_12_2.numberOfDisparities = bm_12_2.state->numberOfDisparities = disparities12_2;
		//
		//
		//			sgbm_13.preFilterCap = sgbm_12.preFilterCap = sgbm_12_2.preFilterCap  = bm_13.state-> preFilterCap = bm_12.state-> preFilterCap = bm_12_2.state-> preFilterCap = bm.state-> preFilterCap;;
		//
		//			bm_13.state->SADWindowSize = bm_12.state->SADWindowSize = bm_12_2.state->SADWindowSize = bm.state->SADWindowSize;
		//			sgbm_13.SADWindowSize = sgbm_12.SADWindowSize = sgbm_12_2.SADWindowSize = bm.state->SADWindowSize;
		//
		//		}
		//
		//		sgbm_13.fullDP = sgbm_12.fullDP = sgbm_12_2.fullDP = false;
		//
		//		sgbm_13.P1 = 8 * sgbm_13.SADWindowSize * sgbm_13.SADWindowSize * SGBM_SmoothFactor_;
		//		sgbm_13.P2 = 32 * sgbm_13.SADWindowSize * sgbm_13.SADWindowSize * SGBM_SmoothFactor_;
		//
		//		sgbm_12.P1 = 8 * sgbm_12.SADWindowSize * sgbm_12.SADWindowSize * SGBM_SmoothFactor_;
		//		sgbm_12.P2 = 32 * sgbm_12.SADWindowSize * sgbm_12.SADWindowSize * SGBM_SmoothFactor_;
		//
		//		sgbm_12_2.P1 = 8 * sgbm_12_2.SADWindowSize * sgbm_12_2.SADWindowSize * SGBM_SmoothFactor_;
		//		sgbm_12_2.P2 = 32 * sgbm_12_2.SADWindowSize * sgbm_12_2.SADWindowSize * SGBM_SmoothFactor_;
		//
		//
		//		sgbm_13.speckleWindowSize = sgbm_12.speckleWindowSize = bm.state->speckleWindowSize;
		//		sgbm_12_2.speckleWindowSize = bm.state->speckleWindowSize / 4;
		//
		//		bm_13.state->speckleWindowSize = bm_12.state->speckleWindowSize  = bm.state->speckleWindowSize;
		//		bm_12_2.state->speckleWindowSize  = bm.state->speckleWindowSize / 4;
		//
		//		sgbm_13.speckleRange = sgbm_12.speckleRange = sgbm_12_2.speckleRange = bm.state->speckleRange * 16;
		//
		//		bm_12.state->speckleRange = bm.state->speckleRange ;
		//		bm_13.state->speckleRange = bm.state->speckleRange ;
		//		bm_12_2.state->speckleRange = bm.state->speckleRange ;
		//
		//		sgbm_13.uniquenessRatio = sgbm_12.uniquenessRatio = sgbm_12_2.uniquenessRatio  = bm.state->uniquenessRatio;
		//		bm_13.state->uniquenessRatio = bm_12.state->uniquenessRatio = bm_12_2.state->uniquenessRatio = bm.state->uniquenessRatio;
		//
		//		sgbm_13.disp12MaxDiff = sgbm_12.disp12MaxDiff = sgbm_12_2.disp12MaxDiff  = bm.state->disp12MaxDiff;
		//		bm_13.state->disp12MaxDiff = bm_12.state->disp12MaxDiff = bm_12_2.state->disp12MaxDiff = bm.state->disp12MaxDiff;
		//
		//		bm_13.state->textureThreshold = _textureThreshold * bm_13.state->SADWindowSize * bm_13.state->SADWindowSize;
		//		bm_12.state->textureThreshold = _textureThreshold * bm_12.state->SADWindowSize * bm_12.state->SADWindowSize;
		//		bm_12_2.state->textureThreshold = _textureThreshold * bm_12_2.state->SADWindowSize * bm_12_2.state->SADWindowSize;
		//
		cerr << "3 Cams not available with OpenCV 3. Contact your developpper\n";
	}
	return 0;
}





int CimgStereo::rectifyOnly( const cimg_library::CImg<unsigned char> & left,   const cimg_library::CImg<unsigned char> & right)
{
	int r = 0;
	inputImageSize.width = left.width();
	inputImageSize.height = left.height();
	if(_nf == 0) {
		r = start();
		if(r)
			return r;
		_nf = 1;
		checkLR_Layout(left,right);
		_nf = 0;
	}
	_nf ++;

	if ( !left.is_sameXYZ(right) )
	{
		std::cerr << "Left and Right Images sizes mismatch\n";
		return -1;
	}
	/*
	if (left.width() != imageSize.width)
	{
		std::cerr << "Image width != calibration\n";
		return -1;
	}

	if (left.height() != imageSize.height)
	{
		std::cerr << "Image height != calibration\n";
		return -1;
	}
	 */
	if(left.depth() > 1)
	{
		std::cerr << "Only 2D input images allowed\n";
		return -1;
	}

	left_input = left.get_shared();
	if(left.spectrum() > 1)
		left_gray = left.get_resize(-100,-100,-100,1,2);
	else
		left_gray = left.get_shared();

	if(right.spectrum() > 1)
		right_gray = right.get_resize(-100,-100,-100,1,2);
	else
		right_gray = right.get_shared();

	//img1 and img2 are graylevel opencv input images

	cv::Mat img1 ( left_gray.height(), left_gray.width(), CV_8U, left_gray.data(), left_gray.width() );
	cv::Mat img2 ( right_gray.height(), right_gray.width(), CV_8U, right_gray.data(), right_gray.width() );

	remap(img1, img1r, map11, map12,cv::INTER_LINEAR);
	remap(img2, img2r, map21, map22,cv::INTER_LINEAR);

	return 0;
}

int CimgStereo::rectifyOnly( const cimg_library::CImg<unsigned char> & left,   const cimg_library::CImg<unsigned char> & center, const cimg_library::CImg<unsigned char> & right)
{
	left.display("Left Input");
	int r = 0;
	inputImageSize.width = left.width();
	inputImageSize.height = left.height();
	if(_nf == 0)
		r = start();
	if(r)
		return r;

	_nf ++;

	if ( !left.is_sameXYZ(right) )
	{
		std::cerr << "Left and Right Images sizes mismatch\n";
		return -1;
	}
	if ( !left.is_sameXYZ(center) )
	{
		std::cerr << "Left and center Images sizes mismatch\n";
		return -1;
	}

	if(left.depth() > 1)
	{
		std::cerr << "Only 2D input images allowed\n";
		return -1;
	}

	left_input = left.get_shared();
	left_input.display("Left Input");
	if(left.spectrum() > 1)
		left_gray = left.get_resize(-100,-100,-100,1,2);
	else
		left_gray = left.get_shared();

	if(right.spectrum() > 1)
		right_gray = right.get_resize(-100,-100,-100,1,2);
	else
		right_gray = right.get_shared();

	if(center.spectrum() > 1)
		center_gray = center.get_resize(-100,-100,-100,1,2);
	else
		center_gray = center.get_shared();

	//img1 and img2 are graylevel opencv input images

	cv::Mat img1 ( left_gray.height(), left_gray.width(), CV_8U, left_gray.data(), left_gray.width() );
	cv::Mat img2 ( center_gray.height(), center_gray.width(), CV_8U, center_gray.data(), center_gray.width() );
	cv::Mat img3 ( right_gray.height(), right_gray.width(), CV_8U, right_gray.data(), right_gray.width() );

	remap(img1, imgleft_r, map11, map12,INTER_LINEAR);
	remap(img2, imgcenter_r, map21, map22,INTER_LINEAR);
	remap(img3, imgright_r, map31, map32,INTER_LINEAR);

	return 0;
}







int CimgStereo::rectify(  const cimg_library::CImg<unsigned char> & left,   const cimg_library::CImg<unsigned char> & right, cimg_library::CImg<unsigned char> & rectifiedLeft,  cimg_library::CImg<unsigned char> & rectifiedRight)
{
	int r = rectifyOnly(left, right);
	if (r)
		return r;
	rectifiedLeft.assign( img1r.data, imageSize_rect.width, imageSize_rect.height, 1, 1,true);//shared
	rectifiedRight.assign(img2r.data, imageSize_rect.width, imageSize_rect.height, 1, 1,true);
	return r;
}







int CimgStereo::disparity()
{

	if (layoutCenter_ < 0 ) { //2 cams
		if (decimate == 1 || decimate == 0){

			if (method_ == METHOD_BM) {
				if (vertBlur_ >0) {
					CImg<unsigned char> c_left((unsigned char *)img1r.data, img1r.cols, img1r.rows, 1 , 1, true);

					CImg<unsigned char> c_right((unsigned char *)img2r.data, img2r.cols, img2r.rows, 1 , 1, true);
					CImg<unsigned char> leftBlurred  = c_left.get_blur(0.0,vertBlur_, 0.0);
					CImg<unsigned char> rightBlurred  = c_right.get_blur(0.0,vertBlur_,0.0);
					memcpy(img1r.data, leftBlurred.data(), leftBlurred.size());
					memcpy(img2r.data, rightBlurred.data(), rightBlurred.size());

				}

				Mat t;
				bm->compute(img1r, img2r, t);
				t.convertTo(disp,CV_32FC1);
				disp = disp / 16.0;
			}
			else if (method_ == METHOD_SGBM){
				cv::Mat disp_short; //Used in sgbm
				sgbm->compute(img1r, img2r, disp_short);
				disp_short.convertTo(disp, CV_32FC1);
				disp = disp / 16.0;
				//dispshort2dispfloat(disp_short, disp);
			}
			//			else if (method_ == METHOD_OFLOW){
			//				double pyrScale = 0.5; //size reduction factor between pyramid layers
			//				int polyN = 5; //or 7
			//				int iterations = 2;
			//
			//				//Values from opencv documentation
			//				double polySigma;
			//				if( polyN == 5)
			//					polySigma = 1.1;
			//				else if(polyN == 7)
			//					polySigma = 1.5;
			//
			//				int levels = 5;
			//				int flags = 0;
			//				Mat flowmat;
			//				Mat rflowmat;
			//				int winsize = SADWindowSize();
			//				calcOpticalFlowFarneback(img1r, img2r, flowmat, pyrScale, levels, winsize,  iterations,  polyN,  polySigma, flags);
			//				calcOpticalFlowFarneback(img2r, img1r, rflowmat, pyrScale, levels, winsize,  iterations,  polyN,  polySigma, flags);
			//				CImg<float> c_flow( (float  *)flowmat.data, 2, flowmat.cols, flowmat.rows, 1, true);
			//				CImg<float> c_rflow( (float  *)rflowmat.data, 2, rflowmat.cols, rflowmat.rows, 1, true);
			//
			//				cv::Size S1;
			//				S1 = img1r.size();
			//
			//				disp.create(S1, CV_32F);
			//
			//				CImg<float> c_disp( (float  *)disp.data, disp.cols, disp.rows, 1, 1, true);
			//
			//				cimg_forXY(c_disp, x,y) {
			//					int  endx = 0.5 + x  + c_flow(0,x,y);
			//					int  endy = 0.5 + y  + c_flow(1,x,y);
			//
			//					if (endx < 0 || endy < 0 || endx >= img1r.cols || endy >= img1r.rows ) {
			//						c_disp(x,y) = NAN;
			//						continue;
			//					}
			//
			//					float xx = endx + c_rflow(0,endx,endy);
			//					float yy = endy + c_rflow(1,endx,endy);
			//					float dx = xx-x;
			//					float dy = yy-y;
			//					if (dx*dx+dy*dy > 1) {
			//						c_disp(x,y) = NAN;
			//						continue;
			//					}
			//					c_disp(x,y) = -c_flow(0,x,y);
			//					if (c_disp(x,y) < 0.0)
			//						c_disp(x,y) = NAN;
			//
			//				}
			//
			//			}



		}
		if(1==decimate)
			disparity_type.assign(disp.cols, disp.rows,1,1).fill(CAM2_DECIMATE_1);

		if( decimate == 2 || decimate == 0){
			disparityDecimate2();
		}

		if(decimate == 0) // Multiscale
		{
			disparity_type.assign(disp.cols, disp.rows,1,1);
			mergeMultiscale();
		}

		if(decimate == 2){
			disp = disp2; //Sin copia f��sica. Depth & xyz are computed from disp;
			disparity_type.assign(disp.cols, disp.rows,1,1).fill(CAM2_DECIMATE_2);
		}
	}
	else { //3 cams
		cerr << "3 cams not supported in opencv3 yet.\n";
		//		cv::Mat disp12;
		//		cv::Mat disp13;
		//		cv::Mat disp12_2;
		//
		//		//Decimate left and center
		//		CImg<unsigned char> c_left((unsigned char *)imgleft_r.data, imgleft_r.cols, imgleft_r.rows, 1 , 1, true);
		//		CImg<unsigned char> c_center((unsigned char *)imgcenter_r.data, imgcenter_r.cols, imgcenter_r.rows, 1 , 1, true);
		//
		//		CImg<unsigned char> left_decimated = c_left.get_resize(-50,-50,-100,-100,2);
		//		CImg<unsigned char> center_decimated = c_center.get_resize(-50,-50,-100,-100,2);
		//
		//		cv::Mat imgleftrd ( left_decimated.height() , left_decimated.width() , CV_8U, left_decimated.data(), left_decimated.width() );
		//		cv::Mat imgcenterd( center_decimated.height() , center_decimated.width() , CV_8U, center_decimated.data(), center_decimated.width() );
		//
		//		if (method_ == METHOD_BM) {
		//			Mat t;
		//			bm_12->compute(imgleft_r, imgcenter_r, t);
		//			t.convertTo(disp12,CV_32F);
		//			bm_13(imgleft_r, imgright_r, disp13, CV_32F);
		//			bm_12_2(imgleftrd, imgcenterd, disp12_2, CV_32F);
		//
		//		}
		//		else if (method_ == METHOD_SGBM){
		//			cv::Mat disp_short; //Used in sgbm
		//
		//
		//			sgbm_12(imgleft_r, imgcenter_r, disp_short);
		//			dispshort2dispfloat(disp_short, disp12);
		//
		//			sgbm_13(imgleft_r, imgright_r, disp_short);
		//			dispshort2dispfloat(disp_short, disp13);
		//
		//			sgbm_12_2(imgleftrd, imgcenterd, disp_short);
		//			dispshort2dispfloat(disp_short, disp12_2);
		//		}
		//		else
		//		{
		//			std::cerr << "Invalid 3 cam correspondence method\n";
		//			exit(0);
		//		}
		//		disparity_type.assign(disp12.cols, disp12.rows,1,1);
		//		merge3camsDisparities( disp12, disp13, disp12_2);
		//		disp = disp12 ;; //No real pixel copy
		//		minDisparity = minDisparity12;
	}
	return 0;
}








int CimgStereo::disparityDecimate2()
{
	CImg<unsigned char> c_left((unsigned char *)img1r.data, img1r.cols, img1r.rows, 1 , 1, true);
	CImg<unsigned char> c_right((unsigned char *)img2r.data, img2r.cols, img2r.rows, 1 , 1, true);

	CImg<unsigned char> left_decimated = c_left.get_resize(-50,-50,-100,-100,2);
	CImg<unsigned char> right_decimated = c_right.get_resize(-50,-50,-100,-100,2);

	cv::Mat img1rd ( left_decimated.height() , left_decimated.width() , CV_8U, left_decimated.data(), left_decimated.width() );
	cv::Mat img2rd( right_decimated.height() , right_decimated.width() , CV_8U, right_decimated.data(), right_decimated.width() );
	cv::Mat dispd;
	Mat t;
	if(method_ == METHOD_BM)
		bm2->compute( img1rd,  img2rd, t);
	else if (method_ == METHOD_SGBM)
		sgbm2->compute(img1rd, img2rd, t);

	t.convertTo(dispd,CV_32F);
	t= t / 16.0;

	CImg<float> disp_decimated((float *) dispd.data, dispd.cols, dispd.rows, 1, 1,true);

	disp2.create(img1r.rows , img1r.cols, CV_32F);
	CImg<float> c_disp((float *) disp2.data, disp2.cols, disp2.rows, 1, 1,true);
	//Interpolate disparity
	int width = c_disp.width();
	int diag = width + 1;
	float *disp_ptr = c_disp.data();
	float min_disparity = bm2->getMinDisparity();//.state->minDisparity;
	cimg_forXY(disp_decimated,x,y)
	{
		int x2 = 2 * x;
		int y2 = 2 * y;
		float d = disp_decimated(x,y);
		int off = x2 + y2 * width;
		float *pf = disp_ptr + off;
		if ( d >= min_disparity)
		{
			float d2 = 2.0 * d;
			*pf = pf[1] = pf[width] = pf[diag] = d2;
		}
		else
			*pf = pf[1] = pf[width] = pf[diag] = bm->getMinDisparity()-1;//.state->minDisparity - 1;
	}

	return 0;
}



int CimgStereo::disparity( const cimg_library::CImg<unsigned char> & left,  const cimg_library::CImg<unsigned char> & right)
{
	int r = rectifyOnly(left, right);
	if (r)
		return r;

	r = disparity();
	return r;
}

int CimgStereo::disparity( const cimg_library::CImg<unsigned char> & left,  const cimg_library::CImg<unsigned char> & center, const cimg_library::CImg<unsigned char> & right)
{
	int r = rectifyOnly(left, center, right);
	if (r)
		return r;

	r = disparity();
	return r;
}

//int CimgStereo::disparity( const cimg_library::CImg<unsigned char> & left,   const cimg_library::CImg<unsigned char> & right, cimg_library::CImg<float> & _disparity)
//{
//	int r = rectifyOnly(left, right);
//	if (r)
//		return r;
//
//	r = disparity();
//	_disparity.assign((float *) disp.data, disp.cols, disp.rows, 1, 1,true);
//	return r;
//}



int CimgStereo::disparity(cimg_library::CImg<float> & _disparity)
{
	int r = disparity();
	if (r)
		return r;

	_disparity.assign((float *) disp.data, disp.cols, disp.rows, 1, 1,true);
	return r;
}

/*
int CimgStereo::depth()
{

	CImg<float> c_disparity((float *) disp.data, disp.cols, disp.rows, 1, 1,true);

	c_depth3D.assign(c_disparity.width() , c_disparity.height(), 1, 3);
	CImg<float> Zp = c_depth3D.get_shared_channel(2);

	CImg<double> C_Q((double *)Q.data, Q.cols, Q.rows, 1, 1, true);
	double aw, bw, cw, dw;
	double az, bz, cz, dz;

	aw = C_Q(0,3);
	bw = C_Q(1,3);
	cw = C_Q(2,3);
	dw = C_Q(3,3);

	az = C_Q(0,2);
	bz = C_Q(1,2);
	cz = C_Q(2,2);
	dz = C_Q(3,2);

	cw *= 1000.0; //Output in metres
	cimg_forXY(c_disparity, x, y)
	{
		int offset = x + y * c_disparity.width();
		double dd = c_disparity[offset];

		double W =  cw * dd ;
		double Z =  dz;

		if(W !=0 || dd < minDisparity)
		{
			double d = Z / W;
			if (d < maximumDistance && d > minimumDistance)
				Zp[offset] = Z / W;
			else
				Zp[offset] = NAN;
		}
		else
		{
			Zp[offset] = NAN;
		}
	}

	return 0;
}
 */
int CimgStereo::reproject3D()
{

	CImg<float> c_disparity((float *) disp.data, disp.cols, disp.rows, 1, 1,true);

	c_depth3D.assign(c_disparity.width() , c_disparity.height(), 1, 3);

	CImg<double> C_Q( (double *)Q.data, Q.cols, Q.rows, 1, 1, true);

	double aw, bw, cw, dw;
	double az, bz, cz, dz;
	double ay, by, cy, dy;
	double ax, bx, cx, dx;

	aw = C_Q(0,3);
	bw = C_Q(1,3);
	cw = C_Q(2,3);
	dw = C_Q(3,3);

	az = C_Q(0,2);
	bz = C_Q(1,2);
	cz = C_Q(2,2);
	dz = C_Q(3,2);

	ay = C_Q(0,1);
	by = C_Q(1,1);
	cy = C_Q(2,1);
	dy = C_Q(3,1);

	ax = C_Q(0,0);
	bx = C_Q(1,0);
	cx = C_Q(2,0);
	dx = C_Q(3,0);


	CImg<float> Xp = c_depth3D.get_shared_channel(0);
	CImg<float> Yp = c_depth3D.get_shared_channel(1);
	CImg<float> Zp = c_depth3D.get_shared_channel(2);



	cw *= 1000.0;  //Output in metres
	//	C_Q.display("Q");

	cimg_forXY(c_disparity, x, y)
	{
		int offset = x + y * c_disparity.width();
		double dd = c_disparity[offset];
		if( dd >= minDisparity) {
			double W =  cw * dd ;
			double Z =  dz;
			//		double Y =  by * y + dy;
			//		double X = ax * x  + dx;

			//double Y = y_inv - y;
			double Y = - ( y + dy);
			//double Y = y + dy;
			double X =  x  + dx; //by ==1

			if(W !=0 )
			{
				double d = Z / W;
				if (fabs(d) < maximumDistance && fabs(d) > minimumDistance)
				{
					Xp[offset] =  X / W;
					Yp[offset] =  Y / W;
					Zp[offset] = d;
				}
				else
				{
					Xp[offset]= Yp[offset] = Zp[offset] = NAN;
					disparity_type[offset] = UNKNOWN;
				}
			}
			else
			{
				Xp[offset]= Yp[offset] = Zp[offset] = NAN;
				disparity_type[offset] = UNKNOWN;
			}
		}
		else
		{
			Xp[offset]= Yp[offset] = Zp[offset] = NAN;
			disparity_type[offset] = UNKNOWN;
		}

	}

	return 0;
}

/*

int CimgStereo::depth(cimg_library::CImg<float> & _depth)
{
	depth();
	_depth = c_depth3D.get_shared_plane(2);
	return 0;
}



int CimgStereo::depth( cimg_library::CImg<unsigned char> & left,  cimg_library::CImg<unsigned char> & right, cimg_library::CImg<float> & _depth)
{
	int r = rectifyOnly(left, right);
	if (r)
		return r;

	r = disparity();
	if (r)
		return r;

	r=reproject3D();
	r=depth(_depth);
	return r;
}
 */

void CimgStereo::setBMDefaults()
{
	//bm.state->preFilterType = CV_STEREO_BM_XSOBEL; // 0 for now Sobel
	//bm.state->preFilterSize=11; //No use with Sobel
	//bm.state->preFilterCap=33;

	bm->setPreFilterType(cv::StereoBM::PREFILTER_XSOBEL);
	bm->setPreFilterSize(11);//No use with Sobel
	bm->setPreFilterCap(33);

	//bm.state->SADWindowSize= DEFAULT_SAD_WINSIZE;
	//bm.state->minDisparity = 0;
	//bm.state->numberOfDisparities= 192;//96;//192;//80; //192;    //tiene que ser divisible por 16
	bm->setBlockSize(DEFAULT_SAD_WINSIZE);
	bm->setMinDisparity(0);
	bm->setNumDisparities(192);//tiene que ser divisible por 16

	_textureThreshold = DEFAULT_TEXTURE_THRESHOLD;  //original 15
	//bm.state->uniquenessRatio = DEFAULT_UNIQUENESS_RATIO;    //original 15
	bm->setUniquenessRatio(DEFAULT_UNIQUENESS_RATIO);
	//bm.state->speckleWindowSize = DEFAULT_SPECKLE_WINSIZE; // the maximum area of speckles to remove
	bm->setSpeckleWindowSize(DEFAULT_SPECKLE_WINSIZE);
	// (set to 0 to disable speckle filtering)

	//bm.state->speckleRange = 32;// acceptable range of disparity variation in each connected component
	bm->setSpeckleRange(32);
	//bm.state->disp12MaxDiff = -1; // maximum allowed disparity difference in the left-right check
	bm->setDisp12MaxDiff(-1);


	//bm2.state->preFilterType = CV_STEREO_BM_XSOBEL; // 0 for now Sobel
	//bm2.state->preFilterSize=11; //No use with Sobel
	//bm2.state->preFilterCap=33;
	bm2->setPreFilterCap(cv::StereoBM::PREFILTER_XSOBEL);
	bm2->setPreFilterSize(11);//No use with Sobel
	bm2->setPreFilterCap(33);




	//sgbm.preFilterCap=33;
	sgbm->setPreFilterCap(33);

	//sgbm.SADWindowSize= DEFAULT_SAD_WINSIZE;
	//sgbm.minDisparity = 0;
	//sgbm.numberOfDisparities= 192;//96;//192;//80; //192;    //tiene que ser divisible por 16
	sgbm->setBlockSize(DEFAULT_SAD_WINSIZE);
	sgbm->setMinDisparity(0);
	sgbm->setNumDisparities(192);//tiene que ser divisible por 16

	//sgbm.uniquenessRatio = DEFAULT_UNIQUENESS_RATIO;    //original 15
	//sgbm.speckleWindowSize = DEFAULT_SPECKLE_WINSIZE; // the maximum area of speckles to remove
	sgbm->setUniquenessRatio(DEFAULT_UNIQUENESS_RATIO);
	sgbm->setSpeckleWindowSize(DEFAULT_SPECKLE_WINSIZE);


	// (set to 0 to disable speckle filtering)
	//	sgbm.speckleRange = 32;// acceptable range of disparity variation in each connected component
	//	sgbm.disp12MaxDiff = -1; // maximum allowed disparity difference in the left-right check
	sgbm->setSpeckleRange(32);
	sgbm->setDisp12MaxDiff(-1);

	//sgbm.fullDP = false;
	sgbm->setMode(StereoSGBM::MODE_SGBM);
	sgbm->setP1(8 * sgbm->getBlockSize() *  sgbm->getBlockSize());
	sgbm->setP2(32 * sgbm->getBlockSize() *  sgbm->getBlockSize());
	//	sgbm.P1 = 8 * sgbm.SADWindowSize * sgbm.SADWindowSize;
	//	sgbm.P2 = 32 * sgbm.SADWindowSize * sgbm.SADWindowSize;




	//sgbm2.SADWindowSize= DEFAULT_SAD_WINSIZE;
	//sgbm2.minDisparity = 0;
	//sgbm.numberOfDisparities= 192;//96;//192;//80; //192;    //tiene que ser divisible por 16
	//sgbm2.preFilterCap=33;
	//sgbm2.fullDP = false;
	//sgbm2.uniquenessRatio = DEFAULT_UNIQUENESS_RATIO;    //original 15
	//sgbm2.speckleWindowSize = DEFAULT_SPECKLE_WINSIZE; // the maximum area of speckles to remove

	//	sgbm2.speckleRange = 32;// acceptable range of disparity variation in each connected component
	//	sgbm2.disp12MaxDiff = -1; // maximum allowed disparity difference in the left-right check
	//	sgbm2.P1 = 8 * sgbm2.SADWindowSize * sgbm2.SADWindowSize;
	//	sgbm2.P2 = 32 * sgbm2.SADWindowSize * sgbm2.SADWindowSize;

	sgbm2->setBlockSize(DEFAULT_SAD_WINSIZE);
	sgbm2->setMinDisparity(0);
	sgbm2->setNumDisparities(192);//tiene que ser divisible por 16
	sgbm2->setPreFilterCap(33);
	sgbm2->setMode(StereoSGBM::MODE_SGBM);
	sgbm2->setUniquenessRatio(DEFAULT_UNIQUENESS_RATIO);
	sgbm2->setSpeckleWindowSize(DEFAULT_SPECKLE_WINSIZE);
	sgbm2->setSpeckleRange(32);
	sgbm2->setDisp12MaxDiff(-1);
	sgbm2->setP1(8 * sgbm->getBlockSize() *  sgbm->getBlockSize());
	sgbm2->setP2(32 * sgbm->getBlockSize() *  sgbm->getBlockSize());



}









cimg_library::CImg<float> &  CimgStereo::xyz()
{
	reproject3D();
	if( rotationMatrix_.size() == 9)
		rotateCoordinates();
	return c_depth3D;
}




cimg_library::CImg<float> &  CimgStereo::xyz(const cimg_library::CImg<unsigned char> & left,  const cimg_library::CImg<unsigned char> & right)
{
	disparity(left, right);

	return xyz();
}


cimg_library::CImg<float> &  CimgStereo::xyz(const cimg_library::CImg<unsigned char> & left,
		const cimg_library::CImg<unsigned char> & center, const cimg_library::CImg<unsigned char> & right)
{
	disparity(left, center, right);

	return xyz();
}


void CimgStereo::rotateCoordinates()
{
	CImg<float> X = c_depth3D.get_shared_slice(0);
	CImg<float> Y = c_depth3D.get_shared_slice(1);
	CImg<float> Z = c_depth3D.get_shared_slice(2);

	float axx = rotationMatrix_(0,0);
	float ayx = rotationMatrix_(1,0);
	float azx = rotationMatrix_(2,0);

	float axy = rotationMatrix_(0,1);
	float ayy = rotationMatrix_(1,1);
	float azy = rotationMatrix_(2,1);

	float axz = rotationMatrix_(0,2);
	float ayz = rotationMatrix_(1,2);
	float azz = rotationMatrix_(2,2);

	double camHeight_metres = camHeight_ /1000.0;
	cimg_foroff(X,o)
	{
		float x = X[o];
		float y = Y[o];
		float z = Z[o];
		if( ! std::isnan(z))
		{
			X[o] = axx * x + ayx * y + azx * z;
			Y[o] = axy * x + ayy * y + azy * z;
			Z[o] = camHeight_metres + axz * x + ayz * y + azz * z;
		}
		else
			X[o] = Y[o] = Z[o] = NAN;
	}

}



void CimgStereo::Range()
{

	c_Range.assign(disp.cols, disp.rows, 1, 1);
	CImg<float> X = c_depth3D.get_shared_slice(0);
	CImg<float> Y = c_depth3D.get_shared_slice(1);
	CImg<float> Z = c_depth3D.get_shared_slice(2);

	cimg_foroff(X,o)
	{
		float x = X[o];
		float y = Y[o];
		float z = Z[o];
		if( ! std::isnan(z))
		{
			c_Range[o] = sqrt(x * x + y * y + z * z);
		}
		else
			c_Range[o] = NAN;
	}

}

void CimgStereo::Range(cimg_library::CImg<float> & _range)
{
	Range();
	_range = c_Range.get_shared();
}


void CimgStereo::rangeAlarm(cimg_library::CImg<unsigned char> & range_alarm, float thres)
{
	if(!range_alarm.is_sameXY(c_Range) || range_alarm.spectrum() != 3)
	{
		std::cerr << "Error en rangeAlarm\n";
		return;
	}

	cimg_forX(range_alarm, x)
	cimg_forY(range_alarm, y){
		float ff = c_Range(x,y);
		if ( isnan( ff ) || ff > thres)
			continue;
		range_alarm(x,y,0) = 255;
		range_alarm(x,y,1) >>= 1;//>> 3;  // shift 2 bits: divide by 4 G,B. To change white pixels by red
		range_alarm(x,y,2) >>= 1;// >> 3;
	}
}


void CimgStereo::rectifyImage(cimg_library::CImg<unsigned char> & original,  cimg_library::CImg<unsigned char> & rectified)
{
	int ncolors = original.spectrum();
	rectified.assign(imageSize_rect.width, imageSize_rect.height, 1, ncolors);

	for (int c = 0; c< ncolors; c++)
	{
		CImg<unsigned char> component = original.get_shared_channel(c);
		CImg<unsigned char> component_rect = rectified.get_shared_channel(c);
		cv::Mat img1 ( component.height(), component.width(), CV_8U, component.data(), component.width() );
		cv::Mat img1r;
		remap(img1, img1r, map11, map12,INTER_LINEAR);
		unsigned char * src = img1r.data;
		int npixels = component_rect.width() * component_rect.height();
		unsigned char * dst = component_rect.data();
		memcpy( dst, src, npixels);
	}
}




void CimgStereo::mergeMultiscale()
{
	CImg<float> c_disp((float *) disp.data, disp.cols, disp.rows, 1, 1,true);
	CImg<float> c_disp2((float *) disp2.data, disp2.cols, disp2.rows, 1, 1,true);

	if( !c_disp.is_sameXY(c_disp2) )
	{
		std::cerr<<"Error in multiscale\n";
		c_disp.print("disp");
		c_disp2.print("disp2");
		exit(0);
	}

	disparity_type.fill(UNKNOWN);
	if( multiscaleMode_ == MODE_CLASSIC) {
		for(int o=0; o < c_disp.size(); o++ )

		{
			float d1 = c_disp[o];
			float d2 = c_disp2[o];



			if( d1 < minDisparity1 && d2 < minDisparity1)
			{
				c_disp[o] = minDisparity1 - 1.0;
			}
			else if( d1 < minDisparity1 && d2 >= minDisparity1)
			{
				c_disp[o] = c_disp2[o];
				disparity_type[o] = CAM2_DECIMATE_2;
			}
			else
				//En el resto de casos tomo c_disp que es el que se ha calculado con m��s resoluci��n espacial.
			{
				disparity_type[o] = CAM2_DECIMATE_1;
			}

		}

	}

	if( multiscaleMode_ == MODE_ROBUST) {

		int sad_ws = SADWindowSize();
		int sad_ws_2 = 2* sad_ws;
		int xlimit = c_disp.width() - sad_ws_2;

		//Upper portion only full resolution data available
		for(int y= 0; y < sad_ws_2; y++ )
			for(int x=0; x < c_disp.width(); x++ )
			{
				int o = c_disp.offset(x,y);
				float d1 = c_disp[o];

				//If any is invalid ---> pixel invalid
				if( d1 >= minDisparity1 )
					disparity_type[o] = CAM2_DECIMATE_1;
			}
		//Central section
		for(int y= sad_ws_2; y < c_disp.height() - sad_ws_2 ; y++ )
			for(int x=0; x < c_disp.width(); x++ )
			{
				int o = c_disp.offset(x,y);
				float d1 = c_disp[o];
				float d2 = c_disp2[o];

				if ( x < sad_ws_2 || x > xlimit) { //No data available from lowres
					if( d1 >= minDisparity1 )
						disparity_type[o] = CAM2_DECIMATE_1;
					continue;
				}
				//If any is invalid ---> pixel invalid
				if( d1 < minDisparity1 || d2 < minDisparity1)
					c_disp[o] = minDisparity1 - 1.0;
				//If disparities are very different		--> pixel invalid
				else if( fabs( d1 -d2)  > 1.0 )
					c_disp[o] = minDisparity1 - 1.0;
				// Else leave the disparity estimated with more resolution
				else
					disparity_type[o] = CAM2_DECIMATE_1;
			}

		//bottom section
		for(int y= c_disp.height() - sad_ws_2; y < c_disp.height(); y++ )
			for(int x=0; x < c_disp.width(); x++ )
			{
				int o = c_disp.offset(x,y);
				float d1 = c_disp[o];
				//If any is invalid ---> pixel invalid
				if( d1 >= minDisparity1 )
					disparity_type[o] = CAM2_DECIMATE_1;
			}
	}
}



void  CimgStereo::Zplane2XYZ(float zplane_metres, const cimg_library::CImg<unsigned int> & mask, cimg_library::CImg<float> & xyz)
{
	if(rotationMatrix_.size() == 0)
	{
		xyz.assign();
		return;
	}

	rectifyOnly(mask, mask);
	CImg<unsigned char> maskr;
	getLeftRectifiedGray(maskr);
	xyz.assign( maskr.width(), maskr.height(), 3); //In room coordinates

	CImg<double> C_Q( (double *)Q.data, Q.cols, Q.rows, 1, 1, true);
	double centerx = - C_Q(3,0);
	double centery = - C_Q(3,1);
	double alt_1_centery = maskr.height() - 1 - centery;
	double focalDistance = rectifiedCamFocal;



	float axx = rotationMatrix_(0,0);
	float ayx = rotationMatrix_(1,0);
	float azx = rotationMatrix_(2,0);

	float axy = rotationMatrix_(0,1);
	float ayy = rotationMatrix_(1,1);
	float azy = rotationMatrix_(2,1);

	float axz = rotationMatrix_(0,2);
	float ayz = rotationMatrix_(1,2);
	float azz = rotationMatrix_(2,2);

	CImg<float> xx = xyz.get_shared_slice(0);
	CImg<float> yy = xyz.get_shared_slice(1);
	CImg<float> zz = xyz.get_shared_slice(2);

	double camHeightMetres = camHeight_ / 1000.0;

	for(unsigned int x= 0; x< maskr.width(); x ++)
	{
		double xcam = x - centerx;
		for(unsigned int y= 0; y< maskr.height(); y ++ )
		{
			int off = y * maskr.width() + x;
			if(mask[off] < 128 )
			{
				xx[off] = yy[off] = zz[off] = NAN;
				continue;
			}
			double ycam = alt_1_centery - y;
			double zcam = focalDistance;
			//xcam ycam zcam: coordinates of pixel in camera centered and aligned coordinates. Units: pixels
			double xroom = axx * xcam +  ayx * ycam    + azx * zcam;
			double yroom = axy * xcam +  ayy * ycam    + azy * zcam;
			double zroom = camHeightMetres + axz * xcam +  ayz * ycam    + azz * zcam;
			//xroom yroom zroom: coordinates of pixel in roome centered and aligned coordinates. Units: metres
			//Pcentre = (0,0,_cameraHeight): camera position coordinate in room axis.
			//line = Pcentre + lambda (Proom - Pcentre)
			//The ray passing by pixel is on the line
			double lambda = (zplane_metres - camHeightMetres) / (zroom - camHeightMetres);

			xx[off] = lambda * xroom;
			yy[off] = lambda * yroom;
			zz[off] = zplane_metres;
		}
	}

}


cimg_library::CImg<float> &  CimgStereo::composite2xyz( const cimg_library::CImg<unsigned char> & composite)
{
	CImg<unsigned char> left;
	CImg<unsigned char> right;
	CImg<unsigned char> center;

	if ( layoutCenter_ < 0) {
		splitLR( composite, left, right);
		(left,right).display("Left-Right");
		return xyz( left, right);
	}
	else {
		splitLCR( composite, left, center, right);
		return xyz( left, center, right);
	}
}

void   CimgStereo::rectifyOnly( const cimg_library::CImg<unsigned char> & composite)
{
	CImg<unsigned char> left;
	CImg<unsigned char> right;
	CImg<unsigned char> center;

	if ( layoutCenter_ < 0) {
		splitLR( composite, left, right);
		rectifyOnly( left, right);

	}
	else {
		splitLCR( composite, left, center, right);
		rectifyOnly( left, center, right);
	}
}

void CimgStereo::splitLR( const cimg_library::CImg<unsigned char> & composite , cimg_library::CImg<unsigned char> & left, cimg_library::CImg<unsigned char> & right)
{
	int width = composite.width();
	int height = composite.height();
	int width2 = width /  2;
	int height2 = height / 2;
	left.assign();
	right.assign();
	if( layoutLeft_ == COMPOSITE_DOWN)
		left = composite.get_crop( 0, height2, width - 1, height - 1 );
	if( layoutLeft_ == COMPOSITE_UP)
		left = composite.get_crop( 0, 0 , width - 1, height2 - 1 );
	if( layoutLeft_ == COMPOSITE_LEFT)
		left = composite.get_crop( 0, 0 , width2 - 1, height - 1 );
	if( layoutLeft_ == COMPOSITE_RIGHT)
		left = composite.get_crop( width2, 0 , width - 1, height - 1 );
	if( layoutLeft_ == COMPOSITE_UP_LEFT)
		left = composite.get_crop( 0, 0 , width2 - 1, height2 - 1 );
	if( layoutLeft_ == COMPOSITE_UP_RIGHT)
		left = composite.get_crop( width2, 0 , width - 1, height2 - 1 );
	if( layoutLeft_ == COMPOSITE_DOWN_LEFT)
		left = composite.get_crop( 0, height2 , width2- 1, height - 1 );
	if( layoutLeft_ == COMPOSITE_DOWN_RIGHT)
		left = composite.get_crop( width2, height2  , width - 1, height - 1 );


	if( layoutRight_ == COMPOSITE_DOWN)
		right = composite.get_crop( 0, height2, width - 1, height - 1 );
	if( layoutRight_ == COMPOSITE_UP)
		right = composite.get_crop( 0, 0 , width - 1, height2 - 1 );
	if( layoutRight_ == COMPOSITE_LEFT)
		right = composite.get_crop( 0, 0 , width2 - 1, height - 1 );
	if( layoutRight_ == COMPOSITE_RIGHT)
		right = composite.get_crop( width2, 0 , width - 1, height - 1 );
	if( layoutRight_ == COMPOSITE_UP_LEFT)
		right = composite.get_crop( 0, 0 , width2 - 1, height2 - 1 );
	if( layoutRight_ == COMPOSITE_UP_RIGHT)
		right = composite.get_crop( width2, 0 , width - 1, height2 - 1 );
	if( layoutRight_ == COMPOSITE_DOWN_LEFT)
		right = composite.get_crop( 0, height2 , width2- 1, height - 1 );
	if( layoutRight_ == COMPOSITE_DOWN_RIGHT)
		right = composite.get_crop( width2, height2  , width - 1, height - 1 );


}



void CimgStereo::splitLCR( const cimg_library::CImg<unsigned char> & composite , cimg_library::CImg<unsigned char> & left, cimg_library::CImg<unsigned char> & center, cimg_library::CImg<unsigned char> & right)
{
	int width = composite.width();
	int height = composite.height();
	int width2 = width /  2;
	int height2 = height / 2;
	left.assign();
	right.assign();
	if( layoutLeft_ == COMPOSITE_DOWN)
		left = composite.get_crop( 0, height2, width - 1, height - 1 );
	if( layoutLeft_ == COMPOSITE_UP)
		left = composite.get_crop( 0, 0 , width - 1, height2 - 1 );
	if( layoutLeft_ == COMPOSITE_LEFT)
		left = composite.get_crop( 0, 0 , width2 - 1, height - 1 );
	if( layoutLeft_ == COMPOSITE_RIGHT)
		left = composite.get_crop( width2, 0 , width - 1, height - 1 );
	if( layoutLeft_ == COMPOSITE_UP_LEFT)
		left = composite.get_crop( 0, 0 , width2 - 1, height2 - 1 );
	if( layoutLeft_ == COMPOSITE_UP_RIGHT)
		left = composite.get_crop( width2, 0 , width - 1, height2 - 1 );
	if( layoutLeft_ == COMPOSITE_DOWN_LEFT)
		left = composite.get_crop( 0, height2 , width2- 1, height - 1 );
	if( layoutLeft_ == COMPOSITE_DOWN_RIGHT)
		left = composite.get_crop( width2, height2  , width - 1, height - 1 );


	if( layoutRight_ == COMPOSITE_DOWN)
		right = composite.get_crop( 0, height2, width - 1, height - 1 );
	if( layoutRight_ == COMPOSITE_UP)
		right = composite.get_crop( 0, 0 , width - 1, height2 - 1 );
	if( layoutRight_ == COMPOSITE_LEFT)
		right = composite.get_crop( 0, 0 , width2 - 1, height - 1 );
	if( layoutRight_ == COMPOSITE_RIGHT)
		right = composite.get_crop( width2, 0 , width - 1, height - 1 );
	if( layoutRight_ == COMPOSITE_UP_LEFT)
		right = composite.get_crop( 0, 0 , width2 - 1, height2 - 1 );
	if( layoutRight_ == COMPOSITE_UP_RIGHT)
		right = composite.get_crop( width2, 0 , width - 1, height2 - 1 );
	if( layoutRight_ == COMPOSITE_DOWN_LEFT)
		right = composite.get_crop( 0, height2 , width2- 1, height - 1 );
	if( layoutRight_ == COMPOSITE_DOWN_RIGHT)
		right = composite.get_crop( width2, height2  , width - 1, height - 1 );



	if( layoutCenter_ == COMPOSITE_DOWN)
		center = composite.get_crop( 0, height2, width - 1, height - 1 );
	if( layoutCenter_ == COMPOSITE_UP)
		center = composite.get_crop( 0, 0 , width - 1, height2 - 1 );
	if( layoutCenter_ == COMPOSITE_LEFT)
		center = composite.get_crop( 0, 0 , width2 - 1, height - 1 );
	if( layoutCenter_ == COMPOSITE_RIGHT)
		center = composite.get_crop( width2, 0 , width - 1, height - 1 );
	if( layoutCenter_ == COMPOSITE_UP_LEFT)
		center = composite.get_crop( 0, 0 , width2 - 1, height2 - 1 );
	if( layoutCenter_ == COMPOSITE_UP_RIGHT)
		center = composite.get_crop( width2, 0 , width - 1, height2 - 1 );
	if( layoutCenter_ == COMPOSITE_DOWN_LEFT)
		center = composite.get_crop( 0, height2 , width2- 1, height - 1 );
	if( layoutCenter_ == COMPOSITE_DOWN_RIGHT)
		center = composite.get_crop( width2, height2  , width - 1, height - 1 );

}



int layoutDecode(const std::string & s)
{
	if ( s == "l" || s == "L")
		return COMPOSITE_LEFT;
	if ( s == "r" || s == "R")
		return COMPOSITE_RIGHT;
	if ( s == "u" || s == "U")
		return COMPOSITE_UP;
	if ( s == "d" || s == "D")
		return COMPOSITE_DOWN;
	if ( s == "ld" || s == "LD"  || s == "dl" || s == "DL")
		return COMPOSITE_DOWN_LEFT;
	if ( s == "lu" || s == "LU"  || s == "ul" || s == "UL")
		return COMPOSITE_UP_LEFT;
	if ( s == "ru" || s == "RU"  || s == "ur" || s == "UR")
		return COMPOSITE_UP_RIGHT;
	if ( s == "rd" || s == "RD"  || s == "dr" || s == "DR")
		return COMPOSITE_DOWN_RIGHT;
	//else
	return -1;

}


//void CimgStereo::dispshort2dispfloat(const cv::Mat & dsh, cv::Mat &dfl)
//{
//	if (dsh.type() != CV_16S)
//	{
//		std::cerr << "Error in dispshort2dispfloat\n";
//		exit(0);
//	}
//	dfl.create(dsh.rows, dsh.cols, CV_32F);
//	short * s = (short *) dsh.data;
//	float * f = (float *) dfl.data;
//	int np = dsh.rows * dsh.cols;
//
//	for( int k = 0; k < np; k++){
//		f[k] = float( s[k] ) / 16.0;
//	}
//
//}



void CimgStereo::dispuchar2dispfloat(const cv::Mat & dsh, cv::Mat &dfl)
{
	if (dsh.type() != CV_8U)
	{
		std::cerr << "Error in dispuchar2dispfloat\n";
		exit(0);
	}
	dfl.create(dsh.rows, dsh.cols, CV_32F);
	unsigned char * s = (unsigned char *) dsh.data;
	float * f = (float *) dfl.data;
	int np = dsh.rows * dsh.cols;

	for( int k = 0; k < np; k++){
		f[k] = float( s[k] );
	}

}

void CimgStereo::depthAsColor(cimg_library::CImg<unsigned char> & depthcolor)
{
	if (LUT.size() == 0)
		createLUT();
	CImg<float> de = xyz().get_shared_slice(2);

	//	depthcolor.assign ( de.width(), de.height(), 0, 3);
	if(!depthcolor.is_sameXY(de) || depthcolor.spectrum() != 3)
	{
		std::cerr << "Error en depthAsColor\n";
		return;
	}
	float opacity = 0.5;
	float opacity_1 = 1- opacity;
	isnan(5.0);
	cimg_forX(depthcolor, x)
	cimg_forY(depthcolor, y){
		float f = de(x,y);
		if ( isnan( f ) )
			continue;
		f = (f - minimumDistance) / (maximumDistance - minimumDistance) * 255.0;

		int ff = f;
		if(ff > 255)
		{
			if(!warning_bad_depthasColor)
			{
				std::cerr << "warning 2 en depthAsColor, (" << x <<","<< y << ")" << "level (min0,max255)=" << ff<<"\n";
				warning_bad_depthasColor=1;
			}
			ff=255;
		}
		else if (ff <0)
		{
			if(!warning_bad_depthasColor)
			{
				std::cerr << "warning 2 en depthAsColor, (" << x <<","<< y << ")" << "level (min0,max255)=" << ff<<"\n";
				warning_bad_depthasColor=1;
			}
			ff=0;
		}
		cimg_forC( depthcolor, c)
		depthcolor(x,y,0,c) = opacity * LUT(ff,0,0,c) + opacity_1 * depthcolor(x,y,0,c);

	}
}

// same as depthAsColor, but based on y-coord instead z-coord
void CimgStereo::Y_AsColor(cimg_library::CImg<unsigned char> & depthcolor)
{
	if (LUT.size() == 0)
		createLUT();
	CImg<float> de = xyz().get_shared_slice(1);

	//	depthcolor.assign ( de.width(), de.height(), 0, 3);
	if(!depthcolor.is_sameXY(de) || depthcolor.spectrum() != 3)
	{
		std::cerr << "Error en Y_AsColor\n";
		return;
	}
	float opacity = 0.5;
	float opacity_1 = 1- opacity;
	isnan(5.0);
	cimg_forX(depthcolor, x)
	cimg_forY(depthcolor, y){
		if(156==x && 54==y){
		}
		float f = de(x,y);
		if ( isnan( f ) )
			continue;
		f = (f - minimumDistance) / (maximumDistance - minimumDistance) * 255.0;

		int ff = f;
		if(ff > 255)
		{
			if(!warning_bad_depthasColor)
			{
				std::cerr << "warning 2 en Y_AsColor, (" << x <<","<< y << ")" << "level (min0,max255)=" << ff<<"\n";
				warning_bad_depthasColor=1;
			}
			ff=255;
		}
		else if (ff <0)
		{
			if(!warning_bad_depthasColor)
			{
				std::cerr << "warning 2 en Y_AsColor, (" << x <<","<< y << ")" << "level (min0,max255)=" << ff<<"\n";
				warning_bad_depthasColor=1;
			}
			ff=0;
		}

		cimg_forC( depthcolor, c)
		depthcolor(x,y,0,c) = opacity * LUT(ff,0,0,c) + opacity_1 * depthcolor(x,y,0,c);

	}
}

void CimgStereo::getXYZ(cimg_library::CImg<float> & XYZ)
{
	XYZ=c_depth3D;//.assign( c_depth3D.,  c_depth3D.width(), c_depth3D.height(), 3, 1,false);
}

void CimgStereo::createLUT()
{
	LUT.assign(255,1,1,3);
	int cc;
	for (cc=0; cc<64; cc++) {    // de rojo a amarillo
		LUT(cc,0,0,0) = 255;
		LUT(cc,0,0,1) = 4 *cc;
		LUT(cc,0,0,2) = 0;
	}
	for (cc=0; cc<64; cc++) {    // de amarillo a verde
		int cc1 = cc + 64;
		LUT(cc1,0,0,0) = 255 - 4 * cc;
		LUT(cc1,0,0,1) = 255;
		LUT(cc1,0,0,2) = 0;
	}
	for (cc=0; cc<64; cc++) {    // de verde a cyan
		int cc1 = cc + 128;
		LUT(cc1,0,0,0) = 0;
		LUT(cc1,0,0,1) = 255;
		LUT(cc1,0,0,2) = 4 * cc;
	}
	for (cc=0; cc<64; cc++) {    // de cyan a azul
		int cc1 = cc + 192;
		LUT(cc1,0,0,0) = 0;
		LUT(cc1,0,0,1) = 255 - 4 * cc;
		LUT(cc1,0,0,2) = 255;
	}
	//		LUT.display("LUT",false);
}






float determineRealSize(float sizex, float centerx, float focalx, float k1, float k2 , float k3)
{
	float W;
	if ( centerx > (sizex - 1.0) / 2.0 )
		W= centerx;
	else
		W= sizex -1 - centerx;

	W = sizex / 2.0 ;

	float a1 = k1 / std::pow( focalx, 2);
	float a2 = k2 / std::pow( focalx, 4);
	float a3 = k3 / std::pow( focalx, 6);
	float w;
	for (w = 0.0; w < 5 * W; w ++) {
		float we = w + a1 * std::pow(w,3) + a2 * std::pow(w,5) + a3 * std::pow(w,7);
		//		std::cout << "w= " << w << "  we = " << we << "\n";
		if (we > W)
			break;
	}

	//	std::cout << "\n\n ***** Apparent size = " << sizex << "  Real Size = " << 2.0 * w << "\n";
	return 2.0 * w;
}



void CimgStereo::getLeftRectified(cimg_library::CImg<unsigned char> & rectifiedLeft) {
	cv::Mat leftcolor = left_input.get_MAT();
	cv::Mat leftcolorr;
	remap(leftcolor, leftcolorr, map11, map12,cv::INTER_LINEAR);
	rectifiedLeft = leftcolorr;
}


void CimgStereo::getLeftRectifiedGray(cimg_library::CImg<unsigned char> & rectifiedLeft)
{
	if ( getNumCameras() == 2)
		rectifiedLeft.assign( img1r.data, img1r.cols, img1r.rows, 1, 1,false);
	else
		rectifiedLeft.assign( imgleft_r.data, imgleft_r.cols, imgleft_r.rows, 1, 1,false);
} //return internally stored info. Does not compute anything

void CimgStereo::getRightRectifiedGray(cimg_library::CImg<unsigned char> & rectifiedRight)
{
	if ( getNumCameras() == 2)
		rectifiedRight.assign( img2r.data, img2r.cols, img2r.rows, 1, 1,false);
	else
		rectifiedRight.assign( imgright_r.data, imgright_r.cols, imgright_r.rows, 1, 1,false);

}



void CimgStereo::getCenterRectifiedGray(cimg_library::CImg<unsigned char> & rectifiedCenter)
{
	if ( getNumCameras() == 2)
		return;
	else
		rectifiedCenter.assign( imgcenter_r.data, imgcenter_r.cols, imgcenter_r.rows, 1, 1,false);

}



void CimgStereo::merge3camsDisparities(const cv::Mat & disp12, const cv::Mat & disp13, const cv::Mat &disp12_2)
{
	CImg<float> c_disp12((float *) disp12.data, disp12.cols, disp12.rows, 1, 1,true);
	CImg<float> c_disp13((float *) disp13.data, disp13.cols, disp13.rows, 1, 1,true);
	CImg<float> c_disp12_2((float *) disp12_2.data, disp12_2.cols, disp12_2.rows, 1, 1,true);

	if( !c_disp12.is_sameXY(c_disp13) )
	{
		std::cerr<<"Error in merge3camsDisparities\n";
		exit(0);
	}

	int width = c_disp12.width();
	int height = c_disp12.height();
	int width2 = c_disp12_2.width();

	disparity_type.fill(UNKNOWN);

	if( multiscaleMode_ == MODE_CLASSIC) {
		for(int y = 0; y < height; y ++) {
			int offsetrow = y * width;
			int offsetrow2 = (y/2) * width2 ; //y/2 * width/2

			for (int x= 0; x < width; x++){
				int offset = x + offsetrow;
				int offset2 = x/2 + offsetrow2;
				float d12 = c_disp12[offset];
				float d13 = c_disp13[offset];
				float d12_2 = c_disp12_2[offset2];
				//Default
				c_disp12[offset] =  - 1.0; //None of them is valid
				if( (d13 > minDisparity13) && (d13 < maxDisparity13) )
				{
					c_disp12[offset] = d13 / ratio12_13;
					disparity_type[offset]= CAM3_LR_DECIMATE_1;
				}
				else if( (d12 > minDisparity12) && (d12 < maxDisparity12) )
				{
					c_disp12[offset] = d12;
					disparity_type[offset]= CAM3_LC_DECIMATE_1;
				}
				else if( (d12_2 > minDisparity12_2) && (d12_2 < maxDisparity12_2) )
				{
					c_disp12[offset] = d12_2 * 2; //Because of half resolution
					disparity_type[offset]= CAM3_LC_DECIMATE_2;
				}
			}
		}
	}
	if( multiscaleMode_ == MODE_ROBUST) {
		for(int y = 0; y < height; y ++) {
			int offsetrow = y * width;
			int offsetrow2 = (y/2) * width2 ; //y/2 * width/2

			for (int x= 0; x < width; x++){
				int offset = x + offsetrow;
				int offset2 = x/2 + offsetrow2;
				float d12 = c_disp12[offset];
				float d13 = c_disp13[offset]/ ratio12_13;;
				float d12_2 = c_disp12_2[offset2] * 2;
				//Default
				c_disp12[offset] =  - 1.0; //None of them is valid
				if( (d13 >= minDisparity13) && (d12 >= minDisparity12) )
				{
					if( fabs (d13 - d12)  < 1.0 ) {
						c_disp12[offset] = d13 ;
						disparity_type[offset]= CAM3_LR_DECIMATE_1;
						continue;
					}
				}
				else if( (d12 >=  minDisparity12) && (d12_2 >= minDisparity12_2) )
				{
					if( fabs (d12_2 - d12)  < 1.0 ) {
						c_disp12[offset] = d12;
						disparity_type[offset]= CAM3_LC_DECIMATE_1;
						continue;
					}
				}

			}
		}
	}
}


void CimgStereo::checkLR_Layout(const cimg_library::CImg<unsigned char> & left,   const cimg_library::CImg<unsigned char> & right) {

	if(layoutCenter_ > 0) //3 cams
		return;

	//Remember original values
	int mindis = bm->getMinDisparity();// bm.state->minDisparity;
	int num_disps = bm->getNumDisparities();// bm.state->numberOfDisparities;

	//int mindis2 = bm2->getMinDisparity();//bm2.state->minDisparity;
	//int num_disps2 = bm2->getNumDisparities();//bm2.state->numberOfDisparities;

	int method = method_;

	int maxDisp = ceil( rectifiedCamFocal * baseLine / (minimumDistance * 1000.0) );
	int disps = ceil( ( 2* maxDisp ) / 16.0) * 16.0;
	int disps2 = ceil( ( maxDisp ) / 16.0) * 16.0;


	//Configure values for test
	//bm.state->minDisparity = -maxDisp;
	//bm.state->numberOfDisparities = disps;
	bm->setMinDisparity(-maxDisp);
	bm->setNumDisparities(disps);

//	bm2.state->minDisparity = -maxDisp/2;
//	bm2.state->numberOfDisparities = disps2;
	bm->setMinDisparity(-maxDisp / 2);
	bm->setNumDisparities(disps2);

	method_ = METHOD_BM;

	//First assume that it is OK
	rectifyOnly( left , right);
	disparity();
	CImg<float> c_disparity((float *) disp.data, disp.cols, disp.rows, 1, 1,true);
	int count_pos = 0;
	int count_neg = 0;
	cimg_foroff(c_disparity,o ) {
		float d= c_disparity [o];
		if ( d < bm->getMinDisparity())
			continue;
		if( d > 1 )
			count_pos ++;
		if( d < - 1 )
			count_neg ++;
	}

	//First assume that it is reversed
	rectifyOnly( right , left);
	disparity();
	CImg<float> c_disparity_r((float *) disp.data, disp.cols, disp.rows, 1, 1,true);
	int count_pos_r = 0;
	int count_neg_r = 0;
	cimg_foroff(c_disparity_r,o ) {
		float d= c_disparity [o];
		if ( d < bm->getMinDisparity())
			continue;
		if( d > 1 )
			count_pos_r ++;
		if( d < - 1 )
			count_neg_r ++;
	}

	std::cout << "=====================================================\n";
	std::cout <<" Pos = " << count_pos;
	std::cout <<" Neg = " << count_neg;
	std::cout << " *** ";
	std::cout <<" Pos_r = " << count_pos_r;
	std::cout <<" Neg_r = " << count_neg_r;
	std::cout << "\n=====================================================\n";


//	bm.state->minDisparity = mindis;
//	bm.state->numberOfDisparities = num_disps;
	bm->setMinDisparity(mindis);
	bm->setNumDisparities(num_disps);

//	bm2.state->minDisparity = mindis2;
//	bm2.state->numberOfDisparities = num_disps2;
	bm2->setMinDisparity(mindis);
	bm2->setNumDisparities(num_disps);
	method_ = method;

	if( (count_pos_r - count_neg_r) > (count_pos - count_neg) ) {
		int tmp = layoutLeft_ ;
		layoutLeft_ = layoutRight_;
		layoutRight_ = tmp;
		std::cout << "\n============ LEFT <----> RIGHT ============================\n";
	}
}





float estimate_Deltat(const cimg_library::CImg<float> & c_dispar, const cimg_library::CImg<float> & motion) {
	float S1 = 0.0;
	float S2 = 0.0;

	CImg<float> localdisp(motion.height(), motion.depth(),1,4);
	//	cimg_forXY(localdisp, x,  y) {
	//		localdisp(x,y,0,0) = motion(0,x,y);
	//		localdisp(x,y,0,1) = motion(1,x,y);
	//	}
	//
	//	localdisp.display("oflow_disp");

	cimg_forYZ(c_dispar,x,y) {

		float m = motion(1,x,y);
		float d = c_dispar(1,x,y);

		if( std::fabs( m ) >= 1.0)
			S1 += m * d;
		S2 += m * m;
	}
	c_dispar.save_cimg("dispar.cimg",false);
	motion.save_cimg("motion.cimg",false);
	if(S2==0.0)
		return 0.0;
	//	std::cout << "S1=" << S1 << " S2=" << S2 << "\n";
	float DeltaT = - S1 / S2;
	return DeltaT;

}





void savePCD(std::string & filename, const cimg_library::CImg<unsigned char> & imgrect, const cimg_library::CImg<float> & xyz) {
	int nvalidPoints = xyz.width()*xyz.height();
	cimg_forXY(xyz,x,y) {
		float zz = xyz(x,y,0,2);
		if (isnan(zz))
			nvalidPoints --;
	}

	string pcd_filename = filename;
	FILE *fd = fopen(pcd_filename.c_str(),"w");
	if( 0 == fd) {
		std::cerr<< "Could not open " << pcd_filename << "for writing\n";
		return;
	}

	fprintf(fd,"# .PCD v.5 - Point Cloud Data file format\n");
	fprintf(fd,"FIELDS x y z rgb\n");
	fprintf(fd,"SIZE 4 4 4 4\n");
	fprintf(fd,"TYPE F F F F\n");
	fprintf(fd,"WIDTH %d\n",nvalidPoints);
	fprintf(fd,"HEIGHT %d\n",1);
	fprintf(fd,"POINTS %d\n",nvalidPoints);
	//	fprintf(fd,"DATA ascii\n");
	fprintf(fd,"DATA binary\n");
	float rgb = 0;
	unsigned char * b =(unsigned char *)(&rgb);
	unsigned char * g = b+1;
	unsigned char * r = b+2;
	r[3] = 0;
	//_input_sin_proyeccion.get_mul(_points_3D.get_shared_channel(2).get_threshold(1e-9)).display("Colores");
	float p[4];
	cimg_forXY(xyz,x,y) {

		float xx = xyz(x,y,0,0);
		float yy = -xyz(x,y,0,1);
		float zz = xyz(x,y,0,2);
		if (isnan(zz))
			continue;
		*r = imgrect(x,y,0,0);
		*g = imgrect(x,y,0,1);
		*b= imgrect(x,y,0,2);
		p[0] = xx;
		p[1] = yy;
		p[2] = zz;
		p[3] = rgb;
		//fprintf(fd,"%f %f %f %f\n", xx, yy, zz, rgb);
		fwrite(p,sizeof(float),4,fd);
	}
	fclose(fd);

}
