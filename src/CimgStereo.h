/*
 * CimgStereo.h
 *
 *  Created on: Sep 7, 2011
 *      Author: aalbiol
 */

#ifndef CIMGSTEREO_H_
#define CIMGSTEREO_H_

#include <string>
#include <vector>
#include <opencv2/calib3d.hpp>


//#include "highgui.h"
#define DEFAULT_SAD_WINSIZE 21
#define DEFAULT_TEXTURE_THRESHOLD 15
#define DEFAULT_UNIQUENESS_RATIO 15
#define DEFAULT_SPECKLE_WINSIZE 100
#define COMPOSITE_UP 1
#define COMPOSITE_DOWN 2
#define COMPOSITE_LEFT 3
#define COMPOSITE_RIGHT 4
#define COMPOSITE_UP_LEFT 5
#define COMPOSITE_UP_RIGHT 6
#define COMPOSITE_DOWN_LEFT 7
#define COMPOSITE_DOWN_RIGHT 8

#define METHOD_BM 1
#define METHOD_SGBM 2
#define METHOD_SVAR 3
#define METHOD_OFLOW 4


#define MODE_CLASSIC 0
#define MODE_ROBUST 1

enum disp_type{UNKNOWN, CAM2_DECIMATE_2, CAM2_DECIMATE_1, CAM3_LC_DECIMATE_2, CAM3_LC_DECIMATE_1, CAM3_LR_DECIMATE_1};

void savePCD(std::string & filename, const cimg_library::CImg<unsigned char> & imgrect, const cimg_library::CImg<float> & xyz);

class CimgStereo {
public:
	CimgStereo();
	CimgStereo(const CimgStereo & cs); //copy constructor
	void loadStereoCalibration(const std::string & stereoCalibDirectory);

	void method( int i ){method_ = i;}

	void SADWindowSize(int ws) { bm->setBlockSize(ws);}
	void SADWindowSizeRel(float f) {SADWindowSizeRel_ = f;}
	void textureThreshold(int t) { _textureThreshold = t;}
	void uniquenessRatio (int u) {bm->setUniquenessRatio(u);}
	void speckleWindowSize (int ws) { bm->setSpeckleWindowSize(ws);}
	void speckleRange (int ws) { bm->setSpeckleRange(ws);}
	void disp12MaxDiff (int d) { bm->setDisp12MaxDiff(d);}
	void decimateDisparity ( int d) {decimate = d;};
	void scale( float s ){ scale_ = s; }

	std::string stereoCalibDirectory(){ return _stereoCalibDirectory; }
	int SADWindowSize(){ return bm->getBlockSize(); }
	int method(){ return method_; }
	int decimateDisparity(){ return decimate; }
	float scale(){ return scale_; }

	void minDistance(float f){minimumDistance = f;}
	void maxDistance(float f){maximumDistance = f;}


	int rectifyOnly( const cimg_library::CImg<unsigned char> & left,  const cimg_library::CImg<unsigned char> & right);
	void rectifyOnly( const cimg_library::CImg<unsigned char> & composite);
	int rectifyOnly( const cimg_library::CImg<unsigned char> & left,  const cimg_library::CImg<unsigned char> & center, const cimg_library::CImg<unsigned char> & right);
	int rectify( const cimg_library::CImg<unsigned char> & left,  const cimg_library::CImg<unsigned char> & right, cimg_library::CImg<unsigned char> & rectifiedLeft,  cimg_library::CImg<unsigned char> & rectifiedRight);

	int disparity(); //Computes disparity from previous rectified images
	int disparity(cimg_library::CImg<float> & _disparity);//Computes disparity from previous rectified images and return it shared
	int disparity( const cimg_library::CImg<unsigned char> & left,  const cimg_library::CImg<unsigned char> & right); //Rectifies images and computes disparity
	int disparity( const cimg_library::CImg<unsigned char> & left,  const cimg_library::CImg<unsigned char> & center, const cimg_library::CImg<unsigned char> & right); //Rectifies images and computes disparity
//	int disparity( const cimg_library::CImg<unsigned char> & left,  const cimg_library::CImg<unsigned char> & right, cimg_library::CImg<float> & _disparity);//Rectifies input images, computes dispariy and returns a shared copy

//	int depth(); //Computes depth from previously computed disparity
//	int depth(cimg_library::CImg<float> & _depth); //Computes depth from previously computed disparity and return a shared copy of depth
	cimg_library::CImg<float> &  xyz(); //Computes depth from previously computed disparity XYZ map
	cimg_library::CImg<float> &  xyz( const cimg_library::CImg<unsigned char> & left,  const cimg_library::CImg<unsigned char> & right);
	cimg_library::CImg<float> &  xyz( const cimg_library::CImg<unsigned char> & left,  const cimg_library::CImg<unsigned char> & center , const cimg_library::CImg<unsigned char> & right);
	cimg_library::CImg<float> &  composite2xyz( const cimg_library::CImg<unsigned char> & composite);
	void splitLR( const cimg_library::CImg<unsigned char> & comp , cimg_library::CImg<unsigned char> & left, cimg_library::CImg<unsigned char> & right);
	void splitLCR( const cimg_library::CImg<unsigned char> & comp , cimg_library::CImg<unsigned char> & left, cimg_library::CImg<unsigned char> & center, cimg_library::CImg<unsigned char> & right);

	void getLeftRectified(cimg_library::CImg<unsigned char> & rectifiedLeft);
	void getLeftRectifiedGray(cimg_library::CImg<unsigned char> & rectifiedLeft);
	void getRightRectifiedGray(cimg_library::CImg<unsigned char> & rectifiedRight)  ;//return internally stored info. Does not compute anything
	void getCenterRectifiedGray(cimg_library::CImg<unsigned char> & rectifiedCenter) ;//return internally stored info. Does not compute anything
	void getDisparity(cimg_library::CImg<float> & disparity) {	disparity.assign( (float *)disp.data, disp.cols, disp.rows, 1, 1,true);}//return internally stored info. Does not compute anything
	void getDisparity_type(cimg_library::CImg<disp_type> & disparity_typeOut) {	disparity_typeOut=disparity_type;}//return internally stored info. Does not compute anything
	void getXYZ(cimg_library::CImg<float> & XYZ);
	const cimg_library::CImg<double> &  getMatQ() const {return cimg_Q;};
	void setDecimate(int dec) { decimate = dec;}

	int getMaxDisparity() const { return maxDisparity;}
	int getMinDisparity() const { return minDisparity;}
	int getTextureTh() const { return _textureThreshold;}
	int getUniquenessRatio() const { return bm->getUniquenessRatio();}

	//Rectifies, computes disparity and depth and returns a shared copy of depth
//	int depth( cimg_library::CImg<unsigned char> & left,  cimg_library::CImg<unsigned char> & right, cimg_library::CImg<float> & _depth);

	void camHeight_mm(float f){ camHeight_ = f;}
	float camHeight_mm(){ return camHeight_ ;}

	void rotationMatrix( const cimg_library::CImg<float> & m){ rotationMatrix_ = m ;}
	const cimg_library::CImg<float> & rotationMatrix() const { return rotationMatrix_;}
	const cv::Mat & Q_Matrix() const { return Q;}

	void rectifyImage(cimg_library::CImg<unsigned char> & original,  cimg_library::CImg<unsigned char> & rectified);
	void Zplane2XYZ(float zplane, const cimg_library::CImg<unsigned int> & mask, cimg_library::CImg<float> & xyz);

	int read_config( const char * filename);
	int read_input_config( const char * filename);

	void SGBM_SmoothFactor( float f){ SGBM_SmoothFactor_ = f;}
	void depthAsColor(cimg_library::CImg<float> & depthcolor);
	void Y_AsColor(cimg_library::CImg<unsigned char> & depthcolor);
	int getNumCameras() const { return (layoutCenter_ > 0? 3 :2) ;}

	cimg_library::CImg<float> getLeftProjectionMatrix() const ;

	int exist_rotationMatrix(){return rotationMatrix_.size() == 9 ?  1 : 0;}


private:
	int _nf ; //frame counter;
	int decimate;
	float scale_;
	int _textureThreshold;

	int method_; //For disparity estimation
	//Input images
	cv::Mat img1, img2; //left right
	cv::Mat img3; //center

	cv::Mat img1r, img2r; //left-right

	cv::Mat prevLeftRectified_;

	//Recified input images: 3cam
	cv::Mat imgleft_r, imgright_r; //left-right
	cv::Mat imgcenter_r; //center

	//Disparity and depth
	cv::Mat disp;
	cv::Mat disp2; //Computed with decimation

	cimg_library::CImg<unsigned char> LUT;

	cv::Rect roi1, roi2;
	cimg_library::CImg<float> c_depth3D;
	cimg_library::CImg<unsigned char> left_input; //as it was color or gray
	cimg_library::CImg<unsigned char> left_gray;
	cimg_library::CImg<unsigned char> right_gray;
	cimg_library::CImg<unsigned char> center_gray;

	cimg_library::CImg<disp_type> disparity_type;   //to know for each pixel if disparity comes from low or high resolution and if left-center or left-right camera pair has been used for that pixel



	//Stereo params

	float SADWindowSizeRel_;
	float vertBlur_; // To compensate synchronization errors

	//2-cams: 1:Left 2:Right
	//3-cams : 1:Left 2:center 3:right
    cv::Mat M1, D1, M2, D2;
    cv::Mat M3, D3;
    cv::Mat R, T; //for 2 cam
    cv::Mat R12, T12; //for 3 cam  Left-center
    cv::Mat R13, T13; //for 3 cam  Left-right
    cv::Mat R1, P1, R2, P2;
    cv::Mat R3, P3;

    cv::Mat Q;
    cimg_library::CImg<double> cimg_Q;
    cv::Mat map11, map12, map21, map22;
    cv::Mat map31, map32;


    cimg_library::CImg<float> points_image1_calib_;
    cimg_library::CImg<float> points_image3_calib_;
    std::vector < std::vector <cv::Point2f> > points1_at_input_scale_;
    std::vector < std::vector <cv::Point2f> > points3_at_input_scale_;

    float ratio12_13;
	cv::Size calibImageSize;
	cv::Size inputImageSize;
	cv::Size imageSize_rect;

	double rectifiedCamFocal;
	double baseLine;
	double minimumDistance;
	double maximumDistance;
	double meanDistance_;

	int minDisparity;
	int maxDisparity;

	//Multiscale
	int multiscaleMode_ ;
	int minDisparity1;
	int maxDisparity1;
	int minDisparity2;
	int maxDisparity2;
	//For 3 cams
	int minDisparity12;
	int maxDisparity12;
	int minDisparity12_2;
	int maxDisparity12_2;
	int minDisparity13;
	int maxDisparity13;
	float alphaRectify_;
	//Stereo processor
	cv::Ptr<cv::StereoBM>  bm;
	cv::Ptr<cv::StereoBM>  bm2;
//	cv::StereoBM bm;
//	cv::StereoBM bm2; //Decimation 2

	cv::Ptr<cv::StereoBM> bm_12_2;
	cv::Ptr<cv::StereoBM> bm_12;
	cv::Ptr<cv::StereoBM> bm_13;

	float SGBM_SmoothFactor_;
	cv::Ptr<cv::StereoSGBM> sgbm;
	cv::Ptr<cv::StereoSGBM> sgbm2;
	cv::Ptr<cv::StereoSGBM> sgbm_12;
	cv::Ptr<cv::StereoSGBM> sgbm_12_2;
	cv::Ptr<cv::StereoSGBM> sgbm_13;


	cimg_library::CImg<float> rotationMatrix_ ;

	float camHeight_;
	std::string _stereoCalibDirectory;

	//Input Format
	int layoutLeft_;
	int layoutRight_;
	int layoutCenter_ ; //If -1: two cam stereo

	int warning_bad_depthasColor;

	//Private funcs
	int start();
	int reproject3D();
	void setBMDefaults();
	int disparityDecimate2();
	void rotateCoordinates();
	void mergeMultiscale();
	void dispshort2dispfloat(const cv::Mat & dsh, cv::Mat &dfl);
	void dispuchar2dispfloat(const cv::Mat & dsh, cv::Mat &dfl);
	void merge3camsDisparities(const cv::Mat & disp12, const cv::Mat & disp13, const cv::Mat &disp12_2);

	void createLUT();
	void checkLR_Layout(const cimg_library::CImg<unsigned char> & left,   const cimg_library::CImg<unsigned char> & right);
	
protected:
	cimg_library::CImg<float> c_Range;  //jmmossi
public:	
	void Range();  
	void Range(cimg_library::CImg<float> & _range);
	void rangeAlarm(cimg_library::CImg<unsigned char> & range_alarm, float threshold);


};


int layoutDecode(const std::string & s);
float determineRealSize(float sizex, float centerx, float focalx, float k1, float k2 , float k3);

float estimate_Deltat(const cimg_library::CImg<float> & c_dispar, const cimg_library::CImg<float> & motion);

#endif /* CIMGSTEREO_H_ */
