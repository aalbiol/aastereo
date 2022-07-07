/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

/*
 This is a modification of the variational stereo correspondence algorithm, described in:
 S. Kosov, T. Thormaehlen, H.-P. Seidel "Accurate Real-Time Disparity Estimation with Variational Methods"
 Proceedings of the 5th International Symposium on Visual Computing, Vegas, USA

 This code is written by Sergey G. Kosov for "Visir PX" application as part of Project X (www.project-10.de)
 */ 
#include <vector>
#include "cv.h"
#include <limits.h>
#include "stereovar_upv.h"
namespace upvsoft
{

using namespace cv;
//StereoVar::StereoVar() : levels(3), pyrScale(0.5), nIt(5), minDisp(0), maxDisp(16), poly_n(3), poly_sigma(0), fi(25.0f), lambda(0.03f), penalization(PENALIZATION_TICHONOV), cycle(CYCLE_V), flags(USE_SMART_ID | USE_AUTO_PARAMS)
StereoVar::StereoVar() : levels(3), pyrScale(0.5), nIt(250), minDisp(0), maxDisp(16), poly_n(3), poly_sigma(0), fi(750.0f), lambda(0.03f), penalization(PENALIZATION_TICHONOV), cycle(CYCLE_V), flags(USE_SMART_ID | USE_AUTO_PARAMS)
{
}

StereoVar::StereoVar(int _levels, double _pyrScale, int _nIt, int _minDisp, int _maxDisp, int _poly_n, double _poly_sigma, float _fi, float _lambda, int _penalization, int _cycle, int _flags) : levels(_levels), pyrScale(_pyrScale), nIt(_nIt), minDisp(_minDisp), maxDisp(_maxDisp), poly_n(_poly_n), poly_sigma(_poly_sigma), fi(_fi), lambda(_lambda), penalization(_penalization), cycle(_cycle), flags(_flags)
{ // No Parameters check, since they are all public
}

StereoVar::~StereoVar()
{
}

static cv::Mat diffX(Mat &src)
{
	register int x, y, cols = src.cols - 1;
	Mat dst(src.size(), src.type());
	for(y = 0; y < src.rows; y++){
        const float* pSrc = src.ptr<float>(y);
        float* pDst = dst.ptr<float>(y);
#if CV_SSE2
        for (x = 0; x <= cols - 8; x += 8) {
            __m128 a0 = _mm_loadu_ps(pSrc + x);
            __m128 b0 = _mm_loadu_ps(pSrc + x + 1);
            __m128 a1 = _mm_loadu_ps(pSrc + x + 4);
            __m128 b1 = _mm_loadu_ps(pSrc + x + 5);
            b0 = _mm_sub_ps(b0, a0);
            b1 = _mm_sub_ps(b1, a1);
            _mm_storeu_ps(pDst + x, b0);
            _mm_storeu_ps(pDst + x + 4, b1);
        }
#endif
        for( ; x < cols; x++) pDst[x] = pSrc[x+1] - pSrc[x];
        pDst[cols] = 0.f;
    }
    return dst;
}

static Mat getGradient(Mat &src)
{
	register int x, y;
	Mat dst(src.size(), src.type());
	dst.setTo(0);
	for (y = 0; y < src.rows - 1; y++) {
		float *pSrc = src.ptr<float>(y);
		float *pSrcF = src.ptr<float>(y + 1);
		float *pDst = dst.ptr<float>(y);
		for (x = 0; x < src.cols - 1; x++)
			pDst[x] = fabs(pSrc[x + 1] - pSrc[x]) + fabs(pSrcF[x] - pSrc[x]); 
	}
	return dst;
}

static Mat getG_c(Mat &src, float l)
{
	Mat dst(src.size(), src.type());
	for (register int y = 0; y < src.rows; y++) {
		float *pSrc = src.ptr<float>(y);
		float *pDst = dst.ptr<float>(y);
		for (register int x = 0; x < src.cols; x++)
			pDst[x] = 0.5f*l / sqrtf(l*l + pSrc[x]*pSrc[x]);
	}
	return dst;
}

static Mat getG_p(Mat &src, float l)
{
	Mat dst(src.size(), src.type());
	for (register int y = 0; y < src.rows; y++) {
		float *pSrc = src.ptr<float>(y);
		float *pDst = dst.ptr<float>(y);
		for (register int x = 0; x < src.cols; x++)
			pDst[x] = 0.5f*l*l / (l*l + pSrc[x]*pSrc[x]);
	}
	return dst;
}

void StereoVar::VariationalSolver(Mat &I1, Mat &I2, Mat &I2x, Mat &u, int level)
{
	register int n, x, y;
	float gl = 1, gr = 1, gu = 1, gd = 1, gc = 4;
	Mat g_c, g_p;
	Mat U; 
	u.copyTo(U);

	int		N = nIt;
	float	l = lambda;
	float	Fi = fi;

	
	if (flags & USE_SMART_ID) {
		double scale = pow(pyrScale, (double) level) * (1 + pyrScale);	
		N = (int) (N / scale);
	}

	double scale = pow(pyrScale, (double) level);
	Fi /= (float) scale;
	l *= (float) scale;

	int width	= u.cols - 1;
	int height	= u.rows - 1;
	for (n = 0; n < N; n++) {
		if (penalization != PENALIZATION_TICHONOV) {
			Mat gradient = getGradient(U);
			switch (penalization) {
				case PENALIZATION_CHARBONNIER:	g_c = getG_c(gradient, l); break;
				case PENALIZATION_PERONA_MALIK: g_p = getG_p(gradient, l); break;
			}
			gradient.release();
		}
		for (y = 1 ; y < height; y++) {
			float *pU	= U.ptr<float>(y);
			float *pUu	= U.ptr<float>(y + 1);
			float *pUd	= U.ptr<float>(y - 1);
			float *pu	= u.ptr<float>(y);
			float *pI1	= I1.ptr<float>(y);
			float *pI2	= I2.ptr<float>(y);
			float *pI2x = I2x.ptr<float>(y);
			float *pG_c = 0, *pG_cu = 0, *pG_cd = 0;
			float *pG_p = 0, *pG_pu = 0, *pG_pd = 0;
			switch (penalization) {
				case PENALIZATION_CHARBONNIER:	
					pG_c	= g_c.ptr<float>(y); 
					pG_cu	= g_c.ptr<float>(y + 1); 
					pG_cd	= g_c.ptr<float>(y - 1); 
					break;
				case PENALIZATION_PERONA_MALIK: 
					pG_p	= g_p.ptr<float>(y); 
					pG_pu	= g_p.ptr<float>(y + 1); 
					pG_pd	= g_p.ptr<float>(y - 1); 
					break;
			}
			for (x = 1; x < width; x++) {
				switch (penalization) {
					case PENALIZATION_CHARBONNIER:
						gc = pG_c[x];
						gl = gc + pG_c[x - 1];
						gr = gc + pG_c[x + 1];
						gu = gc + pG_cu[x];
						gd = gc + pG_cd[x];
						gc = gl + gr + gu + gd;						
						break;
					case PENALIZATION_PERONA_MALIK:
						gc = pG_p[x];
						gl = gc + pG_p[x - 1];
						gr = gc + pG_p[x + 1];
						gu = gc + pG_pu[x];
						gd = gc + pG_pd[x];
						gc = gl + gr + gu + gd;
						break;
				}

				float fi = Fi;
				if (maxDisp > minDisp) {
					if (pU[x] > maxDisp * scale) {fi *= 1000; pU[x] = static_cast<float>(maxDisp * scale);} 
					if (pU[x] < minDisp * scale) {fi *= 1000; pU[x] = static_cast<float>(minDisp * scale);} 
				}

				int A = static_cast<int>(pU[x]);
				int neg = 0; if (pU[x] <= 0) neg = -1;

				if (x + A > width)
					pu[x] = pU[width - A];
				else if (x + A + neg < 0)
					pu[x] = pU[- A + 2];
				else { 
					pu[x] = A + (pI2x[x + A + neg] * (pI1[x] - pI2[x + A])
							  + fi * (gr * pU[x + 1] + gl * pU[x - 1] + gu * pUu[x] + gd * pUd[x] - gc * A)) 
							  / (pI2x[x + A + neg] * pI2x[x + A + neg] + gc * fi) ; 
				}
			}// x
			pu[0] = pu[1];
			pu[width] = pu[width - 1];
		}// y
		for (x = 0; x <= width; x++) {
			u.at<float>(0, x) = u.at<float>(1, x);
			u.at<float>(height, x) = u.at<float>(height - 1, x);
		}
		u.copyTo(U);
		if (!g_c.empty()) g_c.release();
		if (!g_p.empty()) g_p.release();
	}//n
}

void StereoVar::VCycle_MyFAS(Mat &I1, Mat &I2, Mat &I2x, Mat &_u, int level)
{
	CvSize imgSize = _u.size();
	CvSize frmSize = cvSize((int) (imgSize.width * pyrScale + 0.5), (int) (imgSize.height * pyrScale + 0.5));
	Mat I1_h, I2_h, I2x_h, u_h, U, U_h;

	//PRE relaxation
	VariationalSolver(I1, I2, I2x, _u, level);

	if (level >= levels - 1) return;
	level ++;

	//scaling DOWN
	resize(I1, I1_h, frmSize, 0, 0, INTER_AREA);
	resize(I2, I2_h, frmSize, 0, 0, INTER_AREA);
	resize(_u, u_h, frmSize, 0, 0, INTER_AREA);
	u_h.convertTo(u_h, u_h.type(), pyrScale);
	I2x_h = diffX(I2_h);

	//Next level
	U_h = u_h.clone();
	VCycle_MyFAS(I1_h, I2_h, I2x_h, U_h, level);

	subtract(U_h, u_h, U_h);
	U_h.convertTo(U_h, U_h.type(), 1.0 / pyrScale);

	//scaling UP
	resize(U_h, U, imgSize);

	//correcting the solution
	add(_u, U, _u);

	//POST relaxation
	VariationalSolver(I1, I2, I2x, _u, level - 1);

	if (flags & USE_MEDIAN_FILTERING) medianBlur(_u, _u, 3);

	I1_h.release();
	I2_h.release();
	I2x_h.release();
	u_h.release();
	U.release();
	U_h.release();
}

void StereoVar::FMG(Mat &I1, Mat &I2, Mat &I2x, Mat &u, int level)
{
	double	scale = pow(pyrScale, (double) level);
	CvSize	frmSize = cvSize((int) (u.cols * scale + 0.5), (int) (u.rows * scale + 0.5));
	Mat I1_h, I2_h, I2x_h, u_h;

	//scaling DOWN
	resize(I1, I1_h, frmSize, 0, 0, INTER_AREA);
	resize(I2, I2_h, frmSize, 0, 0, INTER_AREA);
	resize(u, u_h, frmSize, 0, 0, INTER_AREA);
	u_h.convertTo(u_h, u_h.type(), scale);
	I2x_h = diffX(I2_h);

	switch (cycle) {
		case CYCLE_O:
			VariationalSolver(I1_h, I2_h, I2x_h, u_h, level);
			break;
		case CYCLE_V:
			VCycle_MyFAS(I1_h, I2_h, I2x_h, u_h, level);
			break;
	}

	u_h.convertTo(u_h, u_h.type(), 1.0 / scale);

	//scaling UP
	resize(u_h, u, u.size(), 0, 0, INTER_CUBIC);

	I1_h.release();
	I2_h.release();
	I2x_h.release();
	u_h.release();

	level--;
	if ((flags & USE_AUTO_PARAMS) && (level < levels / 3)) { 
		penalization = PENALIZATION_PERONA_MALIK;
		fi *= 100;
		flags -= USE_AUTO_PARAMS;
		autoParams();
	}
	if (flags & USE_MEDIAN_FILTERING) medianBlur(u, u, 3);
	if (level >= 0) FMG(I1, I2, I2x, u, level);
}

void StereoVar::autoParams()
{	
	int maxD = MAX(labs(maxDisp), labs(minDisp));
	
	if (!maxD) pyrScale = 0.85;
	else if (maxD < 8) pyrScale = 0.5;
	else if (maxD < 64) pyrScale = 0.5 + static_cast<double>(maxD - 8) * 0.00625;
	else pyrScale = 0.85;
	
	if (maxD) {
		levels = 0;
		while ( pow(pyrScale, levels) * maxD > 1.5) levels ++;
		levels++;
	}

	switch(penalization) {
		case PENALIZATION_TICHONOV: cycle = CYCLE_V; break;
		case PENALIZATION_CHARBONNIER: cycle = CYCLE_O; break;
		case PENALIZATION_PERONA_MALIK: cycle = CYCLE_O; break;
	}
}

void StereoVar::operator ()( const Mat& left, const Mat& right, Mat& disp )
{
	CV_Assert(left.size() == right.size() && left.type() == right.type());
	CvSize imgSize = left.size();
	int MaxD = MAX(labs(minDisp), labs(maxDisp)); 
	int SignD = 1; if (MIN(minDisp, maxDisp) < 0) SignD = -1;
	if (minDisp >= maxDisp) {MaxD = 256; SignD = 1;}
		
	Mat u;
	if ((flags & USE_INITIAL_DISPARITY) && (!disp.empty())) {
		CV_Assert(disp.size() == left.size() && disp.type() == CV_8UC1);
		disp.convertTo(u, CV_32FC1, static_cast<double>(SignD * MaxD) / 256);
	} else {
		u.create(imgSize, CV_32FC1);
		u.setTo(0);
	}

	// Preprocessing
	Mat leftgray, rightgray;
	if (left.type() != CV_8UC1) {
		cvtColor(left, leftgray, CV_BGR2GRAY);
		cvtColor(right, rightgray, CV_BGR2GRAY);
	} else {
		left.copyTo(leftgray);
		right.copyTo(rightgray);
	}
	if (flags & USE_EQUALIZE_HIST) {
		equalizeHist(leftgray, leftgray);
		equalizeHist(rightgray, rightgray);
	}
	if (poly_sigma > 0.0001) {
		GaussianBlur(leftgray, leftgray, cvSize(poly_n, poly_n), poly_sigma);
		GaussianBlur(rightgray, rightgray, cvSize(poly_n, poly_n), poly_sigma);
	}
		
	if (flags & USE_AUTO_PARAMS) {
		penalization = PENALIZATION_TICHONOV;
		autoParams();
	}

	Mat I1, I2;
	leftgray.convertTo(I1, CV_32FC1);
	rightgray.convertTo(I2, CV_32FC1);
	leftgray.release();
	rightgray.release();

	Mat I2x = diffX(I2);
		
	FMG(I1, I2, I2x, u, levels - 1);		
		
	I1.release();
	I2.release();
	I2x.release();
	
/*
	disp.create( left.size(), CV_8UC1 );
	u = abs(u);
	u.convertTo(disp, disp.type(), 256 / MaxD, 0);	

	u.release();
	*/
	disp = u;
}




void adjust3rdMatrix(InputArrayOfArrays _imgpt1_0,
                            InputArrayOfArrays _imgpt3_0,
                            const Mat& cameraMatrix1, const Mat& distCoeffs1,
                            const Mat& cameraMatrix3, const Mat& distCoeffs3,
                            const Mat& R1, const Mat& R3, const Mat& P1, Mat& P3 )
{
    size_t n1 = _imgpt1_0.total(), n3 = _imgpt3_0.total();
    std::vector<Point2f> imgpt1, imgpt3;

    for( int i = 0; i < (int)std::min(n1, n3); i++ )
    {
        Mat pt1 = _imgpt1_0.getMat(i), pt3 = _imgpt3_0.getMat(i);
        int ni1 = pt1.checkVector(2, CV_32F), ni3 = pt3.checkVector(2, CV_32F);
        CV_Assert( ni1 > 0 && ni1 == ni3 );
        const Point2f* pt1data = pt1.ptr<Point2f>();
        const Point2f* pt3data = pt3.ptr<Point2f>();
        std::copy(pt1data, pt1data + ni1, std::back_inserter(imgpt1));
        std::copy(pt3data, pt3data + ni3, std::back_inserter(imgpt3));
    }

    undistortPoints(imgpt1, imgpt1, cameraMatrix1, distCoeffs1, R1, P1);
    undistortPoints(imgpt3, imgpt3, cameraMatrix3, distCoeffs3, R3, P3);

    double y1_ = 0, y2_ = 0, y1y1_ = 0, y1y2_ = 0;
    size_t n = imgpt1.size();

    for( size_t i = 0; i < n; i++ )
    {
        double y1 = imgpt3[i].y, y2 = imgpt1[i].y;

        y1_ += y1; y2_ += y2;
        y1y1_ += y1*y1; y1y2_ += y1*y2;
    }

    y1_ /= n;
    y2_ /= n;
    y1y1_ /= n;
    y1y2_ /= n;

    double a = (y1y2_ - y1_*y2_)/(y1y1_ - y1_*y1_);
    double b = y2_ - a*y1_;

    P3.at<double>(0,0) *= a;
    P3.at<double>(1,1) *= a;
    P3.at<double>(0,2) = P3.at<double>(0,2)*a;
    P3.at<double>(1,2) = P3.at<double>(1,2)*a + b;
    P3.at<double>(0,3) *= a;
    P3.at<double>(1,3) *= a;
}



float rectify3Collinear( InputArray _cameraMatrix1, InputArray _distCoeffs1,
                   InputArray _cameraMatrix2, InputArray _distCoeffs2,
                   InputArray _cameraMatrix3, InputArray _distCoeffs3,
                   InputArrayOfArrays _imgpt1,
                   InputArrayOfArrays _imgpt3,
                   Size imageSize, InputArray _Rmat12, InputArray _Tmat12,
                   InputArray _Rmat13, InputArray _Tmat13,
                   OutputArray _Rmat1, OutputArray _Rmat2, OutputArray _Rmat3,
                   OutputArray _Pmat1, OutputArray _Pmat2, OutputArray _Pmat3,
                   OutputArray _Qmat,
                   double alpha, Size newImgSize,
                   Rect* roi1, Rect* roi2, int flags )
{
    // first, rectify the 1-2 stereo pair
    upvsoft::stereoRectify( _cameraMatrix1, _distCoeffs1, _cameraMatrix2, _distCoeffs2,
                   imageSize, _Rmat12, _Tmat12, _Rmat1, _Rmat2, _Pmat1, _Pmat2, _Qmat,
                   flags, alpha, newImgSize, roi1, roi2 );

    Mat R12 = _Rmat12.getMat(), R13 = _Rmat13.getMat(), T12 = _Tmat12.getMat(), T13 = _Tmat13.getMat();

    _Rmat3.create(3, 3, CV_64F);
    _Pmat3.create(3, 4, CV_64F);

    Mat P1 = _Pmat1.getMat(), P2 = _Pmat2.getMat();
    Mat R3 = _Rmat3.getMat(), P3 = _Pmat3.getMat();

    // recompute rectification transforms for cameras 1 & 2.
    Mat om, r_r, r_r13;

    if( R13.size() != Size(3,3) )
        Rodrigues(R13, r_r13);
    else
        R13.copyTo(r_r13);

    if( R12.size() == Size(3,3) )
        Rodrigues(R12, om);
    else
        R12.copyTo(om);

    om *= -0.5;
    Rodrigues(om, r_r); // rotate cameras to same orientation by averaging
    Mat_<double> t12 = r_r * T12;

    int idx = fabs(t12(0,0)) > fabs(t12(1,0)) ? 0 : 1;
    double c = t12(idx,0), nt = norm(t12, CV_L2);
    Mat_<double> uu = Mat_<double>::zeros(3,1);
    uu(idx, 0) = c > 0 ? 1 : -1;

    // calculate global Z rotation
    Mat_<double> ww = t12.cross(uu), wR;
    double nw = norm(ww, CV_L2);
    ww *= acos(fabs(c)/nt)/nw;
    Rodrigues(ww, wR);

    // now rotate camera 3 to make its optical axis parallel to cameras 1 and 2.
    R3 = wR*r_r.t()*r_r13.t();
    Mat_<double> t13 = R3 * T13;

    P2.copyTo(P3);
    Mat t = P3.col(3);
    t13.copyTo(t);
    P3.at<double>(0,3) *= P3.at<double>(0,0);
    P3.at<double>(1,3) *= P3.at<double>(1,1);

    if( !_imgpt1.empty() && _imgpt3.empty() )
        adjust3rdMatrix(_imgpt1, _imgpt3, _cameraMatrix1.getMat(), _distCoeffs1.getMat(),
                        _cameraMatrix3.getMat(), _distCoeffs3.getMat(), _Rmat1.getMat(), R3, P1, P3);

    return (float)((P3.at<double>(idx,3)/P3.at<double>(idx,idx))/
                   (P2.at<double>(idx,3)/P2.at<double>(idx,idx)));
}


void
icvGetRectangles( const CvMat* cameraMatrix, const CvMat* distCoeffs,
                 const CvMat* R, const CvMat* newCameraMatrix, CvSize imgSize,
                 cv::Rect_<float>& inner, cv::Rect_<float>& outer )
{
    const int N = 9;
    int x, y, k;
    cv::Ptr<CvMat> _pts = cvCreateMat(1, N*N, CV_32FC2);
    CvPoint2D32f* pts = (CvPoint2D32f*)(_pts->data.ptr);

    for( y = k = 0; y < N; y++ )
        for( x = 0; x < N; x++ )
            pts[k++] = cvPoint2D32f((float)x*imgSize.width/(N-1),
                                    (float)y*imgSize.height/(N-1));

    cvUndistortPoints(_pts, _pts, cameraMatrix, distCoeffs, R, newCameraMatrix);

    float iX0=-FLT_MAX, iX1=FLT_MAX, iY0=-FLT_MAX, iY1=FLT_MAX;
    float oX0=FLT_MAX, oX1=-FLT_MAX, oY0=FLT_MAX, oY1=-FLT_MAX;
    // find the inscribed rectangle.
    // the code will likely not work with extreme rotation matrices (R) (>45%)
    for( y = k = 0; y < N; y++ )
        for( x = 0; x < N; x++ )
        {
            CvPoint2D32f p = pts[k++];
            oX0 = MIN(oX0, p.x);
            oX1 = MAX(oX1, p.x);
            oY0 = MIN(oY0, p.y);
            oY1 = MAX(oY1, p.y);

            if( x == 0 )
                iX0 = MAX(iX0, p.x);
            if( x == N-1 )
                iX1 = MIN(iX1, p.x);
            if( y == 0 )
                iY0 = MAX(iY0, p.y);
            if( y == N-1 )
                iY1 = MIN(iY1, p.y);
        }
    inner = cv::Rect_<float>(iX0, iY0, iX1-iX0, iY1-iY0);
    outer = cv::Rect_<float>(oX0, oY0, oX1-oX0, oY1-oY0);
}



void cvStereoRectify( const CvMat* _cameraMatrix1, const CvMat* _cameraMatrix2,
                      const CvMat* _distCoeffs1, const CvMat* _distCoeffs2,
                      CvSize imageSize, const CvMat* matR, const CvMat* matT,
                      CvMat* _R1, CvMat* _R2, CvMat* _P1, CvMat* _P2,
                      CvMat* matQ, int flags, double alpha, CvSize newImgSize,
                      CvRect* roi1, CvRect* roi2 )
{
    double _om[3], _t[3], _uu[3]={0,0,0}, _r_r[3][3], _pp[3][4];
    double _ww[3], _wr[3][3], _z[3] = {0,0,0}, _ri[3][3];
    cv::Rect_<float> inner1, inner2, outer1, outer2;

    CvMat om  = cvMat(3, 1, CV_64F, _om);
    CvMat t   = cvMat(3, 1, CV_64F, _t);
    CvMat uu  = cvMat(3, 1, CV_64F, _uu);
    CvMat r_r = cvMat(3, 3, CV_64F, _r_r);
    CvMat pp  = cvMat(3, 4, CV_64F, _pp);
    CvMat ww  = cvMat(3, 1, CV_64F, _ww); // temps
    CvMat wR  = cvMat(3, 3, CV_64F, _wr);
    CvMat Z   = cvMat(3, 1, CV_64F, _z);
    CvMat Ri  = cvMat(3, 3, CV_64F, _ri);
    double nx = imageSize.width, ny = imageSize.height;
    int i, k;

    if( matR->rows == 3 && matR->cols == 3 )
        cvRodrigues2(matR, &om);          // get vector rotation
    else
        cvConvert(matR, &om); // it's already a rotation vector
    cvConvertScale(&om, &om, -0.5); // get average rotation
    cvRodrigues2(&om, &r_r);        // rotate cameras to same orientation by averaging
    cvMatMul(&r_r, matT, &t);

    int idx = fabs(_t[0]) > fabs(_t[1]) ? 0 : 1;
    double c = _t[idx], nt = cvNorm(&t, 0, CV_L2);
    _uu[idx] = c > 0 ? 1 : -1;

    // calculate global Z rotation
    cvCrossProduct(&t,&uu,&ww);
    double nw = cvNorm(&ww, 0, CV_L2);
    cvConvertScale(&ww, &ww, acos(fabs(c)/nt)/nw);
    cvRodrigues2(&ww, &wR);

    // apply to both views
    cvGEMM(&wR, &r_r, 1, 0, 0, &Ri, CV_GEMM_B_T);
    cvConvert( &Ri, _R1 );
    cvGEMM(&wR, &r_r, 1, 0, 0, &Ri, 0);
    cvConvert( &Ri, _R2 );
    cvMatMul(&Ri, matT, &t);

    // calculate projection/camera matrices
    // these contain the relevant rectified image internal params (fx, fy=fx, cx, cy)
    double fc_new = DBL_MAX;
    CvPoint2D64f cc_new[2] = {{0,0}, {0,0}};

    for( k = 0; k < 2; k++ ) {
        const CvMat* A = k == 0 ? _cameraMatrix1 : _cameraMatrix2;
        const CvMat* Dk = k == 0 ? _distCoeffs1 : _distCoeffs2;
        double dk1 = Dk ? cvmGet(Dk, 0, 0) : 0;
        double fc = cvmGet(A,idx^1,idx^1);
        if( dk1 < 0 ) {
            fc *= 1 + dk1*(nx*nx + ny*ny)/(4*fc*fc);
        }
        fc_new = MIN(fc_new, fc);
    }

    for( k = 0; k < 2; k++ )
    {
        const CvMat* A = k == 0 ? _cameraMatrix1 : _cameraMatrix2;
        const CvMat* Dk = k == 0 ? _distCoeffs1 : _distCoeffs2;
        CvPoint2D32f _pts[4];
        CvPoint3D32f _pts_3[4];
        CvMat pts = cvMat(1, 4, CV_32FC2, _pts);
        CvMat pts_3 = cvMat(1, 4, CV_32FC3, _pts_3);

        for( i = 0; i < 4; i++ )
        {
            int j = (i<2) ? 0 : 1;
            _pts[i].x = (float)((i % 2)*(nx-1));
            _pts[i].y = (float)(j*(ny-1));
        }
        cvUndistortPoints( &pts, &pts, A, Dk, 0, 0 );
        cvConvertPointsHomogeneous( &pts, &pts_3 );

        //Change camera matrix to have cc=[0,0] and fc = fc_new
        double _a_tmp[3][3];
        CvMat A_tmp  = cvMat(3, 3, CV_64F, _a_tmp);
        _a_tmp[0][0]=fc_new;
        _a_tmp[1][1]=fc_new;
        _a_tmp[0][2]=0.0;
        _a_tmp[1][2]=0.0;
        cvProjectPoints2( &pts_3, k == 0 ? _R1 : _R2, &Z, &A_tmp, 0, &pts );
        CvScalar avg = cvAvg(&pts);
        cc_new[k].x = (nx-1)/2 - avg.val[0];
        cc_new[k].y = (ny-1)/2 - avg.val[1];
    }

    // vertical focal length must be the same for both images to keep the epipolar constraint
    // (for horizontal epipolar lines -- TBD: check for vertical epipolar lines)
    // use fy for fx also, for simplicity

    // For simplicity, set the principal points for both cameras to be the average
    // of the two principal points (either one of or both x- and y- coordinates)
    if( flags & CV_CALIB_ZERO_DISPARITY )
    {
        cc_new[0].x = cc_new[1].x = (cc_new[0].x + cc_new[1].x)*0.5;
        cc_new[0].y = cc_new[1].y = (cc_new[0].y + cc_new[1].y)*0.5;
    }
    else if( idx == 0 ) // horizontal stereo
        cc_new[0].y = cc_new[1].y = (cc_new[0].y + cc_new[1].y)*0.5;
    else // vertical stereo
        cc_new[0].x = cc_new[1].x = (cc_new[0].x + cc_new[1].x)*0.5;

    cvZero( &pp );
    _pp[0][0] = _pp[1][1] = fc_new;
    _pp[0][2] = cc_new[0].x;
    _pp[1][2] = cc_new[0].y;
    _pp[2][2] = 1;
    cvConvert(&pp, _P1);

    _pp[0][2] = cc_new[1].x;
    _pp[1][2] = cc_new[1].y;
    _pp[idx][3] = _t[idx]*fc_new; // baseline * focal length
    cvConvert(&pp, _P2);

    alpha = MIN(alpha, 1.);

    icvGetRectangles( _cameraMatrix1, _distCoeffs1, _R1, _P1, imageSize, inner1, outer1 );
    icvGetRectangles( _cameraMatrix2, _distCoeffs2, _R2, _P2, imageSize, inner2, outer2 );

    {
    newImgSize = newImgSize.width*newImgSize.height != 0 ? newImgSize : imageSize;
    double cx1_0 = cc_new[0].x;
    double cy1_0 = cc_new[0].y;
    double cx2_0 = cc_new[1].x;
    double cy2_0 = cc_new[1].y;
    double cx1 = newImgSize.width*cx1_0/imageSize.width;
    double cy1 = newImgSize.height*cy1_0/imageSize.height;
    double cx2 = newImgSize.width*cx2_0/imageSize.width;
    double cy2 = newImgSize.height*cy2_0/imageSize.height;
    double s = 1.;

    if( alpha >= 0 )
    {
        double s0 = std::max(std::max(std::max((double)cx1/(cx1_0 - inner1.x), (double)cy1/(cy1_0 - inner1.y)),
                            (double)(newImgSize.width - cx1)/(inner1.x + inner1.width - cx1_0)),
                        (double)(newImgSize.height - cy1)/(inner1.y + inner1.height - cy1_0));
        s0 = std::max(std::max(std::max(std::max((double)cx2/(cx2_0 - inner2.x), (double)cy2/(cy2_0 - inner2.y)),
                         (double)(newImgSize.width - cx2)/(inner2.x + inner2.width - cx2_0)),
                     (double)(newImgSize.height - cy2)/(inner2.y + inner2.height - cy2_0)),
                 s0);

        double s1 = std::min(std::min(std::min((double)cx1/(cx1_0 - outer1.x), (double)cy1/(cy1_0 - outer1.y)),
                            (double)(newImgSize.width - cx1)/(outer1.x + outer1.width - cx1_0)),
                        (double)(newImgSize.height - cy1)/(outer1.y + outer1.height - cy1_0));
        s1 = std::min(std::min(std::min(std::min((double)cx2/(cx2_0 - outer2.x), (double)cy2/(cy2_0 - outer2.y)),
                         (double)(newImgSize.width - cx2)/(outer2.x + outer2.width - cx2_0)),
                     (double)(newImgSize.height - cy2)/(outer2.y + outer2.height - cy2_0)),
                 s1);

        s = s0*(1 - alpha) + s1*alpha;
    }

    fc_new *= s;
    cc_new[0] = cvPoint2D64f(cx1, cy1);
    cc_new[1] = cvPoint2D64f(cx2, cy2);

    cvmSet(_P1, 0, 0, fc_new);
    cvmSet(_P1, 1, 1, fc_new);
    cvmSet(_P1, 0, 2, cx1);
    cvmSet(_P1, 1, 2, cy1);

    cvmSet(_P2, 0, 0, fc_new);
    cvmSet(_P2, 1, 1, fc_new);
    cvmSet(_P2, 0, 2, cx2);
    cvmSet(_P2, 1, 2, cy2);
    cvmSet(_P2, idx, 3, s*cvmGet(_P2, idx, 3));

    if(roi1)
    {
        *roi1 = cv::Rect(cvCeil((inner1.x - cx1_0)*s + cx1),
                     cvCeil((inner1.y - cy1_0)*s + cy1),
                     cvFloor(inner1.width*s), cvFloor(inner1.height*s))
            & cv::Rect(0, 0, newImgSize.width, newImgSize.height);
    }

    if(roi2)
    {
        *roi2 = cv::Rect(cvCeil((inner2.x - cx2_0)*s + cx2),
                     cvCeil((inner2.y - cy2_0)*s + cy2),
                     cvFloor(inner2.width*s), cvFloor(inner2.height*s))
            & cv::Rect(0, 0, newImgSize.width, newImgSize.height);
    }
    }

    if( matQ )
    {
        double q[] =
        {
            1, 0, 0, -cc_new[0].x,
            0, 1, 0, -cc_new[0].y,
            0, 0, 0, fc_new,
            0, 0, 1./_t[idx], //Antonio para mantener compatibilidad con lo que ten’amos de Mossi
            (idx == 0 ? cc_new[0].x - cc_new[1].x : cc_new[0].y - cc_new[1].y)/_t[idx]
        };
        CvMat Q = cvMat(4, 4, CV_64F, q);
        cvConvert( &Q, matQ );
    }
}






void stereoRectify( InputArray _cameraMatrix1, InputArray _distCoeffs1,
                        InputArray _cameraMatrix2, InputArray _distCoeffs2,
                        Size imageSize, InputArray _Rmat, InputArray _Tmat,
                        OutputArray _Rmat1, OutputArray _Rmat2,
                        OutputArray _Pmat1, OutputArray _Pmat2,
                        OutputArray _Qmat, int flags,
                        double alpha, Size newImageSize,
                        Rect* validPixROI1, Rect* validPixROI2 )
{
    Mat cameraMatrix1 = _cameraMatrix1.getMat(), cameraMatrix2 = _cameraMatrix2.getMat();
    Mat distCoeffs1 = _distCoeffs1.getMat(), distCoeffs2 = _distCoeffs2.getMat();
    Mat Rmat = _Rmat.getMat(), Tmat = _Tmat.getMat();
    CvMat c_cameraMatrix1 = cameraMatrix1;
    CvMat c_cameraMatrix2 = cameraMatrix2;
    CvMat c_distCoeffs1 = distCoeffs1;
    CvMat c_distCoeffs2 = distCoeffs2;
    CvMat c_R = Rmat, c_T = Tmat;

    int rtype = CV_64F;
    _Rmat1.create(3, 3, rtype);
    _Rmat2.create(3, 3, rtype);
    _Pmat1.create(3, 4, rtype);
    _Pmat2.create(3, 4, rtype);
    CvMat c_R1 = _Rmat1.getMat(), c_R2 = _Rmat2.getMat(), c_P1 = _Pmat1.getMat(), c_P2 = _Pmat2.getMat();
    CvMat c_Q, *p_Q = 0;

    if( _Qmat.needed() )
    {
        _Qmat.create(4, 4, rtype);
        p_Q = &(c_Q = _Qmat.getMat());
    }

    upvsoft::cvStereoRectify( &c_cameraMatrix1, &c_cameraMatrix2, &c_distCoeffs1, &c_distCoeffs2,
        imageSize, &c_R, &c_T, &c_R1, &c_R2, &c_P1, &c_P2, p_Q, flags, alpha,
        newImageSize, (CvRect*)validPixROI1, (CvRect*)validPixROI2);
}

} // namespace
