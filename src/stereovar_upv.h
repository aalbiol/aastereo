/*
 * stereovar_upv.h
 *
 *  Created on: Feb 10, 2012
 *      Author: aalbiol
 */

#ifndef STEREOVAR_UPV_H_
#define STEREOVAR_UPV_H_


namespace upvsoft
{


    class  StereoVar
    {
    public:
        // Flags
        enum {USE_INITIAL_DISPARITY = 1, USE_EQUALIZE_HIST = 2, USE_SMART_ID = 4, USE_AUTO_PARAMS = 8, USE_MEDIAN_FILTERING = 16};
        enum {CYCLE_O, CYCLE_V};
        enum {PENALIZATION_TICHONOV, PENALIZATION_CHARBONNIER, PENALIZATION_PERONA_MALIK};

        //! the default constructor
         StereoVar();

        //! the full constructor taking all the necessary algorithm parameters
         StereoVar(int levels, double pyrScale, int nIt, int minDisp, int maxDisp, int poly_n, double poly_sigma, float fi, float lambda, int penalization, int cycle, int flags);

        //! the destructor
        virtual ~StereoVar();

        //! the stereo correspondence operator that computes disparity map for the specified rectified stereo pair
        virtual void operator()(const cv::Mat& left, const cv::Mat& right, cv::Mat& disp);

         int		levels;
         double	pyrScale;
         int		nIt;
         int		minDisp;
         int		maxDisp;
         int		poly_n;
         double	poly_sigma;
         float	fi;
         float	lambda;
         int		penalization;
         int		cycle;
         int		flags;

    private:
        void autoParams();
		void FMG(cv::Mat &I1, cv::Mat &I2, cv::Mat &I2x, cv::Mat &u, int level);
        void VCycle_MyFAS(cv::Mat &I1_h, cv::Mat &I2_h, cv::Mat &I2x_h, cv::Mat &u_h, int level);
        void VariationalSolver(cv::Mat &I1_h, cv::Mat &I2_h, cv::Mat &I2x_h, cv::Mat &u_h, int level);
    };

    float rectify3Collinear( cv::InputArray _cameraMatrix1, cv::InputArray _distCoeffs1,
                       cv::InputArray _cameraMatrix2, cv::InputArray _distCoeffs2,
                       cv::InputArray _cameraMatrix3, cv::InputArray _distCoeffs3,
                       cv::InputArrayOfArrays _imgpt1,
                       cv::InputArrayOfArrays _imgpt3,
                       cv::Size imageSize, cv::InputArray _Rmat12, cv::InputArray _Tmat12,
                       cv::InputArray _Rmat13, cv::InputArray _Tmat13,
                       cv::OutputArray _Rmat1, cv::OutputArray _Rmat2, cv::OutputArray _Rmat3,
                       cv::OutputArray _Pmat1, cv::OutputArray _Pmat2, cv::OutputArray _Pmat3,
                       cv::OutputArray _Qmat,
                       double alpha, cv::Size newImgSize,
                       cv::Rect* roi1, cv::Rect* roi2, int flags );



    void stereoRectify( cv::InputArray _cameraMatrix1, cv::InputArray _distCoeffs1,
    		cv::InputArray _cameraMatrix2, cv::InputArray _distCoeffs2,
    		cv::Size imageSize, cv::InputArray _Rmat, cv::InputArray _Tmat,
    		cv::OutputArray _Rmat1, cv::OutputArray _Rmat2,
    		cv::OutputArray _Pmat1, cv::OutputArray _Pmat2,
    		cv::OutputArray _Qmat, int flags,
    		double alpha, cv::Size newImageSize,
    		cv::Rect* validPixROI1, cv::Rect* validPixROI2 );
}
//    CV_EXPORTS void polyfit(const Mat& srcx, const Mat& srcy, Mat& dst, int order);

#endif /* STEREOVAR_UPV_H_ */
