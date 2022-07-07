#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

void distort(const cv::Mat& src, cv::Mat& dst, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs){

	cv::Mat distort_x = cv::Mat(src.size(), CV_32F);
	cv::Mat distort_y = cv::Mat(src.size(), CV_32F);

	cv::Mat pixel_locations_src = cv::Mat(1,src.total(), CV_32FC2);

	int t=0;
	for (int i = 0; i < src.size().height; i++) {
		for (int j = 0; j < src.size().width; j++,t++) {
			pixel_locations_src.at<cv::Point2f>(0,t) = cv::Point2f(j,i);
		}
	}

	cv::Mat fractional_locations_dst; // = cv::Mat(src.size(), CV_32FC2);

	cv::undistortPoints(pixel_locations_src, fractional_locations_dst, cameraMatrix, distCoeffs);

	cv::Mat pixel_locations_dst = cv::Mat(src.size(), CV_32FC2);

	const float fx = cameraMatrix.at<double>(0,0);
	const float fy = cameraMatrix.at<double>(1,1);
	const float cx = cameraMatrix.at<double>(0,2);
	const float cy = cameraMatrix.at<double>(1,2);

	// is there a faster way to do this?
	t=0;
	for (int i = 0; i < fractional_locations_dst.size().height; i++) {
		for (int j = 0; j < fractional_locations_dst.size().width; j++,t++) {
			const float x = fractional_locations_dst.at<cv::Point2f>(0,t).x*fx + cx;
			const float y = fractional_locations_dst.at<cv::Point2f>(0,t).y*fy + cy;
			pixel_locations_dst.at<cv::Point2f>(i,j) = cv::Point2f(x,y);
		}
	}

	std::vector<cv::Mat> channels(pixel_locations_dst.channels());
	split(pixel_locations_dst,channels);

	cv::remap(src, dst, channels[0], channels[1],  cv::INTER_LINEAR);
}
