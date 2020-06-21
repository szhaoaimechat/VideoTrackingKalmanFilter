/*
 * kalman.cpp
 *
 *  Created on: May 12, 2019
 *      Author: shanshan
 */

#include "blobfuns.hpp"

cv::Point2f kalmanTracking(KalmanFilter &KF, Mat img, Point2f &statePt,
		Point2f measurePt) {
	Point2f predictPt;
	Mat state = Mat::zeros(STATE_SIZE, 1, CV_32F); /* x xv y yv */
	Mat measurement = Mat::zeros(MEASURE_SIZE, 1, CV_32F); //	Mat_<float> measurement(2, 1);

	if (statePt.x == -1) {
		KF.statePost.at<float>(0) = measurePt.x;
		KF.statePost.at<float>(2) = measurePt.y;
		statePt.x = measurePt.x;
		statePt.y = measurePt.y;
		return measurePt;
	}

//	1. PREDICT step:
	KF.predict();
//	2. MEASURE step:
	measurement.at<float>(0) = measurePt.x;
	measurement.at<float>(1) = measurePt.y;
//	3. UPDATE step:

	if (measurePt.x == -1) {
		cv::setIdentity(KF.measurementNoiseCov, cv::Scalar(10000));
	} else {
		cv::setIdentity(KF.measurementNoiseCov, cv::Scalar(0));
	}
	KF.correct(measurement);

	predictPt = Point2f(KF.statePost.at<float>(0), KF.statePost.at<float>(2)); //calcPoint(center, R, predictAngle);

	statePt.x = predictPt.x;
	statePt.y = predictPt.y;
	return predictPt;
}

void plotTracking(Mat img, std::vector<cv::Point2f>& pointlistPre,
		std::vector<cv::Point2f> &pointlistMea) {

#define drawCross( center, color, d )                                        \
		                line( img, Point( center.x - d, center.y - d ),                          \
		                             Point( center.x + d, center.y + d ), color, 1, LINE_AA, 0); \
		                line( img, Point( center.x + d, center.y - d ),                          \
		                             Point( center.x - d, center.y + d ), color, 1, LINE_AA, 0 )

	int Npre = pointlistPre.size();
	int Nmea = pointlistMea.size();
// plot prediction
	for (int i = 0; i < Npre; i++) {
		drawCross(pointlistPre[i], Scalar(0, 255, 0), 5);
		if (i > 0)
			line(img, pointlistPre[i - 1], pointlistPre[i], Scalar(0, 255, 0),
					3, LINE_AA, 0);

	}
// plot measurment
	for (int i = 0; i < Nmea; i++) {
		if (pointlistMea[i].x != -1) {
			drawCross(pointlistMea[i], Scalar(0, 0, 255), 5);
			if (i > 0 & pointlistMea[i-1].x != -1)
				line(img, pointlistMea[i - 1], pointlistMea[i],
						Scalar(0, 0, 255), 2, LINE_AA, 0);
		}

	}
	imshow("Kalman", img);

}

