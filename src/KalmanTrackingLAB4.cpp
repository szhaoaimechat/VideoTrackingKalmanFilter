//opencv libraries
#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui.hpp"
#include <stdio.h>
#include <iostream>

#include "blobfuns.hpp"
using namespace cv;
static inline Point calcPoint(Point2f center, double R, double angle) {
	return center + Point2f((float) cos(angle), (float) -sin(angle)) * (float) R;
}
static void help() {
	printf(
			"\nExample of c calls to OpenCV's Kalman filter.\n"
					"   Tracking of rotating point.\n"
					"   Rotation speed is constant.\n"
					"   Both state and measurements vectors are 1D (a point angle),\n"
					"   Measurement is the real point angle + gaussian noise.\n"
					"   The real and the estimated points are connected with yellow line segment,\n"
					"   the real and the measured points are connected with red line segment.\n"
					"   (if Kalman filter works correctly,\n"
					"    the yellow segment should be shorter than the red one).\n"
					"\n"
					"   Pressing any key (except ESC) will reset the tracking with a different speed.\n"
					"   Pressing ESC will stop the program.\n");
}
int main(int, char**) {

	Mat frame; // current Frame
	Mat fgmask; // foreground mask
	double learningRate = 0.001;
	std::vector<cvBlob> bloblist; // list for blobs
	std::vector<cv::Point2f> pointlistPre; // list for the predicted points
	std::vector<cv::Point2f> pointlistMea; // list for the measured points

	//MOG2 approach
	Ptr<BackgroundSubtractor> pMOG2 = cv::createBackgroundSubtractorMOG2();

	// video sequence
	std::string inputvideo = "task1_videos_kalman/singleball.mp4";
//	std::string inputvideo = "task1_videos_kalman/video3.mp4";

//	std::string inputvideo = "task1_videos_kalman/video3.mp4"; //path for the videofile to process

	VideoCapture cap; //reader to grab videoframes
	/**************** initialize of Kalman Filter****************/

	KalmanFilter KF(STATE_SIZE, MEASURE_SIZE, 0);
	KF.transitionMatrix =
			(Mat_<float>(4, 4) << 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1);
	KF.measurementMatrix = cv::Mat::zeros(MEASURE_SIZE, STATE_SIZE, CV_32F);
	KF.measurementMatrix.at<float>(0) = 1.0f;
	KF.measurementMatrix.at<float>(6) = 1.0f;
	KF.processNoiseCov.at<float>(0) = 1;
	KF.processNoiseCov.at<float>(5) = 1e-1;
	KF.processNoiseCov.at<float>(10) = 1;
	KF.processNoiseCov.at<float>(15) = 1e-1;
//	cv::setIdentity(KF.processNoiseCov, cv::Scalar(1e-5));// uncomment this for noise scenes
	cv::setIdentity(KF.measurementNoiseCov, cv::Scalar(25));
	Point2f statePt(-1, -1);
	Point2f predictPt;
	/**************** end initialize of Kalman Filter****************/

	//open the videofile to check if it exists
	cap.open(inputvideo);
	if (!cap.isOpened()) {
		std::cout << "Could not open video file " << inputvideo << std::endl;
		return -1;
	}
	cap >> frame; //get first video frame
	imshow("frame", frame);
	int iframe = 0;
	char code = (char) -1;
	for (;;) {
		//get frame & check if we achieved the end of the file (e.g. frame.data is empty)
		cap >> frame;
		if (!frame.data)
			break;
		imshow("frame", frame);

		// update the background model (compute fgmask) slow learning rate for abandoned objects!
		pMOG2->apply(frame, fgmask, learningRate);
//		imshow("fgmask",fgmask);
		// Extract the blobs in fgmask
		extractBlobs(fgmask, bloblist);
		//Paint the blob results
//		imshow("measured position", paintBlobImage(frame, bloblist));
		Point2f measurePt(-1, -1);

		if (bloblist.size() > 0) {
			measurePt.x = bloblist[0].x + bloblist[0].w / 2;
			measurePt.y = bloblist[0].y + bloblist[0].h / 2;
		}

		predictPt = kalmanTracking(KF, frame, statePt, measurePt);

		if (statePt.x != -1) {
			pointlistPre.push_back(predictPt);
			pointlistMea.push_back(measurePt);
			plotTracking(frame, pointlistPre, pointlistMea);
		}

		//exit if ESC key is pressed
		if (waitKey(30) == 27)
			break;
	}

	//release all resource
	cap.release();
	destroyAllWindows();
	return 0;
}
