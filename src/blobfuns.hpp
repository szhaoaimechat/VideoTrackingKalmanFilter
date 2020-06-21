/* Applied Video Analysis of Sequences (AVSA)
 *
 * Headers of blob-based functions
 *
 * Author: Juan C. SanMiguel (juancarlos.sanmiguel@uam.es)
 */

#ifndef BLOBFUNS_H_INCLUDE
#define BLOBFUNS_H_INCLUDE
#define STATE_SIZE 4
#define MEASURE_SIZE 2

#include "basicblob.hpp"

//blob extraction functions (included in 'blobextract.cpp')
int extractBlobs(Mat fgmask, std::vector<cvBlob> &bloblist);
Mat paintBlobImage(Mat frame, std::vector<cvBlob> &bloblist);
cv::Point2f kalmanTracking(KalmanFilter &KF, Mat img, Point2f & statePt,
		Point2f measurePt);
void plotTracking(Mat img, std::vector<cv::Point2f> &pointlistPre,
		std::vector<cv::Point2f>& pointlistMea);
#endif
