/* Applied Video Analysis of Sequences (AVSA)
 *
 * Blob-based functions for extraction and painting
 *
 * Author: Juan C. SanMiguel (juancarlos.sanmiguel@uam.es)
 */
#include "blobfuns.hpp"

#define MIN_WIDTH 15
#define MIN_HEIGHT 15

/**
 *	Blob extraction from 1-channel image (binary). The extraction is performed based
 *	on the analysis of the connected components. All the input arguments must be 
 *  initialized when using this function.
 *
 * \param fgmask Foreground/Background segmentation mask (1-channel binary image) 
 * \param bloblist List with found blobs
 *
 * \return Operation code (negative if not succesfull operation) 
 */
int extractBlobs(cv::Mat fgmask, std::vector<cvBlob> &bloblist) {
	//check input conditions and return -1 if any is not satisfied
	//...

	//required variables for connected component analysis 
	Point seed;
	Mat mask;
	Rect ccomp;

	// counter for blob ID
	int blobID = 0;

	//clear blob list (to fill with this function)
	bloblist.clear();
	fgmask.copyTo(mask);

	//Connected component analysis
	//scan the FG mask to find blob pixels (use one of the two following choices)
	for (int x = 0; x < mask.cols; x++)
		for (int y = 0; y < mask.rows; y++) {
			//extract connected component (blob)
			if (mask.at<uchar>(y, x) == 255) {
				seed = Point(x, y);

				floodFill(mask, seed, 0, &ccomp, Scalar(0, 0, 0),
						Scalar(255, 255, 255), 8); // connectivity of 8 neighboring pixels

				//check size of blob using MIN_WIDTH & HEIGHT (valid = true)
				// for singleball video
				if (ccomp.width > MIN_WIDTH && ccomp.height > MIN_HEIGHT)
//				if(ccomp.width >fgmask.cols*0.1 && float(ccomp.height) / float(ccomp.width) > 0.8)// uncomment this in noise videos
				//include blob in 'bloblist' if it is valid
				{
					cvBlob B = initBlob(blobID, ccomp.x, ccomp.y, ccomp.width,
							ccomp.height);
					if (bloblist.size() == 0)
						bloblist.push_back(B);
					else {
						if (bloblist[0].w * bloblist[0].h < B.w * B.h) {
							bloblist.pop_back();
							bloblist.push_back(B);
						}

					}
					blobID++;

				}
			}
		}

	//destroy all resources
	//...

	//return OK code
	return 1;
}

/**
 *	Draws blobs with different rectangles on the image 'frame'. All the input arguments must be 
 *  initialized when using this function.
 *
 * \param frame Input image 
 * \param pBlobList List to store the blobs found 
 *
 * \return Image containing the draw blobs. If no blobs have to be painted 
 *  or arguments are wrong, the function returns a copy of the original "frame".  
 *
 */
Mat paintBlobImage(cv::Mat frame, std::vector<cvBlob> &bloblist) {
	cv::Mat blobImage;
	frame.copyTo(blobImage);

	//check input conditions and return NULL if any is not satisfied
	if (bloblist.size() == 0)
		return blobImage;

	//required variables to paint
	Rect rect2draw;
	CvPoint center;
	cvBlob blob;
	//paint each blob of the list
	for (int i = 0; i < (int) bloblist.size(); i++) {
		blob = bloblist[i]; //get ith blob
		rect2draw = Rect(blob.x, blob.y, blob.w, blob.h);
		rectangle(blobImage, rect2draw, Scalar(0, 0, 255));
		center.x = blob.x + blob.w / 2;
		center.y = blob.y + blob.h / 2;
		int radius = blob.w / 8;
		int thickness = 8;
		circle(blobImage, center, radius, Scalar(100, 100, 100), thickness, 1,
				0);
	}

	//destroy all resources (if required)
	//...

	//return the image to show
	return blobImage;
}
