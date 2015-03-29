#include <iostream>
#include <opencv/cv.h>
#include<vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "package_bgs/pt/PixelBasedAdaptiveSegmenter.h"
#include "package_tracking/BlobTracking.h"
#include "package_analysis/VehicleCouting.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{

	int const numberOfVideos = 50;
	// resize_factor: 50% of original image
	int resize_factor = 100, width = 640, height = 480, key = 0, h, w;

	CvCapture *capture[numberOfVideos] = { 0 };
	IplImage *frame_aux[numberOfVideos];
	IplImage *frame[numberOfVideos];

	/* Background Subtraction Methods */
	IBGS *bgs[numberOfVideos];
	/* Blob Tracking */
	Mat img_blob[numberOfVideos];
	//vector<Mat> img_blob;
	BlobTracking* blobTracking[numberOfVideos];
	/* Vehicle Counting Algorithm */
	VehicleCouting* vehicleCouting[numberOfVideos];
	Mat img_input[numberOfVideos];
	Mat img_mask[numberOfVideos];
	Mat img[numberOfVideos];
	Mat dst(Size(10 * width, 5 * height), CV_8UC3);

	int counter2;
	while (key != 'q')
	{
		h = 0; w = 640, counter2 = -1;
		for (int it = 0; it <= 49; it++){
			char * filename = new char[100];
			int counter = it;
			if (it >= 6)
				counter = 0;
			sprintf(filename, "C:/Users/mohamedtarek/Desktop/GP Project/Dataset_ Myvideos/test%03d.mp4", counter);

			capture[it] = cvCaptureFromFile(filename);
			if (!capture[it]){
				cerr << "Cannot open video!" << endl;
				return 1;
			}


			frame_aux[it] = cvQueryFrame(capture[it]);
			if (!frame_aux[it]) break;

			frame[it] = cvCreateImage(cvSize((int)((frame_aux[it]->width*resize_factor) / 100), (int)((frame_aux[it]->height*resize_factor) / 100)), frame_aux[it]->depth, frame_aux[it]->nChannels);
			frame[it] = frame_aux[it];

			bgs[it] = new PixelBasedAdaptiveSegmenter;
			blobTracking[it] = new BlobTracking;
			vehicleCouting[it] = new VehicleCouting;

			img[it].create(Size(width, height), CV_8UC3);

			++counter2;

			/*** PT Package (adapted from Hofmann) ***/
			img_input[it] = frame[it];
			// bgs->process(...) method internally shows the foreground mask image   
			bgs[it]->process(img_input[it], img_mask[it]);

			if (!img_mask[it].empty())
			{
				// Perform blob tracking
				blobTracking[it]->process(img_input[it], img_mask[it], img_blob[it]);

				if (img_blob[it].data){
					cv::resize(img_blob[it], img_blob[it], img_blob[0].size());
					img[it] = img_blob[it];

					// cout << "dst " << dst->width << " " << dst->height << "\nimg1 " << img1.width << " " << img1.height << "\nimg2 " << img2.width << " " << img2.height << endl;

					resize(img[it], img[it], Size(640, 480));
					//cout << it << "\n";
					if (it != 0 && it % 10 == 0){
						++h;
						counter2 = 0;
					}
					// Copy first image to dst
					Mat imgPanelRoi(dst, Rect(counter2*w, h*img[it].size().height, img[it].size().width, img[it].size().height));
					img[it].copyTo(imgPanelRoi);


					/*IplImage *destination = cvCreateImage(cvSize(1300, 650), dst->depth, dst->nChannels);
					cvResize(dst, destination);*/

					Mat destination;
					destination.create(Size(1300, 650), dst.type());
					resize(dst, destination, destination.size());
					imshow("dst", destination);

				}

				// Perform vehicle counting
				vehicleCouting[it]->setInput(img_blob[0]);
				vehicleCouting[it]->setTracks(blobTracking[0]->getTracks());
				vehicleCouting[it]->process();

			}// end if mask
			waitKey(5);
		} // end inner for loop

	} //end while

	delete vehicleCouting;
	delete blobTracking;
	delete bgs;

	cvDestroyAllWindows();

	for (int i = 0; i <= 49; i++)
		cvReleaseCapture(&capture[i]);

	return 0;
}
