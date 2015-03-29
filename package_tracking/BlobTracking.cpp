#include "BlobTracking.h"

BlobTracking::BlobTracking() : firstTime(true), minArea(500), maxArea(20000), debugTrack(false), debugBlob(false), showBlobMask(false), showOutput(true)
{
	std::cout << "BlobTracking()" << std::endl;
}

BlobTracking::~BlobTracking()
{
	std::cout << "~BlobTracking()" << std::endl;
}

const cvb::CvTracks BlobTracking::getTracks()
{
	return tracks;
}

void BlobTracking::process(const cv::Mat &img_input, const cv::Mat &img_mask, cv::Mat &img_output)
{
	if (img_input.empty() || img_mask.empty())
		return;

	loadConfig();

	if (firstTime)
		saveConfig();

	IplImage* frame = new IplImage(img_input);
	cvConvertScale(frame, frame, 1, 0);

	IplImage* segmentated = new IplImage(img_mask);

	IplConvKernel* morphKernel = cvCreateStructuringElementEx(5, 5, 1, 1, CV_SHAPE_RECT, NULL);
	cvMorphologyEx(segmentated, segmentated, NULL, morphKernel, CV_MOP_OPEN, 1);

	//msh byd5ol el if hna 
	if (showBlobMask)
		cvShowImage("Blob Mask-000", segmentated);

	IplImage* labelImg = cvCreateImage(cvGetSize(frame), IPL_DEPTH_LABEL, 1);

	/// A map is used to access each blob from its label number.
	/// \see CvLabel
	/// \see CvBlob
	cvb::CvBlobs blobs;
	//d bta5od el black and wihte image ely haya el segmented ..w w bta5od lableImg fadya .. w bta5do blobs fadya 
	//f haya t2rebn b t assign el blobs lable w t7ot el bobs b el lable bta3ha f el labelImg
	unsigned int result = cvb::cvLabel(segmentated, labelImg, blobs);

	//cvb::cvFilterByArea(blobs, 500, 1000000);
	//hna bams7 el blobs ely area bta3tha bara el range ely ana m7ddo ely hoa minArea, maxArea
	cvb::cvFilterByArea(blobs, minArea, maxArea);

	if (debugBlob)
		cvb::cvRenderBlobs(labelImg, blobs, frame, frame, CV_BLOB_RENDER_BOUNDING_BOX | CV_BLOB_RENDER_CENTROID | CV_BLOB_RENDER_ANGLE | CV_BLOB_RENDER_TO_STD);
	else
		cvb::cvRenderBlobs(labelImg, blobs, frame, frame, CV_BLOB_RENDER_BOUNDING_BOX | CV_BLOB_RENDER_CENTROID | CV_BLOB_RENDER_ANGLE);

	cvb::cvUpdateTracks(blobs, tracks, 200., 5);

	if (debugTrack)
		cvb::cvRenderTracks(tracks, frame, frame, CV_TRACK_RENDER_ID | CV_TRACK_RENDER_BOUNDING_BOX | CV_TRACK_RENDER_TO_STD);
	else
		cvb::cvRenderTracks(tracks, frame, frame, CV_TRACK_RENDER_ID | CV_TRACK_RENDER_BOUNDING_BOX);

	//std::map<CvID, CvTrack *> CvTracks
	
	/*if (showOutput)
		cvShowImage("Blob Tracking", frame);*/

	cv::Mat img_result(frame);
	img_result.copyTo(img_output);

	//cvReleaseImage(&frame);
	//cvReleaseImage(&segmentated);
	cvReleaseImage(&labelImg);
	delete frame;
	delete segmentated;
	cvReleaseBlobs(blobs);
	cvReleaseStructuringElement(&morphKernel);

	firstTime = false;
}

void BlobTracking::saveConfig()
{
	CvFileStorage* fs = cvOpenFileStorage("./config/BlobTracking.xml", 0, CV_STORAGE_WRITE);

	cvWriteInt(fs, "minArea", minArea);
	cvWriteInt(fs, "maxArea", maxArea);

	cvWriteInt(fs, "debugTrack", debugTrack);
	cvWriteInt(fs, "debugBlob", debugBlob);
	cvWriteInt(fs, "showBlobMask", showBlobMask);
	cvWriteInt(fs, "showOutput", showOutput);

	cvReleaseFileStorage(&fs);
}

void BlobTracking::loadConfig()
{
	CvFileStorage* fs = cvOpenFileStorage("./config/BlobTracking.xml", 0, CV_STORAGE_READ);

	minArea = cvReadIntByName(fs, 0, "minArea", 500);
	maxArea = cvReadIntByName(fs, 0, "maxArea", 20000);

	debugTrack = cvReadIntByName(fs, 0, "debugTrack", false);
	debugBlob = cvReadIntByName(fs, 0, "debugBlob", false);
	showBlobMask = cvReadIntByName(fs, 0, "showBlobMask", false);
	showOutput = cvReadIntByName(fs, 0, "showOutput", true);

	cvReleaseFileStorage(&fs);
}