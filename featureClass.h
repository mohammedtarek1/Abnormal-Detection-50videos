#pragma once
#include<opencv\cv.h>
//#include "BlobTracking.h"
#include "package_tracking\BlobTracking.h"

class featureClass
{
	static int id;

public:
	const int ID;
	bool abnormal;
	int size;
	char* direction;
	float speed;
	cvb::CvTrack drawTrack;
	featureClass() : ID(++id){
		abnormal = 0;
		speed = 0;
		direction = " ";
	}
};

