// Line & Junction detection, Matching, and reconstruction
// Author: Ji Zhao
// Date:   12/06/2016
// Email:  zhaoji84@gmail.com
// Shanghai ReadSense Tech., Inc.
// All rights reserved

#ifndef _JUNCTION_DETECTOR_H
#define _JUNCTION_DETECTOR_H

#include <vector>
#include <map>
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "geometry.h"
#include "lineDetectorWrapper.h"
#include "junctionBasic.h"
#include "junctionSift.h"

using namespace std;
using namespace cv;

#define THRESHOLD_LINE_LENGTH 6

inline int keyForJunctionRetrieval(int nLine, int i, int j){
	return min(i, j) * nLine + max(i, j);
}

Mat visualizeJunction(Mat img, map<int, Junction> mJunction, string winName, bool isShow = true);

void visualizeJunctionTwo(Mat img_1, map<int, Junction> mapJunc1, Mat img_2, map<int, Junction> mapJunc2, string winName, int margin=0);

void visualizeJunctionDebug(Mat img, map<int, Junction> mJunction, string winName);

string junctionTypeString(junctionType t);

void printJunctionStatistics(Junction J);

class junctionDetector{
public:
	junctionDetector();
	junctionDetector(int dscpMtd);
	~junctionDetector();

	void set(vector<Line> vL, int h, int w);
	map<int, Junction> get();
	void detect();
	void runDescriptor(Mat img);
	void getDescriptor(vector<KeyPoint>& kp, Mat& desc);

private:
	float szWindow;
	float angleThresh; // angles below this threshold is discard
	float dotProductThresh;  // cos(angleThresh)
	int height;
	int width;
	int nLine;
	int nJunc;
	vector<Line> vecLine;
	map<int, Junction> mapJunction;

	// keypoints and descriptors
	int descriptorMethod;
	cv::Ptr<Feature2D> f2d;
	float featureScale;
	vector<junctKeyPt> vecKeyPts;
	vector<KeyPoint> keypoints;
	Mat descriptors;
};


#endif