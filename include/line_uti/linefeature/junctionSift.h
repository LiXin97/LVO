// Line & Junction detection, Matching, and reconstruction
// Author: Ji Zhao
// Date:   12/06/2016
// Email:  zhaoji84@gmail.com
// Shanghai ReadSense Tech., Inc.
// All rights reserved

#ifndef _JUNCTION_SIFT_H
#define _JUNCTION_SIFT_H

#include <vector>
#include <iostream>
#include "geometry.h"
#include "junctionBasic.h"

#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

//#define SIFT_DEBUG 
#define DIM_DESCRIPTOR 128

void siftExtraction(Mat img, vector<junctKeyPt> vecKeyPts, float* descr, float r = 10.0f);

#endif