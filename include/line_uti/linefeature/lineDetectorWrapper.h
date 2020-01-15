// Line & Junction detection, Matching, and reconstruction
// Author: Ji Zhao
// Date:   12/06/2016
// Email:  zhaoji84@gmail.com
// Shanghai ReadSense Tech., Inc.
// All rights reserved

#ifndef _LINE_DETECTOR_WRAPPER_H
#define _LINE_DETECTOR_WRAPPER_H


#include "LVL/Detection.h" // lvl
#include "LSD/lsd.h"
#include "basicDraw.h"

struct Line
{
	Point2f StartPt;
	Point2f EndPt;
	float lineWidth;
	Point2f Vp;

	Point2f Center;
	Point2f unitDir; // [cos(theta), sin(theta)]
	float length;
	float theta;

	// para_a * x + para_b * y + c = 0
	float para_a;
	float para_b;
	float para_c;

	float image_dx;
	float image_dy;
    float line_grad_avg;

	float xMin;
	float xMax;
	float yMin;
	float yMax;
	unsigned short id;
	int colorIdx;
};

vector<Line> LineDetectLsd(Mat imgGray);

vector<Line> LineDetectEdline(Mat imgGray);

vector<Line> LineDetectLvl(Mat imgGray);

vector<Line> LineDetectSlslam(Mat imgGray);

vector<Line> LineDetectCannyLine(Mat imgGray);

vector<Line> LineDetect_NFA_RNFA(Mat imgGray);

vector<Line> LineDetect_RNFAEdge(Mat imgGray);

Line LineParameters(float x1, float y1, float x2, float y2, float lineWidth = 1.0f);

class lineDetector{
public:
	lineDetector(int lMtd, bool is_roi, float x_min, float x_max, float y_min, float y_max);

	vector<Line> detect(Mat imgGray);

	void removeBoundary(vector<Line>& vecLine);
    void line_grad_discriptor(Mat imgGray, vector<Line>& vecLine);
private:
	int lineMethod;
	bool isROI;
	float xMin;
	float xMax;
	float yMin;
	float yMax;
};

Mat visualizeLineSegment(Mat img, vector<Line> vecLine, string winName, bool isShow = true);

void visualizeLineSegmentTwo(Mat img_1, vector<Line> vecLine1, Mat img_2, vector<Line> vecLine2, string winName, int margin = 0);

void printLineStatistics(Line L);

#endif
