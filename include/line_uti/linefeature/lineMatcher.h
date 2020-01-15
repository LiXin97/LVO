// Line & Junction detection, Matching, and reconstruction
// Author: Ji Zhao
// Date:   12/06/2016
// Email:  zhaoji84@gmail.com
// Shanghai ReadSense Tech., Inc.
// All rights reserved

#ifndef _LINE_MATCHER_H
#define _LINE_MATCHER_H

#include <vector>
#include <map>
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "junctionBasic.h"
#include "junctionDetector.h"
#include "junctionSift.h"
#include "VFC_LPM/vfc.h"
#include "VFC_LPM/lpm.h"

using namespace std;
using namespace cv;

class lineMatcher{
public:
	lineMatcher(int dscpMtd, int matchMtd, bool enRansac, bool isStereo, float minLen);
	void set(map<int, Junction> mapJ1, map<int, Junction> mapJ2,
		Mat im1, Mat im2, vector<KeyPoint> kp1, vector<KeyPoint> kp2, Mat desc1, Mat desc2, int n1, int n2);
	void get(vector<pair<int, int>>& lMatches, vector<pair<int, int>>& jcMatch, Mat& FundMat);

	void lineMatch();
	void juncMatch();

	vector<pair<int, int>> junctionMatcherCore();

	Mat ransacFundMatrixMatch(
		std::vector<cv::KeyPoint> &mvKeys1, std::vector<cv::KeyPoint> &mvKeys2,
		std::vector<DMatch> &matches, std::vector<DMatch> &correctMatches);


	// for customed SIFT descriptors
	vector<pair<int, int>> junctionMatcherCoreSift();

	vector<pair<int, int>> juncInitMatch(
		float* descr1, float* descr2, float distThresh);

	void thresholdMatchForOnePtV1(float* descr1, int i, float* descr2, int nJunc2,
		junctionType type1, junctionType type2,
		vector<pair<int, int>>& siftMatch, float thresh);

	float distanceDescriptor(float* descr1, float* descr2,
		junctionType type1, junctionType type2, float diff_ang_1, float diff_ang_2);

	float distanceDescriptorSimple(float* descr1, float* descr2,
		junctionType type1, junctionType type2, float diff_ang_1, float diff_ang_2);

	Mat ransacFundMatrixMatch(
		vector<junctKeyPt>& vecKeyPts1, vector<junctKeyPt>& vecKeyPts2,
		vector<pair<int, int>>& initMatch, vector<pair<int, int>>& ransacMatch);
	
private:
	// parameters by input
	int descriptorMethod;
	int matchMethod;
	bool enableRansac;
	bool enableTopoFlt;
	bool isStereoPair;
	float minLength;
	// internal parameter
	float featureScale;
	float angleThresh;
	float positionThresh;
	float ransacDist_Orb;
	float ransacProb_Orb;
	float ransacDist_Sift;
	float ransacProb_Sift;
//	cv::Ptr<Feature2D> f2d;
	BFMatcher OrbMatcher;

	// input data
	map<int, Junction> mapJunc1;
	map<int, Junction> mapJunc2;
	Mat img_1;
	Mat img_2;
	int nLine1;
	int nLine2;
	// results
	vector<junctKeyPt> vecKeyPts1;
	vector<junctKeyPt> vecKeyPts2;
	vector<KeyPoint> keypoints1, keypoints2;
	Mat descriptors1, descriptors2;
	Mat F;
	vector<pair<int, int>> juncMatches;
	vector<pair<int, int>> lineMatches;
};

void visualizeJunctionMatch(
	map<int, Junction> mapJunc1, map<int, Junction> mapJunc2,
	vector<pair<int, int>> juncMatch,
	Mat& img_1, Mat& img_2, string winname, int margin = 0, bool isHorizonal = true);

void visualizeJunctionMatch(
	vector<junctKeyPt> vecKeyPts1, vector<junctKeyPt> vecKeyPts2,
	vector<pair<int, int>> juncMatch,
	Mat& img_1, Mat& img_2, string winname, int margin = 0, bool isHorizonal = true);

void SaveFrameLinewithID(vector<Line> vecLine1, vector<int> lineID,
						 Mat& img_1_ori,string name);
void visualizeLinewithID(vector<Line> vecLine1, vector<int> lineID,
						 Mat& img_1_ori,string name);
void visualizeLineTrackCandidate(vector<Line> vecLine1,Mat& img_1_ori,string name);

void visualizeLineMatch(
	vector<Line> vecLine1, vector<Line> vecLine2,
	vector<pair<int, int>> lineMatches,
	Mat& img_1_ori, Mat& img_2_ori, string winname, 
	int margin = 0, bool isHorizonal = true, string str = "");

vector<Point2f> ExtractEndpoints(Junction J1, Junction J2);

void visualizeJunctionMatchDebug(
	map<int, Junction> mapJunc1, map<int, Junction> mapJunc2,
	vector<pair<int, int>> juncMatch,
	Mat& img_1_ori, Mat& img_2_ori, string winname, int margin = 0, bool isHorizonal = true);

void visualizeLineMatchDebug(
	vector<Line> vecLine1, vector<Line> vecLine2,
	vector<pair<int, int>> lineMatches,
	Mat& img_1_ori, Mat& img_2_ori, string winname, int margin = 0, bool isHorizonal = true);

void visualizeEpipolarLine(Mat& img1, Mat& img2,
	Mat F, map<int, Junction> mapJunc1, map<int, Junction> mapJunc2,
	vector<pair<int, int>> juncMatch, string winname);

void visualizeEpipolarLine(Mat& img1, Mat& img2,
	Mat F, vector<junctKeyPt> vecKeyPts1, vector<junctKeyPt> vecKeyPts2,
	vector<pair<int, int>> juncMatch, string winname);

#endif
