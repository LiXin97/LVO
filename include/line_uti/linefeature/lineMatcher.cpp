// Line & Junction detection, Matching, and reconstruction
// Author: Ji Zhao
// Date:   12/06/2016
// Email:  zhaoji84@gmail.com
// Shanghai ReadSense Tech., Inc.
// All rights reserved

#include "lineMatcher.h"

// when the orientation adds 90/180/270 degrees,
// we can re-arrange the descriptors to avoid repeat calculation
// refer: http://www.vlfeat.org/api/sift.html
int idx_0_deg[128] = {
	0, 1, 2, 3, 4, 5, 6, 7, 
	8, 9, 10, 11, 12, 13, 14, 15, 
	16, 17, 18, 19, 20, 21, 22, 23, 
	24, 25, 26, 27, 28, 29, 30, 31, 
	32, 33, 34, 35, 36, 37, 38, 39, 
	40, 41, 42, 43, 44, 45, 46, 47, 
	48, 49, 50, 51, 52, 53, 54, 55, 
	56, 57, 58, 59, 60, 61, 62, 63, 
	64, 65, 66, 67, 68, 69, 70, 71, 
	72, 73, 74, 75, 76, 77, 78, 79, 
	80, 81, 82, 83, 84, 85, 86, 87, 
	88, 89, 90, 91, 92, 93, 94, 95, 
	96, 97, 98, 99, 100, 101, 102, 103, 
	104, 105, 106, 107, 108, 109, 110, 111, 
	112, 113, 114, 115, 116, 117, 118, 119, 
	120, 121, 122, 123, 124, 125, 126, 127};

int idx_90_deg[128] ={
	26, 27, 28, 29, 30, 31, 24, 25,
	58, 59, 60, 61, 62, 63, 56, 57, 
	90, 91, 92, 93, 94, 95, 88, 89, 
	122, 123, 124, 125, 126, 127, 120, 121, 
	18, 19, 20, 21, 22, 23, 16, 17, 
	50, 51, 52, 53, 54, 55, 48, 49, 
	82, 83, 84, 85, 86, 87, 80, 81, 
	114, 115, 116, 117, 118, 119, 112, 113, 
	10, 11, 12, 13, 14, 15, 8, 9, 
	42, 43, 44, 45, 46, 47, 40, 41, 
	74, 75, 76, 77, 78, 79, 72, 73, 
	106, 107, 108, 109, 110, 111, 104, 105, 
	2, 3, 4, 5, 6, 7, 0, 1, 
	34, 35, 36, 37, 38, 39, 32, 33, 
	66, 67, 68, 69, 70, 71, 64, 65, 
	98, 99, 100, 101, 102, 103, 96, 97};

int idx_180_deg[128] = {
	124, 125, 126, 127, 120, 121, 122, 123, 
	116, 117, 118, 119, 112, 113, 114, 115, 
	108, 109, 110, 111, 104, 105, 106, 107, 
	100, 101, 102, 103, 96, 97, 98, 99, 
	92, 93, 94, 95, 88, 89, 90, 91, 
	84, 85, 86, 87, 80, 81, 82, 83, 
	76, 77, 78, 79, 72, 73, 74, 75, 
	68, 69, 70, 71, 64, 65, 66, 67, 
	60, 61, 62, 63, 56, 57, 58, 59, 
	52, 53, 54, 55, 48, 49, 50, 51, 
	44, 45, 46, 47, 40, 41, 42, 43, 
	36, 37, 38, 39, 32, 33, 34, 35, 
	28, 29, 30, 31, 24, 25, 26, 27, 
	20, 21, 22, 23, 16, 17, 18, 19, 
	12, 13, 14, 15, 8, 9, 10, 11, 
	4, 5, 6, 7, 0, 1, 2, 3};

int idx_270_deg[128] = {
	102, 103, 96, 97, 98, 99, 100, 101, 
	70, 71, 64, 65, 66, 67, 68, 69, 
	38, 39, 32, 33, 34, 35, 36, 37, 
	6, 7, 0, 1, 2, 3, 4, 5, 
	110, 111, 104, 105, 106, 107, 108, 109, 
	78, 79, 72, 73, 74, 75, 76, 77, 
	46, 47, 40, 41, 42, 43, 44, 45, 
	14, 15, 8, 9, 10, 11, 12, 13, 
	118, 119, 112, 113, 114, 115, 116, 117, 
	86, 87, 80, 81, 82, 83, 84, 85, 
	54, 55, 48, 49, 50, 51, 52, 53, 
	22, 23, 16, 17, 18, 19, 20, 21, 
	126, 127, 120, 121, 122, 123, 124, 125, 
	94, 95, 88, 89, 90, 91, 92, 93, 
	62, 63, 56, 57, 58, 59, 60, 61, 
	30, 31, 24, 25, 26, 27, 28, 29};

void vfcMatch(std::vector<cv::KeyPoint> keypoints_1, std::vector<cv::KeyPoint> keypoints_2,
	std::vector<DMatch> matches, std::vector<DMatch> &vfcMatches)
{
	// preprocess data format
	vector<Point2f> X;
	vector<Point2f> Y;
	for (unsigned int i = 0; i < matches.size(); i++) {
		int idx1 = matches[i].queryIdx;
		int idx2 = matches[i].trainIdx;
		X.push_back(keypoints_1[idx1].pt);
		Y.push_back(keypoints_2[idx2].pt);
	}
	// main process
	VFC myvfc;
	myvfc.setData(X, Y);
	myvfc.optimize();
	vector<int> matchIdx = myvfc.obtainCorrectMatch();

	// postprocess data format

	for (unsigned int i = 0; i < matchIdx.size(); i++) {
		int idx = matchIdx[i];
		vfcMatches.push_back(matches[idx]);
	}

	return;
}

void lpmMatch(std::vector<cv::KeyPoint> keypoints_1, std::vector<cv::KeyPoint> keypoints_2,
	std::vector<DMatch> matches, std::vector<DMatch> &correctMatches)
{
	float lambda1 = 6.0f;
	int numNeigh1 = 4;
	float lambda2 = 4.0f;
	int numNeigh2 = 4;
	// preprocess data format
	int numPoint = matches.size();
	if (numPoint < numNeigh1 || numPoint < numNeigh2)
		return;

	float* X = new float[numPoint * 2];
	float* Y = new float[numPoint * 2];
	bool* p = new bool[numPoint * 2];
	for (unsigned int i = 0; i < matches.size(); i++) {
		int idx1 = matches[i].queryIdx;
		int idx2 = matches[i].trainIdx;
		X[i * 2] = keypoints_1[idx1].pt.x;
		X[i * 2 + 1] = keypoints_1[idx1].pt.y;
		Y[i * 2] = keypoints_2[idx2].pt.x;
		Y[i * 2 + 1] = keypoints_2[idx2].pt.y;
	}

	LPMatching(X, Y, p, lambda1, numNeigh1, lambda2, numNeigh2, numPoint);

	// postprocess data format
	correctMatches.clear();
	for (int i = 0; i < numPoint; i++) {
		if (p[i]){
			correctMatches.push_back(matches[i]);
		}
	}
	delete[] X;
	delete[] Y;
	delete[] p;
}

bool pairCompare(const pair<pair<int, int>, float>& firstElem, const pair<pair<int, int>, float>& secondElem) {
	return firstElem.second > secondElem.second;
}

lineMatcher::lineMatcher(int dscpMtd, int matchMtd, bool enRansac, bool isStereo, float minLen)
{
	descriptorMethod = dscpMtd;
	matchMethod = matchMtd;
	enableRansac = enRansac;
	isStereoPair = isStereo;
	minLength = minLen;

	if (descriptorMethod == 0){ // ORB
//		f2d = ORB::create();
		OrbMatcher = BFMatcher(NORM_HAMMING, true);
	}

	featureScale = 10.0f;
	angleThresh = MY_PI / 6;
	positionThresh = 3.0f;
	ransacDist_Orb = 1.0f;
	ransacProb_Orb = 0.9999f;
	ransacDist_Sift = 3.0f;
	ransacProb_Sift = 0.999f;

	enableTopoFlt = false;
	juncMatches.clear();
	lineMatches.clear();
}

void lineMatcher::set(map<int, Junction> mapJ1, map<int, Junction> mapJ2, 
	Mat im1, Mat im2, vector<KeyPoint> kp1, vector<KeyPoint> kp2, Mat desc1, Mat desc2, int n1, int n2)
{
	mapJunc1 = mapJ1;
	mapJunc2 = mapJ2;
	img_1 = im1;
	img_2 = im2;
	nLine1 = n1;
	nLine2 = n2;

	vecKeyPts1.clear();
	vecKeyPts2.clear();
	convertJuncToKeyPts(mapJunc1, vecKeyPts1);
	convertJuncToKeyPts(mapJunc2, vecKeyPts2);

	descriptors1 = desc1;
	descriptors2 = desc2;
	keypoints1 = kp1;
	keypoints2 = kp2;

	juncMatches.clear();
	lineMatches.clear();
	return;
}

void lineMatcher::get(vector<pair<int, int>>& lMatches, vector<pair<int, int>>& jcMatch, Mat& FundMat)
{
	FundMat = F;
	jcMatch = juncMatches;
	lMatches = lineMatches;
}

void lineMatcher::lineMatch()
{
	juncMatch();

	int nPutatives = juncMatches.size();
	map<int, Junction>::const_iterator it;
	vector<pair<pair<int,int>, float>> lineMatchAffinty;
	for (int i = 0; i < nPutatives; i++)
	{
		int idx1 = juncMatches[i].first;
		int idx2 = juncMatches[i].second;

		int key1 = vecKeyPts1[idx1].key;
		it = mapJunc1.find(key1);
		//		if (key1 != it->first)
		//			cout << "error!" << endl;
		Junction J1 = it->second;

		int key2 = vecKeyPts2[idx2].key;
		it = mapJunc2.find(key2);
		Junction J2 = it->second;

		// putative line match (line 1, line 2) and their total length
		// remove short line segments
		if (J1.L1_IS_BASE_EDGE == J2.L1_IS_BASE_EDGE){
			if (J1.line1.length >= minLength || J2.line1.length >=minLength)
				lineMatchAffinty.push_back(make_pair(make_pair(J1.idxLine1, J2.idxLine1), J1.line1.length + J2.line1.length));
			if (J1.line2.length >= minLength || J2.line2.length >= minLength)
				lineMatchAffinty.push_back(make_pair(make_pair(J1.idxLine2, J2.idxLine2), J1.line2.length + J2.line2.length));
		}
		else{
			if (J1.line1.length >= minLength || J2.line2.length >= minLength)
				lineMatchAffinty.push_back(make_pair(make_pair(J1.idxLine1, J2.idxLine2), J1.line1.length + J2.line2.length));
			if (J1.line2.length >= minLength || J2.line1.length >= minLength)
				lineMatchAffinty.push_back(make_pair(make_pair(J1.idxLine2, J2.idxLine1), J1.line2.length + J2.line1.length));
		}
	}

	// longer line segments first
	std::sort(lineMatchAffinty.begin(), lineMatchAffinty.end(), pairCompare);
	bool* indicator1 = new bool[nLine1];
	bool* indicator2 = new bool[nLine2];
	for (int i = 0; i < nLine1; i++)
		indicator1[i] = false;
	for (int i = 0; i < nLine2; i++)
		indicator2[i] = false;
	for (size_t i = 0; i < lineMatchAffinty.size(); i++){
		pair<int, int> aMatch = lineMatchAffinty[i].first;
		int idx_left = aMatch.first;
		int idx_right = aMatch.second;
		if (indicator1[idx_left] == false && indicator2[idx_right] == false){
			lineMatches.push_back(aMatch);
			indicator1[idx_left] = true;
			indicator2[idx_right] = true;
//			cout << idx_left << " " << idx_right << endl;
		}
	}
	delete[] indicator1;
	delete[] indicator2;

	return;
}

void lineMatcher::juncMatch()
{
	vector<pair<int, int>> juncMatchOri;
	if (descriptorMethod == 0){ // ORB
		juncMatchOri = junctionMatcherCore();
	}
	else if (descriptorMethod == 1){ // SIFT
		juncMatchOri = junctionMatcherCoreSift();
	}

	juncMatches.clear();
	int nPutatives = juncMatchOri.size();
	map<int, Junction>::const_iterator it;
	for (int i = 0; i < nPutatives; i++)
	{
		int idx1 = juncMatchOri[i].first;
		int idx2 = juncMatchOri[i].second;
		junctKeyPt pt1 = vecKeyPts1[idx1];
		junctKeyPt pt2 = vecKeyPts2[idx2];
/*
		int key1 = vecKeyPts1[idx1].key;
		it = mapJunc1.find(key1);
		if (key1 != it->first)
			cout << "error!" << endl;
		Junction J1 = it->second;

		int key2 = vecKeyPts1[idx2].key;
		it = mapJunc1.find(key2);
		if (key2 != it->first)
			cout << "error!" << endl;
		Junction J2 = it->second;
*/
		if (isStereoPair){
			float d_posy = abs(pt1.y - pt2.y);
			if (d_posy > positionThresh)
				continue;
		}

		float d_ang1 = abs(pt1.angle1 - pt2.angle1);
		float d_ang2 = abs(pt1.angle2 - pt2.angle2);
		
		if (d_ang1 <= angleThresh && d_ang2 <= angleThresh)
			juncMatches.push_back(juncMatchOri[i]);
	}
	return;
}


vector<pair<int, int>> lineMatcher::junctionMatcherCore()
{
	vector<pair<int, int>> initMatch, ransacMatch;

	// ORB
//	double t = (double)getTickCount();
//	vector<KeyPoint> keypoints_1, keypoints_2;
//	Mat descriptors_1, descriptors_2;
	vector<DMatch> matches, vfcMatches, ransacMatches;
/*
	for (int i = 0; i < vecKeyPts1.size(); i++)
	{
		junctKeyPt Pt = vecKeyPts1[i];
		float x = Pt.x;
		float y = Pt.y;
		float angle0 = Pt.mid_ang1_ang2;
		KeyPoint a(x, y, featureScale, angle0, 0, 0, i);
		keypoints_1.push_back(a);
	}
	for (int i = 0; i < vecKeyPts2.size(); i++)
	{
		junctKeyPt Pt = vecKeyPts2[i];
		float x = Pt.x;
		float y = Pt.y;
		float angle0 = Pt.mid_ang1_ang2;
		KeyPoint a(x, y, featureScale, angle0, 0, 0, i);
		keypoints_2.push_back(a);
	}
*/
//	cv::Ptr<Feature2D> f2d = ORB::create();

	// test only
	//		keypoints_1.clear();
	//		keypoints_2.clear();
	//		f2d->detect(img_1, keypoints_1);
	//		f2d->detect(img_2, keypoints_2);
	//-- Step 2: Calculate descriptors (feature vectors)
//	f2d->compute(img_1, keypoints_1, descriptors_1);
//	f2d->compute(img_2, keypoints_2, descriptors_2);
//	t = 1000 * ((double)getTickCount() - t) / getTickFrequency();
//	cout << "ORB extraction time (ms): " << t << endl;

	//-- Step 3: Matching descriptor vectors with a brute force matcher
//	t = (double)getTickCount();
//	BFMatcher OrbMatcher(NORM_HAMMING, true);
	OrbMatcher.match(descriptors1, descriptors2, matches);
//	t = 1000 * ((double)getTickCount() - t) / getTickFrequency();
//	cout << "ORB matching time (ms): " << t << endl;

	// vector field consensus
	if (matchMethod == 0){ // vfc
		vfcMatch(keypoints1, keypoints2, matches, vfcMatches);
	}
	else if (matchMethod == 1){ // lpm
		lpmMatch(keypoints1, keypoints2, matches, vfcMatches);
	}
	else{
		vfcMatches = matches;
	}

	// RANSAC
//	t = (double)getTickCount();
	if (enableRansac){
		F = ransacFundMatrixMatch(keypoints1, keypoints2,	vfcMatches, ransacMatches);
	}
	else{
		ransacMatches = vfcMatches;
	}
	// convert vector<DMatch> to vector<pair<int, int>>
	int nPutatives = ransacMatches.size();
	for (int i = 0; i < nPutatives; i++) {
		int idx1 = ransacMatches[i].queryIdx;
		int idx2 = ransacMatches[i].trainIdx;
		// In orb->compute, Keypoints for which a descriptor cannot be computed are removed.
		idx1 = keypoints1[idx1].class_id;
		idx2 = keypoints2[idx2].class_id;
		ransacMatch.push_back(make_pair(idx1, idx2));
	}
//	t = 1000 * ((double)getTickCount() - t) / getTickFrequency();
//	cout << "RANSAC matching time (ms): " << t << endl;
	//		Mat img_matches;
	//		drawMatches(img_1, keypoints1, img_2, keypoints2, matches, img_matches);
	//		imshow("orb matches", img_matches);


	return ransacMatch;
};

Mat lineMatcher::ransacFundMatrixMatch(std::vector<cv::KeyPoint> &mvKeys1, std::vector<cv::KeyPoint> &mvKeys2,
	std::vector<DMatch> &matches, std::vector<DMatch> &correctMatches)
{
	Mat F;
	if (mvKeys1.size() < 8 || matches.size() < 8)
		return F;

	//	double t = (double)getTickCount();

	vector<Point2f>imgpts1, imgpts2;
	for (size_t i = 0; i < matches.size(); i++)
	{
		// queryIdx is the "left" image
		imgpts1.push_back(mvKeys1[matches[i].queryIdx].pt);
		// trainIdx is the "right" image
		imgpts2.push_back(mvKeys2[matches[i].trainIdx].pt);
	}
	vector<uchar> status;
	F = findFundamentalMat(imgpts1, imgpts2, CV_FM_RANSAC, ransacDist_Orb, ransacProb_Orb, status);

	// extract the surviving (inliers) matches
	for (size_t i = 0; i < status.size(); i++){
		if (status[i]){
			correctMatches.push_back(matches[i]);
		}
	}

	//	t = 1000 * ((double)getTickCount() - t) / getTickFrequency();
	//	cout << "RANSAC time (ms): " << t << endl;
	return F;
}

vector<pair<int, int>> lineMatcher::junctionMatcherCoreSift()
{
	vector<pair<int, int>> initMatch, ransacMatch;

	// SIFT feature extraction
//	double t = (double)getTickCount();
	float* descr1 = new float[DIM_DESCRIPTOR * vecKeyPts1.size()];
	float* descr2 = new float[DIM_DESCRIPTOR * vecKeyPts2.size()];
	siftExtraction(img_1, vecKeyPts1, descr1, featureScale);
	siftExtraction(img_2, vecKeyPts2, descr2, featureScale);
//	t = 1000 * ((double)getTickCount() - t) / getTickFrequency();
//	std::cout << "SIFT extraction time (ms): " << t << endl;

	// SIFT init matching
//	t = (double)getTickCount();
	float distThresh = 0.15f;
	initMatch = juncInitMatch(descr1, descr2, distThresh);
	delete[] descr1;
	delete[] descr2;
//	t = 1000 * ((double)getTickCount() - t) / getTickFrequency();
//	std::cout << "SIFT matching time (ms): " << t << endl;

	// ransac refined matching
	if (enableRansac){
//		t = (double)getTickCount();
		F = ransacFundMatrixMatch(vecKeyPts1, vecKeyPts2, initMatch, ransacMatch);
//		t = 1000 * ((double)getTickCount() - t) / getTickFrequency();
//		cout << "RANSAC matching time (ms): " << t << endl;
	}
	else{
		ransacMatch = initMatch;
	}
	return ransacMatch;
}

vector<pair<int, int>> lineMatcher::juncInitMatch(
	float* descr1, float* descr2, float distThresh)
{
	vector<pair<int, int>> siftMatch;

	int nJunc1 = vecKeyPts1.size();
	int nJunc2 = vecKeyPts2.size();
	if (nJunc1 < 1 || nJunc2 < 1)
		return siftMatch;
	
//#define _USE_SIMPLE_METHOD

#ifdef _USE_SIMPLE_METHOD
	for (int i = 0; i < nJunc1; i++){
		junctionType type1 = vecKeyPts1[i].type;
		junctionType type2 = vecKeyPts2[i].type;
		thresholdMatchForOnePtV1(descr1, i, descr2, nJunc2, type1, type2, siftMatch, distThresh);
	}
#else
	for (int i = 0; i < nJunc1; i++)
	{
		junctKeyPt P1 = vecKeyPts1[i];
		junctionType type1 = P1.type;

		for (int j = 0; j < nJunc2; j++){
			junctKeyPt P2 = vecKeyPts2[j];
			junctionType type2 = P2.type;

//			float d = distanceDescriptor(
//				descr1 + i*DIM_DESCRIPTOR, descr2+j*DIM_DESCRIPTOR,
//				type1, type2, P1.dif_ang1_ang2, P2.dif_ang1_ang2);
			float d = distanceDescriptorSimple(
				descr1 + i*DIM_DESCRIPTOR, descr2+j*DIM_DESCRIPTOR,
				type1, type2, P1.dif_ang1_ang2, P2.dif_ang1_ang2);
			if (d < distThresh){
				siftMatch.push_back(make_pair(i, j));
			}
		}
	}
#endif
	return siftMatch;
}

float lineMatcher::distanceDescriptor(float* descr1, float* descr2,
	junctionType type1, junctionType type2, float diff_ang_1, float diff_ang_2)
{
	float s = FLT_MAX;

	int N1(0), N2(0);
	if (type1 == RAY_POINT_RAY)
		N1 = 1;
	else if (type1 == T_INTERSECTION)
		N1 = 2;
	else if (type1 == X_INTERSECTION)
		N1 = 4;

	if (type2 == RAY_POINT_RAY)
		N2 = 1;
	else if (type2 == T_INTERSECTION)
		N2 = 2;
	else if (type2 == X_INTERSECTION)
		N2 = 4;
	
	float dif1, dif2;
	int *p1 = NULL;
	int *p2 = NULL;
	for (int i = 0; i < N1; i++){
		if (i == 0){
			dif1 = diff_ang_1;
			p1 = idx_0_deg;
		}
		else if (i == 1){
			dif1 = MY_PI - diff_ang_1;
			p1 = idx_90_deg;
		}
		else if (i == 2){
			dif1 = diff_ang_1;
			p1 = idx_180_deg;
		}
		else if (i == 3){
			dif1 = MY_PI - diff_ang_1;
			p1 = idx_270_deg;
		}
		
		float s_tmp = 0;
		for (int j = 0; j < N2; j++){
			if (j == 0){
				dif2 = diff_ang_2;
				p2 = idx_0_deg;
			}
			else if (i == 1){
				dif2 = MY_PI - diff_ang_2;
				p2 = idx_90_deg;
			}
			else if (i == 2){
				dif2 = diff_ang_2;
				p2 = idx_180_deg;
			}
			else if (i == 3){
				dif2 = MY_PI - diff_ang_2;
				p2 = idx_270_deg;
			}

			if ((i > 0 && j > 0) && (i == j))
				continue;
			if (fabs(dif1 - dif2) > MY_PI / 6.0f)
				continue;

			for (int k = 0; k < DIM_DESCRIPTOR; k++){
				//int k1, k2;
				//k1 = *(p1 + k);
				//k2 = *(p2 + k);

				float v1 = *(descr1 + k);
				float v2 = *(descr2 + k);
				float d = v1 - v2;
				s_tmp += d*d;
			}
			s = min(s, s_tmp);
		}
	}


	return s;
}

float lineMatcher::distanceDescriptorSimple(float* descr1, float* descr2,
	junctionType type1, junctionType type2, float diff_ang_1, float diff_ang_2)
{
	if (fabs(diff_ang_1 - diff_ang_2) > MY_PI / 6.0f)
		return FLT_MAX;

	float s = 0;
	for (int k = 0; k < DIM_DESCRIPTOR; k++){
		float v1 = *(descr1 + k);
		float v2 = *(descr2 + k);
		float d = v1 - v2;
		s += d*d;
	}
	return s;
}

void lineMatcher::thresholdMatchForOnePtV1(float* descr1, int i, float* descr2, int nJunc2,
	junctionType type1, junctionType type2, 
	vector<pair<int, int>>& siftMatch, float thresh)
{
	int p1 = DIM_DESCRIPTOR * i;
	int p2 = 0;
	for (int j = 0; j < nJunc2; j++){
		float s = 0;
		for (int k = 0; k < DIM_DESCRIPTOR; k++){
			float v1 = *(descr1 + p1 + k);
			float v2 = *(descr2 + p2 + k);
			float d = v1 - v2;
			s += d*d;
		}
		if (s < thresh){
			siftMatch.push_back(make_pair(i, j));
		}
		p2 += DIM_DESCRIPTOR;
	}
}

Mat lineMatcher::ransacFundMatrixMatch(
	vector<junctKeyPt>& vecKeyPts1, vector<junctKeyPt>& vecKeyPts2,
	vector<pair<int, int>>& initMatch, vector<pair<int, int>>& ransacMatch)
{
	Mat F;
	if (vecKeyPts1.size()<8 || vecKeyPts2.size() < 8 || initMatch.size() < 8){
		cout << "do not have enough matches" << endl;
		ransacMatch = initMatch;
		return F;
	}
	// prepare data for RANSAC
	//vectors to store the coordinates of the feature points
	vector<Point2f> points1, points2;
	for (unsigned int i = 0; i < initMatch.size(); i++) {
		int idx1 = initMatch[i].first;
		int idx2 = initMatch[i].second;
		points1.push_back(Point2f(vecKeyPts1[idx1].x, vecKeyPts1[idx1].y));
		points2.push_back(Point2f(vecKeyPts2[idx2].x, vecKeyPts2[idx2].y));
	}
	Mat mask;
	F = findFundamentalMat(points1, points2, CV_FM_RANSAC, ransacDist_Sift, ransacProb_Sift, mask);
	for (size_t i = 0; i < initMatch.size(); i++){
		if (mask.at<uchar>(i, 0)){
			ransacMatch.push_back(initMatch[i]);
		}
	}
	//	for (int i = 0; i < mask.rows; i++)
	//		cout << int( mask.at<uchar>(i, 0) ) << endl;
	return F;
}


void visualizeJunctionMatch(
	map<int, Junction> mapJunc1, map<int, Junction> mapJunc2,
	vector<pair<int, int>> juncMatch,
	Mat& img_1, Mat& img_2, string winname, int margin, bool isHorizonal)
{
	vector<junctKeyPt> vecKeyPts1, vecKeyPts2;
	convertJuncToKeyPts(mapJunc1, vecKeyPts1);
	convertJuncToKeyPts(mapJunc2, vecKeyPts2);

	visualizeJunctionMatch(vecKeyPts1, vecKeyPts2, 
		juncMatch, img_1, img_2, winname, margin, isHorizonal);
	return;
}

void visualizeJunctionMatch(
	vector<junctKeyPt> vecKeyPts1, vector<junctKeyPt> vecKeyPts2,
	vector<pair<int, int>> juncMatch,
	Mat& img_1_ori, Mat& img_2_ori, string winname, int margin, bool isHorizonal)
{
	Mat img_1, img_2;
	if (img_1_ori.channels() != 3){
		cv::cvtColor(img_1_ori, img_1, cv::COLOR_GRAY2BGR);
	}
	else{
		img_1 = img_1_ori;
	}
	if (img_2_ori.channels() != 3){
		cv::cvtColor(img_2_ori, img_2, cv::COLOR_GRAY2BGR);
	}
	else{
		img_2 = img_2_ori;
	}

	int h = img_1.rows;
	int w = img_1.cols;
	Mat img_matches;
//	int margin = 20; // unit: pixel

	int nPutatives = juncMatch.size();
	cout << nPutatives << " putative matched junctions" << endl;
	if (isHorizonal){
		img_matches.create(h, w * 2 + margin, CV_8UC3);
		img_matches.setTo(255);
		img_1.copyTo(img_matches(cv::Rect_<int>(0, 0, w, h)));
		img_2.copyTo(img_matches(cv::Rect_<int>(w + margin, 0, w, h)));
		for (int i = 0; i < nPutatives; i++)
		{
			int idx1 = juncMatch[i].first;
			int idx2 = juncMatch[i].second;

			float x1 = vecKeyPts1[idx1].x;
			float y1 = vecKeyPts1[idx1].y;
			float x2 = vecKeyPts2[idx2].x + w + margin;
			float y2 = vecKeyPts2[idx2].y;

			CvPoint pt1(int(x1 + 0.5f), int(y1 + 0.5f));
			CvPoint pt2(int(x2 + 0.5f), int(y2 + 0.5f));
			circle(img_matches, pt1, 3, CV_RGB(255, 0, 0), 4, 8, 0);
			circle(img_matches, pt2, 3, CV_RGB(255, 0, 0), 4, 8, 0);
			line(img_matches, pt1, pt2, colormap(rand()), 2, 8); // CV_RGB(0, 255, 0)
		}
	}
	else{ // vertical
		img_matches.create(h*2 + margin, w, CV_8UC3);
		img_matches.setTo(255);
		img_1.copyTo(img_matches(cv::Rect_<int>(0, 0, w, h)));
		img_2.copyTo(img_matches(cv::Rect_<int>(0, h + margin, w, h)));
		for (int i = 0; i < nPutatives; i++)
		{
			int idx1 = juncMatch[i].first;
			int idx2 = juncMatch[i].second;

			float x1 = vecKeyPts1[idx1].x;
			float y1 = vecKeyPts1[idx1].y;
			float x2 = vecKeyPts2[idx2].x;
			float y2 = vecKeyPts2[idx2].y + h + margin;

			CvPoint pt1(int(x1 + 0.5f), int(y1 + 0.5f));
			CvPoint pt2(int(x2 + 0.5f), int(y2 + 0.5f));
			circle(img_matches, pt1, 3, CV_RGB(255, 0, 0), 4, 8, 0);
			circle(img_matches, pt2, 3, CV_RGB(255, 0, 0), 4, 8, 0);
			line(img_matches, pt1, pt2, colormap(rand()), 2, 8); // CV_RGB(0, 255, 0)
		}
	}
	imshow(winname, img_matches);
	waitKey(1);
	return;
}


void visualizeLineTrackCandidate(vector<Line> vecLine1,Mat& img_1_ori,string name)
{
	Mat img_1;
	if (img_1_ori.channels() != 3){
		cv::cvtColor(img_1_ori, img_1, cv::COLOR_GRAY2BGR);
	}
	else{
		img_1 = img_1_ori;
	}

	for (size_t i = 0; i < vecLine1.size(); i++)
	{

		Line aLine1 = vecLine1[i];
		Scalar color = colormap(i+1);

		line(img_1, aLine1.StartPt, aLine1.EndPt, color, 2, 8);
		putText(img_1, num2str(i), aLine1.Center, FONT_HERSHEY_PLAIN, 1, color, 1, 8, false);
	}
	imshow(name, img_1);
	waitKey(1);
	return;

}

void visualizeLinewithID(vector<Line> vecLine1, vector<int> lineID,
						 Mat& img_1_ori,string name)
{
	Mat img_1;
	if (img_1_ori.channels() != 3){
		cv::cvtColor(img_1_ori, img_1, cv::COLOR_GRAY2BGR);
	}
	else{
		img_1 = img_1_ori;
	}

	for (size_t i = 0; i < vecLine1.size(); i++)
	{

		Line aLine1 = vecLine1[i];
		int id = lineID[i];
		Scalar color = colormap(id);

		line(img_1, aLine1.StartPt, aLine1.EndPt, color, 2, 8);

        cv::Point2f dxdy;
        if(aLine1.StartPt.x > aLine1.EndPt.x)  // 统一直线方向的计算都是从右到左做差，不然有的计算出来是dxdy(1,1),翻过来可能就是（-1,-1）,会影响后续计算
        {
            dxdy = aLine1.StartPt - aLine1.EndPt;
        } else{
            dxdy = aLine1.EndPt - aLine1.StartPt;
        }
        cv::Point2f delta_xy = 4 * dxdy/std::sqrt(dxdy.dot(dxdy));   // two pixel along perpendicular line
        float dx = delta_xy.x;
        float dy = delta_xy.y;

        cv::Point2f c_prependicular_point1 = aLine1.Center+ Point2f(-dy, dx);
        cv::Point2f c_prependicular_point2 = aLine1.Center+ Point2f(dy, -dx);
        //std::cout <<"prependicuar: "<<std::endl;
        //std::cout<< c_prependicular_point1<<" "<< aLine1.Center <<std::endl;
        if(c_prependicular_point1.x < img_1.cols && c_prependicular_point1.x > 0 &&
           c_prependicular_point1.y < img_1.rows && c_prependicular_point1.y > 0 &&
           c_prependicular_point2.x < img_1.cols && c_prependicular_point2.x > 0 &&
           c_prependicular_point2.y < img_1.rows && c_prependicular_point2.y > 0)
        {
            //std::cout << c_prependicular_point2 <<std::endl;
            line(img_1, c_prependicular_point1, c_prependicular_point2, color, 2, 8);
            line(img_1, c_prependicular_point1 + 2 * delta_xy, c_prependicular_point2 + 2 * delta_xy, color, 2, 8);

        }

		putText(img_1, num2str(id), aLine1.Center, FONT_HERSHEY_PLAIN, 1, color, 1, 8, false);
	}
	imshow(name, img_1);
	waitKey(1);
	return;

}

void SaveFrameLinewithID(vector<Line> vecLine1, vector<int> lineID,
                         Mat& img_1_ori,string name)
{
    Mat img_1;
    if (img_1_ori.channels() != 3){
        cv::cvtColor(img_1_ori, img_1, cv::COLOR_GRAY2BGR);
    }
    else{
        img_1 = img_1_ori;
    }

    for (size_t i = 0; i < vecLine1.size(); i++)
    {

        Line aLine1 = vecLine1[i];
        int id = lineID[i];
        Scalar color = colormap(id);

        line(img_1, aLine1.StartPt, aLine1.EndPt, color, 2, 8);

        cv::Point2f dxdy;
        if(aLine1.StartPt.x > aLine1.EndPt.x)  // 统一直线方向的计算都是从右到左做差，不然有的计算出来是dxdy(1,1),翻过来可能就是（-1,-1）,会影响后续计算
        {
            dxdy = aLine1.StartPt - aLine1.EndPt;
        } else{
            dxdy = aLine1.EndPt - aLine1.StartPt;
        }
        cv::Point2f delta_xy = 4 * dxdy/std::sqrt(dxdy.dot(dxdy));   // two pixel along perpendicular line
        float dx = delta_xy.x;
        float dy = delta_xy.y;

        cv::Point2f c_prependicular_point1 = aLine1.Center+ Point2f(-dy, dx);
        cv::Point2f c_prependicular_point2 = aLine1.Center+ Point2f(dy, -dx);
        //std::cout <<"prependicuar: "<<std::endl;
        //std::cout<< c_prependicular_point1<<" "<< aLine1.Center <<std::endl;
        if(c_prependicular_point1.x < img_1.cols && c_prependicular_point1.x > 0 &&
           c_prependicular_point1.y < img_1.rows && c_prependicular_point1.y > 0 &&
           c_prependicular_point2.x < img_1.cols && c_prependicular_point2.x > 0 &&
           c_prependicular_point2.y < img_1.rows && c_prependicular_point2.y > 0)
        {
            //std::cout << c_prependicular_point2 <<std::endl;
            line(img_1, c_prependicular_point1, c_prependicular_point2, color, 2, 8);
            line(img_1, c_prependicular_point1 + 2 * delta_xy, c_prependicular_point2 + 2 * delta_xy, color, 2, 8);

        }

        putText(img_1, num2str(id), aLine1.Center, FONT_HERSHEY_PLAIN, 1, color, 1, 8, false);
    }
    imwrite(name, img_1);
    waitKey(1);
    return;

}

void visualizeLineMatch(
	vector<Line> vecLine1, vector<Line> vecLine2,
	vector<pair<int, int>> lineMatches,
	Mat& img_1_ori, Mat& img_2_ori, string winname, 
	int margin, bool isHorizonal, string str)
{
	Mat img_1, img_2;
	if (img_1_ori.channels() != 3){
		cv::cvtColor(img_1_ori, img_1, cv::COLOR_GRAY2BGR);
	}
	else{
		img_1 = img_1_ori;
	}
	if (img_2_ori.channels() != 3){
		cv::cvtColor(img_2_ori, img_2, cv::COLOR_GRAY2BGR);
	}
	else{
		img_2 = img_2_ori;
	}

	int h = img_1.rows;
	int w = img_1.cols;
	Mat img_matches;

	int nPutatives = lineMatches.size();
	cout << nPutatives << " putative matched lines" << endl;
//	map<int, Junction>::const_iterator it;

	img_matches.create(h, w * 2 + margin, CV_8UC3);
	img_matches.setTo(255);
	img_1.copyTo(img_matches(cv::Rect_<int>(0, 0, w, h)));
	img_2.copyTo(img_matches(cv::Rect_<int>(w + margin, 0, w, h)));

	Point2f offset = Point2f(w + margin, 0);
	for (int i = 0; i < nPutatives; i++)
	{
		int idx1 = lineMatches[i].first;
		int idx2 = lineMatches[i].second;

		Line aLine1 = vecLine1[idx1];
		Line aLine2 = vecLine2[idx2];
		Scalar color = colormap(aLine1.colorIdx);
		
		line(img_matches, aLine1.StartPt, aLine1.EndPt, color, 2, 8);
		line(img_matches, aLine2.StartPt + offset, aLine2.EndPt + offset, color, 2, 8);
		putText(img_matches, num2str(i), aLine1.Center, FONT_HERSHEY_PLAIN, 1, color, 1, 8, false);
		putText(img_matches, num2str(i), aLine2.Center + offset, FONT_HERSHEY_PLAIN, 1, color, 1, 8, false);
	}
	putText(img_matches, str, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2);
	imshow(winname, img_matches);
	waitKey(1);
	return;
}

vector<Point2f> ExtractEndpoints(Junction J1, Junction J2)
{
	Point2f L1, L2, L3, L4;;
	Point2f R1, R2, R3, R4;
	bool f;
	// left image
	f = J1.L1_IS_BASE_EDGE;
	if (f){
		L1 = J1.line1.StartPt;
		L2 = J1.line1.EndPt;
		L3 = J1.line2.StartPt;
		L4 = J1.line2.EndPt;
	}
	else{
		L1 = J1.line2.StartPt;
		L2 = J1.line2.EndPt;
		L3 = J1.line1.StartPt;
		L4 = J1.line1.EndPt;
	}
	// right image
	f = J2.L1_IS_BASE_EDGE;
	if (f){
		R1 = J2.line1.StartPt;
		R2 = J2.line1.EndPt;
		R3 = J2.line2.StartPt;
		R4 = J2.line2.EndPt;
	}
	else{
		R1 = J2.line2.StartPt;
		R2 = J2.line2.EndPt;
		R3 = J2.line1.StartPt;
		R4 = J2.line1.EndPt;
	}
	vector<Point2f> V;
	V.push_back(L1);
	V.push_back(L2);
	V.push_back(R1);
	V.push_back(R2);
	V.push_back(L3);
	V.push_back(L4);
	V.push_back(R3);
	V.push_back(R4);
	return V;
}

void visualizeJunctionMatchDebug(
	map<int, Junction> mapJunc1, map<int, Junction> mapJunc2,
	vector<pair<int, int>> juncMatch,
	Mat& img_1_ori, Mat& img_2_ori, string winname, int margin, bool isHorizonal)
{
	Mat img_1, img_2;
	if (img_1_ori.channels() != 3){
		cv::cvtColor(img_1_ori, img_1, cv::COLOR_GRAY2BGR);
	}
	else{
		img_1 = img_1_ori;
	}
	if (img_2_ori.channels() != 3){
		cv::cvtColor(img_2_ori, img_2, cv::COLOR_GRAY2BGR);
	}
	else{
		img_2 = img_2_ori;
	}

	int h = img_1.rows;
	int w = img_1.cols;
	Mat img0;

	vector<junctKeyPt> vecKeyPts1, vecKeyPts2;
	convertJuncToKeyPts(mapJunc1, vecKeyPts1);
	convertJuncToKeyPts(mapJunc2, vecKeyPts2);

	int nPutatives = juncMatch.size();
	map<int, Junction>::const_iterator it;

	img0.create(h, w * 2 + margin, CV_8UC3);
	img0.setTo(255);
	img_1.copyTo(img0(cv::Rect_<int>(0, 0, w, h)));
	img_2.copyTo(img0(cv::Rect_<int>(w + margin, 0, w, h)));

	Point2f offset = Point2f(w + margin, 0);
	double thresh = 0.0f;
	for (int i = 0; i < nPutatives; i++)
	{
		Mat img_matches = img0.clone();

		int idx1 = juncMatch[i].first;
		int idx2 = juncMatch[i].second;

		int key1 = vecKeyPts1[idx1].key;
		it = mapJunc1.find(key1);
		//		if (key1 != it->first)
		//			cout << "error!" << endl;
		Junction J1 = it->second;

		int key2 = vecKeyPts2[idx2].key;
		it = mapJunc2.find(key2);
		Junction J2 = it->second;

		// plot junction
		float x1 = vecKeyPts1[idx1].x;
		float y1 = vecKeyPts1[idx1].y;
		float x2 = vecKeyPts2[idx2].x + w + margin;
		float y2 = vecKeyPts2[idx2].y;

		CvPoint pt1(int(x1 + 0.5f), int(y1 + 0.5f));
		CvPoint pt2(int(x2 + 0.5f), int(y2 + 0.5f));
		circle(img_matches, pt1, 3, CV_RGB(0, 255, 0), 4, 8, 0);
		circle(img_matches, pt2, 3, CV_RGB(0, 255, 0), 4, 8, 0);
		line(img_matches, pt1, pt2, CV_RGB(128, 128, 128), 2, 8); // colormap(rand())

		// plot line segments
		vector<Point2f> V = ExtractEndpoints(J1, J2);
		
		if (cv::norm(cv::Mat(V[0]), cv::Mat(V[1]))>thresh)
			line(img_matches, V[0], V[1], CV_RGB(255, 0, 0), 2, 8);
		if (cv::norm(cv::Mat(V[4]), cv::Mat(V[5]))>thresh)
			line(img_matches, V[4], V[5], CV_RGB(0, 0, 255), 2, 8);
		if (cv::norm(cv::Mat(V[2]), cv::Mat(V[3]))>thresh)
			line(img_matches, V[2] + offset, V[3] + offset, CV_RGB(255, 0, 0), 2, 8);
		if (cv::norm(cv::Mat(V[6]), cv::Mat(V[7]))>thresh)
			line(img_matches, V[6] + offset, V[7] + offset, CV_RGB(0, 0, 255), 2, 8);

		imshow(winname, img_matches);
		waitKey(0);
	}
	return;
}


void visualizeLineMatchDebug(
	vector<Line> vecLine1, vector<Line> vecLine2,
	vector<pair<int, int>> lineMatches,
	Mat& img_1_ori, Mat& img_2_ori, string winname, int margin, bool isHorizonal)
{
	Mat img_1, img_2;
	if (img_1_ori.channels() != 3){
		cv::cvtColor(img_1_ori, img_1, cv::COLOR_GRAY2BGR);
	}
	else{
		img_1 = img_1_ori;
	}
	if (img_2_ori.channels() != 3){
		cv::cvtColor(img_2_ori, img_2, cv::COLOR_GRAY2BGR);
	}
	else{
		img_2 = img_2_ori;
	}

	int h = img_1.rows;
	int w = img_1.cols;
	Mat img0;

	int nPutatives = lineMatches.size();
	map<int, Junction>::const_iterator it;

	img0.create(h, w * 2 + margin, CV_8UC3);
	img0.setTo(255);
	img_1.copyTo(img0(cv::Rect_<int>(0, 0, w, h)));
	img_2.copyTo(img0(cv::Rect_<int>(w + margin, 0, w, h)));

	Point2f offset = Point2f(w + margin, 0);
	for (int i = 0; i < nPutatives; i++)
	{
		Mat img_matches = img0.clone();

		int idx1 = lineMatches[i].first;
		int idx2 = lineMatches[i].second;

		line(img_matches, vecLine1[idx1].StartPt, vecLine1[idx1].EndPt, CV_RGB(255, 0, 0), 2, 8);
		line(img_matches, vecLine2[idx2].StartPt + offset, vecLine2[idx2].EndPt + offset, CV_RGB(255, 0, 0), 2, 8);
		imshow(winname, img_matches);
		waitKey(0);
	}
	return;
}

void visualizeEpipolarLine(Mat& img1, Mat& img2,
	Mat F, map<int, Junction> mapJunc1, map<int, Junction> mapJunc2,
	vector<pair<int, int>> juncMatch, string winname)
{
	vector<junctKeyPt> vecKeyPts1, vecKeyPts2;
	convertJuncToKeyPts(mapJunc1, vecKeyPts1);
	convertJuncToKeyPts(mapJunc2, vecKeyPts2);
	visualizeEpipolarLine(img1, img2, F, vecKeyPts1, vecKeyPts2, juncMatch, winname);
	return;
}

void visualizeEpipolarLine(Mat& img1, Mat& img2, 
	Mat F, vector<junctKeyPt> vecKeyPts1, vector<junctKeyPt> vecKeyPts2, 
	vector<pair<int, int>> juncMatch, string winname)
{
	if (F.cols != 3 || F.rows != 3 ||
		vecKeyPts1.size() == 0 || vecKeyPts2.size() == 0 || juncMatch.size() == 0)
	{
		return;
	}

	vector<Point2f> points1, points2;
	points1.clear();
	points2.clear();
	for (unsigned int i = 0; i < juncMatch.size(); i++) {
		int idx1 = juncMatch[i].first;
		int idx2 = juncMatch[i].second;
		points1.push_back(Point2f(vecKeyPts1[idx1].x, vecKeyPts1[idx1].y));
		points2.push_back(Point2f(vecKeyPts2[idx2].x, vecKeyPts2[idx2].y));
	}

	// Compute corresponding epipolar lines
	std::vector<cv::Vec3f> lines1;
	if (F.cols == 3 && F.rows == 3){
		cv::computeCorrespondEpilines(
			cv::Mat(points1), // image points
			1, // in image 1 (can also be 2)
			F, // F matrix
			lines1); // vector of epipolar lines
	}
	// for all epipolar lines
	Mat outImg1, outImg2;
	if (img1.type() == CV_8U)
		cvtColor(img1, outImg1, COLOR_GRAY2BGR);
	else
		img1.copyTo(outImg1);

	if (img2.type() == CV_8U)
		cvtColor(img2, outImg2, COLOR_GRAY2BGR);
	else
		img2.copyTo(outImg2);

	for (std::vector<cv::Vec3f>::const_iterator it = lines1.begin(); it != lines1.end(); ++it)
	{
		// Draw the line between first and last column
		cv::line(outImg2,
			cv::Point(0, -(*it)[2] / (*it)[1]),
			cv::Point(outImg2.cols, -((*it)[2] + (*it)[0] * outImg2.cols) / (*it)[1]),
			cv::Scalar(0, 255, 0));
	}
	imshow(winname, outImg2);
	waitKey(1);
	return;
}