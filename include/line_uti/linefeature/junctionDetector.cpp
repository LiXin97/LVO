// Line & Junction detection, Matching, and reconstruction
// Author: Ji Zhao
// Date:   12/06/2016
// Email:  zhaoji84@gmail.com
// Shanghai ReadSense Tech., Inc.
// All rights reserved

#include "junctionDetector.h"

Mat visualizeJunction(Mat img_ori, map<int, Junction> mJunction, string winName, bool isShow)
{
	//int height = img_ori.rows;
	//int width = img_ori.cols;
	Mat img; // = img_ori.clone();
	// convert original to 3-channel grayscale image
	if (img_ori.channels() == 3){
		cv::cvtColor(img_ori, img, cv::COLOR_BGR2GRAY);
		cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
	}
	else{
		cv::cvtColor(img_ori, img, cv::COLOR_GRAY2BGR);
	}

	int nJunction = mJunction.size();
	for (map<int, Junction>::const_iterator it = mJunction.begin();
		it != mJunction.end(); ++it)
	{
		Junction aJunc = it->second;
		// plot one line segment
		line(img, aJunc.line1.StartPt, aJunc.line1.EndPt, colormap(aJunc.line1.colorIdx), 2, 8); // CV_RGB(0, 255, 0)
		// plot another line segment
		line(img, aJunc.line2.StartPt, aJunc.line2.EndPt, colormap(aJunc.line2.colorIdx), 2, 8); // CV_RGB(0, 255, 0)
		// plot intersection point
		circle(img, aJunc.origin, 3, colormap(rand()), 4, 8, 0);
	}
	if (isShow){
		cout << nJunction << " junction detected" << endl;
		imshow(winName, img);
		waitKey(1);
	}
	return img;
}

void visualizeJunctionTwo(Mat img_1, map<int, Junction> mapJunc1, Mat img_2, map<int, Junction> mapJunc2, string winName, int margin)
{
	Mat img1_tmp = visualizeJunction(img_1, mapJunc1, "", false);
	Mat img2_tmp = visualizeJunction(img_2, mapJunc2, "", false);

	std::vector<cv::Mat> matrices;
	cv::Mat R;

	cv::Mat imgMargin(img_1.rows, margin, CV_8UC3, Scalar(255, 255, 255));
	matrices.push_back(img1_tmp);
	matrices.push_back(imgMargin);
	matrices.push_back(img2_tmp);

	hconcat(matrices, R);
	//putText(R, "left", Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(255, 0, 0), 2);
	//putText(R, "right", Point(10 + img_1.cols + margin, 30), FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(255, 0, 0), 2);
	imshow(winName, R);
	waitKey(1);
}

void visualizeJunctionDebug(Mat img_ori, map<int, Junction> mJunction, string winName)
{
	Mat img0; // = img_ori.clone();
	// convert original to 3-channel grayscale image
	if (img_ori.channels() == 3){
		cv::cvtColor(img_ori, img0, cv::COLOR_BGR2GRAY);
		cv::cvtColor(img0, img0, cv::COLOR_GRAY2BGR);
	}
	else{
		cv::cvtColor(img_ori, img0, cv::COLOR_GRAY2BGR);
	}

	int nJunction = mJunction.size();
	cout << nJunction << " junction detected" << endl;
	int cnt = 0;
	for (map<int, Junction>::const_iterator it = mJunction.begin();
		it != mJunction.end(); ++it)
	{
		Mat img = img0.clone();
		Junction aJunc = it->second;

		bool E1_IS_FAR_POINT = aJunc.E1_IS_FAR_POINT;
		bool E2_IS_FAR_POINT = aJunc.E2_IS_FAR_POINT;
		bool L1_IS_BASE_EDGE = aJunc.L1_IS_BASE_EDGE;
		// plot intersection point
		circle(img, aJunc.origin, 3, CV_RGB(0, 255, 0), 4, 8, 0);
		// plot one line segment
		CvPoint pt_s1(aJunc.line1.StartPt);
		CvPoint pt_e1(aJunc.line1.EndPt);
		CvPoint pt_o(aJunc.origin);

		// base edge is red, another edge is blue
		Scalar L1_color, L2_color;
		if (L1_IS_BASE_EDGE){
			L1_color = CV_RGB(255, 0, 0);
			L2_color = CV_RGB(0, 0, 255);
		}
		else{
			L1_color = CV_RGB(0, 0, 255);
			L2_color = CV_RGB(255, 0, 0);
		}

		if (E1_IS_FAR_POINT){
			circle(img, pt_e1, 3, L1_color, 4, 8, 0);
		}
		else{
			circle(img, pt_s1, 3, L1_color, 4, 8, 0);
		}
		line(img, pt_s1, pt_e1, L1_color, 2, 8);
		

		// plot another line segment
		CvPoint pt_s2(aJunc.line2.StartPt);
		CvPoint pt_e2(aJunc.line2.EndPt);
		if (E2_IS_FAR_POINT){
			circle(img, pt_e2, 3, L2_color, 4, 8, 0);
		}
		else{
			circle(img, pt_s2, 3, L2_color, 4, 8, 0);
		}
		line(img, pt_s2, pt_e2, L2_color, 2, 8);

		string str = junctionTypeString(aJunc.type);
		putText(img, "#"+num2str(cnt) + ": " + str,
			Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2);

		printLineStatistics(aJunc.line1);
		printLineStatistics(aJunc.line2);
		printJunctionStatistics(aJunc);

		imshow(winName, img);
		waitKey(0);
//		imwrite("1_" + num2str(cnt) + ".jpg", img);
		cnt++;
	}
}

string junctionTypeString(junctionType t)
{
	string str;
	if (t == RAY_POINT_RAY){
		str = "ray-point-ray";
	}
	else if (t == T_INTERSECTION){
		str = "T-Junction";
	}
	else if (t == X_INTERSECTION){
		str = "X-Junction";
	}
	return str;
}

void printJunctionStatistics(Junction aJunc)
{
	bool E1_IS_FAR_POINT = aJunc.E1_IS_FAR_POINT;
	bool E2_IS_FAR_POINT = aJunc.E2_IS_FAR_POINT;
	bool L1_IS_BASE_EDGE = aJunc.L1_IS_BASE_EDGE;

	cout << "--------- Junction statistics ---------" << endl;
	cout << "line 1 is base edge: " << L1_IS_BASE_EDGE << endl;
	cout << "E1 is far point    : " << E1_IS_FAR_POINT << endl;
	cout << "E2 is far point    : " << E2_IS_FAR_POINT << endl;
	string str = junctionTypeString(aJunc.type);
	cout << "junction type is   : " << str << endl;
}

junctionDetector::junctionDetector() {
	szWindow = 20.0f;
	angleThresh = 30.0f;
	height = 0;
	width = 0;
	nLine = 0;
	dotProductThresh = fabs( cos(angleThresh * MY_PI / 180.f) );

	descriptorMethod = 0;
	f2d = ORB::create();
	featureScale = 10.0f;
}

junctionDetector::junctionDetector(int dscpMtd)
{
	szWindow = 20.0f;
	angleThresh = 30.0f;
	height = 0;
	width = 0;
	nLine = 0;
	dotProductThresh = fabs(cos(angleThresh * MY_PI / 180.f));

	descriptorMethod = dscpMtd;
	if (descriptorMethod == 0){ // ORB
		f2d = ORB::create();
	}
	featureScale = 10.0f;
}

junctionDetector::~junctionDetector() {

}

void junctionDetector::set(vector<Line> vL, int h, int w){
	vecLine.clear();
	vecLine = vL;
	height = h;
	width = w;
	nLine = vecLine.size();
}

map<int, Junction> junctionDetector::get(){
	return mapJunction;
}

void junctionDetector::detect()
{
	mapJunction.clear();
	if (nLine < 2)
		return;

	float marginBound = szWindow *SQRT_OF_2;

//	int num_dup = 0;
	int cnt_0, cnt_1, cnt_2;
	cnt_0 = cnt_1 = cnt_2  = 0;
	junctionType type;
	int cnt = 0;
	for (int i = 0; i < nLine; i++){
		Line line1 = vecLine[i];
		float length1 = line1.length;
		if (length1 < THRESHOLD_LINE_LENGTH)
			continue;

		Point2f S1 = line1.StartPt;
		Point2f E1 = line1.EndPt;
		float theta1 = line1.theta;
		float x1 = S1.x;
		float y1 = S1.y;
		float x2 = E1.x;
		float y2 = E1.y;
		float cos_theta = line1.unitDir.x;
		float sin_theta = line1.unitDir.y;
		float center_x = line1.Center.x;
		float center_y = line1.Center.y;
		
		float xMin1 = line1.xMin - marginBound;
		float xMax1 = line1.xMax + marginBound;
		float yMin1 = line1.yMin - marginBound;
		float yMax1 = line1.yMax + marginBound;

		for (int j = 0; j < nLine; j++){
			if (j == i)
				continue;

			Line line2 = vecLine[j];
			float length2 = line2.length;
			if (length2 < THRESHOLD_LINE_LENGTH)
				continue;
			Point2f S2 = line2.StartPt;
			Point2f E2 = line2.EndPt;
			float theta2 = line2.theta;
			float x3 = S2.x;
			float y3 = S2.y;
			float x4 = E2.x;
			float y4 = E2.y;
			float xMin2 = line2.xMin;
			float xMax2 = line2.xMax;
			float yMin2 = line2.yMin;
			float yMax2 = line2.yMax;

			// use a quick and coarse method to discard lines 
			//    that are obviously outside of the affect region
			if (xMin2 > xMax1 || xMax2 < xMin1 || yMin2 > yMax1 || yMax2 < yMin1)
				continue;

			float px, py;
			bool flag = intersectionTwoLines(x1, y1, x2, y2, x3, y3, x4, y4, px, py, dotProductThresh);
			// near parallel, or intersection point is outside of the image
			if (!flag || px<0 || px>width-1 || py<0 || py>height-1)
				continue;

			bool L1_IS_BASE_EDGE, E1_IS_FAR_POINT, E2_IS_FAR_POINT;
			float d1 = pointLineSegmentDist(x1, y1, x2, y2, px, py, E1_IS_FAR_POINT);
			// intersection is outside of the affect region
			if (d1 > szWindow)
				continue;

			// avoid duplicate computation and saving
			int key = keyForJunctionRetrieval(nLine, i, j);
			map<int, Junction>::const_iterator it = mapJunction.find(key);
			if (it != mapJunction.end()){ // exist already
//				num_dup++;
				continue;
			}

			// use complex and exact method to remove lines 
			//    that are outside of the affect region
			float tx3, ty3, tx4, ty4;
			transform_RT(cos_theta, sin_theta, center_x, center_y, x3, y3, tx3, ty3);
			transform_RT(cos_theta, sin_theta, center_x, center_y, x4, y4, tx4, ty4);
			if (!lineRectIntersection(length1, szWindow, tx3, ty3, tx4, ty4))
				continue;
			/*
			float tx1, ty1, tx2, ty2;
			transform_RT(cos_theta, sin_theta, center_x, center_y, x1, y1, tx1, ty1);
			transform_RT(cos_theta, sin_theta, center_x, center_y, x2, y2, tx2, ty2);
			if (!nearZero(ty1) || !nearZero(ty2)){
				cout << tx1 << " " << ty1 << endl;
				cout << tx2 << " " << ty2 << endl;
			}
			*/

			// determind the junction type
			float d2 = pointLineSegmentDist(x3, y3, x4, y4, px, py, E2_IS_FAR_POINT);
			
			if (d1 >= 0 && d2 >= 0){
				type = RAY_POINT_RAY;
				coordinateForRayPointRay(theta1, theta2, 
					E1_IS_FAR_POINT, E2_IS_FAR_POINT, L1_IS_BASE_EDGE);
				cnt_0++;
			}
			else if ((d1 < 0 && d2 >= 0) || (d1 >= 0 && d2 < 0)){
				type = T_INTERSECTION;
				coordinateFor_T_Junction(theta1, theta2, d1, d2,
					E1_IS_FAR_POINT, E2_IS_FAR_POINT, L1_IS_BASE_EDGE);
				cnt_1++;
			}
			else if (d1 < 0 && d2 < 0){
				type = X_INTERSECTION;
				coordinateFor_X_Junction(theta1, theta2, d1, d2,
					E1_IS_FAR_POINT, E2_IS_FAR_POINT, L1_IS_BASE_EDGE);
				cnt_2++;
			}

			// other parameters
			Junction oneJunc;
			oneJunc.line1 = line1;
			oneJunc.line2 = line2;
			oneJunc.origin = Point2f(px, py);
			oneJunc.type = type;
			oneJunc.idxLine1 = i;
			oneJunc.idxLine2 = j;
			oneJunc.E1_IS_FAR_POINT = E1_IS_FAR_POINT;
			oneJunc.E2_IS_FAR_POINT = E2_IS_FAR_POINT;
			oneJunc.L1_IS_BASE_EDGE = L1_IS_BASE_EDGE;

			mapJunction.insert(std::pair<int, Junction>(key, oneJunc));

//			printLineStatistics(oneJunc.line1);
//			printLineStatistics(oneJunc.line2);
			cnt++;
		}
	}

//	int nJunc = mapJunction.size();
//	cout << nJunc << " junctions detected" << endl;
//	cout << num_dup << " duplications" << endl;
//	cout << cnt_0 << " RAY_POINT_RAY" << endl;
//	cout << cnt_1 << " T_INTERSECTION" << endl;
//	cout << cnt_2 << " X_INTERSECTION" << endl << endl;

	return;
}

void junctionDetector::runDescriptor(Mat img)
{
	vecKeyPts.clear();
	keypoints.clear();
	if (descriptorMethod == 0){ //ORB
		convertJuncToKeyPts(mapJunction, vecKeyPts);

		for (size_t i = 0; i < vecKeyPts.size(); i++)
		{
			junctKeyPt Pt = vecKeyPts[i];
			float x = Pt.x;
			float y = Pt.y;
			float angle0 = Pt.mid_ang1_ang2;
			KeyPoint a(x, y, featureScale, angle0, 0, 0, i);
			keypoints.push_back(a);
		}

		f2d->compute(img, keypoints, descriptors);
	}
	return;
}

void junctionDetector::getDescriptor(vector<KeyPoint>& kp, Mat& desc)
{
	kp = keypoints;
	desc = descriptors;
	return;
}