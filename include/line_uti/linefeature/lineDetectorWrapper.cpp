// Line & Junction detection, Matching, and reconstruction
// Author: Ji Zhao
// Date:   12/06/2016
// Email:  zhaoji84@gmail.com
// Shanghai ReadSense Tech., Inc.
// All rights reserved

#include "lineDetectorWrapper.h"

vector<Line> LineDetectLsd(Mat imgGray)
{
    int width, height;
    height = imgGray.rows;
    width = imgGray.cols;

    // data format conversion: Mat --> LSD format
    double *image = (double *)malloc(height * width * sizeof(double));
    double *out;
    int cnt = 0;
    for (int i = 0; i < height; i++) {
        unsigned char *p = imgGray.ptr<unsigned char>(i);
        for (int j = 0; j < width; j++) {
            image[cnt] = double(p[j]);
            cnt++;
        }
    }
    int nLines;
    out = lsd(&nLines, image, width, height);
    //	cout << nLines << " line segments found" << endl;

    // convert LSD format to customized format,
    // and record parameters for line segments
    unsigned short nCount = 0;
    vector<Line> vecLines;
    for (int i = 0; i < nLines; i++)
    {
        float x1 = static_cast<float>(out[7 * i]);
        float y1 = static_cast<float>(out[7 * i + 1]);
        float x2 = static_cast<float>(out[7 * i + 2]);
        float y2 = static_cast<float>(out[7 * i + 3]);
        float lineWidth = static_cast<float>(out[7 * i + 4]);

        Line aLine = LineParameters(x1, y1, x2, y2, lineWidth);
        aLine.id = nCount;

        vecLines.push_back(aLine);
        nCount++;
    }

    /* free memory */
    free((void *)image);
    free((void *)out);
    return vecLines;

}

vector<Line> LineDetectEdline(Mat imgGray)
{
	// Sorry for that I removed this line detect method, by heyijia
	vector<Line> vecLines;
	return vecLines;
}


vector<Line> LineDetectSlslam(Mat imgGray)
{
	// Sorry for that I removed this line detect method, by heyijia
	vector<Line> vecLines;
	return vecLines;
}

vector<Line> LineDetectCannyLine(Mat imgGray)
{
	// Sorry for that I removed this line detect method, by heyijia
	vector<Line> vecLines;
	return vecLines;
}

vector<Line> LineDetect_NFA_RNFA(Mat imgGray)
{
	// Sorry for that I removed this line detect method, by heyijia
	vector<Line> vecLines;
	return vecLines;
}

vector<Line> LineDetect_RNFAEdge(Mat imgGray)
{
	// Sorry for that I removed this line detect method, by heyijia
	vector<Line> vecLines;
	return vecLines;
}

vector<Line> LineDetectLvl(Mat imgGray)
{
	// LineMap.h
	typedef lvl::GradientLineFeature LineFeature;
	typedef lvl::Line<double, 3, LineFeature> Line3;
	typedef std::vector<Line3> Line3Vec;

	typedef lvl::Line<typename Line3Vec::value_type::Scalar, 2, lvl::GradientLineFeature> Line2;
	typedef std::vector<Line2, Eigen::aligned_allocator<Line2> > Line2Vec;

	lvl::LineDetectorParameters lineDetectorParams;
	lineDetectorParams.edgeFilterKernelSize = 3;
	lineDetectorParams.edgeDetectorHighThreshold = 300.0f;
	lineDetectorParams.preSmoothingGaussianBlurSigma = -1.0f;
	lineDetectorParams.douglasPeuckerLineEps = 1.6f;
	lineDetectorParams.minLineLength = 30;
	//	edgeFilterKernelSize = 3; // 5;
	//	edgeDetectorHighThreshold = 300.0f; // 500.0;
	//	preSmoothingGaussianBlurSigma = 0.0f; // 1.0;
	//	douglasPeuckerLineEps = 0.8f; // 1.6f
	//	minLineLength = 15;

	static Line2Vec lines;

	detectLines(imgGray, lines, lineDetectorParams);

	// convert lvl format to customized format, 
	// and record parameters for line segments
	vector<Line> vecLines;

	unsigned short nCount = 0;
	for (size_t i = 0; i<lines.size(); i++) {
		Eigen::Vector2d p1 = lines[i].p1;
		Eigen::Vector2d p2 = lines[i].p2;

        Line aLine = LineParameters(p1[0], p1[1], p2[0], p2[1]);

		aLine.id = nCount;
		vecLines.push_back(aLine);
		nCount++;
	}
	return vecLines;
}



Line LineParameters(float x1, float y1, float x2, float y2, float lineWidth)
{
	float dx = x2 - x1;
	float dy = y2 - y1;
	float len = dx * dx + dy * dy;
	len = sqrtf(len);

	Line aLine;
	aLine.StartPt = Point2f(x1, y1);
	aLine.EndPt = Point2f(x2, y2);
	aLine.lineWidth = lineWidth;
	aLine.length = len;
	aLine.theta = atan(dy/dx);  // [-pi/2 , pi/2]
	aLine.Center = Point2f((x1 + x2) / 2, (y1 + y2) / 2);
	aLine.para_a = -dy;
	aLine.para_b = dx;
	aLine.para_c = x1 * y2 - x2 * y1;
	aLine.unitDir = Point2f(dx / len, dy / len);
	aLine.xMin = min(x1, x2);
	aLine.xMax = max(x1, x2);
	aLine.yMin = min(y1, y2);
	aLine.yMax = max(y1, y2);
	aLine.colorIdx = rand();
	return aLine;
}

lineDetector::lineDetector(int lMtd, bool is_roi, float x_min, float x_max, float y_min, float y_max)
{
	lineMethod = lMtd;
	isROI = is_roi;
	xMin = x_min;
	xMax = x_max;
	yMin = y_min;
	yMax = y_max;
}

vector<Line> lineDetector::detect(Mat imgGray)
{
	vector<Line> vecLines;
	switch (lineMethod){
	case 0:
		//cout << "LSD is used" << endl;
		vecLines = LineDetectLsd(imgGray);
		break;
	case 1:
		//cout << "EDline is used" << endl;
		vecLines = LineDetectEdline(imgGray);
		break;
	case 2:
		//cout << "lvl is used" << endl;
		vecLines = LineDetectLvl(imgGray);
		break;
	case 3:
		//cout << "SLSLAM is used" << endl;
		vecLines = LineDetectSlslam(imgGray);
		break;
	case 4:
		//cout << "CannyLine is used" << endl;
		vecLines = LineDetectCannyLine(imgGray);
		break;
	default:
		//cout << "LSD is used" << endl;
		vecLines = LineDetectLsd(imgGray);
	}

    /*
	// remove short line
	float MinLen = 20;
	for (vector<Line>::iterator it = vecLines.begin();
		 it != vecLines.end();)
	{
		Line L = *it;
		if (L.length < MinLen)
			it = vecLines.erase(it);
		else
			++it;
	}
   */
	if (isROI){
		removeBoundary(vecLines);
	}

    line_grad_discriptor(imgGray,vecLines);
	return vecLines;
}

void lineDetector::removeBoundary(vector<Line>& vecLine)
{
	for (vector<Line>::iterator it = vecLine.begin();
		it != vecLine.end(); /*it++*/)
	{
		Line L = *it;
		int x = L.Center.x;
		int y = L.Center.y;
		bool flag;
		flag = (x>=xMin) & (x<=xMax) & (y>=yMin) & (y<=yMax);
		if (!flag)
			it = vecLine.erase(it);
		else
			++it;
	}

	for (size_t i = 0; i < vecLine.size(); i++){
		vecLine[i].id = i;
	}

	return;
}
void lineDetector::line_grad_discriptor(Mat imgGray, vector<Line>& lines)
{
    double minLineLength = 30;
    vector<Line> vecLines;

    for (size_t i = 0; i<lines.size(); i++) {

        Line aLine = lines[i];

        if(aLine.length < minLineLength)
            continue;

        // line grad descriptor
        cv::Point2f dxdy;
        if(aLine.StartPt.x > aLine.EndPt.x)  // 统一直线方向的计算都是从右到左做差，不然有的计算出来是dxdy(1,1),翻过来可能就是（-1,-1）,会影响后续计算
        {
            dxdy = aLine.StartPt - aLine.EndPt;
        } else{
            dxdy = aLine.EndPt - aLine.StartPt;
        }

        cv::Point2f delta_xy = 2 * dxdy/std::sqrt(dxdy.dot(dxdy));   // two pixel along perpendicular line
        float dx = delta_xy.x;
        float dy = delta_xy.y;

        cv::Point2f c_prependicular_point1 = aLine.Center+ Point2f(-dy, dx);  // 垂直方向 移动两个像素 采集 pt1
        cv::Point2f c_prependicular_point2 = aLine.Center+ Point2f(dy, -dx);  // 垂直方向 移动两个像素 采集 pt2
        float line_grad = 0 ;
        int sum_cnt = 0;

        for(int i = -5; i< 5; i++)  // 沿着直线方向采集10个点
        {
            cv::Point2f pt1 = c_prependicular_point1 + i * delta_xy;
            cv::Point2f pt2 = c_prependicular_point2 + i * delta_xy;
            if( pt1.x < imgGray.cols && pt1.x > 0 && pt1.y < imgGray.rows && pt1.y > 0 &&
                pt2.x < imgGray.cols && pt2.x > 0 && pt2.y < imgGray.rows && pt2.y > 0)
            {
                line_grad += imgGray.at<uchar>(pt1) - imgGray.at<uchar>(pt2);
                sum_cnt++;
            }

        }

        aLine.line_grad_avg = line_grad/sum_cnt;
        vecLines.push_back(aLine);
    }

    lines = vecLines;
}
Mat visualizeLineSegment(Mat img_ori, vector<Line> vecLine, string winName, bool isShow)
{
	Mat img; // = img_ori.clone();
	// convert original to 3-channel grayscale image
	if (img_ori.channels() == 3){
		cv::cvtColor(img_ori, img, cv::COLOR_BGR2GRAY);
		cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
	}
	else{
		cv::cvtColor(img_ori, img, cv::COLOR_GRAY2BGR);
	}

	int nLines = vecLine.size();
	for (int i = 0; i < nLines; i++) {
		Line aLine = vecLine[i];
		float x1 = aLine.StartPt.x;
		float y1 = aLine.StartPt.y;
		float x2 = aLine.EndPt.x;
		float y2 = aLine.EndPt.y;

		CvPoint pt1(int(x1 + 0.5f), int(y1 + 0.5f));
		CvPoint pt2(int(x2 + 0.5f), int(y2 + 0.5f));
		line(img, pt1, pt2, colormap(aLine.colorIdx), 2, 8); // CV_RGB(0, 255, 0)
	}
	if (isShow){
		cout << nLines << " line segments detected" << endl;
		imshow(winName, img);
		waitKey(1);
	}
	return img;
}

void visualizeLineSegmentTwo(Mat img_1, vector<Line> vecLine1, Mat img_2, vector<Line> vecLine2, string winName, int margin)
{
	Mat img1_tmp = visualizeLineSegment(img_1, vecLine1, "", false);
	Mat img2_tmp = visualizeLineSegment(img_2, vecLine2, "", false);

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

void printLineStatistics(Line L)
{
	cout << "--------- line statistics ---------" << endl;
	cout << "Line index: " << L.id << endl;
	cout << "start / end points: (" << L.StartPt.x << ", " << L.StartPt.y
		<< ") and (" << L.EndPt.x << ", " << L.EndPt.y << ")." << endl;
	cout << "length: " << L.length << endl;
}
