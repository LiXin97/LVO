
#pragma once

#include <iostream>
#include <queue>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "camodocal/camera_models/EquidistantCamera.h"

#include "../parameters.h"
#include "../tic_toc.h"

#include "camera_model.h"           // svo camera pinhole model
#include "loadConfig.h"
#include "lineDetectorWrapper.h"
#include "lineMatcher.h"
#include "lineReconstruction.h"

#include "optflow_line_matcher.h"
#include <opencv2/line_descriptor.hpp>
#include <opencv2/features2d.hpp>

using namespace cv::line_descriptor;
using namespace std;
using namespace cv;
using namespace camodocal;

class FrameLines
{
public:
    int frame_id;
    Mat img;

    // zhao ji method
    vector<Line> vecLine;
    vector< int > lineID;
    map<int, Junction> Junc;
    Mat JuncDesc;
    vector<KeyPoint> kp;

    // opencv3 lsd+lbd
    std::vector<KeyLine> keylsd;
    Mat lbd_descr;
};
typedef shared_ptr< FrameLines > FrameLinesPtr;

class LineFeatureTracker
{
  public:
    LineFeatureTracker();

    void readIntrinsicParameter(const string &calib_file);
    void NearbyLineTracking(const vector<Line> forw_lines, const vector<Line> cur_lines, vector<pair<int, int> >& lineMatches);

    vector<Line> undistortedLineEndPoints();

    void readImage(const cv::Mat &_img);

    FrameLinesPtr curframe_, forwframe_;

    cv::Mat undist_map1_, undist_map2_ , K_;

    camodocal::CameraPtr m_camera;       // pinhole camera
    Camera_Model::PinholeCamera* pinhole;

    int frame_cnt;
    vector<int> ids;                     // 每个特征点的id
    vector<int> linetrack_cnt;           // 记录某个特征已经跟踪多少帧了，即被多少帧看到了
    int allfeature_cnt;                  // 用来统计整个地图中有了多少条线，它将用来赋值

    double sum_time;
    double mean_time;
};
