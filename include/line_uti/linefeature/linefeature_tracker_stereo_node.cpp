#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "loadConfig.h"
#include "stereolinefeature_tracker.h"
#include "stereo.h"

// #include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::ImageConstPtr> img_buf;

ros::Publisher pub_img,pub_match;

//LineFeatureTracker trackerData;
stereoLineFeatureTracker stereoTrackerData;
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;

class ImageGrabber
{
public:
//    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}
    ImageGrabber(){do_rectify = true;}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = cv_ptrLeft->header.stamp.toSec();
    }

    // frequency control, 如果图像频率低于一个值
    if (round(1.0 * pub_count / (cv_ptrLeft->header.stamp.toSec() - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (cv_ptrLeft->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = cv_ptrLeft->header.stamp.toSec();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    TicToc t_r;
    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::imshow("origl",cv_ptrLeft->image);
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        stereoTrackerData.readImage(imLeft,imRight);
//        mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        stereoTrackerData.readImage(cv_ptrLeft->image,cv_ptrRight->image);
//        mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

    if (PUB_THIS_FRAME)
    {
        pub_count++;
        sensor_msgs::PointCloudPtr feature_lines(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_line;   //  feature id
        sensor_msgs::ChannelFloat32 u_of_endpoint;    //  u
        sensor_msgs::ChannelFloat32 v_of_endpoint;    //  v
        // right camera matched line
        sensor_msgs::ChannelFloat32 u_of_startpoint_R;    //  u
        sensor_msgs::ChannelFloat32 v_of_startpoint_R;    //  v
        sensor_msgs::ChannelFloat32 u_of_endpoint_R;    //  u
        sensor_msgs::ChannelFloat32 v_of_endpoint_R;    //  v

        feature_lines->header = cv_ptrLeft->header;
        feature_lines->header.frame_id = "world";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            if (i != 1 || !STEREO_TRACK)  // 单目
            {
                vector<Line> un_lines_R;
                auto un_lines = stereoTrackerData.undistortedLineEndPoints(un_lines_R);
//                std::cout <<"1111111111111111111"<<std::endl;
                //auto &cur_lines = trackerData.curframe_->vecLine;
                auto &ids = stereoTrackerData.curframe_->lineID;

                for (unsigned int j = 0; j < ids.size(); j++)
                {

                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::Point32 p;
                    p.x = un_lines[j].StartPt.x;
                    p.y = un_lines[j].StartPt.y;
                    p.z = 1;

                    feature_lines->points.push_back(p);
                    id_of_line.values.push_back(p_id * NUM_OF_CAM + i);
                    //std::cout<< "feature tracking id: " <<p_id * NUM_OF_CAM + i<<" "<<p_id<<"\n";
                    u_of_endpoint.values.push_back(un_lines[j].EndPt.x);
                    v_of_endpoint.values.push_back(un_lines[j].EndPt.y);

                    u_of_startpoint_R.values.push_back(un_lines_R[j].StartPt.x);
                    v_of_startpoint_R.values.push_back(un_lines_R[j].StartPt.y);
                    u_of_endpoint_R.values.push_back(un_lines_R[j].EndPt.x);
                    v_of_endpoint_R.values.push_back(un_lines_R[j].EndPt.y);
                    //ROS_ASSERT(inBorder(cur_pts[j]));
                }
            }
        }
        feature_lines->channels.push_back(id_of_line);
        feature_lines->channels.push_back(u_of_endpoint);
        feature_lines->channels.push_back(v_of_endpoint);

        feature_lines->channels.push_back(u_of_startpoint_R);
        feature_lines->channels.push_back(v_of_startpoint_R);
        feature_lines->channels.push_back(u_of_endpoint_R);
        feature_lines->channels.push_back(v_of_endpoint_R);
        ROS_INFO("publish %f, at %f", feature_lines->header.stamp.toSec(), ros::Time::now().toSec());
        pub_img.publish(feature_lines);

    }

    ROS_INFO("whole feature tracker processing costs: %f", t_r.toc());
}
//
//void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
//{
//    if(first_image_flag)
//    {
//        first_image_flag = false;
//        first_image_time = img_msg->header.stamp.toSec();
//    }
//
//    // frequency control, 如果图像频率低于一个值
//    if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
//    {
//        PUB_THIS_FRAME = true;
//        // reset the frequency control
//        if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
//        {
//            first_image_time = img_msg->header.stamp.toSec();
//            pub_count = 0;
//        }
//    }
//    else
//        PUB_THIS_FRAME = false;
//
//    cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
//    cv::Mat show_img = ptr->image;
////    cv::imshow("lineimg",show_img);
////    cv::waitKey(1);
//    TicToc t_r;
//
//    trackerData.readImage(ptr->image.rowRange(0 , ROW));   // rowRange(i,j) 取图像的i～j行
//
//    if (PUB_THIS_FRAME)
//    {
//        pub_count++;
//        sensor_msgs::PointCloudPtr feature_lines(new sensor_msgs::PointCloud);
//        sensor_msgs::ChannelFloat32 id_of_line;   //  feature id
//        sensor_msgs::ChannelFloat32 u_of_endpoint;    //  u
//        sensor_msgs::ChannelFloat32 v_of_endpoint;    //  v
//
//        feature_lines->header = img_msg->header;
//        feature_lines->header.frame_id = "world";
//
//        vector<set<int>> hash_ids(NUM_OF_CAM);
//        for (int i = 0; i < NUM_OF_CAM; i++)
//        {
//            if (i != 1 || !STEREO_TRACK)  // 单目
//            {
//                auto un_lines = trackerData.undistortedLineEndPoints();
//
//                //auto &cur_lines = trackerData.curframe_->vecLine;
//                auto &ids = trackerData.curframe_->lineID;
//
//                for (unsigned int j = 0; j < ids.size(); j++)
//                {
//
//                    int p_id = ids[j];
//                    hash_ids[i].insert(p_id);
//                    geometry_msgs::Point32 p;
//                    p.x = un_lines[j].StartPt.x;
//                    p.y = un_lines[j].StartPt.y;
//                    p.z = 1;
//
//                    feature_lines->points.push_back(p);
//                    id_of_line.values.push_back(p_id * NUM_OF_CAM + i);
//                    //std::cout<< "feature tracking id: " <<p_id * NUM_OF_CAM + i<<" "<<p_id<<"\n";
//                    u_of_endpoint.values.push_back(un_lines[j].EndPt.x);
//                    v_of_endpoint.values.push_back(un_lines[j].EndPt.y);
//                    //ROS_ASSERT(inBorder(cur_pts[j]));
//                }
//            }
//
//        }
//        feature_lines->channels.push_back(id_of_line);
//        feature_lines->channels.push_back(u_of_endpoint);
//        feature_lines->channels.push_back(v_of_endpoint);
//        ROS_DEBUG("publish %f, at %f", feature_lines->header.stamp.toSec(), ros::Time::now().toSec());
//        pub_img.publish(feature_lines);
//
//    }
//
//    ROS_INFO("whole feature tracker processing costs: %f", t_r.toc());
//}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo linefeature_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);

    stereoTrackerData.readIntrinsicParameter(CAM_NAMES[0]);

    ImageGrabber igb;
    if(igb.do_rectify)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings("/home/lixin04/catkin_ws/src/VINS-Mono/config/euroc/stereoEuRoC.yaml", cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
           rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    ROS_INFO("start stereo line feature");

    pub_img = n.advertise<sensor_msgs::PointCloud>("linefeature", 1000);
    pub_match = n.advertise<sensor_msgs::Image>("linefeature_img",1000);

    message_filters::Subscriber<sensor_msgs::Image> left_sub(n, "/cam0/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(n, "/cam1/image_raw", 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));
    ros::spin();

    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */

    return 0;
}
