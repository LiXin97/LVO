//
// Created by lixin04 on 20年1月16日.
//

#include "common.hpp"
#include "StereoFrame.hpp"
#include "Odometry.hpp"
#include "Feature.hpp"
#include "line_uti/line_geometry.hpp"
#include <opencv2/core/eigen.hpp>

//std::string PathToSequence = "/home/lixin04/data/photo/MH_05";
std::string PathToSequence = "/home/lixin04/data/photo/V1_02_medium/mav0";
const std::string ParameterFile = "../config/EuRoc_VR.yaml";
//const std::string GroundTrueFile = "/home/lixin04/output/MH05_TUM.txt";
const std::string GroundTrueFile = "/home/lixin04/output/V102_TUM.txt";

void LoadStereoCalib( const std::string& ParameterFile, cv::Mat& M1l,cv::Mat& M1r,cv::Mat&M2l,cv::Mat& M2r );
void LoadImages(const std::string &strPathToSequence, std::vector<std::string> &vstrImageLeft,
                std::vector<std::string> &vstrImageRight, std::vector<double> &vTimestamps);

void LoadTrueTraj( const std::string &GroundtruthFile, std::vector< Eigen::Matrix4d >& Twcs, std::vector<double> &vTimestamps );

void print_T01()
{
    cv::FileStorage fsSettings(ParameterFile, cv::FileStorage::READ);
    cv::Mat cv_T0;
    fsSettings["body_T_cam0"] >> cv_T0;
    Eigen::Matrix4d TI0;
    cv::cv2eigen(cv_T0, TI0);


    cv::Mat cv_T1;
    fsSettings["body_T_cam1"] >> cv_T1;
    Eigen::Matrix4d TI1;
    cv::cv2eigen(cv_T1, TI1);

    std::cout << TI0.inverse()*TI1 << std::endl;

//    0.999997  -0.00231714 -0.000343393     0.110074
//    0.00231207     0.999898   -0.0140907 -0.000156612
//    0.000376008    0.0140898     0.999901  0.000889383
//    0            0            0            1
}

Eigen::Vector4d Getobserve4d( const Eigen::Vector2d& startPoint, const Eigen::Vector2d& endPoint )
{
    cv::FileStorage fsSettings(ParameterFile, cv::FileStorage::READ);
    double fx, fy, cx, cy;
    fx = fsSettings["Camera.fx"];
    fy = fsSettings["Camera.fy"];
    cx = fsSettings["Camera.cx"];
    cy = fsSettings["Camera.cy"];

    return Eigen::Vector4d( ( startPoint(0) - cx )/fx,
                            ( startPoint(1) - cy )/fy,
                            ( endPoint(0) - cx )/fx,
                            ( endPoint(1) - cy )/fy);
}

int main(int argc, char **argv)
{
//    print_T01(); return 0;
    // Retrieve paths to images
    std::vector<std::string> vstrImageLeft;
    std::vector<std::string> vstrImageRight;
    std::vector<double> vTimestamps;
    std::vector< Eigen::Matrix4d > Twcs_true;

    LoadImages(PathToSequence, vstrImageLeft, vstrImageRight, vTimestamps);

    cv::Mat M1l,M2l,M1r,M2r;
    LoadStereoCalib(ParameterFile, M1l ,M1r ,M2l ,M2r);

    LoadTrueTraj( GroundTrueFile, Twcs_true, vTimestamps );

    std::cout << "Twcs_true.size() = " << Twcs_true.size() << std::endl;
    std::cout << "vstrImageLeft.size() = " << vstrImageLeft.size() << std::endl;
    std::cout << "vTimestamps.size() = " << vTimestamps.size() << std::endl;


    const int nImages = static_cast<const int>(vstrImageLeft.size());

    cv::Mat imLeft, imRight, imLeftRect, imRightRect;
    //error
    int begin = 200;
    int end   = 1500;

    std::shared_ptr< LVO::View > view = std::make_shared< LVO::View >();
    LVO::Odometry odo(view);

    std::vector<cv::Mat> T_all;
    std::vector<long> Time_all;

    Eigen::Matrix4d init = Eigen::Matrix4d::Identity();
    bool get_init = true;
    // Main loop

    std::shared_ptr<LVO::MonoParam> paraml = std::make_shared< LVO::MonoParam >(), paramr = std::make_shared< LVO::MonoParam >();
    std::shared_ptr<LVO::StereoMatchParam> stereoparam = std::make_shared< LVO::StereoMatchParam >();

    for(int ni=begin ; ni<end; ni++)
    {
        TicToc time;
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],0);
        imRight = cv::imread(vstrImageRight[ni],0);
        long tframe = vTimestamps[ni];

        if(imLeft.empty() || imRight.empty())
        {
            std::cerr << std::endl << "Failed to load image at: "
                      << std::string(vstrImageLeft[ni]) << std::endl;
            continue;
            //return 1;
        }

        cv::remap(imLeft,imLeftRect,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(imRight,imRightRect,M1r,M2r,cv::INTER_LINEAR);

        cv::imshow("imLeftRect", imLeftRect);

        std::shared_ptr< LVO::StereoFrame > stereoframe = std::make_shared< LVO::StereoFrame >(ni, tframe, imLeftRect, paraml, imRightRect, paramr, stereoparam);
        odo.input_frame(stereoframe);

        if( odo.get_state() == LVO::Odometry::OK && get_init )
        {
            init = Twcs_true[ni];
            get_init = false;
        }

        std::cout << "GT Twc = " << std::endl << Twcs_true[ni].inverse() * init  << std::endl;

        if(cv::waitKey( 1 ) == 27) break;

    }

    return 0;
}


void LoadTrueTraj( const std::string &GroundtruthFile, std::vector< Eigen::Matrix4d >& Twcs, std::vector<double> &vTimestamps )
{
    std::ifstream fTruth(GroundtruthFile.c_str());
    if (!fTruth)
    {
        std::cout << "can not open GroudtruthFile" << std::endl;
        return ;
    }
    while (!fTruth.eof())
    {
        for(int index = 0; index < vTimestamps.size(); ++index)
        {
//            std::cout << std::setprecision(20) << "vTimestamps[index] = " << vTimestamps[index] << std::endl;

            std::vector<double> data_need(8);
            {
                double data[8];
                for (auto &d:data)
                {
                    fTruth >> d;
                }
                for(int i=0;i<7;++i) data_need[i] = data[i];

                double error = data_need[0] - vTimestamps[index];
                while( error > 0.01 )
                {
                    ++index;
                    error = data_need[0] - vTimestamps[index];
                    Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
                    Twcs.push_back(Twc);
                }

//                std::cout << std::setprecision(20) << "data[0] = " << data_need[0] << std::endl;
//                std::cout << std::setprecision(20) << "std::abs(data_need[0] - vTimestamps[index])  = " << std::abs(data_need[0] - vTimestamps[index])  << std::endl;
            }

            do{
                if(fTruth.eof()) return;
//                data.clear();
                double data[8];
                for (auto &d:data)
                {
                    fTruth >> d;
                }
                for(int i=0;i<7;++i) data_need[i] = data[i];
//                std::cout << std::setprecision(20) << "data[0] = " << data_need[0] << std::endl;
//                std::cout << std::setprecision(20) << "std::abs(data_need[0] - vTimestamps[index])  = " << data_need[0] - vTimestamps[index]  << std::endl;
            }
            while( std::abs(data_need[0] - vTimestamps[index]) > 0.01 );


            std::vector<double> t;
            std::vector<double> R;

            t.push_back(data_need[1]);
            t.push_back(data_need[2]);
            t.push_back(data_need[3]);

            Eigen::Quaterniond qua( data_need[7], data_need[3], data_need[4], data_need[5] );

            Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
            Twc.block(0,0,3,3) = qua.toRotationMatrix();
            Twc(0,3) = t[0];
            Twc(1,3) = t[1];
            Twc(2,3) = t[2];

            Twcs.push_back(Twc);
        }
    }
}

Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;

    M << cvMat3.at<double>(0,0), cvMat3.at<double>(0,1), cvMat3.at<double>(0,2),
            cvMat3.at<double>(1,0), cvMat3.at<double>(1,1), cvMat3.at<double>(1,2),
            cvMat3.at<double>(2,0), cvMat3.at<double>(2,1), cvMat3.at<double>(2,2);

    return M;
}

std::vector<double> toQuaternion(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    Eigen::Quaterniond q(eigMat);

    std::vector<double> v(4);
    v[0] = q.x();
    v[1] = q.y();
    v[2] = q.z();
    v[3] = q.w();

    return v;
}


void LoadImages(const std::string &strPathToSequence, std::vector<std::string> &vstrImageLeft,
                std::vector<std::string> &vstrImageRight, std::vector<double> &vTimestamps)
{
    std::string strPathTimeFile = strPathToSequence + "/cam0/data.csv";
    std::ifstream fTimes(strPathTimeFile.c_str());
    if ( !fTimes )
    {
        std::cout << "can not open times.txt" << std::endl;
        return;
    }
    std::string strPrefixLeft = strPathToSequence + "/cam0/data/";
    std::string strPrefixRight = strPathToSequence + "/cam1/data/";
    while(!fTimes.eof())
    {
        std::string s;
        getline(fTimes,s);
        for (auto i = 0; i < s.size(); ++i) {
            while(s[i] == ','){
                s.erase(static_cast<unsigned long>(i), s.size() - 1);
                break;
            }
        }
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            vstrImageLeft.push_back( strPrefixLeft + ss.str() +".png" );
            vstrImageRight.push_back( strPrefixRight + ss.str() +".png");
            double t;
            ss >> t;
            t /= 1e9;
            vTimestamps.push_back(t);
        }
    }
}

void LoadStereoCalib( const std::string& ParameterFile, cv::Mat& M1l,cv::Mat& M1r,cv::Mat&M2l,cv::Mat& M2r )
{
    cv::FileStorage fsSettings(ParameterFile, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
        return;
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
        std::cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << std::endl;
        return;
    }

    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
}