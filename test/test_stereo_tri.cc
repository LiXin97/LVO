//
// Created by lixin04 on 20年1月15日.
//

#include "common.hpp"
#include "StereoFrame.hpp"
#include "Feature.hpp"
#include "line_uti/line_geometry.hpp"
#include <opencv2/core/eigen.hpp>

//std::string PathToSequence = "/home/lixin04/data/photo/MH_05";
std::string PathToSequence = "/home/lixin04/data/photo/V1_02_medium/mav0";
const std::string ParameterFile = "../config/EuRoc_VR.yaml";

void LoadStereoCalib( const std::string& ParameterFile, cv::Mat& M1l,cv::Mat& M1r,cv::Mat&M2l,cv::Mat& M2r );
void LoadImages(const std::string &strPathToSequence, std::vector<std::string> &vstrImageLeft,
                std::vector<std::string> &vstrImageRight, std::vector<double> &vTimestamps);

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
    LoadImages(PathToSequence, vstrImageLeft, vstrImageRight, vTimestamps);

    cv::Mat M1l,M2l,M1r,M2r;
    LoadStereoCalib(ParameterFile, M1l ,M1r ,M2l ,M2r);


    const int nImages = static_cast<const int>(vstrImageLeft.size());

    cv::Mat imLeft, imRight, imLeftRect, imRightRect;
    //error
    int begin = 200;
    int end   = 1500;

    std::vector<cv::Mat> T_all;
    std::vector<long> Time_all;
    // Main loop
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

        LVO::MonoParam paraml, paramr;
        LVO::StereoMatchParam stereoparam;

        Eigen::Matrix4d Trl;
        Trl = Eigen::Matrix4d::Identity();
        Trl(0,3) = -(0.110074);

        Eigen::Matrix4d Tlr;
        Tlr = Eigen::Matrix4d::Identity();
        Tlr(0,3) = (0.110074);

        LVO::StereoFrame stereoframe(ni, tframe, imLeftRect, paraml, imRightRect, paramr, Trl, stereoparam);

        Eigen::Matrix4d Twc0 = Eigen::Matrix4d::Identity();
        std::map<long, Eigen::Matrix4d> Twcs;
        Twcs.emplace( ni*2, Twc0 );
        Twcs.emplace( ni*2+1, Tlr );
        {
            std::vector<LVO::LineFeature> linefeatures;
            std::vector<cv::line_descriptor::KeyLine> left_lines, right_lines;
            std::tie(left_lines, right_lines) = stereoframe.get_stereo_match_line();
            for(int index = 0; index < left_lines.size(); ++index)
            {
                auto &left_line = left_lines[index];
                Eigen::Vector2d leftstartPoint(left_line.startPointX, left_line.startPointY);
                Eigen::Vector2d leftendPoint(left_line.endPointX, left_line.endPointY);

                auto &right_line = right_lines[index];
                Eigen::Vector2d rightstartPoint(right_line.startPointX, right_line.startPointY);
                Eigen::Vector2d rightendPoint(right_line.endPointX, right_line.endPointY);

                Eigen::Vector4d left_ob = Getobserve4d(leftstartPoint, leftendPoint), right_ob = Getobserve4d(rightstartPoint, rightendPoint);
                LVO::LineFeature linefeature;
                linefeature.insert_ob(ni, left_ob, right_ob);
                bool tri_flag; double tri_plane_angle;
                std::tie(tri_flag, tri_plane_angle) = linefeature.tri_two_plane( Twcs );
                if(tri_flag)
                    std::cout << "tri_plane_angle = " << tri_plane_angle << std::endl;
                linefeatures.push_back(linefeature);
            }
        }


////        auto [ left_result, right_result ] = stereoframe.get_stereo_match();
        cv::Mat left_result, right_result;
        std::tie( left_result, right_result ) = stereoframe.get_stereo_match();
//
//
        cv::imshow("left_result", left_result);
        cv::imshow("right_result", right_result);

        if(cv::waitKey(  ) == 27) break;

    }

    return 0;
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