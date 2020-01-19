//
// Created by lixin04 on 20年1月19日.
//

#ifndef LVO_IMAGEMANGER_HPP
#define LVO_IMAGEMANGER_HPP

#include "common.hpp"

namespace LVO
{
    class ImageManger
    {

    protected:
        std::string ParameterFile;
        std::string PathToSequence;
        std::string GroundTruthFile;


        std::vector<std::string> ImageLeftPaths;
        std::vector<std::string> ImageRightPaths;
        std::vector<double> Timestamps;
        std::vector< Eigen::Matrix4d > Twcs_true;

        cv::Mat M1l,M2l,M1r,M2r;
    };

    class EuRocData : public ImageManger
    {
    public:
        EuRocData( const std::string& PF, const std::string& PS, const std::string& GF )
        {
            ParameterFile = PF;
            PathToSequence = PS;
            GroundTruthFile = GF;
            LoadImages();
            LoadTrueTraj();
            LoadStereoCalib();
        }
        ~EuRocData() = default;

        void LoadImages( )
        {
            std::string strPathTimeFile = PathToSequence + "/cam0/data.csv";
            std::ifstream fTimes(strPathTimeFile.c_str());
            if ( !fTimes )
            {
                std::cout << "can not open times.txt" << std::endl;
                return;
            }
            std::string strPrefixLeft = PathToSequence + "/cam0/data/";
            std::string strPrefixRight = PathToSequence + "/cam1/data/";
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
                    ImageLeftPaths.push_back( strPrefixLeft + ss.str() +".png" );
                    ImageRightPaths.push_back( strPrefixRight + ss.str() +".png");
                    double t;
                    ss >> t;
                    t /= 1e9;
                    Timestamps.push_back(t);
                }
            }
        }

        void LoadTrueTraj( )
        {
            std::ifstream fTruth(GroundTruthFile.c_str());
            if (!fTruth)
            {
                std::cout << "can not open GroudtruthFile" << std::endl;
                return ;
            }
            while (!fTruth.eof())
            {
                for(int index = 0; index < Timestamps.size(); ++index)
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

                        double error = data_need[0] - Timestamps[index];
                        while( error > 0.01 )
                        {
                            ++index;
                            error = data_need[0] - Timestamps[index];
                            Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
                            Twcs_true.push_back(Twc);
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
                    while( std::abs(data_need[0] - Timestamps[index]) > 0.01 );


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

                    Twcs_true.push_back(Twc);
                }
            }
        }

        void LoadStereoCalib( )
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

        std::tuple<cv::Mat, cv::Mat> ReactStereoImg( cv::Mat& left, cv::Mat& right )
        {
            cv::Mat left_rect, right_rect;
            cv::remap(left,left_rect,M1l,M2l,cv::INTER_LINEAR);
            cv::remap(right,right_rect,M1r,M2r,cv::INTER_LINEAR);

            return std::make_tuple(left_rect, right_rect);
        }

        std::tuple< std::vector<std::string>, std::vector<std::string>, std::vector<double>, std::vector<Eigen::Matrix4d> > get_LRTT()
        {
            return std::make_tuple( ImageLeftPaths, ImageRightPaths, Timestamps, Twcs_true );
        }
    };
}

#endif //LVO_IMAGEMANGER_HPP
