//
// Created by lixin04 on 20年1月14日.
//

#ifndef LVO_PARAM_HPP
#define LVO_PARAM_HPP

#include "common.hpp"
#include <opencv2/core/eigen.hpp>

namespace LVO{
    enum LineExtractType{ LSD, FLD };
// 单目线提取参数
    class MonoParam{
    public:
        // TODO
        MonoParam()
        {
            const std::string ParameterFile = "../config/EuRoc_VR.yaml";
            cv::FileStorage fsSettings(ParameterFile, cv::FileStorage::READ);
            fx = fsSettings["Camera.fx"];
            fy = fsSettings["Camera.fy"];
            cx = fsSettings["Camera.cx"];
            cy = fsSettings["Camera.cy"];
        }
        Eigen::Vector4d Getobserve4d( const Eigen::Vector2d& startPoint, const Eigen::Vector2d& endPoint )
        {
            return Eigen::Vector4d( ( startPoint(0) - cx )/fx,
                                    ( startPoint(1) - cy )/fy,
                                    ( endPoint(0) - cx )/fx,
                                    ( endPoint(1) - cy )/fy);
        }

        Eigen::Vector4d Getobserve4d( const Eigen::Vector4d& PointObserve )
        {
            return Eigen::Vector4d( ( PointObserve(0) - cx )/fx,
                                    ( PointObserve(1) - cy )/fy,
                                    ( PointObserve(2) - cx )/fx,
                                    ( PointObserve(3) - cy )/fy);
        }
        
        Eigen::Vector4d GetPixel4d( const Eigen::Vector4d& observe )
        {
            return Eigen::Vector4d( ( observe(0) * fx )+cx,
                                    ( observe(1) * fy )+cy,
                                    ( observe(2) * fx )+cx,
                                    ( observe(3) * fy )+cy);
        }


        bool equalize = true;
        LineExtractType line_extract_type = LineExtractType::FLD;

        int MaxNumLineFeatures = 300;
        double fx, fy, cx, cy;
    };

    class StereoMatchParam{
    public:
        StereoMatchParam()
        {
            const std::string ParameterFile = "../config/EuRoc_VR.yaml";
            cv::FileStorage fsSettings(ParameterFile, cv::FileStorage::READ);
            cv::Mat cv_T0;
            fsSettings["body_T_cam0"] >> cv_T0;
            Eigen::Matrix4d TI0;
            cv::cv2eigen(cv_T0, TI0);


            cv::Mat cv_T1;
            fsSettings["body_T_cam1"] >> cv_T1;
            Eigen::Matrix4d TI1;
            cv::cv2eigen(cv_T1, TI1);

            Tlr =  TI0.inverse()*TI1 ;
        }



        float match_dist_thread = 30;
        Eigen::Matrix4d Tlr;

//    private:

    };

    class OdoParam{
    public:
        OdoParam()
        {
        }



        float match_dist_thread = 50;
        double nrr_thread = 0.85;

        int SW_frame_size = 10;

        int track_motion_mini_line_num = 5;
        int track_SW_mini_line_num = 5;

//    private:

    };

    struct sort_flines_by_length
    {
        inline bool operator()(const cv::Vec4f& a, const cv::Vec4f& b){
            return ( sqrt(pow(a(0)-a(2),2.0)+pow(a(1)-a(3),2.0)) > sqrt(pow(b(0)-b(2),2.0)+pow(b(1)-b(3),2.0)) );
        }
    };
}

#endif //LVO_PARAM_HPP
