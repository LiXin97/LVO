//
// Created by lixin04 on 20年1月14日.
//

#ifndef LVO_PARAM_HPP
#define LVO_PARAM_HPP

#include "common.hpp"
#include <opencv2/core/eigen.hpp>

namespace LVO{
    enum LineExtractType{ LSD,  };
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


        bool equalize = true;
        LineExtractType line_extract_type = LineExtractType::LSD;

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
}

#endif //LVO_PARAM_HPP
