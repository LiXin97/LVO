//
// Created by lixin04 on 20年1月19日.
//

#ifndef LVO_VIEW_HPP
#define LVO_VIEW_HPP

#include "common.hpp"
#include <pangolin/pangolin.h>
#include <thread>

namespace LVO
{
    class View
    {
    public:
        View();
        ~View() = default;

        void run();

        void draw_lines();
        inline void draw_cam(pangolin::OpenGlMatrix& Twc);
        void draw_cams();
        void draw_leftcams();

        void set_elem(std::vector<Eigen::Matrix4d>& _Twcs, std::vector< Eigen::Vector3d >& _Lines, cv::Mat& image);

    private:
//        std::vector< Eigen::Matrix4d > Twcs;
        std::vector<pangolin::OpenGlMatrix> Twcs;
        std::vector< Eigen::Vector3d > Lines;

        cv::Mat img;

        std::mutex gui_mutex;

        std::shared_ptr<std::thread> view_thread;
    };
}

#endif //LVO_VIEW_HPP
