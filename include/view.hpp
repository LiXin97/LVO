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
        void draw_cam(pangolin::OpenGlMatrix& Twc);
        void draw_cams();

        void set_elem(std::vector<Eigen::Matrix4d>& _Twcs, std::vector< Eigen::Vector3d >& _Lines);

    private:
//        std::vector< Eigen::Matrix4d > Twcs;
        std::vector<pangolin::OpenGlMatrix> Twcs;
        std::vector< Eigen::Vector3d > Lines;

        std::mutex gui_mutex;

        std::shared_ptr<std::thread> view_thread;
    };
}

#endif //LVO_VIEW_HPP
