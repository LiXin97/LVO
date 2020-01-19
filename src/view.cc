//
// Created by lixin04 on 20年1月19日.
//

#include "view.hpp"

namespace LVO
{
    View::View()
    {
        auto proc_func = [&]{run();};
        view_thread.reset( new std::thread(proc_func) );
    }

    void View::run()
    {
        pangolin::CreateWindowAndBind("LVO",1024,768);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // 新建按钮和选择框，第一个参数为按钮的名字，第二个为默认状态，第三个为是否有选择框
        pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
        pangolin::Var<bool> menuWhite("menu.Show White",false,true);
        pangolin::Var<bool> menuShowLine("menu.Show Line",true,true);
        pangolin::Var<bool> menuShowCamera("menu.Show Camera",true,true);


        // Define Camera Render Object (for view / scene browsing)
        // 定义相机投影模型：ProjectionMatrix(w, h, fu, fv, u0, v0, zNear, zFar)
        // 定义观测方位向量：观测点位置：(mViewpointX mViewpointY mViewpointZ)
        //                观测目标位置：(0, 0, 0)
        //                观测的方位向量：(0.0,-1.0, 0.0)
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
                pangolin::ModelViewLookAt(0,-0.7,-1.8, 0,0,0,0.0,-1.0, 0.0)
        );

        // Add named OpenGL viewport to window and provide 3D Handler
        // 定义显示面板大小，orbslam中有左右两个面板，昨天显示一些按钮，右边显示图形
        // 前两个参数（0.0, 1.0）表明宽度和面板纵向宽度和窗口大小相同
        // 中间两个参数（pangolin::Attach::Pix(175), 1.0）表明右边所有部分用于显示图形
        // 最后一个参数（-1024.0f/768.0f）为显示长宽比
        pangolin::View& d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
                .SetHandler(new pangolin::Handler3D(s_cam));
        d_cam.Activate(s_cam);

        while( !pangolin::ShouldQuit() )
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            if(menuWhite)
                glClearColor(1.0f,1.0f,1.0f,1.0f);
            else
                glClearColor(0.0f,0.0f,0.0f,0.0f);

            d_cam.Activate(s_cam);
            const double lens = 0.5;
            glLineWidth(2.0);
            glBegin(GL_LINES);
            glColor3f(1.0f,0.0f,0.0f);
            glVertex3f(0,0,0);
            glVertex3f(lens,0,0);


            glColor3f(0.0f,1.0f,0.0f);
            glVertex3f(0,0,0);
            glVertex3f(0,lens,0);

            glColor3f(0.0f,0.0f,1.0f);
            glVertex3f(0,0,0);
            glVertex3f(0,0,lens);

            glEnd();

            if(menuShowLine)
                draw_lines();
            if(menuShowCamera)
                draw_cams();


            pangolin::FinishFrame();
        }
    }

    void View::draw_lines()
    {
        std::lock_guard<std::mutex> lock(gui_mutex);

        if(Lines.empty()) return;

        glLineWidth(4);
        glBegin(GL_LINES);
        glColor3f(1.0,0.0,0.0);
        for(int i=1;i<Lines.size(); i=i+2)
        {
            glVertex3f(Lines[i][0], Lines[i][1], Lines[i][2]);
            glVertex3f(Lines[i-1][0], Lines[i-1][1], Lines[i-1][2]);
        }
        glEnd();
    }

    void View::draw_cam(pangolin::OpenGlMatrix &Twc)
    {
        //相机模型大小：宽度占总宽度比例为0.08
        const float &w = 0.3;
        const float h = w*0.638;
        const float z = w*0.6;

        //百度搜索：glPushMatrix 百度百科
        glPushMatrix();

        //将4*4的矩阵Twc.m右乘一个当前矩阵
        //（由于使用了glPushMatrix函数，因此当前帧矩阵为世界坐标系下的单位矩阵）
        //因为OpenGL中的矩阵为列优先存储，因此实际为Tcw，即相机在世界坐标下的位姿
        glMultMatrixd(Twc.m);

        //    设置绘制图形时线的宽度
        glLineWidth(1.0);
        //设置当前颜色为绿色(相机图标显示为绿色)
        glColor3f(0.0f,1.0f,0.0f);
        //用线将下面的顶点两两相连
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();


        //双缓存交换缓存以显示图像
        //    glutSwapBuffers();

        glPopMatrix();
    }

    void View::draw_cams()
    {
        std::lock_guard<std::mutex> lock(gui_mutex);
        if(Twcs.empty()) return;

        for(auto &Twc:Twcs)
        {
            draw_cam(Twc);
        }
    }

    void View::set_elem(std::vector<Eigen::Matrix4d> &_Twcs, std::vector<Eigen::Vector3d> &_Lines)
    {
        std::lock_guard<std::mutex> lock(gui_mutex);

        Twcs.clear();
        for(const auto & _Twc : _Twcs)
        {
            pangolin::OpenGlMatrix M;

            Eigen::Matrix3d Rwc = _Twc.block(0,0,3,3);
            Eigen::Vector3d twc = _Twc.block(0,3,3,1);

            M.m[0] = Rwc(0,0);
            M.m[1] = Rwc(1,0);
            M.m[2] = Rwc(2,0);
            M.m[3]  = 0.0;

            M.m[4] = Rwc(0,1);
            M.m[5] = Rwc(1,1);
            M.m[6] = Rwc(2,1);
            M.m[7]  = 0.0;

            M.m[8] = Rwc(0,2);
            M.m[9] = Rwc(1,2);
            M.m[10] = Rwc(2,2);
            M.m[11]  = 0.0;

            M.m[12] = twc(0);
            M.m[13] = twc(1);
            M.m[14] = twc(2);
            M.m[15]  = 1.0;

            Twcs.push_back(M);
        }

        for(const auto &line:_Lines)
        {
            Lines.push_back(line);
        }
    }
}