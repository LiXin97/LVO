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

        pangolin::View& d_video = pangolin::Display("imgVideo")
                .SetAspect(752/(float)480);

        pangolin::View& d_kfDepth = pangolin::Display("imgKFDepth")
                .SetAspect(752/(float)480);

        pangolin::View& d_residual = pangolin::Display("imgResidual")
                .SetAspect(752/(float)480);

        pangolin::CreateDisplay()
                .SetBounds(0.0, 0.3, pangolin::Attach::Pix(180), 1.0)
                .SetLayout(pangolin::LayoutEqual)
                .AddDisplay(d_video)
                .AddDisplay(d_kfDepth)
                .AddDisplay(d_residual);


        // 新建按钮和选择框，第一个参数为按钮的名字，第二个为默认状态，第三个为是否有选择框
        pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
        pangolin::Var<bool> menuWhite("menu.Show White",false,true);
        pangolin::Var<bool> menuShowimg("menu.Show img",true,true);
        pangolin::Var<bool> menuShowLine("menu.Show Line",true,true);
        pangolin::Var<bool> menuShowCamera("menu.Show Camera",true,true);
        pangolin::Var<bool> menuShowStereoCamera("menu.Show Stereo Camera",false,true);


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



        pangolin::GlTexture imageTexture(752, 480, GL_RGB,false,0,GL_BGR,GL_UNSIGNED_BYTE);

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
            if(menuShowCamera && menuShowStereoCamera)
                draw_cams();
            else if(menuShowCamera)
                draw_leftcams();


            if(menuShowimg)
            {
                std::unique_lock<std::mutex> lock(gui_mutex);
                imageTexture.Upload(img.data,GL_BGR,GL_UNSIGNED_BYTE);
                d_video.Activate();
                glColor3f(1.0,1.0,1.0);
                // 注意，这里由于Upload函数无法将cv::Mat格式的图片数据作为输入，因此使用 opencv 的data函数将Mat格式的数据变为uchar格式，但是opencv中Mat存储图片是自下而上的，单纯的渲染所渲染出来的图片是倒置的，因此需使用RenderToViewportFlipY（）函数进行渲染，将原本上下倒置的图片进行自下而上渲染，使显示的图片是正的。
                // https://blog.csdn.net/u013270341/article/details/74995530
                imageTexture.RenderToViewportFlipY();
            }

            pangolin::FinishFrame();
        }
    }

    void View::draw_lines()
    {
//        std::cout << " now in draw_lines " << std::endl;
        std::unique_lock<std::mutex> lock(gui_mutex);
//        std::cout << " now is draw_lines " << std::endl;

//        std::unique_lock<std::mutex> lock(gui_mutex, std::chrono::seconds(1));
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
//        std::cout << " now over draw_lines " << std::endl;
    }

    inline void View::draw_cam(pangolin::OpenGlMatrix &Twc)
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
        std::unique_lock<std::mutex> lock(gui_mutex);
//        std::unique_lock<std::mutex> lock(gui_mutex, std::chrono::seconds(1));
        if(Twcs.empty()) return;

        for(auto &Twc:Twcs)
        {
            draw_cam(Twc);
        }
    }

    void View::draw_leftcams()
    {
//        std::cout << " now in draw_leftcams " << std::endl;
        std::unique_lock<std::mutex> lock(gui_mutex);
//        std::cout << " now is draw_leftcams " << std::endl;
//        std::unique_lock<std::mutex> lock(gui_mutex, std::chrono::seconds(1));
        if(Twcs.empty()) return;

        for(int i=0;i<Twcs.size();i=i+2)
        {
            auto Twc = Twcs[i];
            draw_cam(Twc);
        }
//        std::cout << " now over draw_leftcams " << std::endl;
    }

    void View::set_elem(std::vector<Eigen::Matrix4d>& _Twcs, std::vector< Eigen::Vector3d >& _Lines, cv::Mat& image)
    {
//        std::cout << " now in set_elem " << std::endl;
        std::unique_lock<std::mutex> lock(gui_mutex);
//        std::cout << " now is set_elem " << std::endl;

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

        Lines.clear();
        for(const auto &line:_Lines)
        {
            Lines.push_back(line);
        }

        if(image.channels() != 3) cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
        img = image.clone();
//        img.resize(  );

//        std::cout << " now over set_elem " << std::endl;
    }
}