//
// Created by lixin04 on 20年1月19日.
//

#include "ImageManger.hpp"
#include "Odometry.hpp"

std::string ParameterFile = "../config/EuRoc_VR.yaml";
std::string GroundTrueFile = "/home/lixin04/output/V102_TUM.txt";
std::string PathToSequence = "/home/lixin04/data/photo/V1_02_medium/mav0";

int main()
{
    LVO::EuRocData dataset( ParameterFile, PathToSequence, GroundTrueFile );

    std::vector<std::string> ImageLeftPaths;
    std::vector<std::string> ImageRightPaths;
    std::vector<double> Timestamps;
    std::vector< Eigen::Matrix4d > Twcs_true;

    std::tie( ImageLeftPaths, ImageRightPaths, Timestamps, Twcs_true ) = dataset.get_LRTT();

    std::shared_ptr<LVO::MonoParam> paraml = std::make_shared< LVO::MonoParam >(), paramr = std::make_shared< LVO::MonoParam >();
    std::shared_ptr<LVO::StereoMatchParam> stereoparam = std::make_shared< LVO::StereoMatchParam >();

    std::shared_ptr< LVO::View > view = std::make_shared< LVO::View >();

    LVO::Odometry odo(view);

    int begin = 200;
    int end = 1500;
    for(int ni=begin ; ni<end; ni++)
    {
        // Read left and right images from file
        cv::Mat imLeft = cv::imread(ImageLeftPaths[ni],0);
        cv::Mat imRight = cv::imread(ImageRightPaths[ni],0);
        double tframe = Timestamps[ni];

        if(imLeft.empty() || imRight.empty())
        {
            std::cerr << std::endl << "Failed to load image at: "
                      << std::string(ImageLeftPaths[ni]) << std::endl;
            continue;
            //return 1;
        }
        cv::Mat left_rect, right_rect;
        std::tie( left_rect, right_rect ) = dataset.ReactStereoImg(imLeft, imRight);

        std::shared_ptr< LVO::StereoFrame > stereoframe = std::make_shared< LVO::StereoFrame >(ni, tframe, left_rect, paraml, right_rect, paramr, stereoparam);
        odo.input_frame(stereoframe);

//        cv::imshow("imLeftRect", left_rect);


        std::cout << "GT Twc = " << std::endl << Twcs_true[ni] << std::endl;

//        if(cv::waitKey( 1 ) == 27) break;

    }
    return 0;
}