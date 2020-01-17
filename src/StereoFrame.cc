//
// Created by lixin04 on 20年1月13日.
//

#include "StereoFrame.hpp"

namespace LVO
{

    Frame::Frame(
            long _FrameId,
            long _TimeStamp,
            const cv::Mat& Img,
            const std::shared_ptr< MonoParam >& param
            ) : timestamp(_TimeStamp), img_src(Img), frame_id(_FrameId), mono_param(param)
    {
        img = img_src.clone();
        if(mono_param->equalize)
        {
            cv::equalizeHist(img, img);
        }


        switch (mono_param->line_extract_type)
        {
            case LineExtractType::LSD :
                extract_line_lsd();
                break;

            default:
                std::cerr << " Wrong Line Extract type " << std::endl;
                break;
        }
    }


    // TODO 提取这里还可以研究研究
    void Frame::extract_line_lsd()
    {

        // step 1: line extraction
        cv::Ptr<cv::line_descriptor::LSDDetector> lsd_detector;
        lsd_detector = cv::line_descriptor::LSDDetector::createLSDDetector();
        lsd_detector->detect( img, line, 2, 2 );

        if(line.size() > mono_param->MaxNumLineFeatures)
        {
            std::sort( line.begin(), line.end(),
                    [](cv::line_descriptor::KeyLine a, cv::line_descriptor::KeyLine b){ return a.response > b.response; }
                    );
            line.resize(mono_param->MaxNumLineFeatures);
        }


        // step 2: lbd descriptor
        cv::Ptr<cv::line_descriptor::BinaryDescriptor> bd_ = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor( );;
        //bd_ = BinaryDescriptor::createBinaryDescriptor(  );
        bd_->compute( img, line, lbd_descr );

        for ( int i = 0; i < (int) line.size(); i++ )
        {

            if( line[i].octave == 0 && line[i].lineLength >= 30)
            {
                keyline.push_back( line[i] );
                keylbd_descr.push_back( lbd_descr.row( i ) );
            }
        }
    }

    cv::Mat Frame::get_extract_img()
    {
        cv::Mat show = img.clone();
        if(show.channels() != 3) cv::cvtColor(show, show, cv::COLOR_GRAY2BGR);

        int lowest = 0, highest = 255;
        int range = (highest - lowest) + 1;
        for(auto &line:keyline)
        {
            unsigned int r = lowest + int(rand() % range);
            unsigned int g = lowest + int(rand() % range);
            unsigned int b = lowest + int(rand() % range);
            cv::Point startPoint = cv::Point(int(line.startPointX), int(line.startPointY));
            cv::Point endPoint = cv::Point(int(line.endPointX), int(line.endPointY));
            cv::line(show, startPoint, endPoint, cv::Scalar(r, g, b),2 ,8);
        }

        return show;
    }

    // TODO 2 thread line extract
    StereoFrame::StereoFrame(
            long _StereoframeId,
            long _TimeStamp,
            const cv::Mat& _LeftImg,
            const std::shared_ptr< MonoParam >& leftparam,
            const cv::Mat& _RightImg,
            const std::shared_ptr< MonoParam >& rightparam,
            const std::shared_ptr< StereoMatchParam >& stereoparam
            ): stereo_param(stereoparam), TimeStamp(_TimeStamp), StereoFrameId(_StereoframeId)
    {
        left = std::make_shared<Frame>(_StereoframeId*2, _TimeStamp, _LeftImg, leftparam);
        right = std::make_shared<Frame>(_StereoframeId*2 + 1, _TimeStamp, _RightImg, rightparam);

        Eigen::Matrix4d Iden = Eigen::Matrix4d::Identity();
        this->set_Twc(Iden);
        //stereo match
        match_stereo_line();
    }

    void StereoFrame::match_stereo_line()
    {
        std::vector<cv::DMatch> lsd_matches;
        cv::Ptr<cv::line_descriptor::BinaryDescriptorMatcher> bdm_;
        bdm_ = cv::line_descriptor::BinaryDescriptorMatcher::createBinaryDescriptorMatcher();

        auto keyline_l = left->get_line_extract();
        auto keyline_r = right->get_line_extract();
        cv::Mat lbd_l = left->get_lbd();
        cv::Mat lbd_r = right->get_lbd();
        bdm_->match(lbd_l, lbd_r, lsd_matches);

        /* select best matches */
        std::vector<cv::DMatch> good_matchesLR;
        std::vector<cv::line_descriptor::KeyLine> good_KeylinesLR;
        good_matchesLR.clear();
        for (auto & lsd_matche : lsd_matches)
        {
            if( lsd_matche.distance < stereo_param->match_dist_thread ){

                cv::DMatch mt = lsd_matche;
                cv::line_descriptor::KeyLine line1 =  keyline_l[mt.queryIdx] ;
                cv::line_descriptor::KeyLine line2 =  keyline_r[mt.trainIdx] ;
                cv::Point2f serr = line1.getStartPoint() - line2.getStartPoint();
                cv::Point2f eerr = line1.getEndPoint() - line2.getEndPoint();
                if((serr.dot(serr) < 60 * 60) && (eerr.dot(eerr) < 60 * 60))   // 线段在图像里不会跑得特别远
                    good_matchesLR.push_back( lsd_matche );
            }
        }

        std::set<size_t> stereo_lines_left, stereo_lines_right;


//        std::cout << "keyline_l.size() = " << keyline_l.size() << std::endl;
//        std::cout << "keyline_r.size() = " << keyline_r.size() << std::endl;
//        std::cout << "all_obersves.size() = " << all_obersves.size() << std::endl;
        for (auto mt : good_matchesLR)
        {


            left_match_lines.push_back( keyline_l[mt.queryIdx] );
            right_match_lines.push_back( keyline_r[mt.trainIdx] );

            Eigen::Vector2d leftstartPoint(keyline_l[mt.queryIdx].startPointX, keyline_l[mt.queryIdx].startPointY);
            Eigen::Vector2d leftendPoint(keyline_l[mt.queryIdx].endPointX, keyline_l[mt.queryIdx].endPointY);
            Eigen::Vector4d left_obs =  left->get_monoparam()->Getobserve4d(leftstartPoint, leftendPoint);

            Eigen::Vector2d rightstartPoint(keyline_r[mt.trainIdx].startPointX, keyline_r[mt.trainIdx].startPointY);
            Eigen::Vector2d rightendPoint(keyline_r[mt.trainIdx].endPointX, keyline_r[mt.trainIdx].endPointY);
            Eigen::Vector4d right_obs =  right->get_monoparam()->Getobserve4d(rightstartPoint, rightendPoint);

            LineFeature lineFeature;
            lineFeature.insert_ob( get_StereoId(), left_obs, right_obs );

            stereo_lines_left.insert(mt.queryIdx);
            stereo_lines_right.insert(mt.trainIdx);

            cv::Mat lbd_descr;

            // TODO 描述子取平均
            lbd_descr.push_back( lbd_l.row( mt.queryIdx ) );
            lineFeature.set_descri(lbd_descr);

            all_obersves.push_back(lineFeature);

//            all_line_obs.push_back( left->get_monoparam()->Getobserve4d(leftstartPoint, leftendPoint) );
//            all_line_obs.push_back( right->get_monoparam()->Getobserve4d(rightstartPoint, rightendPoint) );
//            all_line_id.push_back( feature_id - 1 );
//            all_line_id.push_back( feature_id - 1 );
//            all_line_desc.push_back( lbd_l.row( mt.queryIdx ) );
//            all_line_desc.push_back( lbd_l.row( mt.queryIdx ) );
        }

        for(int index=0;index<keyline_l.size();++index)
        {
            if(stereo_lines_left.count(index) > 0) continue;
            {
                Eigen::Vector2d leftstartPoint(keyline_l[index].startPointX, keyline_l[index].startPointY);
                Eigen::Vector2d leftendPoint(keyline_l[index].endPointX, keyline_l[index].endPointY);
                Eigen::Vector4d left_obs =  left->get_monoparam()->Getobserve4d(leftstartPoint, leftendPoint);

                LineFeature lineFeature;
                lineFeature.insert_ob( get_StereoId()*2, left_obs );

                cv::Mat lbd_descr;

                lbd_descr.push_back( lbd_l.row( index ) );
                lineFeature.set_descri(lbd_descr);

                all_obersves.push_back(lineFeature);
            }
        }

        for(int index=0;index<keyline_r.size();++index)
        {
            if(stereo_lines_right.count(index) > 0) continue;
            {
                Eigen::Vector2d rightstartPoint(keyline_r[index].startPointX, keyline_r[index].startPointY);
                Eigen::Vector2d rightendPoint(keyline_r[index].endPointX, keyline_r[index].endPointY);
                Eigen::Vector4d right_obs =  right->get_monoparam()->Getobserve4d(rightstartPoint, rightendPoint);

                LineFeature lineFeature;
                lineFeature.insert_ob( get_StereoId()*2+1, right_obs );

                cv::Mat lbd_descr;

                lbd_descr.push_back( lbd_r.row( index ) );
                lineFeature.set_descri(lbd_descr);

                all_obersves.push_back(lineFeature);
            }
        }

//        std::cout << "all_obersves.size() = " << all_obersves.size() << std::endl;

    }

    std::tuple<cv::Mat ,cv::Mat> StereoFrame::get_stereo_match()
    {


        cv::Mat show_left = left->get_img();
        if(show_left.channels() != 3) cv::cvtColor(show_left, show_left, cv::COLOR_GRAY2BGR);
        cv::Mat show_right = right->get_img();
        if(show_right.channels() != 3) cv::cvtColor(show_right, show_right, cv::COLOR_GRAY2BGR);


        //    srand(time(NULL));
        int lowest = 0, highest = 255;
        int range = (highest - lowest) + 1;
        for(int index = 0;index < left_match_lines.size(); ++index)
        {
            auto left_line = left_match_lines[index];
            auto right_line = right_match_lines[index];

            unsigned int r = lowest + int(rand() % range);
            unsigned int g = lowest + int(rand() % range);
            unsigned int b = lowest + int(rand() % range);
            cv::Point startPoint = cv::Point(int(left_line.startPointX), int(left_line.startPointY));
            cv::Point endPoint = cv::Point(int(left_line.endPointX), int(left_line.endPointY));
            cv::line(show_left, startPoint, endPoint, cv::Scalar(r, g, b),2 ,8);

            cv::Point startPoint2 = cv::Point(int(right_line.startPointX), int(right_line.startPointY));
            cv::Point endPoint2 = cv::Point(int(right_line.endPointX), int(right_line.endPointY));
            cv::line(show_right, startPoint2, endPoint2, cv::Scalar(r, g, b),2, 8);
            cv::line(show_right, startPoint, startPoint2, cv::Scalar(0, 0, 255),1, 8);
            cv::line(show_right, endPoint, endPoint2, cv::Scalar(0, 0, 255),1, 8);
        }
        return std::make_tuple(show_left, show_right);
    }

}
