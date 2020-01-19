//
// Created by lixin04 on 20年1月13日.
//

#include "Odometry.hpp"

namespace LVO
{
    long Odometry::feature_id = 0;

    void Odometry::input_frame(std::shared_ptr<LVO::StereoFrame> &stereoframe)
    {
        cur_frame = stereoframe;

        tracking();

        if(state == OdoState::OK)
        update_gui();
    }

    void Odometry::update_gui()
    {
        std::vector< Eigen::Matrix4d > Twcs;
        for(const auto& id_Twc:SW_frames)
        {
            Twcs.push_back( id_Twc.second );
        }

        std::vector< Eigen::Vector3d > Lines;
        for(auto& id_feature:SW_features)
        {
            if(id_feature.second.is_tri())
            {
                Eigen::Matrix4d Twc = SW_frames.find( id_feature.second.get_first_frameid() )->second;

                auto Line3d = id_feature.second.get3D(Twc);

                Lines.push_back( Line3d.getLeftPoint() );
                Lines.push_back( Line3d.getRightPoint() );
            }
        }

        view->set_elem(Twcs, Lines);
    }

    void Odometry::tracking()
    {
        if(state == OdoState::UNINIT)
        {
            {
                int tri_ok = 0;
                std::map<long, Eigen::Matrix4d> frame_map;
                Eigen::Matrix4d Iden = Eigen::Matrix4d::Identity();
                Eigen::Matrix4d Tlr = cur_frame->get_stereo_param()->Tlr;
                frame_map.emplace( cur_frame->get_StereoId()*2, Iden );
                frame_map.emplace( cur_frame->get_StereoId()*2+1, Tlr );


                auto all_obs = cur_frame->get_obs();
                std::vector<long> all_ids(all_obs.size(), -1);

                for(int i=0;i<all_obs.size(); ++i)
                {
                    auto &obs = all_obs[i];
                    if(obs.num_obs() > 1)
                    {
                        auto tri_state = obs.tri_two_plane(frame_map);
                        if( std::get<0>(tri_state) ) tri_ok++;
                        all_ids[i] = feature_id;
                        SW_features.emplace(feature_id++, obs);
                    }
                }

                if (tri_ok <= 30)
                {
                    std::cerr << "tri line " << tri_ok << std::endl;
                    SW_features.clear();
                }
                else
                {
                    state = OdoState::OK;

                    for (int i = 0; i < all_obs.size(); ++i) {
                        auto &obs = all_obs[i];
                        if (obs.num_obs() == 1) {
                            all_ids[i] = feature_id;
                            SW_features.emplace(feature_id++, obs);
                        }
                    }
                    cur_frame->set_lineid(all_ids);
                    cur_frame->set_Twc(Iden);
                    SW_frames.emplace(cur_frame->get_left_frame()->get_id(), cur_frame->get_left_frame()->get_Twc());
                    SW_frames.emplace(cur_frame->get_right_frame()->get_id(), cur_frame->get_right_frame()->get_Twc());

                    last_key_frame = cur_frame;
                    last_frame = cur_frame;
                    motion_velocity = Eigen::Matrix4d::Identity();
                }

//                std::cout << "tri_ok = " << tri_ok << std::endl;
            }
        }
        else if(state == OdoState::OK)
        {
            bool OK;
            int track_motion_line_num = track_motion_mode();
            OK = track_motion_line_num > odoParam.track_motion_mini_line_num;
            if(!OK)
            {
                std::cerr << "track_motion_mode() failed, only " << track_motion_line_num << std::endl;
            }

            int track_SW_line_num = track_sw();
            OK = track_SW_line_num > odoParam.track_SW_mini_line_num;
            if(!OK)
            {
                std::cerr << "track_sw() failed, only " << track_SW_line_num << std::endl;
            }

            if(OK)
            {

                {
                    SW_frames.emplace(cur_frame->get_left_frame()->get_id(), cur_frame->get_left_frame()->get_Twc());
                    SW_frames.emplace(cur_frame->get_right_frame()->get_id(), cur_frame->get_right_frame()->get_Twc());

                    // 添加当前帧的观测到滑窗
                    {
                        std::vector<long> feature_ids;
                        std::vector< LineFeature > obs;
                        std::tie( feature_ids, obs ) = cur_frame->get_id_obs();

                        for(int index = 0;index<feature_ids.size();++index)
                        {
                            if(feature_ids[index] < 0)
                            {
                                continue;
                            }

                            auto &ob = obs[index];
                            auto observe = ob.get_obs();
                            for(auto &obse:observe)
                                SW_features[ feature_ids[index] ].insert_ob(obse.first, obse.second);
                        }
                    }


                    // 三角化

                    {
                        for(auto &id_feature:SW_features)
                        {
                            auto &feature = id_feature.second;
                            if(feature.is_tri()) continue;
                            feature.tri_two_plane( SW_frames );
                        }
                    }

                    bool need_kf = need_keyframe();

                    // 滑窗优化

                    // TODO 滑窗优化之后是不是应该更新一下速度呢？
                    optimization_SW();
                    {
                        update_velocity();
//                        if(SW_frames.size()>8)
//                        {
//                            auto it0 = SW_frames.rbegin(); it0++;
//                            cur_frame->get_left_frame()->set_Twc( it0->second );
//                            auto it1 = SW_frames.rbegin(); it1++;it1++;it1++;
//                            last_frame->get_left_frame()->set_Twc( it1->second );
//                        }
                    }
                    // 重投影误差过大的线，直接去三角化
                    retri_linefeatrues();

                    // 滑窗
                    {
                        if(need_kf) add_keyframe();

                        std::cout << "SW_frames.size() = " << SW_frames.size() << std::endl;

                        {
                            if( SW_frames.size() > odoParam.SW_frame_size*2 )
                            {
                                if(need_kf)
                                {
                                    auto it0 = SW_frames.begin();
                                    auto it1 = SW_frames.begin();
                                    it1++;
                                    remove_frame( it0->first );
                                    remove_frame( it1->first );
                                    SW_frames.erase(it1);
                                    SW_frames.erase(it0);
                                }
                                else
                                {
                                    auto it0 = SW_frames.rbegin(); it0++;it0++;it0++;it0++;
                                    auto it1 = SW_frames.rbegin(); it1++;it1++;it1++;it1++;
                                    it1++;
                                    remove_frame( it0->first );
                                    remove_frame( it1->first );
                                    SW_frames.erase(it1->first);
                                    SW_frames.erase(it0->first);
                                }
                            }
                        }
                    }
                }

//                if(need_keyframe()) add_keyframe();
                last_frame = cur_frame;
            }
        }
    }

    void Odometry::retri_linefeatrues()
    {
        // 分两种情况，一种是平均重投影误差过大，直接重新tri；一种是某个观测的重投影过大，删去观测
        for(auto &id_feature:SW_features)
        {
            id_feature.second.retri_check(SW_frames);
        }
    }

    //TODO 关键帧判断
    bool Odometry::need_keyframe()
    {
        if( SW_frames.size() < ( odoParam.SW_frame_size * 2 ) ) return true;

        auto last_keyframe_id = last_key_frame->get_StereoId();
        auto cur_frame_id = cur_frame->get_StereoId();

        Eigen::Vector3d last_keyframe_t = std::get<0>(last_key_frame->get_Twc_ex()).block(0,3,3,1);
        Eigen::Vector3d cur_frame_t = std::get<0>(cur_frame->get_Twc_ex()).block(0,3,3,1);
        double distance = (last_keyframe_t - cur_frame_t).norm();

        return distance > 0.05;
    }

    void Odometry::add_keyframe()
    {
        // 将关键帧新的观测加入到SW_featuer中
        std::vector<long> feature_ids;
        std::vector< LineFeature > obs;
        std::tie( feature_ids, obs ) = cur_frame->get_id_obs();

        for(int index = 0;index<feature_ids.size();++index)
        {
            if(feature_ids[index] < 0)
            {
                feature_ids[index] = feature_id;
                SW_features.emplace(feature_id++, obs[index]);
            }
        }

        cur_frame->set_lineid(feature_ids);


        last_key_frame = cur_frame;
    }

    void Odometry::update_velocity()
    {
        Eigen::Matrix4d Twcl = last_frame->get_left_frame()->get_Twc();
        Eigen::Matrix4d Twcc = cur_frame->get_left_frame()->get_Twc();

        motion_velocity = Twcl.inverse()*Twcc;
    }

    void Odometry::optimization_SW()
    {
        lineProjectionFactor::sqrt_info  = 460 / 1.5 * Eigen::Matrix2d::Identity();
        lineProjectionRightFactor::sqrt_info  = 460 / 1.5 * Eigen::Matrix2d::Identity();
        ceres::Problem problem;

        //找到特征和帧的对应关系
        std::map< long, int > frameid_index;
        std::vector< std::pair< long, Eigen::Matrix4d > > frameid_Twcs;
        {
            int index = 0;
            for(auto &frame:SW_frames)
            {
                frameid_Twcs.emplace_back( frame.first, frame.second );
                frameid_index.emplace( frame.first, index++ );
            }
        }

        std::vector< std::vector< std::pair<int, Eigen::Vector4d> >> index_observe_s;
        std::vector< Eigen::Vector4d > features_param;
        {
            for(auto &id_feature:SW_features)
            {
                auto &feature = id_feature.second;
                if(feature.is_tri())
                {
                    Eigen::Vector4d feature_param = feature.get_orth_w(SW_frames);
                    features_param.push_back( feature_param );
                    auto obs = feature.get_obs();
                    std::vector< std::pair<int, Eigen::Vector4d> > index_obs_s;
                    for(auto &ob:obs)
                    {
                        int frameindex = frameid_index[ob.first];
                        Eigen::Vector4d ob4d = ob.second.get4dob();
                        index_obs_s.emplace_back( frameindex, ob4d );
                    }
                    index_observe_s.push_back(index_obs_s);
                }
            }
        }

        Eigen::Matrix4d Tlr = cur_frame->get_stereo_param()->Tlr;


        double para_Feature_line[features_param.size()][4];
        for(int index = 0; index<features_param.size(); ++index)
        {
            Eigen::Vector4d orth = features_param[index];

            para_Feature_line[index][0] = orth[0];
            para_Feature_line[index][1] = orth[1];
            para_Feature_line[index][2] = orth[2];
            para_Feature_line[index][3] = orth[3];

            ceres::LocalParameterization *local_parameterization = new LineOrthParameterization();
            problem.AddParameterBlock(para_Feature_line[index], 4, local_parameterization);
        }

        double para_Pose[frameid_Twcs.size()][7];
        for(int index = 0; index<frameid_Twcs.size(); ++index)
        {
            auto& Twc = frameid_Twcs[index].second;
            Eigen::Matrix3d Rot = Twc.block(0,0,3,3);
            Eigen::Vector3d Tran = Twc.block(0,3,3,1);
            Eigen::Quaterniond qua(Rot);

//            std::cout << "Twc = " << std::endl << Twc << std::endl;

            para_Pose[index][0] = Tran(0);
            para_Pose[index][1] = Tran(1);
            para_Pose[index][2] = Tran(2);
            para_Pose[index][3] = qua.x();
            para_Pose[index][4] = qua.y();
            para_Pose[index][5] = qua.z();
            para_Pose[index][6] = qua.w();

            ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
            problem.AddParameterBlock(para_Pose[index], 7, local_parameterization);
            if(index < 2)
            problem.SetParameterBlockConstant( para_Pose[index] );
        }


        double para_Pose_lr[7];
        {
            Eigen::Matrix3d Rot = Tlr.block(0,0,3,3);
            Eigen::Vector3d Tran = Tlr.block(0,3,3,1);
            Eigen::Quaterniond qua(Rot);

            para_Pose_lr[0] = Tran(0);
            para_Pose_lr[1] = Tran(1);
            para_Pose_lr[2] = Tran(2);
            para_Pose_lr[3] = qua.x();
            para_Pose_lr[4] = qua.y();
            para_Pose_lr[5] = qua.z();
            para_Pose_lr[6] = qua.w();

            ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
            problem.AddParameterBlock(para_Pose_lr, 7, local_parameterization);
            problem.SetParameterBlockConstant( para_Pose_lr );
        }


        for(int index = 0; index < index_observe_s.size(); ++index)
        {
            if(index_observe_s[index].size() < 3) continue;
            for( int index_in=0;index_in<index_observe_s[index].size();++index_in )
            {
                auto& index_ob4d = index_observe_s[index][index_in];
                auto frameindex = index_ob4d.first;
                auto& ob4d = index_ob4d.second;

                if( frameid_Twcs[frameindex].first % 2 == 0 )
                {
                    ceres::LossFunction* loss_function = nullptr;
                    loss_function = new ceres::CauchyLoss(1.);

                    auto cost_function = new lineProjectionFactor( ob4d );
                    problem.AddResidualBlock(cost_function, loss_function, para_Pose[frameindex], para_Feature_line[index]);
                }
                else
                {
                    ceres::LossFunction* loss_function = nullptr;
                    loss_function = new ceres::CauchyLoss(1.);

                    auto cost_function = new lineProjectionRightFactor( ob4d );
                    problem.AddResidualBlock(cost_function, loss_function, para_Pose[frameindex-1], para_Pose_lr, para_Feature_line[index]);
                }
            }
        }


        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_SCHUR;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;  // LEVENBERG_MARQUARDT  DOGLEG
//    options.linear_solver_type = ceres::SPARSE_SCHUR; // SPARSE_NORMAL_CHOLESKY  or SPARSE_SCHUR
        options.max_num_iterations = 20;
        options.minimizer_progress_to_stdout = false;

        TicToc solver_time;
        ceres::Solver::Summary summary;
        ceres::Solve (options, &problem, & summary);

        std::cout << summary.BriefReport()<<std::endl;
//        std::cout << summary.FullReport() <<std::endl;


        {
            int index = 0;
            for(auto &frame:SW_frames)
            {
                if(index % 2 == 0)
                {
                    Eigen::Matrix4d Twc;
                    Eigen::Quaterniond qua(para_Pose[index][6], para_Pose[index][3], para_Pose[index][4], para_Pose[index][5]);
                    Eigen::Matrix3d rot = qua.toRotationMatrix();
                    Eigen::Vector3d tran(para_Pose[index][0], para_Pose[index][1], para_Pose[index][2]);
                    Twc.block(0,3,3,1) = tran;
                    Twc.block(0,0,3,3) = rot;

                    frame.second = Twc;
                }
                else
                {
                    Eigen::Matrix4d Twl;
                    Eigen::Quaterniond qua(para_Pose[index-1][6], para_Pose[index-1][3], para_Pose[index-1][4], para_Pose[index-1][5]);
                    Eigen::Matrix3d rot = qua.toRotationMatrix();
                    Eigen::Vector3d tran(para_Pose[index-1][0], para_Pose[index-1][1], para_Pose[index-1][2]);
                    Twl.block(0,3,3,1) = tran;
                    Twl.block(0,0,3,3) = rot;

                    frame.second = Twl*Tlr;
                }
                index++;
            }
        }

        {
            int index = 0;
            for(auto &id_feature:SW_features)
            {
                auto &feature = id_feature.second;
                if(feature.is_tri())
                {

                    Eigen::Vector4d line_orth(para_Feature_line[index][0], para_Feature_line[index][1], para_Feature_line[index][2], para_Feature_line[index][3]);

                    feature.set_orth_w( line_orth, SW_frames );

                    index++;
                }
            }
        }

    }

#define TRACK_MOTION_DEBUG 0
    int Odometry::track_motion_mode()
    {
        cur_frame->update_Twc( last_frame->get_left_frame()->get_Twc()*motion_velocity );

        cv::Mat last_frame_desc;
        std::vector<Eigen::Vector4d> last_lines_pixel;
        std::vector< long > last_frame_id;
        std::vector< long > feature_ids;
        std::set<long> candite_id;
        {
            std::vector<long> line_id;
            std::vector< LineFeature > obs;
            std::tie(line_id, obs) = last_frame->get_id_obs();

            for( int i=0;i<line_id.size(); ++i )
            {
                if(line_id[i] < 0) continue;
                if(SW_features.count(line_id[i]) == 0) continue;

                auto it = SW_features.find(line_id[i]);
                if(it->second.is_tri())
                {
                    cv::Mat des = obs[i].get_descri();
                    last_frame_desc.push_back( des.row(0) );
                    candite_id.insert( line_id[i] );
                    feature_ids.push_back( it->first );

                    if(TRACK_MOTION_DEBUG)
                    {
                        last_frame_id.push_back( obs[i].get_obs().begin()->first );
                        Eigen::Vector4d ob_norm = obs[i].get_obs().begin()->second.get4dob();
                        Eigen::Vector4d ob_pixel = cur_frame->get_left_frame()->get_monoparam()->GetPixel4d(ob_norm);
                        last_lines_pixel.push_back(ob_pixel);
                    }
                }
            }
        }

        cv::Mat cur_des;
        std::vector<Eigen::Vector4d> cur_lines_pixel;
        std::vector< long > cur_frame_id;
        {
            std::vector<long> stereo_id;
            std::vector< LineFeature > obs;
            std::tie(stereo_id, obs) = cur_frame->get_id_obs();

            for( int i=0;i<obs.size(); ++i )
            {
                cv::Mat des = obs[i].get_descri();
                cur_des.push_back( des.row(0) );

                if(TRACK_MOTION_DEBUG)
                {
                    cur_frame_id.push_back( obs[i].get_obs().begin()->first );
                    Eigen::Vector4d ob_norm = obs[i].get_obs().begin()->second.get4dob();
                    Eigen::Vector4d ob_pixel = cur_frame->get_left_frame()->get_monoparam()->GetPixel4d(ob_norm);
                    cur_lines_pixel.push_back(ob_pixel);
                }
            }
        }

//        auto match_result = matchNNR(last_frame_desc, cur_des);


        auto match_result = matchNNR(cur_des, last_frame_desc);

        if(match_result.size() < odoParam.track_motion_mini_line_num) return match_result.size();
//        std::cout << "motion match_result.size() = " << match_result.size() << std::endl;
//        std::cout << "match_result2.size() = " << match_result2.size() << std::endl;

        std::map<int, int> match_opti;
        for(auto &ma:match_result)
        {
            match_opti.emplace( ma.first, feature_ids[ma.second] );
        }
        Eigen::Matrix4d cur_pos, Tlr;
        std::tie( cur_pos, Tlr ) = cur_frame->get_Twc_ex();
        optimization_curpose( cur_pos, Tlr, match_opti );
        cur_frame->update_Twc( cur_pos );

        if(TRACK_MOTION_DEBUG)
        {
            std::cout << "match_result.size() = " << match_result.size() << std::endl;
            show_match( match_result, cur_lines_pixel, last_lines_pixel, cur_frame_id, last_frame_id );
        }

        return match_result.size();
    }

#define TRACK_SW_DEBUG 0
    int Odometry::track_sw()
    {
        cv::Mat sw_desc;
        std::vector< long > sw_feature_ids;

        for(auto& id_feature:SW_features)
        {
            auto& featureId = id_feature.first;
            auto& feature = id_feature.second;
            if(feature.is_tri())
            {
                cv::Mat des = feature.get_descri();
                sw_desc.push_back( des.row(0) );
                sw_feature_ids.push_back( featureId );
            }
        }

        cv::Mat cur_des;
        std::vector<Eigen::Vector4d> cur_lines_pixel;
        std::vector< long > cur_frame_id;
        {
            std::vector<long> stereo_id;
            std::vector< LineFeature > obs;
            std::tie(stereo_id, obs) = cur_frame->get_id_obs();

            for(auto & ob : obs)
            {
                cv::Mat des = ob.get_descri();
                cur_des.push_back( des.row(0) );

                if(TRACK_SW_DEBUG)
                {
                    cur_frame_id.push_back( ob.get_obs().begin()->first );
                    Eigen::Vector4d ob_norm = ob.get_obs().begin()->second.get4dob();
                    Eigen::Vector4d ob_pixel = cur_frame->get_left_frame()->get_monoparam()->GetPixel4d(ob_norm);
                    cur_lines_pixel.push_back(ob_pixel);
                }
            }
        }

//        auto match_result = matchNNR(last_frame_desc, cur_des);


        auto match_result = matchNNR(cur_des, sw_desc);


        if(match_result.size() < odoParam.track_SW_mini_line_num) return match_result.size();
//        std::cout << "sw match_result.size() = " << match_result.size() << std::endl;

//        std::cout << "match_result.size() = " << match_result.size() << std::endl;
//        std::cout << "match_result2.size() = " << match_result2.size() << std::endl;

        std::map<int, int> match_opti;
        for(auto &ma:match_result)
        {
            match_opti.emplace( ma.first, sw_feature_ids[ma.second] );
        }
        Eigen::Matrix4d cur_pos, Tlr;
        std::tie( cur_pos, Tlr ) = cur_frame->get_Twc_ex();
        optimization_curpose( cur_pos, Tlr, match_opti );
        cur_frame->update_Twc( cur_pos );

        update_cur_frame_feature_id( match_opti );

        if(TRACK_SW_DEBUG)
        {
            std::cout << "match_result.size() = " << match_result.size() << std::endl;
//            show_match( match_result, cur_lines_pixel, last_lines_pixel, cur_frame_id, last_frame_id );
        }

        return match_result.size();
    }

    void Odometry::update_cur_frame_feature_id( const std::map<int, int>& match_opti )
    {
        int obs_nums = cur_frame->get_obs_num();
        std::vector< long > obs_feature_id(obs_nums, -1);

        for(const auto &ma:match_opti)
        {
            obs_feature_id[ma.first] = ma.second;
        }

        cur_frame->set_lineid( obs_feature_id );
    }

    void Odometry::optimization_curpose( Eigen::Matrix4d& Twc, const Eigen::Matrix4d& Tlr, const std::map<int, int>& match )
    {
        lineProjectionFactor::sqrt_info  = 460 / 1.5 * Eigen::Matrix2d::Identity();
        lineProjectionRightFactor::sqrt_info  = 460 / 1.5 * Eigen::Matrix2d::Identity();
        ceres::Problem problem;

        std::vector<long> frameids;
        std::vector<Eigen::Vector4d> obs;
        std::vector<Eigen::Vector4d> orths;
        std::vector< int > obs_line; // 观测是哪条线的

        std::vector<long> stereo_id;
        std::vector< LineFeature > line_obs;
        std::tie(stereo_id, line_obs) = cur_frame->get_id_obs();
        for(auto &ma:match)
        {
            auto obs_single_line = line_obs[ma.first].get_obs();
            for(auto &ob:obs_single_line)
            {
                frameids.push_back(ob.first);
                obs.push_back(ob.second.get4dob());
                obs_line.push_back(orths.size());
            }
            Eigen::Vector4d orth = SW_features[ma.second].get_orth_w( SW_frames );
            orths.push_back(orth);
        }

        double para_Feature_line[orths.size()][4];
        for(int index = 0; index<orths.size(); ++index)
        {
            Eigen::Vector4d orth = orths[index];

            para_Feature_line[index][0] = orth[0];
            para_Feature_line[index][1] = orth[1];
            para_Feature_line[index][2] = orth[2];
            para_Feature_line[index][3] = orth[3];

            ceres::LocalParameterization *local_parameterization = new LineOrthParameterization();
            problem.AddParameterBlock(para_Feature_line[index], 4, local_parameterization);
            problem.SetParameterBlockConstant(para_Feature_line[index]);
        }

        double para_Pose[2][7];
        {
            Eigen::Matrix3d Rot = Twc.block(0,0,3,3);
            Eigen::Vector3d Tran = Twc.block(0,3,3,1);
            Eigen::Quaterniond qua(Rot);

            para_Pose[0][0] = Tran(0);
            para_Pose[0][1] = Tran(1);
            para_Pose[0][2] = Tran(2);
            para_Pose[0][3] = qua.x();
            para_Pose[0][4] = qua.y();
            para_Pose[0][5] = qua.z();
            para_Pose[0][6] = qua.w();

            ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
            problem.AddParameterBlock(para_Pose[0], 7, local_parameterization);
        }
        {
            Eigen::Matrix3d Rot = Tlr.block(0,0,3,3);
            Eigen::Vector3d Tran = Tlr.block(0,3,3,1);
            Eigen::Quaterniond qua(Rot);

            para_Pose[1][0] = Tran(0);
            para_Pose[1][1] = Tran(1);
            para_Pose[1][2] = Tran(2);
            para_Pose[1][3] = qua.x();
            para_Pose[1][4] = qua.y();
            para_Pose[1][5] = qua.z();
            para_Pose[1][6] = qua.w();

            ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
            problem.AddParameterBlock(para_Pose[1], 7, local_parameterization);
            problem.SetParameterBlockConstant( para_Pose[1] );
        }


        for(int index = 0; index < obs.size(); ++index)
        {
            auto observe = obs[index];
            auto index_feature = obs_line[index];
            if( frameids[index]%2 == 0 )
            {
                ceres::LossFunction* loss_function = nullptr;
                loss_function = new ceres::CauchyLoss(1.);

                auto cost_function = new lineProjectionFactor( observe );
                problem.AddResidualBlock(cost_function, loss_function, para_Pose[0], para_Feature_line[index_feature]);

            }
            else
            {
                ceres::LossFunction* loss_function = nullptr;
                loss_function = new ceres::CauchyLoss(1.);

                auto cost_function = new lineProjectionRightFactor( observe );
                problem.AddResidualBlock(cost_function, loss_function, para_Pose[0], para_Pose[1], para_Feature_line[index_feature]);
            }
        }


        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;  // LEVENBERG_MARQUARDT  DOGLEG
//    options.linear_solver_type = ceres::SPARSE_SCHUR; // SPARSE_NORMAL_CHOLESKY  or SPARSE_SCHUR
        options.max_num_iterations = 10;
        options.minimizer_progress_to_stdout = false;

        TicToc solver_time;
        ceres::Solver::Summary summary;
        ceres::Solve (options, &problem, & summary);

//        std::cout << summary.FullReport()<<std::endl;

//        std::cout << "before opti " << std::endl << Twc << std::endl;

        {
            Eigen::Quaterniond qua(para_Pose[0][6], para_Pose[0][3], para_Pose[0][4], para_Pose[0][5]);
            Eigen::Matrix3d rot = qua.toRotationMatrix();
            Eigen::Vector3d tran(para_Pose[0][0], para_Pose[0][1], para_Pose[0][2]);
            Twc.block(0,3,3,1) = tran;
            Twc.block(0,0,3,3) = rot;
        }
//        std::cout << "after opti " << std::endl << Twc << std::endl;
    }

    void Odometry::show_match(
            std::map<int, int> &match_result,
            std::vector<Eigen::Vector4d> &cur_lines_pixel,
            std::vector<Eigen::Vector4d> &last_lines_pixel,
            std::vector< long >& cur_frame_id,
            std::vector< long >& last_frame_id)
    {
        cv::Mat last_img = last_frame->get_left_frame()->get_img();
        cv::Mat cur_img = cur_frame->get_left_frame()->get_img();
        if(last_img.channels() != 3) cv::cvtColor(last_img, last_img, cv::COLOR_GRAY2BGR);
        if(cur_img.channels() != 3) cv::cvtColor(cur_img, cur_img, cv::COLOR_GRAY2BGR);
        int lowest = 0, highest = 255;
        int range = (highest - lowest) + 1;
        for(auto &m : match_result)
        {
            if( cur_frame_id[m.first]%2==0 && last_frame_id[m.second]%2==0 )
            {
                auto &cur_line = cur_lines_pixel[m.second];
                auto &last_line = last_lines_pixel[m.first];
                unsigned int r = lowest + int(rand() % range);
                unsigned int g = lowest + int(rand() % range);
                unsigned int b = lowest + int(rand() % range);
                cv::Point cur_sp = cv::Point(int(cur_line(0)), int(cur_line(1)));
                cv::Point cur_ep = cv::Point(int(cur_line(2)), int(cur_line(3)));
                cv::line(cur_img, cur_sp, cur_ep, cv::Scalar(r, g, b),2 ,8);

                cv::Point last_sp = cv::Point(int(last_line(0)), int(last_line(1)));
                cv::Point last_ep = cv::Point(int(last_line(2)), int(last_line(3)));
                cv::line(last_img, last_sp, last_ep, cv::Scalar(r, g, b),2 ,8);
            }

        }

//        std::cout << "cur_lines_pixel.size() = " << cur_lines_pixel.size() << std::endl;
//        std::cout << "last_lines_pixel.size() = " << last_lines_pixel.size() << std::endl;
//        std::cout << "match_result.size() = " << match_result.size() << std::endl;

        cv::imshow("last_img", last_img);
        cv::imshow("cur_img", cur_img);
        cv::waitKey();
    }


    std::map< int, int > Odometry::matchNNR(const cv::Mat &desc1, const cv::Mat &desc2, float nnr)
    {
        std::map< int, int > match_result_12;


        std::vector<std::vector<cv::DMatch>> matches_;
        cv::Ptr<cv::line_descriptor::BinaryDescriptorMatcher> bdm_;
        bdm_ = cv::line_descriptor::BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
        bdm_->knnMatch( desc1, desc2, matches_, 3 );

        for (int idx = 0; idx < desc1.rows; ++idx)
        {
//            std::cout << "radio = " << matches_[idx][0].distance / matches_[idx][1].distance << std::endl;
//            std::cout << "matches_[idx][0].distance = " <<  matches_[idx][0].distance << std::endl;;
//            std::cout << "matches_[idx][1].distance = " <<  matches_[idx][1].distance << std::endl;
//            std::cout << "matches_[idx][2].distance = " <<  matches_[idx][2].distance << std::endl;
//            if (matches_[idx][0].distance < odoParam.match_dist_thread)
            if (matches_[idx][0].distance < matches_[idx][1].distance * nnr  && matches_[idx][0].distance < odoParam.match_dist_thread)
            {
                match_result_12.emplace( idx, matches_[idx][0].trainIdx );
            }
        }

        return match_result_12;
    }
}