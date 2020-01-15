#include "optflow_line_matcher.h"
#include <algorithm>

namespace line_tracker {

void LineMatcher::opticalFlowBasedLineMatch(const cv::Mat& ref_frame, const cv::Mat& cur_frame,
        const std::vector<KeyLine>& ref_kls, std::vector<KeyLine>& cur_kls, std::vector<bool>& kl_status) {
    cur_kls.clear();
    kl_status.clear();
    if (ref_kls.empty()) {
        std::cout << "There are no KeyLines in reference frame" << std::endl;
        return;
    }
    for (size_t kl_id = 0; kl_id < ref_kls.size(); ++kl_id) {

//        std::cout << kl_id << std::endl;

        lineFlowVectorReset();
        KeyLine cur_kl;
        if(!getLineGoodFtrsToTrack(ref_frame, ref_kls[kl_id])) {
            cur_kls.push_back(cur_kl);
            kl_status.push_back(false);
            continue;
        }

        opticalFlowTrack(ref_frame, cur_frame, kl_flow_vector_.ref_kl_good_kps_,
                         kl_flow_vector_.cur_kl_good_kps_, kl_flow_vector_.status_);

        if (!consistencyCheck(ref_frame, cur_frame, kl_flow_vector_.ref_kl_good_kps_,
                              kl_flow_vector_.cur_kl_good_kps_)) {
            cur_kls.push_back(cur_kl);
            kl_status.push_back(false);
            continue;
        }

        cv::Point2f start_pt, end_pt;
        findStartEndPoints(start_pt, end_pt);
        cur_kl.startPointX = start_pt.x;
        cur_kl.startPointY = start_pt.y;
        cur_kl.endPointX = end_pt.x;
        cur_kl.endPointY = end_pt.y;
        cur_kl.octave = 0;

        cur_kls.push_back(cur_kl);
        kl_status.push_back(true);
    }
}

void LineMatcher::lineFlowVectorReset() {
    kl_flow_vector_.ref_kl_good_kps_.clear();
    kl_flow_vector_.cur_kl_good_kps_.clear();
    kl_flow_vector_.status_.clear();
}

bool numericSort(const std::pair<float, int>& a, const std::pair<float, int>& b) {
    return a.first > b.first;
}

void LineMatcher::findStartEndPoints(cv::Point2f& start_pt, cv::Point2f& end_pt) {
    std::vector<std::pair<float, int> > sort_x, sort_y;
    for (int i = 0; i < (int) kl_flow_vector_.cur_kl_good_kps_.size(); ++i) {
        if (kl_flow_vector_.status_.at(i)) {
            cv::Point2f cur_kp = kl_flow_vector_.cur_kl_good_kps_[i];
            sort_x.push_back(std::make_pair(cur_kp.x, i));
            sort_y.push_back(std::make_pair(cur_kp.y, i));
        }
    }
    std::sort(sort_x.begin(), sort_x.end(), numericSort);
    std::sort(sort_y.begin(), sort_y.end(), numericSort);

    float dist_x = std::fabs(sort_x.front().first - sort_x.back().first);
    float dist_y = std::fabs(sort_y.front().first - sort_y.back().first);

    if (dist_x > dist_y) {
        start_pt = kl_flow_vector_.cur_kl_good_kps_[sort_x.front().second];
        end_pt = kl_flow_vector_.cur_kl_good_kps_[sort_x.back().second];
    } else {
        start_pt = kl_flow_vector_.cur_kl_good_kps_[sort_y.front().second];
        end_pt = kl_flow_vector_.cur_kl_good_kps_[sort_y.back().second];
    }
}

bool LineMatcher::getLineGoodFtrsToTrack(const cv::Mat& ref_frame, const KeyLine& ref_kl) {
    cv::Point2f start_pt = ref_kl.getStartPoint();
    cv::Point2f end_pt = ref_kl.getEndPoint();
    cv::Point2f dir = end_pt - start_pt;
    const float dir_norm = std::sqrt(std::pow(dir.x, 2) + std::pow(dir.y, 2));

    if (std::ceil(dir_norm) < min_kl_length_)
        return false;

    dir.x /= dir_norm;
    dir.y /= dir_norm;

    int n_steps = std::floor(dir_norm / (float) line_step_);
    std::vector<std::pair<cv::Point2f, float> > kl_kp_candidates;
    for (int ii = 0; ii < n_steps; ++ii) {
        int l = ii * line_step_;
        cv::Point2f kp_cand(start_pt.x + l * dir.x, start_pt.y + l * dir.y);
        kl_kp_candidates.push_back(std::make_pair(kp_cand, scoreOfShiTomasi(ref_frame, kp_cand.x, kp_cand.y)));
    }
    kl_kp_candidates.push_back(std::make_pair(end_pt, scoreOfShiTomasi(ref_frame, end_pt.x, end_pt.y)));

    nonMaximumSuppression(kl_kp_candidates);
    kl_flow_vector_.ref_kl_good_kps_.clear();
    for (auto kp_it = kl_kp_candidates.begin(); kp_it != kl_kp_candidates.end(); ++kp_it) {
        if ((*kp_it).second > 1e-8)
            kl_flow_vector_.ref_kl_good_kps_.push_back((*kp_it).first);
    }

    // TODO: only keey 50% better line_kp_candidiates
    if (kl_flow_vector_.ref_kl_good_kps_.size() < min_kl_kps_)
        return false;

    return true;
}

float LineMatcher::scoreOfShiTomasi(const cv::Mat& image, int u, int v) {
    CV_Assert(image.type() == CV_8UC1);

    float dXX = 0.0, dYY = 0.0, dXY = 0.0;
    const int half_box_sz = 4;
    const int box_sz = 2 * half_box_sz;
    const int box_area = box_sz * box_sz;
    const int x_min = u - half_box_sz;
    const int x_max = u + half_box_sz;
    const int y_min = v - half_box_sz;
    const int y_max = v + half_box_sz;

    if (x_min < 1 || x_max >= image.cols - 1 || y_min < 1 || y_max >= image.rows - 1)
        return 0.0;

    const int stride = image.step.p[0];
    for (int y = y_min; y < y_max; ++y) {
        const uint8_t* ptr_l = image.data + stride * y + x_min - 1;
        const uint8_t* ptr_r = image.data + stride * y + x_min + 1;
        const uint8_t* ptr_t = image.data + stride * (y - 1) + x_min;
        const uint8_t* ptr_b = image.data + stride * (y + 1) + x_min;
        for (int x = 0; x < box_sz; ++x, ++ptr_l, ++ptr_r, ++ptr_t, ++ptr_b) {
            float dx = *ptr_r - *ptr_l;
            float dy = *ptr_b - *ptr_t;
            dXX += dx * dx;
            dYY += dy * dy;
            dXY += dx * dy;
        }
    }

    dXX /= (2.0 * box_area);
    dYY /= (2.0 * box_area);
    dXY /= (2.0 * box_area);

    return 0.5 * (dXX + dYY - std::sqrt(std::pow(dXX + dYY, 2) - 4 *(dXX * dYY - dXY * dXY)));
}

bool candSort(const std::pair<cv::Point2f, float>& a, const std::pair<cv::Point2f, float>& b) {
    return a.second > b.second;
}

void LineMatcher::nonMaximumSuppression(std::vector<std::pair<cv::Point2f, float> >& kl_kp_candidates) {
    std::vector<bool> is_max_score(kl_kp_candidates.size(), false);
    float score_l, score_r, score_cand;
    int cand_sz = (int) kl_kp_candidates.size();
    for (int cand_id = 0; cand_id < cand_sz; ++cand_id) {
        score_l = cand_id == 0 ? -1 : kl_kp_candidates.at(cand_id - 1).second;
        score_r = cand_id == (cand_sz - 1) ? -1 : kl_kp_candidates.at(cand_id + 1).second;
        score_cand = kl_kp_candidates[cand_id].second;
        if (score_cand > score_l && score_cand > score_r)
            is_max_score[cand_id] = true;
    }

    for (int i = 0; i < (int) is_max_score.size(); ++i) {
        if (!is_max_score[i])
            kl_kp_candidates[i].second = 0;
    }

    std::sort(kl_kp_candidates.begin(), kl_kp_candidates.end(), candSort);
}

void LineMatcher::opticalFlowTrack(const cv::Mat& ref_frame, const cv::Mat& cur_frame,
                                   const std::vector<cv::Point2f>& ref_kl_good_kps,
                                   std::vector<cv::Point2f>& cur_kl_good_kps,
                                   std::vector<bool>& flow_vector_status) {
    CV_Assert(!ref_kl_good_kps.empty());
    cur_kl_good_kps.clear();
    cur_kl_good_kps.insert(cur_kl_good_kps.begin(), ref_kl_good_kps.begin(), ref_kl_good_kps.end());
    flow_vector_status.resize(ref_kl_good_kps.size(), true);

    const int klt_win_sz = 30;
    const int klt_max_iter = 30;
    const double klt_eps = 0.001;
    std::vector<float> klt_error;
    std::vector<unsigned char> klt_status;

    cv::TermCriteria termcrit(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, klt_max_iter, klt_eps);
    cv::calcOpticalFlowPyrLK(ref_frame, cur_frame,
                             ref_kl_good_kps, cur_kl_good_kps,
                             klt_status, klt_error,
                             cv::Size2i(klt_win_sz, klt_win_sz),
                             4, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW);

    for (int i = 0; i < (int) klt_status.size(); ++i)
        if (!klt_status[i])
            flow_vector_status.at(i) = false;
}

bool LineMatcher::consistencyCheck(const cv::Mat& ref_frame, const cv::Mat& cur_frame,
                                   const std::vector<cv::Point2f>& ref_kl_good_kps,
                                   const std::vector<cv::Point2f>& cur_kl_good_kps) {
    std::vector<bool> temp_flow_vector_status;
    std::vector<cv::Point2f> temp_ref_kl_good_kps;
    opticalFlowTrack(cur_frame, ref_frame, cur_kl_good_kps, temp_ref_kl_good_kps, temp_flow_vector_status);

    for (int i = 0; i < (int) temp_flow_vector_status.size(); ++i) {
        if (!kl_flow_vector_.status_.at(i))
            continue;
        if (!temp_flow_vector_status[i]) {
            kl_flow_vector_.status_.at(i) = false;
            continue;
        }
        cv::Point2f ref_kp_origin = ref_kl_good_kps.at(i);
        cv::Point2f ref_kp_retrack = temp_ref_kl_good_kps.at(i);
        cv::Point2f v = ref_kp_origin - ref_kp_retrack;
        float dist = std::sqrt(std::pow(v.x, 2) + std::pow(v.y, 2));

        if (dist > min_dist_)
            kl_flow_vector_.status_.at(i) = false;
    }

    float valid_kps_rate = (float) getNumOfGoodKeyPoints(kl_flow_vector_.status_) / (int) ref_kl_good_kps.size();
    if (valid_kps_rate < min_valid_kps_rate_)
        return false;

    lineRANSAC(cur_kl_good_kps);

    return true;
}

int LineMatcher::getNumOfGoodKeyPoints(const std::vector<bool>& flow_vector_status) {
    int line_kp_num = 0;
    std::for_each(flow_vector_status.begin(), flow_vector_status.end(), [&](bool kp_statue) {
        if (kp_statue)
            ++line_kp_num;
    });
    return line_kp_num;
}

void LineMatcher::lineRANSAC(const std::vector<cv::Point2f>& cur_kl_good_kps) {
    const double p = 0.99;
    int n_iter = 500;
    int max_support = 0;

    std::vector<int> random_ids, inliers_ids;
    for (int id = 0; id < (int) kl_flow_vector_.status_.size(); ++id)
        if (kl_flow_vector_.status_[id])
            random_ids.push_back(id);

    int k = 0;
    while (k < n_iter) {
        ++k;
        int n_inliers = 0;
        std::random_shuffle(random_ids.begin(), random_ids.end());

        cv::Vec3f k_line_model;
        cv::Point2f a = cur_kl_good_kps.at(random_ids[0]);
        cv::Point2f b = cur_kl_good_kps.at(random_ids[1]);
        getTheLineModel(a, b, k_line_model);

        std::vector<int> k_inliers_ids;
        getInliers(cur_kl_good_kps, random_ids, k_line_model, k_inliers_ids);
        n_inliers = (int) k_inliers_ids.size();

        if (n_inliers > max_support) {
            max_support = n_inliers;
            inliers_ids = k_inliers_ids;
            kl_flow_vector_.cur_kl_model_ = k_line_model;

            double e = 1.0 - (double) n_inliers / (double) random_ids.size();
            n_iter = std::ceil(std::log(1 - p) / std::log(1 - std::pow(1 - e, line_model_param_num_)));
        }
    }

    int sz = kl_flow_vector_.status_.size();
    kl_flow_vector_.status_.clear();
    kl_flow_vector_.status_.resize(sz, false);
    for (int i = 0; i < (int) inliers_ids.size(); ++i)
        kl_flow_vector_.status_.at(inliers_ids[i]) = true;
}

void LineMatcher::getTheLineModel(const cv::Point2f& a, const cv::Point2f& b, cv::Vec3f& line_model) {
    float r = std::sqrt(std::pow(b.y - a.y, 2) + std::pow(a.x - b.x, 2));
    float A = (b.y - a.y) / r;
    float B = (a.x - b.x) / r;
    float C = (a.y * b.x - a.x * b.y) / r;

    line_model = cv::Vec3f(A, B, C);
}

void LineMatcher::getInliers(const std::vector<cv::Point2f>& cur_kl_good_kps,
                             const std::vector<int>& random_ids, const cv::Vec3f& line_model,
                             std::vector<int>& inliers_ids) {
    inliers_ids.clear();
    const float dist_thresh = 1;
    for (int i = 0; i < (int) random_ids.size(); ++i) {
        cv::Point2f kp = cur_kl_good_kps.at(random_ids[i]);
        float dist = std::fabs(kp.x * line_model[0] + kp.y * line_model[1] + line_model[2]);
        if (dist < dist_thresh)
            inliers_ids.push_back(random_ids[i]);
    }
}

} /* line_tracker  */
