#ifndef OPTFLOW_LINE_MATCHER_H
#define OPTFLOW_LINE_MATCHER_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/line_descriptor.hpp>

using namespace cv::line_descriptor;

namespace line_tracker {
struct LineFlowVector {
    bool is_good_match_;
    int num_good_pairs_;
    cv::Vec3f cur_kl_model_;

    std::vector<cv::Point2f> ref_kl_good_kps_;
    std::vector<cv::Point2f> cur_kl_good_kps_;
    std::vector<bool> status_;
};

class LineMatcher {
  private:
    const int min_kl_length_ = 10;
    const int min_kl_kps_ = 8;
    const int line_step_ = 2;
    const int min_dist_ = 2;
    const float min_valid_kps_rate_ = 0.5;

    static const int line_model_param_num_ = 2;
  private:
    LineFlowVector kl_flow_vector_;

  public:
    LineMatcher () {}
    virtual ~LineMatcher () {}
    void opticalFlowBasedLineMatch(const cv::Mat& ref_frame, const cv::Mat& cur_frame,
                                   const std::vector<KeyLine>& ref_kls,
                                   std::vector<KeyLine>& cur_kls, std::vector<bool>& kl_status);
  private:
    void lineFlowVectorReset();
    void findStartEndPoints(cv::Point2f& start_pt, cv::Point2f& end_pt);

    bool getLineGoodFtrsToTrack(const cv::Mat& ref_frame, const KeyLine& ref_kl);
    float scoreOfShiTomasi(const cv::Mat& image, int u, int v);
    void nonMaximumSuppression(std::vector<std::pair<cv::Point2f, float> >& kl_kp_candidates);

    void opticalFlowTrack(const cv::Mat& ref_frame, const cv::Mat& cur_frame,
                          const std::vector<cv::Point2f>& ref_kl_good_kps,
                          std::vector<cv::Point2f>& cur_kl_good_kps,
                          std::vector<bool>& flow_vector_status);

    bool consistencyCheck(const cv::Mat& ref_frame, const cv::Mat& cur_frame,
                          const std::vector<cv::Point2f>& ref_kl_good_kps,
                          const std::vector<cv::Point2f>& cur_kl_good_kps);
    int getNumOfGoodKeyPoints(const std::vector<bool>& flow_vector_status);
    void lineRANSAC(const std::vector<cv::Point2f>& cur_kl_good_kps);
    void getTheLineModel(const cv::Point2f& a, const cv::Point2f& b, cv::Vec3f& line_model);
    void getInliers(const std::vector<cv::Point2f>& cur_kl_good_kps,
                    const std::vector<int>& random_ids, const cv::Vec3f& line_model,
                    std::vector<int>& inliers_ids);
};
} /* line_tracker */

#endif /* OPTFLOW_LINE_MATCHER_H */
