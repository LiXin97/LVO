#ifndef _LINE_RECONSTRUCTION_H
#define _LINE_RECONSTRUCTION_H

#include <Eigen/Eigen>
#include "gc.h"
#include "lineMatcher.h"
#include <fstream>
#include <iomanip>

using namespace std;
using namespace cv;
using namespace Eigen;

const double inverse_depth = 0.1;
const double line_vn_angle_thr = 3 * 3.141592654 / 180;
const double extension_length = 10.0; // 5.0

inline bool IsNumber(double x)
{
	// This looks like it should always be true, 
	// but it's false if x is a NaN.
	return (x == x);
}

inline bool IsFiniteNumber(double x)
{
	return (x <= DBL_MAX && x >= -DBL_MAX);
}

inline bool IsValidVector6d(Vector6d v)
{
	for (int i = 0; i < 6; i++){
		if (!IsNumber(v[i]))
			return false;
	}
	return true;
}
/*
typedef struct _obs_t {
	_obs_t() {}
	_obs_t(int id, Vector8d obs) : id(id), obs(obs) {}

	int id;
	Vector8d obs;
} obs_t;   // id: feature_id or kf_id

typedef struct {
	Vector6d line;
	Vector2d tt;
	Vector3d pvn;
	bool twice_observed;
	bool ba_updated;
	bool currently_visible;
	int init_kfid;
	vector<obs_t> obs_vec;
} landmark_t;
*/

void insert_curr_obs(Vector8d& obs, float fx1, float fy1, float cx1, float cy1);
Vector6d initialize_lm(Vector8d obs, double baseline);
void add_lms(Vector8d obs, double baseline, landmark_t* lm);
void extend_end_points(Vector8d obs, pose_t T, float baseline, landmark_t* lm, Vector2d& tt);
Vector6d save_landmark(Vector6d line, pose_t T, Vector2d tt);

Vector6d line_2D_to_3D(Vector8d& obs, double baseline, float fx1, float fy1, float cx1, float cy1);

float extend_line_seg(float x1, float y1, float x2, float y2, float y);
void stereo_3D_Pt(double ul, double ur, float v, float fx, float fy,
	float baseline, double& x, double& y, double& z);
Vector6d line_2D_to_3D_v2(Vector8d obs, double baseline, float fx1, float fy1, float cx1, float cy1);

Vector8d formatConversion(Line line1, Line line2);

bool lineRecon(
	vector<Line> vecLine1, vector<Line> vecLine2,
	vector<pair<int, int>> lineMatch,
	float baseline, float fx, float fy, float cx, float cy,
	vector<pair<Vector8d, Vector6d>>& line_seg_para,
	vector<pair<int, int>>& line_match_recon,
	int reconMethod = 0);

bool save_stereo_results(
	vector<pair<Vector8d, Vector6d>>& line_seg_para,
	vector<pair<int, int>>& line_match_recon,
	map<int, Junction> mapJunc1,
	int reconMethod, int idxFrame);

#endif