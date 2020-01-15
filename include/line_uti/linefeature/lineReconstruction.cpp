#include "lineReconstruction.h"

// void SLAM::insert_curr_obs( int feature_id, Vector8d obs )
void insert_curr_obs(Vector8d& obs,	float fx1, float fy1, float cx1, float cy1)
{
	obs(0) = obs(0) / fx1 - cx1 / fx1;
	obs(1) = obs(1) / fy1 - cy1 / fy1;
	obs(2) = obs(2) / fx1 - cx1 / fx1;
	obs(3) = obs(3) / fy1 - cy1 / fy1;
	obs(4) = obs(4) / fx1 - cx1 / fx1;
	obs(5) = obs(5) / fy1 - cy1 / fy1;
	obs(6) = obs(6) / fx1 - cx1 / fx1;
	obs(7) = obs(7) / fy1 - cy1 / fy1;
	return;
}

// initialize_lm
Vector6d initialize_lm(Vector8d obs, double baseline) 
{
	Vector6d line;

	Vector3d p1(obs(0), obs(1), 1);
	Vector3d p2(obs(2), obs(3), 1);
	Vector3d p3(obs(4) + baseline, obs(5), 1);
	Vector3d p4(obs(6) + baseline, obs(7), 1);

	Vector4d pi1 = gc_ppp_pi(p1, p2, Vector3d(0, 0, 0));
	Vector4d pi2 = gc_ppp_pi(p3, p4, Vector3d(baseline, 0, 0));

	Vector6d plk = gc_pipi_plk(pi1, pi2);
	Vector3d n = plk.head(3);
	Vector3d v = plk.tail(3);
	Vector3d cp = gc_plucker_origin(n, v);

	double cpn = cp.norm();
	if (cpn < 0.1 || cpn > 10.0) {
		Vector3d cpu = cp / cpn;
		// cout << "cpn = " << cpn << endl;
		cp = cpu / inverse_depth;
	}
	if (cp(2) < 0)
		cp = -cp;

	line.head(3) = cp;
	line.tail(3) = v;

	return line;
}

// void SLAM::add_lms()
void add_lms(Vector8d obs, double baseline, landmark_t* lm)
{
//	landmark_t* lm = new landmark_t;
//	lm->line = initialize_lm(obit->second, baseline);
	lm->line = initialize_lm(obs, baseline);
	lm->twice_observed = false;
	lm->ba_updated = false;
	lm->currently_visible = false;
	lm->tt = Vector2d(0, 0);
//	lm->obs_vec.push_back(obs_t(kfid + 1, obit->second));
	lm->obs_vec.push_back(obs_t(0, obs));
//	lm->init_kfid = kfid + 1;
	lm->init_kfid = 0;
	Vector3d dv = lm->line.tail(3);
	lm->pvn = gc_vec3_normalize(dv);

	return;
}

// void SLAM::extend_end_points()
void extend_end_points(Vector8d obs, pose_t T, float baseline, landmark_t* lm, Vector2d& tt)
{
//	landmark_t* lm = add_lms(obs, baseline);
//	Vector2d tt;
//	lm = add_lms(obs, baseline);
	tt = Vector2d(0, 0);

	const double threshold = extension_length;

	Vector3d dv = lm->line.tail(3);
	Vector3d cvn = gc_vec3_normalize(dv);

	if (gc_angle_normvec(cvn, lm->pvn) > line_vn_angle_thr) {
		lm->pvn = cvn;
		lm->tt = Vector2d(0, 0);
	}

	// main routine

//	pose_t init_pose = kfs[lm->init_kfid]->T;
	pose_t init_pose = T;
	pose_t relative_pose = gc_T_inv(init_pose);
	Vector6d line2 = gc_line_from_pose(lm->line, init_pose);

	Vector3d pc, nc, vc;
	pc = line2.head(3);
	vc = line2.tail(3);
	nc = pc.cross(vc);
	Matrix4d Lc;
	Lc << gc_skew_symmetric(nc), vc, -vc.transpose(), 0;

	obs_t obt = lm->obs_vec.back();
	Vector3d p11 = Vector3d(obt.obs(0), obt.obs(1), 1.0);
	Vector3d p21 = Vector3d(obt.obs(2), obt.obs(3), 1.0);
	Vector2d ln = (p11.cross(p21)).head(2);
	ln = ln / ln.norm();

	Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);
	Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
	Vector3d cam = Vector3d(0, 0, 0);

	Vector4d pi1 = gc_ppp_pi(cam, p11, p12);
	Vector4d pi2 = gc_ppp_pi(cam, p21, p22);
	Vector4d e1 = Lc * pi1;
	Vector4d e2 = Lc * pi2;

	Vector3d p0 = gc_plucker_origin(nc, vc);
	Vector3d vn = vc / vc.norm();

	double p0_dist = p0.norm();

	if (p0_dist > threshold) {
		lm->ba_updated = false;
//		continue;
		return;
	}

	Vector3d pc1 = e1.head(3) / e1(3);
	Vector3d pc2 = e2.head(3) / e2(3);
	if (pc1(2) < 0 || pc2(2) < 0) {
		lm->ba_updated = false;
//		continue;
		return;
	}

	double t1 = vn.dot(pc1 - p0);
	double t2 = vn.dot(pc2 - p0);

//	Vector2d tt;
	if (t2 > t1)
		tt << t1, t2;
	else
		tt << t2, t1;

	double extend_dist = sqrt(threshold * threshold - p0_dist * p0_dist);

	if (fabs(tt(0)) > extend_dist)
		tt(0) = (tt(0) / fabs(tt(0))) * extend_dist;
	if (fabs(tt(1)) > extend_dist)
		tt(1) = (tt(1) / fabs(tt(1))) * extend_dist;
	if (tt(0) == tt(1)) {
		lm->ba_updated = false;
//		continue;
		return;
	}

	Vector3d init_v = lm->line.tail(3);
	Vector2d tt1 = gc_tt_to_pose(lm->tt, init_v, relative_pose);

	if (tt1(0) == 0 && tt1(1) == 0)
		tt1 = tt;
	else
	{
		if (tt(0) < tt1(0))
			tt1(0) = tt(0);

		if (tt(1) > tt1(1))
			tt1(1) = tt(1);
	}

	lm->tt = gc_tt_from_pose(tt1, init_v, relative_pose);

	// printf("%d: tt(0) = %g, tt(1) = %g\n", lmit->first, tt(0), tt(1));

	/////////////////////////////////////////////////////////////////////////

	lm->ba_updated = false;

//	delete lm;
	return;
}

// void SLAM::save_landmark()
// void SLAM::draw()
Vector6d save_landmark(Vector6d line, pose_t T, Vector2d tt){
	Vector3d p, n, v;
	p = line.head(3);
	v = line.tail(3);
	n = p.cross(v);
	Vector3d p0 = v.cross(n) / v.dot(v);
	Vector3d vn = v / v.norm();

//	pose_t T = kfs[lm->init_kfid]->T;

	Vector3d p1c = p0 + vn * tt(0);
	Vector3d p1w = gc_poit_from_pose(T, p1c);
	Vector3d p2c = p0 + vn * tt(1);
	Vector3d p2w = gc_poit_from_pose(T, p2c);

	Vector6d line_endpts;
	line_endpts << p1w(0), p1w(1), p1w(2), p2w(0), p2w(1), p2w(2);
	return line_endpts;
}

Vector6d line_2D_to_3D(Vector8d& obs, double baseline, float fx1, float fy1, float cx1, float cy1)
{
	pose_t T(Matrix3d::Identity(), Vector3d::Zero());
	Vector2d tt;

	insert_curr_obs(obs, fx1, fy1, cx1, cy1);
	landmark_t* lm = new landmark_t;
	add_lms(obs, baseline, lm);;
	extend_end_points(obs, T, baseline, lm, tt);
	Vector6d line_endpts = save_landmark(lm->line, T, tt);
	delete lm;
	return line_endpts;
}

float extend_line_seg(float x1, float y1, float x2, float y2, float y)
{
	// input paras satifsy: 
	// (x1, y1) is above (x2, y2) in image plane
	// y is out of line segment
	// i.e., x1 <= x2, y1 <= y2, (y < min(y1, y2) or y > max(y1, y2))

	if (abs(x1 - x2) < 1e-10){
		return (x1 + x2) / 2.0f;
	}

	// up extend
	if (y < min(y1, y2)){
		return x2 - (x2 - x1) * (y2 - y) / (y2 - y1);
	}
	else{
		// down extend
		return x1 + (x2 - x1) * (y - y1) / (y2 - y1);
	}
}

void stereo_3D_Pt(double ul, double ur, float v, float fx, float fy, 
	float baseline, double& x, double& y, double& z)
{
	double d = baseline / (ul - ur);
	x = ul * d;
	y = v * d;
	z = fx * d;
	return;
}

Vector6d line_2D_to_3D_v2(Vector8d obs, double baseline, float fx, float fy, float cx, float cy)
{
	// obs = [x1, y1, x2, y2, x1r, y1r, x2r, y2r]
	Vector8d new_obs;
	Vector6d line_endpts;
	double x1, y1, x2, y2, x1r, y1r, x2r, y2r;
	// make sure first endpoint is above
	if (obs[1] <= obs[3]){
		x1 = obs[0]; y1 = obs[1];
		x2 = obs[2]; y2 = obs[3];
	}
	else{
		x1 = obs[2]; y1 = obs[3];
		x2 = obs[0]; y2 = obs[1];
	}
	if (obs[5] <= obs[7]){
		x1r = obs[4]; y1r = obs[5];
		x2r = obs[6]; y2r = obs[7];
	}
	else{
		x1r = obs[6]; y1r = obs[7];
		x2r = obs[4]; y2r = obs[5];
	}
	new_obs[0] = x1; new_obs[1] = y1;
	new_obs[2] = x2; new_obs[3] = y2;
	new_obs[4] = x1r; new_obs[5] = y1r;
	new_obs[6] = x2r; new_obs[7] = y2r;

	// up extend left line
//	if (y1 > y1r){
//		new_obs[0] = extend_line_seg(x1, y1, x2, y2, y1r);
//		new_obs[1] = y1r;
//	}
	// up extend right line
	if (y1 < y1r){
		new_obs[4] = extend_line_seg(x1r, y1r, x2r, y2r, y1);
		new_obs[5] = y1;
	}
	// down extend left line
//	if (y2 < y2r){
//		new_obs[2] = extend_line_seg(x1, y1, x2, y2, y2r);
//		new_obs[3] = y2r;
//	}
	// down extend right line
	if (y2 > y2r){
		new_obs[6] = extend_line_seg(x1r, y1r, x2r, y2r, y2);
		new_obs[7] = y2;
	}

	stereo_3D_Pt(new_obs[0]-cx, new_obs[4]-cx, new_obs[1]-cy, fx, fy, baseline, line_endpts[0], line_endpts[1], line_endpts[2]);
	stereo_3D_Pt(new_obs[2]-cx, new_obs[6]-cx, new_obs[3]-cy, fx, fy, baseline, line_endpts[3], line_endpts[4], line_endpts[5]);
	return line_endpts;
}

Vector8d formatConversion(Line line1, Line line2)
{
	// input: L1, L2, R1, R2 for one line's endpoints;
	//        L3, L4, R3, R4 for another line's endpoints
	Vector8d V;

	Point2f L1 = line1.StartPt;
	Point2f L2 = line1.EndPt;
	Point2f R1 = line2.StartPt;
	Point2f R2 = line2.EndPt;

//	float d1 = fabs(L1.x - R1.x) + fabs(L1.y - R1.y) + fabs(L2.x - R2.x) + fabs(L2.y - R2.y);
//	float d2 = fabs(L1.x - R2.x) + fabs(L1.y - R2.y) + fabs(L2.x - R1.x) + fabs(L2.y - R1.y);

//	if (d1<=d2)
		V << L1.x, L1.y, L2.x, L2.y, R1.x, R1.y, R2.x, R2.y;
//	else
//		V << L1.x, L1.y, L2.x, L2.y, R2.x, R2.y, R1.x, R1.y;
	return V;
}

bool lineRecon(
	vector<Line> vecLine1, vector<Line> vecLine2,
	vector<pair<int, int>> lineMatch,
	float baseline, float fx, float fy, float cx, float cy, 
	vector<pair<Vector8d, Vector6d>>& line_seg_para, 
	vector<pair<int, int>>& line_match_recon,
	int reconMethod)
{
	int nPutatives = lineMatch.size();
	if (nPutatives == 0 || vecLine1.size() == 0 || vecLine2.size() == 0)
		return false;

//	vector<pair<Vector8d, Vector6d>> line_seg_para;
//	vector<pair<int, int>> line_match_recon;
	
	vector<Line>::const_iterator it;
//	int nReconLine = 0;
	for (int i = 0; i < nPutatives; i++)
	{
		int idx1 = lineMatch[i].first;
		int idx2 = lineMatch[i].second;

		Vector8d obs = formatConversion(vecLine1[idx1], vecLine2[idx2]);
		Vector8d obs_backup = obs;
		if (abs(obs[5] - obs[7]) < 1e-10){
			// will cause invalid solution
			continue;
		}
		Vector6d line_endpts;
		switch (reconMethod){
		case 0: // plucker
			line_endpts = line_2D_to_3D(obs, baseline, fx, fy, cx, cy);
			break;
		case 1: // stereo
			line_endpts = line_2D_to_3D_v2(obs, baseline, fx, fy, cx, cy);
			break;
		default: // plucker
			line_endpts = line_2D_to_3D(obs, baseline, fx, fy, cx, cy);
		}

		// if the line is near parallel to horizonal line, then skip it
//		bool flag = (fabs(obs_backup[1] - obs_backup[3]) < 10)
//			|| (fabs(obs_backup[5] - obs_backup[7]) < 10);
		// tan(10 deg) = 0.1763269807
		// tan(15 deg) = 0.26794919243
		// tan(25 deg) = 0.46630765815
		/*
		float thresh = 0.26794919243;
		bool flag1 = 
			static_cast<float>(fabs(obs_backup[1] - obs_backup[3])) < \
			static_cast<float>(fabs(obs_backup[0] - obs_backup[2])) * thresh;
		bool flag2 =
			static_cast<float>(fabs(obs_backup[5] - obs_backup[7])) < \
			static_cast<float>(fabs(obs_backup[4] - obs_backup[6])) * thresh	;

		if (!IsValidVector6d(line_endpts) || (flag1 && flag2)){
			continue;
		}
		*/
		line_seg_para.push_back(make_pair(obs_backup, line_endpts));
		line_match_recon.push_back(lineMatch[i]);
//		nReconLine++;
	}

	return true;
}

bool save_stereo_results(
	vector<pair<Vector8d, Vector6d>>& line_seg_para,
	vector<pair<int, int>>& line_match_recon,
	map<int, Junction> mapJunc1, 
	int reconMethod, int idxFrame)
{
	// we only save mapJunc1, which is for left image only

	int nReconLine = line_seg_para.size();
	// construct a vector to indicate which line segments have been reconstructed
	int maxN = -1;
	for (int i = 0; i < nReconLine; i++){
		int idx = line_match_recon[i].first;
		maxN = (maxN > idx) ? maxN : idx;
	}
	maxN++;
	// this line appears in which elements of lineIds
	int* address = new int[maxN];
	for (int i = 0; i < maxN; i++){
		address[i] = -1;
	}
	for (int i = 0; i < nReconLine; i++){
		int idx = line_match_recon[i].first;
		address[idx] = i;
	}
	
	if (nReconLine>0){
		ofstream myfile;
		string fullname;
		if (reconMethod == 0)
			fullname = num2str(idxFrame) + ".txt";
		else
			fullname = num2str(idxFrame) + "_" + num2str(reconMethod) + ".txt";

		myfile.open(fullname.c_str(), ios::out);
		if (!myfile.is_open()) {
			cerr << "Oops! Unable to open file for save!\n";
			delete[] address;
			return false;
		}
		for (int i = 0; i < nReconLine; i++){
			if (i != 0)
				myfile << endl;
			Vector8d obs = line_seg_para[i].first;
			Vector6d line_endpts = line_seg_para[i].second;

			int idx1 = line_match_recon[i].first;

			myfile << idx1 << " ";
			// 2D
			for (int k = 0; k < 8; k++){
				myfile << setprecision(3) << std::fixed << obs[k] << " ";
			}
			// 3D
			for (int k = 0; k < 6; k++){
				myfile << setprecision(6) << std::fixed << line_endpts(k);
				if (k < 5)
					myfile << " ";
			}
		}
	}

	int nJunc1 = mapJunc1.size();
	if (nJunc1 > 0){
		vector<junctKeyPt> vecKeyPts1;
		convertJuncToKeyPts(mapJunc1, vecKeyPts1);

		ofstream myfile;
		string fullname;
		if (reconMethod == 0)
			fullname = num2str(idxFrame) + "_junc.txt";
		else
			fullname = num2str(idxFrame) + "_junc_" + num2str(reconMethod) + ".txt";

		myfile.open(fullname.c_str(), ios::out);
		if (!myfile.is_open()) {
			cerr << "Oops! Unable to open file for save!\n";
			delete[] address;
			return false;
		}
		
		bool is_first = true;
		for (int i = 0; i < nJunc1; i++){
			junctKeyPt aJunc = vecKeyPts1[i];
			int key1 = aJunc.key;
			map<int, Junction>::const_iterator  it = mapJunc1.find(key1);
			Junction J1 = it->second;

			if (address[J1.line1.id] < 0 && address[J1.line2.id] < 0)
			{
				// at least one line of this junction has been reconstructed
				continue;
			}

			if (!is_first){
				myfile << endl;
			}
			else{
				is_first = false;
			}

			myfile << setprecision(3) << std::fixed << aJunc.x << " ";
			myfile << setprecision(3) << std::fixed << aJunc.y << " ";
			myfile << setprecision(5) << std::fixed << aJunc.mid_ang1_ang2 << " ";
			if (J1.L1_IS_BASE_EDGE){
				myfile << J1.line1.id << " " << J1.line2.id;
			}
			else{
				myfile << J1.line2.id << " " << J1.line1.id;
			}
		}

	}
	delete[] address;
	return true;
}
