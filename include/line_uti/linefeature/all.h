// SLSLAM: Stereo Line-based SLAM
// This file is part of SLSLAM
//
// Copyright (C) 2015 Guoxuan Zhang, Jin Han Lee, Jongwoo Lim, Il Hong Suh
// 
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

#ifndef ALL_H_
#define ALL_H_

#include <iostream>
#include <fstream>
#include <algorithm>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "Eigen/Dense"
#include "Eigen/QR"

//#include "glog/logging.h"

using namespace std;
using namespace Eigen;

typedef Matrix<double,8,1> Vector8d;
typedef Matrix<double,6,1> Vector6d;

//////////////////////////////////////////////////////////////////////////////
// Data structure

typedef struct _pose_t {
  _pose_t() : R( Matrix3d::Identity() ), t( Vector3d::Zero() ) {}
  _pose_t( const _pose_t & T ) : R( T.R ), t( T.t ) {}
  _pose_t( Matrix3d R, Vector3d t ) : R(R), t(t) {}

  Matrix3d R;
  Vector3d t;
} pose_t;

typedef struct _obs_t {
    _obs_t() {}
    _obs_t( int id, Vector8d obs ) : id( id ), obs( obs ) {}

    int id;               //  id: 观测到这个观测量obs的kf_id
    Vector8d obs;         // obs: 观测量
} obs_t;

typedef struct vpobs_t {
    vpobs_t() {}
    vpobs_t( int id, Vector2d obs ) : id( id ), obs( obs ) {}

    int id;               //  id: 观测到这个观测量obs的kf_id
    Vector2d obs;         // obs: 观测量
} vpobs_t;

typedef struct {
    Vector6d line;                 // 保存的3d坐标是 第一次观测到这个line的kf坐标系下
    Vector2d tt;
    Vector3d pvn;
    bool twice_observed;
    bool ba_updated;
    bool currently_visible;
    int init_kfid;                 // 第一次观测到这个line的kf
    vector<obs_t> obs_vec;         //  obs_t( kfid, vector8d) ,  记录观测到这个line的关键帧(kfid)上的测量值(vector8d)
    vector<vpobs_t> vpobs_vec;     //  vpobs_vec( kfid, vector2d) ,  记录观测到这个line的关键帧(kfid)上的测量值(vector2d)
} landmark_t;

#endif /* ALL_H_ */
