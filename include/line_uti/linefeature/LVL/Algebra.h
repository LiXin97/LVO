/// \author Jonas Witt
/// \file
/// \brief Various Algebra functions.

#pragma once

//#include <opencv2/core/core.hpp>
#include <cmath>
#include <Eigen/Geometry>
#include "VisionTypes.h"

namespace lvl {

/*template<typename _Tp, int cn>
inline cv::Vec<_Tp, cn> operator*(const cv::Mat& mat, const cv::Vec<_Tp, cn>& vec)
{
	assert(mat.cols == mat.rows && mat.cols == cn);
	cv::Vec<_Tp, cn> resultVec;
	const _Tp* element;

	for(int i=0; i<cn; ++i)
	{
		resultVec[i] = 0.0;
		element = &mat.at<_Tp>(i, 0);
		for(int j=0; j<cn; ++j)
			resultVec[i] += (*element++) * vec[j];
	}

	return resultVec;
}*/


/*
template<typename _Tp> static inline
cv::Vec<_Tp, 2>& operator/=(cv::Vec<_Tp, 2>& vec, _Tp factor)
{
	vec[0] /= factor;
	vec[1] /= factor;

	return vec;
}

template<typename _Tp> static inline
cv::Vec<_Tp, 3>& operator/=(cv::Vec<_Tp, 3>& vec, _Tp factor)
{
	vec[0] /= factor;
	vec[1] /= factor;
	vec[2] /= factor;

	return vec;
}

template<typename _Tp> static inline
cv::Vec<_Tp, 4>& operator/=(cv::Vec<_Tp, 4>& vec, _Tp factor)
{
	vec[0] /= factor;
	vec[1] /= factor;
	vec[2] /= factor;
	vec[3] /= factor;

	return vec;
}

template<typename _Tp, int cn> static inline
cv::Vec<_Tp, cn> operator/(cv::Vec<_Tp, cn> vec, _Tp factor)
{
	return vec /= factor;
}



template<typename _Tp> static inline
_Tp dot(const cv::Vec<_Tp, 4>& v1, const cv::Vec<_Tp, 4>& v2)
{
	return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2] + v1[3]*v2[3];
}

template<typename _Tp> static inline
_Tp dot(const cv::Vec<_Tp, 3>& v1, const cv::Vec<_Tp, 3>& v2)
{
	return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

template<typename _Tp> static inline
_Tp dot(const cv::Vec<_Tp, 2>& v1, const cv::Vec<_Tp, 2>& v2)
{
	return v1[0]*v2[0] + v1[1]*v2[1];
}

template<typename _Tp, int cn> static inline
_Tp normL2Sq(const cv::Vec<_Tp, cn>& vec)
{
	return vec.dot(vec);
}




template<typename _Tp, int cn> static inline
_Tp normL2(const cv::Vec<_Tp, cn>& vec)
{
	return cv::sqrt(normL2Sq(vec));
}*/


/// \brief Builds a rotation matrix from the given normalized orientation vector and the cosine and sine of its rotation angle.
template<typename _Tp>
void rotationMatrixFromOrientationVector(const Eigen::Matrix<_Tp, 3, 1>& n, _Tp cosAlpha, _Tp sinAlpha, Eigen::Matrix<_Tp, 3, 3>& rotationMatrix)
{	
	// First row
	rotationMatrix(0,0) = cosAlpha + n[0]*n[0]*((_Tp)1.0-cosAlpha);
	rotationMatrix(0,1) = n[0]*n[1]*((_Tp)1.0-cosAlpha) - n[2]*sinAlpha;
	rotationMatrix(0,2) = n[0]*n[2]*((_Tp)1.0-cosAlpha) + n[1]*sinAlpha;
	// Second row
	rotationMatrix(1,0) = n[0]*n[1]*((_Tp)1.0-cosAlpha) + n[2]*sinAlpha;
	rotationMatrix(1,1) = cosAlpha + n[1]*n[1]*((_Tp)1.0-cosAlpha);
	rotationMatrix(1,2) = n[1]*n[2]*((_Tp)1.0-cosAlpha) - n[0]*sinAlpha;
	// Third row
	rotationMatrix(2,0) = n[0]*n[2]*((_Tp)1.0-cosAlpha) - n[1]*sinAlpha;
	rotationMatrix(2,1) = n[2]*n[1]*((_Tp)1.0-cosAlpha) + n[0]*sinAlpha;
	rotationMatrix(2,2) = cosAlpha + n[2]*n[2]*((_Tp)1.0-cosAlpha);
}

/*template<typename _Tp>
cv::Vec<_Tp,4> quaternionFromOrientationVector(const cv::Vec<_Tp,3>& v)
{
	cv::Vec<_Tp,4> quaternion;
	_Tp vNorm = normL2(v);
	if (vNorm > 0.0001)
	{
		quaternion[0] = cos(vNorm/2);
		_Tp k = sin(vNorm/2)/vNorm;
		quaternion[1] = k * v[0];
		quaternion[2] = k * v[1];
		quaternion[3] = k * v[2];
	} 
	else 
	{
		quaternion[0] = 1.0;
		quaternion[1] = 0.5 * v[0];
		quaternion[2] = 0.5 * v[1];
		quaternion[3] = 0.5 * v[2];
		quaternion /= normL2(quaternion);
	}
	return quaternion;
}

template<typename _Tp>
void quaternionFromOrientationVector(const cv::Vec<_Tp,3>& v, cv::Vec<_Tp,4>& quaternion)
{
	_Tp vNorm = normL2(v);
	if (vNorm > 0.0001)
	{
		quaternion[0] = cos(vNorm/2);
		_Tp k = sin(vNorm/2)/vNorm;
		quaternion[1] = k * v[0];
		quaternion[2] = k * v[1];
		quaternion[3] = k * v[2];
	} 
	else 
	{
		quaternion[0] = 1.0;
		quaternion[1] = 0.5 * v[0];
		quaternion[2] = 0.5 * v[1];
		quaternion[3] = 0.5 * v[2];
		quaternion /= normL2(quaternion);
	}
}*/


/// \brief Builds a rotation matrix from the given normalized orientation vector and its value/angle alpha (in radians).
template<typename _Tp>
void rotationMatrixFromOrientationVector(const Eigen::Matrix<_Tp, 3, 1>& n, _Tp alpha, Eigen::Matrix<_Tp, 3, 3>& rotationMatrix)
{
	/*const cv::Vec<_Tp,3>& n(orientationVector);

	if(sizeof(_Tp) == 4)
		rotationMatrix.create(3, 3, CV_32FC1);
	else if(sizeof(_Tp) == 8)
		rotationMatrix.create(3, 3, CV_64FC1);
	else
		throw ::std::exception("Only float and double are allowed");

	_Tp *data = (_Tp*)rotationMatrix.data;

	_Tp cosAlpha = cos(alpha);
	_Tp sinAlpha = sin(alpha);

	// First row
	data[0] = cosAlpha + n[0]*n[0]*(1.0-cosAlpha);
	data[1] = n[0]*n[1]*(1.0-cosAlpha) - n[2]*sinAlpha;
	data[2] = n[0]*n[2]*(1.0-cosAlpha) + n[1]*sinAlpha;
	// Second row
	data += 3;
	data[0] = n[0]*n[1]*(1.0-cosAlpha) + n[2]*sinAlpha;
	data[1] = cosAlpha + n[1]*n[1]*(1.0-cosAlpha);
	data[2] = n[1]*n[2]*(1.0-cosAlpha) - n[0]*sinAlpha;
	// Third row
	data += 3;
	data[0] = n[0]*n[2]*(1.0-cosAlpha) - n[1]*sinAlpha;
	data[1] = n[2]*n[1]*(1.0-cosAlpha) + n[0]*sinAlpha;
	data[2] = cosAlpha + n[2]*n[2]*(1.0-cosAlpha);*/

	rotationMatrixFromOrientationVector<_Tp>(n, cos(alpha), sin(alpha), rotationMatrix);
}


/// \brief Builds a rotation matrix from the given orientation vector. If a zero-vector is passed, the identity is returned.
template<typename _Tp>
void rotationMatrixFromOrientationVector(const Eigen::Matrix<_Tp, 3, 1>& orientationVector, Eigen::Matrix<_Tp, 3, 3>& rotationMatrix)
{
	_Tp alpha = orientationVector.norm();
	if(alpha > (_Tp)0)
		rotationMatrixFromOrientationVector<_Tp>(orientationVector / alpha, alpha, rotationMatrix);
	else
		rotationMatrix.setIdentity();
}



/// \brief Builds a rotation matrix to map the src to the dst unit-vectors.
///
/// The following rotation quaternion is computed:
/// v1dst = R * v1src;
/// v2dst = R * v2src;
/// The first rotation equation is fulfilled exactly, but the second equation is not exactly fulfilled. 
/// Instead, the rotation towards the closest vector to v2dst which is reachable with the remaining degree of freedom is computed.
template<typename _Tp>
void rotationMatrixFromVectorMapping(const Eigen::Matrix<_Tp, 3, 1>& v1src, const Eigen::Matrix<_Tp, 3, 1>& v1dst, const Eigen::Matrix<_Tp, 3, 1>& v2src, const Eigen::Matrix<_Tp, 3, 1>& v2dst, Eigen::Matrix<_Tp, 3, 3>& rotationMatrix)
{
	// The rotation orientation vector is perpendicular to both vectors
	Eigen::Matrix<_Tp, 3, 1> n1 = v1src.cross(v1dst);
	
	_Tp cosAlpha = v1src.dot(v1dst);
	_Tp sinAlpha = n1.norm();

	/*n1[0] /= sinAlpha;
	n1[1] /= sinAlpha;
	n1[2] /= sinAlpha;*/

	n1 /= sinAlpha;

	// create R1 rotation matrix about n1
	//static cv::Mat R1;
	Eigen::Matrix<_Tp, 3, 3> tempMatrix;
	if(sinAlpha != 0.0)
		rotationMatrixFromOrientationVector(n1, cosAlpha, sinAlpha, tempMatrix);
	else
		tempMatrix.setIdentity();

	// find the projection of R1*v2src to the plane defined by v1dst
	Eigen::Matrix<_Tp, 3, 1> planeProjection1 = -v1dst.cross(v1dst.cross(tempMatrix*v2src));
	_Tp len = planeProjection1.norm();
	planeProjection1 /= len;

	// find the projection of v2dst to the plane defined by v1dst
	Eigen::Matrix<_Tp, 3, 1> planeProjection2 = -v1dst.cross(v1dst.cross(v2dst));
	len = planeProjection2.norm();
	planeProjection2 /= len;

	Eigen::Matrix<_Tp, 3, 1> n2 = planeProjection1.cross(planeProjection2);

	cosAlpha = planeProjection1.dot(planeProjection2);
	sinAlpha = n2.norm();
	n2 /= sinAlpha;

	// create R2 by rotation about v1dst (leaving it unaltered)
	//static cv::Mat R2;

	if(sinAlpha != 0.0)
		rotationMatrixFromOrientationVector(n2, cosAlpha, sinAlpha, rotationMatrix);
	else
		rotationMatrix.setIdentity();//R2 = cv::Mat::eye(3, 3, CV_32FC1);
	
	// compose final rotation matrix mapping v1src to v1dst and v2src to the closest vector to v2dst
	//rotationMatrix = R2 * R1;
	rotationMatrix *= tempMatrix;
}

/// \brief Builds a rotation quaternion to map the src to the dst unit-vectors.
///
/// The following rotation quaternion is computed:
/// v1dst = q * v1src * q^-1;
/// v2dst = q * v2src * q^-1;
/// The first rotation equation is fulfilled exactly, but the second equation is not exactly fulfilled. 
/// Instead, the rotation towards the closest vector to v2dst which is reachable with the remaining degree of freedom is computed.
template<typename _Tp>
void quaternionFromVectorMapping(	const Eigen::Matrix<_Tp, 3, 1>& v1src, 
									const Eigen::Matrix<_Tp, 3, 1>& v1dst, 
									const Eigen::Matrix<_Tp, 3, 1>& v2src, 
									const Eigen::Matrix<_Tp, 3, 1>& v2dst, 
									Eigen::Quaternion<_Tp>& quaternion)
{
	// The rotation orientation vector is perpendicular to both vectors
	Eigen::Matrix<_Tp, 3, 1> n1 = v1src.cross(v1dst);
	
	//_Tp cosAlpha = v1src.dot(v1dst);
	_Tp sinAlpha = n1.norm();
/*
	n1[0] /= sinAlpha;
	n1[1] /= sinAlpha;
	n1[2] /= sinAlpha;*/

	n1 /= sinAlpha;

	// create q1 rotation quaternion about n1
	Eigen::Quaternion<_Tp> q1(1, 0, 0, 0);
	if(sinAlpha != 0.0)
		q1 = Eigen::AngleAxis<_Tp>(asin(sinAlpha), n1);
	

	// find the projection of R1*v2src to the plane defined by v1dst
	Eigen::Matrix<_Tp, 3, 1> planeProjection1 = -v1dst.cross(v1dst.cross(q1*v2src));
	_Tp len = planeProjection1.norm();
	planeProjection1 /= len;

	// find the projection of v2dst to the plane defined by v1dst
	Eigen::Matrix<_Tp, 3, 1> planeProjection2 = -v1dst.cross(v1dst.cross(v2dst));
	len = planeProjection2.norm();
	planeProjection2 /= len;

	Eigen::Matrix<_Tp, 3, 1> n2 = planeProjection1.cross(planeProjection2);

	//cosAlpha = planeProjection1.dot(planeProjection2);
	sinAlpha = n2.norm();
	n2 /= sinAlpha;

	// create R2 by rotation about v1dst (leaving it unaltered)
	//static cv::Mat R2;

	quaternion = Eigen::Quaternion<_Tp>(1, 0, 0, 0);
	if(sinAlpha != 0.0)
		quaternion = Eigen::AngleAxis<_Tp>(asin(sinAlpha), n2);
	
	// compose final rotation matrix mapping v1src to v1dst and v2src to the closest vector to v2dst
	//rotationMatrix = R2 * R1;
	quaternion *= q1;
}


/// \brief Intersects (2D) line1 with line2 and returns the intersection point p if an intersection exists (return value true)
template<typename Derived1, typename Derived2> static inline
bool lineLineIntersect2D(	const Eigen::MatrixBase<Derived1>& line1P1, 
							const Eigen::MatrixBase<Derived1>& line1P2, 
							const Eigen::MatrixBase<Derived2>& line2P1, 
							const Eigen::MatrixBase<Derived2>& line2P2, 
							Eigen::MatrixBase<Derived1>& p)
{
	typename Derived1::PlainObject b = line1P2 - line1P1;
	typename Derived2::PlainObject d = line2P2 - line2P1;
	/*_Tp bx = x2 - x1;
	_Tp by = y2 - y1;
	_Tp dx = x4 - x3;
	_Tp dy = y4 - y3; 
	_Tp b_dot_d_perp = bx*dy - by*dx;*/
	typename Derived1::Scalar b_dot_d_perp = b[0]*d[1] - b[1]*d[0];

	if(b_dot_d_perp == 0)
		return false;

	typename Derived1::Scalar cx = line2P1[0] - line1P1[0];
	typename Derived1::Scalar cy = line2P1[1] - line1P1[1];
	typename Derived1::Scalar t = (cx*d[1] - cy*d[0]) / b_dot_d_perp;
	//_Tp cx = x3-x1; 
	//_Tp cy = y3-y1;
	//_Tp t = (cx*dy - cy*dx) / b_dot_d_perp; 

	//p[0] = x1+t*bx;
	//p[1] = y1+t*by;

	p = line1P1 + t * b;
	return true;
}

/// \brief Intersects (2D) line1 with line2 and returns the intersection point p = line1P1 + mu1 * line1Dir if an intersection exists (return value true)
template<typename Derived1, typename Derived2> static inline
bool lineLineIntersect2D(	const Eigen::MatrixBase<Derived1>& line1P1, 
							const Eigen::MatrixBase<Derived1>& line1Dir, 
							const Eigen::MatrixBase<Derived2>& line2P1, 
							const Eigen::MatrixBase<Derived2>& line2Dir, 
							Eigen::MatrixBase<Derived1>& p,
							typename Derived1::Scalar& mu1)
{	
	typename Derived1::Scalar dir1_dot_dir2_perp = line1Dir[0]*line2Dir[1] - line1Dir[1]*line2Dir[0];

	if(dir1_dot_dir2_perp == 0)
		return false;

	typename Derived1::Scalar cx = line2P1[0] - line1P1[0];
	typename Derived1::Scalar cy = line2P1[1] - line1P1[1];
	mu1 = (cx*line2Dir[1] - cy*line2Dir[0]) / dir1_dot_dir2_perp;
	
	p = line1P1 + mu1 * line1Dir;
	return true;
}


/// \brief Calculates the line segment Pa_Pb that describes the shortest distance between two lines P1_P2 and P3_P4. 
///
/// Also calculates the values of mua and mub where
///     Pa = P1 + mua (P2 - P1)
///     Pb = P3 + mub (P4 - P3)
/// Returns false if no solution exists (lines are parallel).
template<typename Derived, typename Derived2, typename Derived3, typename Derived4, typename DerivedResult>
bool lineLineIntersect3D(	const Eigen::MatrixBase<Derived>& p1, 
							const Eigen::MatrixBase<Derived2>& p2, 
							const Eigen::MatrixBase<Derived3>& p3, 
							const Eigen::MatrixBase<Derived4>& p4, 
							Eigen::MatrixBase<DerivedResult>& pa, 
							Eigen::MatrixBase<DerivedResult>& pb,
							typename Derived::Scalar& mua, 
							typename Derived::Scalar& mub)
{
	static const typename Derived::Scalar  eps = (typename Derived::Scalar)1e-5;	
	
	// Compute direction of line 2
	typename Derived::PlainObject p43 = p4 - p3;
	if (fabs(p43[0]) < eps && fabs(p43[1]) < eps && fabs(p43[2]) < eps)
		return false;

	// Compute direction of line 1
	typename Derived::PlainObject  p21 = p2 - p1;
	if (fabs(p21[0]) < eps && fabs(p21[1]) < eps && fabs(p21[2]) < eps)
		return false;
	
	typename Derived::PlainObject p13 = p1 - p3;

	typename Derived::Scalar d1343 = p13.dot(p43);
	typename Derived::Scalar d4321 = p43.dot(p21);
	typename Derived::Scalar d1321 = p13.dot(p21);
	typename Derived::Scalar d4343 = p43.dot(p43);
	typename Derived::Scalar d2121 = p21.dot(p21);

	typename Derived::Scalar denom = d2121 * d4343 - d4321 * d4321;
	if (fabs(denom) < eps)
		return false;
	typename Derived::Scalar numer = d1343 * d4321 - d1321 * d4343;

	mua = numer / denom;
	mub = (d1343 + d4321 * mua) / d4343;

	pa = p1 + mua * p21;
	pb = p3 + mub * p43;

	return true;
}


/// \brief Calculates the line segment Pa_Pb that describes the shortest distance between two lines p1+mu1*dir1 and p2+mu2*dir2. 
///
/// dir1 and dir2 have to have unit length. Also calculates the values of mua and mub where
///     Pa = P1 + mua (P2 - P1)
///     Pb = P3 + mub (P4 - P3)
/// Returns false if no solution exists (lines are parallel).
template<typename _Tp>
bool lineLineIntersect3D2(	const Eigen::Matrix<_Tp, 3, 1>& p1, 
							const Eigen::Matrix<_Tp, 3, 1>& dir1, 
							const Eigen::Matrix<_Tp, 3, 1>& p2, 
							const Eigen::Matrix<_Tp, 3, 1>& dir2, 
							Eigen::Matrix<_Tp, 3, 1>& pa, 
							Eigen::Matrix<_Tp, 3, 1>& pb,
							_Tp& mua, 
							_Tp& mub)
{
	static const _Tp  eps = (_Tp)1e-5;	
	
	Eigen::Matrix<_Tp, 3, 1> p12 = p1 - p2;

	_Tp d122 = p12.dot(dir2);
	_Tp d21  = dir2.dot(dir1);
	_Tp d121 = p12.dot(dir1);

	_Tp denom = (_Tp)1.0 - d21 * d21;
	if (fabs(denom) < eps)
		return false;
	_Tp numer = d122 * d21 - d121;

	mua = numer / denom;
	mub = d122 + d21 * mua;

	pa = p1 + mua * dir1;
	pb = p2 + mub * dir2;

	return true;
}

/// \brief Calculates the shortest distance between point p and the given line.
template<typename _Tp, int cn1, int cn2> static inline
_Tp linePointIntersect2D(const Eigen::Matrix<_Tp, cn1, 1>& linePoint, const Eigen::Matrix<_Tp, cn1, 1>& lineDir, const Eigen::Matrix<_Tp, cn2, 1>& p)
{
	return (p[0] - linePoint[0]) * lineDir[1] - (p[1] - linePoint[1]) * lineDir[0];
}


/// \brief Calculates the shortest distance between point p and the given line.
/*template<typename _Tp, int cn1, int cn2> static inline
_Tp linePointIntersect2D(const Eigen::Matrix<_Tp, cn1, 1>& lineDir, _Tp lineOriginDistance, const Eigen::Matrix<_Tp, cn2, 1>& p)
{
	return p[0] * lineDir[1] - p[1] * lineDir[0] - lineOriginDistance;
}*/


/// \brief Calculates the shortest distance between point p and the given line. Also returns mu, where linePoint + mu*lineDir is the closest point on the line to p.
template<typename _Tp, int cn1, int cn2> static inline
void linePointIntersect2D(const Eigen::Matrix<_Tp, cn1, 1>& linePoint, const Eigen::Matrix<_Tp, cn1, 1>& lineDir, const Eigen::Matrix<_Tp, cn2, 1>& p, _Tp& distance, _Tp& mu)
{
  distance = linePointIntersect2D<_Tp, cn1, cn2>(linePoint, lineDir, p);
	mu = -(linePoint.template head<2>() - p.template head<2>()).dot(lineDir.template head<2>());
	/*if(fabs(lineDir[0]) > fabs(lineDir[1]))
	{
		mu = (p[0] - linePoint[0] - lineDir[1] * distance) / lineDir[0];
	}
	else
	{
		mu = (p[1] - linePoint[1] + lineDir[0] * distance) / lineDir[1];
	}*/
}

/// \brief Calculates the shortest distance between point p and the given line. Also returns mu, where linePoint + mu*lineDir is the closest point on the line to p.
/*template<typename _Tp, int cn1, int cn2> static inline
void linePointIntersect2D(const Eigen::Matrix<_Tp, cn1, 1>& linePoint, const Eigen::Matrix<_Tp, cn1, 1>& lineDir, _Tp lineOriginDistance, const Eigen::Matrix<_Tp, cn2, 1>& p, _Tp& distance, _Tp& mu)
{
	distance = linePointIntersect2D<_Tp, cn1, cn2>(lineDir, lineOriginDistance, p);
	mu = -(linePoint.template head<2>() - p.template head<2>()).dot(lineDir.template head<2>());
}*/

/// \brief Calculates the perpendicular distance from a line to the given point p. Also returns mu, where line linePoint + mu*lineDir is the closest point on the line to p.
template<typename _Tp> static inline
void linePointIntersect3D(const Eigen::Matrix<_Tp, 3, 1>& linePoint, const Eigen::Matrix<_Tp, 3, 1>& lineDir, const Eigen::Matrix<_Tp, 3, 1>& p, _Tp& distance, _Tp& mu)
{
	mu = -(linePoint - p).dot(lineDir);
	distance = lineDir.cross(linePoint - p).norm();	
}



/// \brief Calculates the intersection point between a plane and a line
///
/// Returns false if no interection exists and true otherwise.
/// planeNormal and lineDir do not have to be normalized
template<typename _Tp> static inline
bool planeLineIntersect3D(	const Eigen::Matrix<_Tp, 3, 1>& planePoint, 
							const Eigen::Matrix<_Tp, 3, 1>& planeNormal, 
							const Eigen::Matrix<_Tp, 3, 1>& linePoint, 
							const Eigen::Matrix<_Tp, 3, 1>& lineDir,
							Eigen::Matrix<_Tp, 3, 1>& intersectionPoint)
{
	_Tp directionDotProduct = planeNormal.dot(lineDir);

	if(directionDotProduct == 0.0)
		return false;

	_Tp lineMu = - (linePoint - planePoint).dot(planeNormal) / directionDotProduct;
	intersectionPoint = linePoint + lineMu * lineDir;

	return true;
}

/// \brief Computes the x coordinate of a 2D line for a given y coordinate.
template<typename _Tp, typename _LineFeature>
_Tp getLineXCoordinateAt(const Line<_Tp, 2, _LineFeature>& line, _Tp y)
{
	//return (line.p1[0] * line.dir[1] - line.p1[1] * line.dir[0] + y * line.dir[0]) / line.dir[1];
	if(line.dir[1] == 0)
		return line.p1[0];
	else
		return line.p1[0] + (y - line.p1[1]) * line.dir[0] / line.dir[1];// - line.p1[1] * line.dir[0] + y * line.dir[0]) / line.dir[1];

}

/// \brief Computes the 3D stereo/disparity line of two matching 2D lines
template<typename _Tp, typename _LineFeature>
inline void computeLineDisparity(	const Line<_Tp, 2, _LineFeature>& leftLine,
									const Line<_Tp, 2, _LineFeature>& rightLine,
									Line<_Tp, 3, _LineFeature>& stereoLine)
{
	_Tp xl1 = leftLine.p1[0];//getLineXCoordinateAt(leftLine,  leftLine.p1[1]);
	_Tp xl2 = leftLine.p2[0];//getLineXCoordinateAt(leftLine,  leftLine.p2[1]);
	_Tp xr1, xr2;

	if(leftLine.dir.dot(rightLine.dir) > 0)
	{
		xr1 = (leftLine.p1[1] != rightLine.p1[1]) ? getLineXCoordinateAt(rightLine, leftLine.p1[1]) : rightLine.p1[0];
		xr2 = (leftLine.p2[1] != rightLine.p2[1]) ? getLineXCoordinateAt(rightLine, leftLine.p2[1]) : rightLine.p2[0];
	}
	else
	{
		xr1 = (leftLine.p1[1] != rightLine.p2[1]) ? getLineXCoordinateAt(rightLine, leftLine.p1[1]) : rightLine.p2[0];
		xr2 = (leftLine.p2[1] != rightLine.p1[1]) ? getLineXCoordinateAt(rightLine, leftLine.p2[1]) : rightLine.p1[0];
	}

	stereoLine.p1[0] = leftLine.p1[0];
	stereoLine.p1[1] = leftLine.p1[1];
	stereoLine.p1[2] = xl1 - xr1;
	stereoLine.p2[0] = leftLine.p2[0];
	stereoLine.p2[1] = leftLine.p2[1];
	stereoLine.p2[2] = xl2 - xr2;
	stereoLine.dir[0] = leftLine.dir[0];
	stereoLine.dir[1] = leftLine.dir[1];
	stereoLine.dir[2] = 0.0;
	stereoLine.length = leftLine.length;
	stereoLine.feature = leftLine.feature;
}


/// \brief Orthogonalizes the given matrix with a householder QR-decomposition.
template<typename Derived>
void orthogonalize(Eigen::MatrixBase<Derived>& rotationMatrix)
{
	assert(rotationMatrix.cols() == rotationMatrix.rows());

	Eigen::HouseholderQR< Eigen::MatrixBase<Derived> > qr = rotationMatrix.householderQr();
	rotationMatrix = qr.householderQ();

	for(int i = 0; i < rotationMatrix.cols(); ++i)
	{
		if(qr.matrixQR().coeff(i,i) < 0.0f)
			rotationMatrix.col(i) *= -1.0f;
	}				
}



} // namespace lvl
