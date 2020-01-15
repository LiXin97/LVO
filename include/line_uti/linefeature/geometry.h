// Line & Junction detection, Matching, and reconstruction
// Author: Ji Zhao
// Date:   12/06/2016
// Email:  zhaoji84@gmail.com
// Shanghai ReadSense Tech., Inc.
// All rights reserved

#ifndef _GEOMETRY_H
#define _GEOMETRY_H

#include <algorithm>
#include <iostream>
using namespace std;

#define MY_PI 3.1415926535897932f
#define MY_TWO_PI 6.283185307179586f
#define MY_HALF_PI 1.570796326794897f
#define NEAR_ZERO_THRESHOLD 0.0001
#define SQRT_OF_2  1.41421356237f
#define HALF_SQRT_OF_2  0.70710678118f
// #define INTERSEC_DOT_PRODUCT_THRESH 0.86602540378f
// two lines with small angles are rejected, here we set 30 degrees
// cos(5 deg) = 0.99619469809
// cos(10 deg) = 0.98480775301
// cos(15 deg) = 0.96592582628
// cos(20 deg) = 0.93969262078
// cos(25 deg) = 0.90630778703
// cos(30 deg) = 0.86602540378

enum junctionType{
	RAY_POINT_RAY,  // 1 intersection
	T_INTERSECTION, // 2 intersections
	X_INTERSECTION  // 4 intersections
};

struct PointXY{
	float x;
	float y;
};

static inline bool nearZero(float a)
{
	return (a<NEAR_ZERO_THRESHOLD && a > -NEAR_ZERO_THRESHOLD);
}

// Given two points on each line, judge whether they are parallel.
// Here, (x1,y1), (x2, y2) are from one line, and (x3, y3), (x4, y4) are from another line
static inline bool nearParallel(float x1, float y1, float x2, float y2,
	float x3, float y3, float x4, float y4)
{
	float x1_x2 = x1 - x2;
	float y1_y2 = y1 - y2;
	float x3_x4 = x3 - x4;
	float y3_y4 = y3 - y4;

	float A = x1_x2 * y3_y4 - y1_y2 * x3_x4;
	if (nearZero(A)){
		return true;
	}
	return false;
}

// Given two points on each line, calculate the intersection (px, py) of two lines.
// Suppose the angle of these two lines is theta, return false if cos(theta) < dotProductThresh.
// Here, (x1,y1), (x2, y2) are from one line, and (x3, y3), (x4, y4) are from another line
// https://en.wikipedia.org/wiki/Line-line_intersection
static inline bool intersectionTwoLines(float x1, float y1, float x2, float y2,
	float x3, float y3, float x4, float y4, float& px, float& py, float dotProductThresh)
{
	float x1_x2 = x1 - x2;
	float y1_y2 = y1 - y2;
	float x3_x4 = x3 - x4;
	float y3_y4 = y3 - y4;

	float len1 = sqrtf(x1_x2*x1_x2 + y1_y2*y1_y2);
	float len2 = sqrtf(x3_x4*x3_x4 + y3_y4*y3_y4);
	float dotProd = fabs(x1_x2 * x3_x4 + y1_y2 * y3_y4)/(len1*len2);
	if (dotProd > dotProductThresh) {
		return false;
	}

	float x1y2_y1x2 = x1*y2 - y1*x2;
	float x3y4_y3x4 = x3*y4 - y3*x4;
	float A = x1_x2 * y3_y4 - y1_y2 * x3_x4;
	if (!nearZero(A)){ // not parallel
		px = (x1y2_y1x2 * x3_x4 - x1_x2 * x3y4_y3x4) / A;
		py = (x1y2_y1x2 * y3_y4 - y1_y2 * x3y4_y3x4) / A;
		return true;
	}
	else{
		px = 0;
		py = 0;
		return false;
	}
};


// Calculate distance d and perpendicular intersection (px, py) 
// from a point (x0,y0) to a line.
// Here, line is parametered by two points (x1,y1), (x2,y2)
// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
// http://paulbourke.net/geometry/pointlineplane/
static inline float pointLineDistance(float x1, float y1, float x2, float y2,
	float x0, float y0, float& px, float& py)
{
	float dx = x2 - x1;
	float dy = y2 - y1;
	float len2 = dx * dx + dy * dy;
	float d;
	if (!nearZero(len2)){
		float len = sqrtf(len2);
		d = dy*x0 - dx*y0 + x2*y1 - y2*x1;
		d = fabs(d) / len;

		float u = (x0 - x1)*dx + (y0 - y1)*dy;
		u /= len2;
		px = x1 + u*dx;
		py = y1 + u*dy;

		return d;
	}
	d = -1;
	return d;
}

// calculate angles [-pi pi] between two lines
// Here, (x1,y1), (x2, y2) are from one line, and (x3, y3), (x4, y4) are from another line
static float angleTwoLines(float x1, float y1, float x2, float y2,
	float x3, float y3, float x4, float y4)
{
	// not test yet
	float x1_x2 = x1 - x2;
	float y1_y2 = y1 - y2;
	float x3_x4 = x3 - x4;
	float y3_y4 = y3 - y4;
	float len1 = sqrtf(x1_x2*x1_x2 + y1_y2*y1_y2);
	float len2 = sqrtf(x3_x4*x3_x4 + y3_y4*y3_y4);
	float dotProd = fabs(x1_x2 * x3_x4 + y1_y2 * y3_y4) / (len1*len2);

	float angle = acos(dotProd);
	angle = (angle <= MY_HALF_PI) ? angle : (MY_PI - angle);
	return angle;
}

// suppose point (x0, y0) on the line that connects (x1,y1) and (x2,y2)
// judge whether (x0,y0) is on the line segment
static bool pointOnLineSegment(float x1, float y1, float x2, float y2,
	float x0, float y0)
{
	bool flag;
	float xMin = min(x1, x2);
	float xMax = max(x1, x2);
	float yMin = min(y1, y2);
	float yMax = max(y1, y2);
	flag = (x0<xMin) || (x0>xMax) || (y0<yMin) || (y0>yMax);
	return (!flag);
}

// suppose point p0(x0, y0) on the line that connects p1(x1,y1) and p2(x2,y2)
// calculate the distance between (x0,y0) and the line segment, 
//   i.e., min(||p1-p0||, ||p2-p0||).
// If (x0, y0) is on the line segment, distance is negative,
//   i.e., -min(||p1-p0||, ||p2-p0||).
// If ||p2-p0|| >= ||p1-p0||, EndPoint_IS_FAR_POINT is true.
// (When calculating angle at the beginning, we treat p1 as start point, p2 as end point.
//  However, when constructing junctions, the intersection point is treated as start point, 
//  and either p1 or p2 is treated as end point. 
//  We need this flag to indicate whether to opposite the angle)
static float pointLineSegmentDist(float x1, float y1, float x2, float y2,
	float x0, float y0, bool& EndPoint_IS_FAR_POINT)
{
	float xMin = min(x1, x2);
	float xMax = max(x1, x2);
	float yMin = min(y1, y2);
	float yMax = max(y1, y2);
	
	float dx, dy;
	if (fabs(x2 - x0) + fabs(y2 - y0) >= fabs(x1 - x0) + fabs(y1 - y0)){
		EndPoint_IS_FAR_POINT = true;
		dx = x1 - x0;
		dy = y1 - y0;
	}
	else{
		EndPoint_IS_FAR_POINT = false;
		dx = x2 - x0;
		dy = y2 - y0;
	}
	float d = sqrtf(dx*dx + dy*dy);
	bool flag = (x0<xMin) || (x0>xMax || (y0<yMin) || (y0>yMax));
	if (!flag){ // intersection is on the line segment
		d *= -1;
	}
	return d;
}

// transform a point (x, y) to new point (tx, ty)
// rotation is [cos_theta, sin_theta; -sin_theta, cos_theta]
// translation is (center_x, center_y)
inline void transform_RT(float cos_theta, float sin_theta, 
	float center_x, float center_y, float x, float y, float& tx, float& ty)
{
	float xp = x - center_x;
	float yp = y - center_y;
	tx = cos_theta * xp + sin_theta * yp;
	ty = -sin_theta * xp + cos_theta * yp;
}

// decide whether a given line segment intersects a given rectangle
// http://zlethic.com/Ïß¶ÎÏà½»-intersection/
// http://www.cnblogs.com/lidaojian/archive/2012/05/06/2486600.html
inline float direction(PointXY p1, PointXY p2, PointXY p3){//p3p1->p2
	return (p3.x - p1.x) * (p2.y - p1.y) - (p2.x - p1.x) * (p3.y - p1.y);
}

inline bool online(PointXY p1, PointXY p2, PointXY p3){
	return (p3.x >= min(p1.x, p2.x) && p3.x <= max(p1.x, p2.x) && p3.y >= min(p1.y, p2.y) && p3.y <= max(p1.y, p2.y));
}

inline bool intersect(PointXY p1, PointXY p2, PointXY p3, PointXY p4){//p1-p2  p3-p4
	double d1 = direction(p3, p4, p1);
	double d2 = direction(p3, p4, p2);
	double d3 = direction(p1, p2, p3);
	double d4 = direction(p1, p2, p4);
	if (d1 * d2 < 0 && d3 * d4 < 0) return true;
	else if (d1 == 0 && online(p3, p4, p1)) return true;
	else if (d2 == 0 && online(p3, p4, p2)) return true;
	else if (d3 == 0 && online(p1, p2, p3)) return true;
	else if (d4 == 0 && online(p1, p2, p4)) return true;
	return false;
}

// judge whether line segment 1 is in the affact region of line segment 2
// line 1 is parameterized by two points (x1, y1) and (x2, y2)
// line 2 is y = 0; -length/2 <= x <= length/2
// Parameter for affect region is szWindow
// For affect region, refer
// [1] Kai Li, et al. Hierarchical Line Matching Based on Line-Junction-Line Structure 
//     Descriptor and Local Homography Estimation. Neurocomputing, 2016.
inline bool lineRectIntersection(float length, float szWindow,
	float x1, float y1, float x2, float y2)
{
	PointXY beg, end, lu, ru, ll, rl;
	// two points for line segment
	beg.x = x1;
	beg.y = y1;
	end.x = x2;
	end.y = y2;

	// rectangle 
	float xmin = -length/2 - szWindow;
	float xmax = length/2  +szWindow;
	float ymin = -szWindow;
	float ymax = szWindow;
	lu.x = xmin;
	lu.y = ymax;
	ru.x = xmax;
	ru.y = ymax;
	ll.x = xmin;
	ll.y = ymin;
	rl.x = xmax;
	rl.y = ymin;

	bool flag = false;
	if (intersect(lu, ru, beg, end))
		flag = true;
	else if (intersect(ll, rl, beg, end))
		flag = true;
	else if (intersect(lu, ll, beg, end))
		flag = true;
	else if (intersect(ru, rl, beg, end))
		flag = true;
	else if (beg.x > xmin && beg.x < xmax && beg.y > ymin && beg.y < ymax &&
		end.x > xmin && end.x < xmax && end.y > ymin &&end.y < ymax)
		flag = true;

	return flag;
}

// If the junction is ray-point-ray type,
// judge which line is base edge such that the junction angle is between [0, pi]
inline void coordinateForRayPointRay(float theta1, float theta2,
	bool E1_IS_FAR_POINT, bool E2_IS_FAR_POINT, bool& L1_IS_BASE_EDGE)
{
	// theta is calculate assuming EndPoint is the far point.
	//   When constructing junctions, this is not alway the case.
	// When calculating the distance between the intersection and endpoints of line segment 
	// (refer function pointLineSegmentDist), we already checked whether EndPoint is far point, 
	//   and the results is saved ENDPOIN_IS_FAR_POINT
	float new_theta1 = (E1_IS_FAR_POINT) ? theta1 : (theta1+MY_PI);
	float new_theta2 = (E2_IS_FAR_POINT) ? theta2 : (theta2+MY_PI);

	// suppose L1 is base edge that can construct the junction with angle between [0, pi], 
	// check whether it is true
	float diffAngle = new_theta2 - new_theta1;
	while (diffAngle >= MY_TWO_PI)
		diffAngle -= MY_TWO_PI;
	while (diffAngle < 0)
		diffAngle += MY_TWO_PI;

	L1_IS_BASE_EDGE = (diffAngle <= MY_PI) ? true : false;
};

// If the junction is T type,
// judge which line is base edge such that the junction angle is between [0, pi],
// also which endpint should be far points
inline void coordinateFor_T_Junction(float theta1, float theta2, float d1, float d2,
	bool& E1_IS_FAR_POINT, bool& E2_IS_FAR_POINT, bool& L1_IS_BASE_EDGE)
{
	if (d1 < 0){ // junction is on Line 1
		// L1 is base edge
		L1_IS_BASE_EDGE = true;

		// E2_IS_FAR_POINT don't change
		// E1_IS_FAR_POINT need check
		float new_theta2 = (E2_IS_FAR_POINT) ? theta2 : (theta2+MY_PI);
		float diffAngle = new_theta2 - theta1;
		while (diffAngle >= MY_TWO_PI)
			diffAngle -= MY_TWO_PI;
		while (diffAngle < 0)
			diffAngle += MY_TWO_PI;
		E1_IS_FAR_POINT = (diffAngle <= MY_PI) ? true : false;

	}
	else{ // junction is on Line 2
		// L2 is base edge
		L1_IS_BASE_EDGE = false;
		
		// E1_IS_FAR_POINT don't change
		// E2_IS_FAR_POINT need check
		float new_theta1 = (E1_IS_FAR_POINT) ? theta1 : (theta1+MY_PI);
		float diffAngle = new_theta1 - theta2;
		while (diffAngle >= MY_TWO_PI)
			diffAngle -= MY_TWO_PI;
		while (diffAngle < 0)
			diffAngle += MY_TWO_PI;
		E2_IS_FAR_POINT = (diffAngle <= MY_PI) ? true : false;
	}
}

// If the junction is X type, determine whch line is base edge 
//  such that the angle for 2 far points and intersection points is between [0, pi].
inline void coordinateFor_X_Junction(float theta1, float theta2, float d1, float d2,
	bool E1_IS_FAR_POINT, bool E2_IS_FAR_POINT, bool& L1_IS_BASE_EDGE)
{
	float new_theta1 = (E1_IS_FAR_POINT) ? theta1 : (theta1+MY_PI);
	float new_theta2 = (E2_IS_FAR_POINT) ? theta2 : (theta2+MY_PI);

	// suppose L1 is base edge according anti-clockwise direction
	// check whether it is true
	float diffAngle = new_theta2 - new_theta1;
	while (diffAngle >= MY_TWO_PI)
		diffAngle -= MY_TWO_PI;
	while (diffAngle < 0)
		diffAngle += MY_TWO_PI;
	L1_IS_BASE_EDGE = (diffAngle <= MY_PI) ? true : false;
}

#endif