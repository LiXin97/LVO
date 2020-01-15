#ifndef _JUNCTION_BASIC_H
#define _JUNCTION_BASIC_H

#include <vector>
#include <map>
#include <iostream>

#include "lineDetectorWrapper.h"
#include "geometry.h"

struct Junction
{
	Line line1;
	Line line2;
	unsigned short idxLine1;
	unsigned short idxLine2;
	// intersection point
	Point2f origin;
	// junction type: ray-junction-ray, T-type, X-type
	junctionType type;

	// in order to construct the descriptor, we need to define the coordinate of first quadrant
	// we define three boolean variables to determine the coordinate framework
	bool L1_IS_BASE_EDGE; // which line segment is base edge, L1 or L2?
	bool E1_IS_FAR_POINT; // which point is far from interection point in L1, end pint or start point?
	bool E2_IS_FAR_POINT; // which point is far from interection point in L2, end point or start point?
};

struct junctKeyPt{
	int key; // key for corresponding junction map
	float x;
	float y;
	// in order to construct the descriptor, 
	//    define the coordinate of first quadrant
	float angle1;
	float angle2;
	float mid_ang1_ang2;
	float dif_ang1_ang2;
	junctionType type;
};

void convertJuncToKeyPts(map<int, Junction> mapJunction, vector<junctKeyPt>& vecKeyPts);


#endif