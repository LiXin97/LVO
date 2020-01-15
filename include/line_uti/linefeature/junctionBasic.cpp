#include "junctionBasic.h"


void convertJuncToKeyPts(map<int, Junction> mapJunction, vector<junctKeyPt>& vecKeyPts)
{
	vecKeyPts.clear();
	int nJunc = mapJunction.size();
	vecKeyPts.reserve(nJunc);

	map<int, Junction>::const_iterator it;
	for (it = mapJunction.begin(); it != mapJunction.end(); ++it)
	{
		int key = it->first;
		Junction aJunc = it->second;

		float x = aJunc.origin.x;
		float y = aJunc.origin.y;
		float angle1, angle2;

		if (aJunc.E1_IS_FAR_POINT)
			angle1 = aJunc.line1.theta;
		else
			angle1 = aJunc.line1.theta + MY_PI;

		if (aJunc.E2_IS_FAR_POINT)
			angle2 = aJunc.line2.theta;
		else
			angle2 = aJunc.line2.theta + MY_PI;

		if (!aJunc.L1_IS_BASE_EDGE){
			float tmp;
			tmp = angle1;
			angle1 = angle2;
			angle2 = tmp;
		}

		// wrap angle to [0, 2*pi)
		while (angle1 >= MY_TWO_PI)
			angle1 -= MY_TWO_PI;
		while (angle1 < 0)
			angle1 += MY_TWO_PI;
		while (angle2 >= MY_TWO_PI)
			angle2 -= MY_TWO_PI;
		while (angle2 < 0)
			angle2 += MY_TWO_PI;

		float mid_ang1_ang2 = atan2(sin(angle1) + sin(angle2), cos(angle1) + cos(angle2));
		float dif_ang1_ang2 = angle2 - angle1;

		// wrap angle to [0, 2*pi)
		while (mid_ang1_ang2 >= MY_TWO_PI)
			mid_ang1_ang2 -= MY_TWO_PI;
		while (mid_ang1_ang2 < 0)
			mid_ang1_ang2 += MY_TWO_PI;
		while (dif_ang1_ang2 >= MY_TWO_PI)
			dif_ang1_ang2 -= MY_TWO_PI;
		while (dif_ang1_ang2 < 0)
			dif_ang1_ang2 += MY_TWO_PI;

		junctKeyPt Pt;
		Pt.key = key;
		Pt.x = x;
		Pt.y = y;
		Pt.angle1 = angle1;
		Pt.angle2 = angle2;
		Pt.mid_ang1_ang2 = mid_ang1_ang2;
		Pt.dif_ang1_ang2 = dif_ang1_ang2;
		Pt.type = aJunc.type;

		vecKeyPts.push_back(Pt);
	}
}