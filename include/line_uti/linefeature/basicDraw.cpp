// Line & Junction detection, Matching, and reconstruction
// Author: Ji Zhao
// Date:   12/06/2016
// Email:  zhaoji84@gmail.com
// Shanghai ReadSense Tech., Inc.
// All rights reserved

#include "basicDraw.h"

Scalar colormap(int i)
{
	Scalar C;
	int idx = (i % 9);
	switch (idx) {
	case 0:
		C = Scalar(255, 0, 0);
		break;
	case 1:
		C = Scalar(0, 255, 0);
		break;
	case 2:
		C = Scalar(0, 0, 255);
		break;
	case 3:
		C = Scalar(255, 255, 0);
		break;
	case 4:
		C = Scalar(255, 0, 255);
		break;
	case 5:
		C = Scalar(0, 255, 255);
		break;
	case 6:
		C = Scalar(0, 192, 192);
		break;
	case 7:
		C = Scalar(192, 0, 192);
		break;
	case 8:
		C = Scalar(192, 192, 0);
		break;
	default:
		C = Scalar(255, 255, 255);
		break;
	}
	return C;
}

string num2str(int val) {
	stringstream ss;
	ss << val;
	string s1 = ss.str();
	return s1;
}
