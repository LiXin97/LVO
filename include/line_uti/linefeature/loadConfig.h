// Line & Junction detection, Matching, and reconstruction
// Author: Ji Zhao
// Date:   12/06/2016
// Email:  zhaoji84@gmail.com
// Shanghai ReadSense Tech., Inc.
// All rights reserved

#ifndef _LOAD_CONFIG_H
#define _LOAD_CONFIG_H

#include "configFileParser.h"

struct ConfigPara
{
	string pathImg1;
	string pathImg2;
	string left_img_path;
	string right_img_path;
	string camera_para_path;

	bool isStereoPair;
	bool isNeedRectify;
	bool isROI;
	// the following 4 parameters are ignored if (isROI == false)
	float xMin;
	float xMax;
	float yMin;
	float yMax;

	// for line matcher
	// 0: LSD; 1: EDline; 2: lvl; 3: SLSLAM; 4: CannyLines
	int lineMethod;
	int minLength;
	// 0: ORB; 1: SIFT
	int descriptorMethod;
	// mismatch removal. 0: VFC (recommend); 1: LPM; others: none
	int matchMethod;
	bool enableRansac;

	// reconstruction method
	int reconMethod;

	// visualization
	bool isShow;
	int margin;
	bool isHorizonal;
	// save results
	bool is_save_recon_rslt;
};


ConfigPara loadConfigFile(string fn);

typedef ConfigPara benchmarkPara;

#endif