// Line & Junction detection, Matching, and reconstruction
// Author: Ji Zhao
// Date:   12/06/2016
// Email:  zhaoji84@gmail.com
// Shanghai ReadSense Tech., Inc.
// All rights reserved

#include "loadConfig.h"

ConfigPara loadConfigFile(string fn)
{
	// load parameters from configuration file
	cfg::Config cfgFile(fn);
	ConfigPara para;
	para.pathImg1 = cfgFile.pString("image1");
	para.pathImg2 = cfgFile.pString("image2");
	para.left_img_path = cfgFile.pString("left_image_path");
	para.right_img_path = cfgFile.pString("right_image_path");
	para.camera_para_path = cfgFile.pString("camera_para_path");

	para.isStereoPair = cfgFile.pBool("isStereoPair");
	para.isNeedRectify = cfgFile.pBool("isNeedRectify");
	para.isROI = cfgFile.pBool("isROI");
	para.xMin = cfgFile.pDouble("roiMinX");
	para.xMax = cfgFile.pDouble("roiMaxX");
	para.yMin = cfgFile.pDouble("roiMinY");
	para.yMax = cfgFile.pDouble("roiMaxY");


	para.lineMethod = cfgFile.pInt("lineMethod");
	para.minLength = cfgFile.pDouble("minLength");
	para.descriptorMethod = cfgFile.pInt("descriptorMethod");
	para.matchMethod = cfgFile.pInt("matchMethod");
	para.enableRansac = cfgFile.pBool("enableRansac");

	para.reconMethod = cfgFile.pInt("reconMethod");

	para.isShow = cfgFile.pBool("isShow");
	para.margin = cfgFile.pInt("marginBetweenImgPair");
	para.isHorizonal = cfgFile.pBool("isHorizonal");
	para.is_save_recon_rslt = cfgFile.pBool("is_save_recon_rslt");
	return para;
};