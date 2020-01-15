#ifndef _LOCALITY_PRESERVING_MATCHING_H
#define _LOCALITY_PRESERVING_MATCHING_H

#include <iostream>
#include <float.h>
using namespace std;

// Mismatch removal by locality preserving matching (LPM)
// Author: Ji Zhao
// 01/04/2017
// Reference:
// [1] Jiayi Ma, Ji Zhao, Hanqi Guo, et al. 
//     Locality Preserving Matching. IJCAI, 2017.

void calcDistMatrix(float* X, float* Y, float* distMat, int numPoint);

void find_K_Smallest(float* distVec, int num_valid, int K, int* neighbors);

void LPMatching(float* X, float* Y, bool* p, int numPoint, 
	float lambda1 = 6.0f, int numNeigh1 = 4, float lambda2 = 4.0f, int numNeigh2 = 4);

//#define FLT_MAX 20
#endif
