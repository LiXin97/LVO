#include "lpm.h"

void calcDistMatrix(float* X, float* Y, float* distMat, int numPoint)
{
	// calculate distance matrix
	// upper triangle (i<j) is distance matrix for X
	// lower triangle (i>j) is distance matrix for Y
	for (int i = 0; i < numPoint; i++){
		float xL_1 = X[i * 2];
		float xL_2 = X[i * 2 + 1];
		float yL_1 = Y[i * 2];
		float yL_2 = Y[i * 2 + 1];
		distMat[i * numPoint + i] = 0;
		for (int j = i + 1; j < numPoint; j++){
			float xR_1 = X[j * 2];
			float xR_2 = X[j * 2 + 1];
			float yR_1 = Y[j * 2];
			float yR_2 = Y[j * 2 + 1];

			// (row<col): distance matrix for X
			float d1 = xL_1 - xR_1;
			float d2 = xL_2 - xR_2;
			distMat[i * numPoint + j] = d1*d1 + d2*d2;

			// (row>col): distance matrix for Y
			d1 = yL_1 - yR_1;
			d2 = yL_2 - yR_2;
			distMat[j * numPoint + i] = d1*d1 + d2*d2;
		}
	}
}

void find_K_Smallest(float* distVec, int num_valid, int K, int* neighbors)
{
	for (int i = 0; i < K; i++){
		float minValue = FLT_MAX;
		int minIndex = -1;
		for (int j = 0; j < num_valid; j++){
			float curValue = distVec[j];
			if (curValue < minValue){
				minValue = curValue;
				minIndex = j;
			}
		}
		neighbors[i] = minIndex;
		distVec[minIndex] = FLT_MAX;
	}
}

void LPMatching(float* X, float* Y, bool* p, int numPoint,
	float lambda1, int numNeigh1, float lambda2, int numNeigh2)
{
	float* distMat = new float[numPoint*numPoint];
	float* distVecX = new float[numPoint];
	float* distVecY = new float[numPoint];
	int* neighborX = new int[numPoint];
	int* neighborY = new int[numPoint];
	bool* p_old = new bool[numPoint];
	int numNeighCands;
	int numNeigh;
	float lambda;

	calcDistMatrix(X, Y, distMat, numPoint);
	for (int i = 0; i < numPoint; i++){
		p[i] = true;
		p_old[i] = true;
	}
	numNeighCands = numPoint;

	for (int iter = 0; iter < 2; iter++){
		switch (iter){
		case 0:
			lambda = lambda1;
			numNeigh = numNeigh1;
			break;
		case 1:
			lambda = lambda2;
			numNeigh = numNeigh2;
			break;
		default:
			lambda = 4.0f;
			numNeigh = 8;
		}
		lambda += 0.000001f; // modified
		if (numNeighCands < numNeigh + 1){
			continue;
		}

		numNeighCands = 0;
		for (int i = 0; i < numPoint; i++){
			int num_valid = 0;
			for (int j = 0; j < numPoint; j++){
				int idx_min = (i < j) ? i : j;
				int idx_max = (i > j) ? i : j;
				int id1 = idx_min * numPoint + idx_max;
				int id2 = idx_max * numPoint + idx_min;

				if (p_old[j] && i != j){
					distVecX[num_valid] = distMat[id1];
					distVecY[num_valid] = distMat[id2];
					num_valid++;
				}
			}

			find_K_Smallest(distVecX, num_valid, numNeigh, neighborX);
			find_K_Smallest(distVecY, num_valid, numNeigh, neighborY);

			int cost = 0;
			for (int m = 0; m < numNeigh; m++){
				int tmp = neighborX[m];
				bool isMember = false;
				for (int n = 0; n < numNeigh; n++){
					if (neighborY[n] == tmp){
						isMember = true;
						break;
					}
				}
				if (!isMember)
					cost++;
			}
			cost *= 2;

			if (cost <= lambda){
				p[i] = true;
				numNeighCands++;
			}
			else{
				p[i] = false;
			}

		}
		for (int i = 0; i < numPoint; i++){
			p_old[i] = p[i];
		}

	}

//	for (int i = 0; i < numPoint; i++){
//		if (p[i])
//			Prob[i] = 1.0f;
//		else
//			Prob[i] = 0.0f;
//	}

	delete[] distMat;
	delete[] distVecX;
	delete[] distVecY;
	delete[] neighborX;
	delete[] neighborY;
	delete[] p_old;
}