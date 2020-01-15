/// \author Jonas Witt
/// \file
/// \brief %Edge and line detection and extraction algorithms.

#pragma once

#include "VisionTypes.h"
#include "ObjectPool.h"
#include <opencv2/core/core.hpp>

namespace lvl {

struct LineDetectorParameters
{
	template<typename Line2VecType>
	friend void detectLines(const cv::Mat&, Line2VecType&, LineDetectorParameters&);
	
	LineDetectorParameters()
	{
		edgeFilterKernelSize = 3; // 5;
		edgeDetectorHighThreshold = 300.0f; // 500.0;
		preSmoothingGaussianBlurSigma = 0.0f; // 1.0;
		douglasPeuckerLineEps = 0.8f; // 1.6f
		minLineLength = 15;
	}

	int edgeFilterKernelSize;
	double edgeDetectorHighThreshold;
	double preSmoothingGaussianBlurSigma;
	float douglasPeuckerLineEps;
	unsigned int minLineLength;	

	const cv::Mat& getXGradient() const { return dx;}
	const cv::Mat& getYGradient() const { return dy;}
	const cv::Mat& getEdgeXCoords() const { return edgeXCoords;}
	const cv::Mat& getEdgeYCoords() const { return edgeYCoords;}
	const cv::Mat& getBinaryEdgeImage() const { return edgeImage;}
	const ::std::vector<Edge<cv::Point>*>& getEdgeChains() const { return edges;}

protected:
	// Some temporary variables that are used by detectLines
	// can be accessed after the function was called
	cv::Mat edgeXCoords, edgeYCoords, dx, dy, edgeImage, smoothImage;
	::std::vector<Edge<cv::Point>*> edges;
	::std::vector<cv::Point2f> edge;
	ObjectPool<Edge<cv::Point> > edgePool;
};


template<typename T> 
void fastApproxPolyDP(	const ::std::vector<T>& srcPoints, 
						::std::vector<T>& dstPoints, 
						float eps, 
						::std::vector<size_t>& dstIndices );

template<typename T, typename LineVecType>
void approximateLines(	const ::std::vector<T>& points, 
						LineVecType& lines,
						float eps, 
						unsigned int minLinePoints = 3);

template<typename T, int lineDim>
void approximateEdges(	const ::std::vector<Edge<T>*>& edges, 
						::std::vector<Edge<T>*>& approxLineStrips, 
						::std::vector<Edge<T>*>& approxCurveStrips, 
						::std::vector<Line<float, lineDim> >& lines,
						float eps, 
						float curveThreshold = 5.0f, 
						size_t* pNumPointsBefore = NULL, 
						size_t* pNumPointsInLines = NULL, 
						size_t* pNumPointsInCurves = NULL);

template<typename T, int lineDim>
void getTwoPointLinesFromEdges(	const ::std::vector<Edge<T>*>& edges, 
								::std::vector<Line<float, lineDim> >& lines);

template<typename T> 
void findEdges(			const cv::Mat& binaryImage, 
						ObjectPool<Edge<T> >& edgePool,
						::std::vector<Edge<T>*>& edges);

inline void cannySubpix(const cv::Mat& image, cv::Mat& edges,
						cv::Mat& edgeXCoords, cv::Mat& edgeYCoords,
						double lowThresh, double highThresh,
						int apertureSize, cv::Mat& dx, cv::Mat& dy);

inline void cannySubpix(const cv::Mat& image, cv::Mat& edges,
						cv::Mat& edgeXCoords, cv::Mat& edgeYCoords,
						double lowThresh, double highThresh,
						int apertureSize);

template<typename Line2VecType>
void detectLines(	const cv::Mat& image, 
					Line2VecType& lines, 
					LineDetectorParameters& params);


} // namespace lvl


#include "Detection.hpp"
