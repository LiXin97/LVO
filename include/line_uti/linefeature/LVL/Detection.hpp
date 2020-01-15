/// \author Jonas Witt
/// \file
/// \brief Implements Detection.h

#pragma once

#include <iostream>
//#include <boost/static_assert.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <set>
#include "Algebra.h"
//#include "StereoEdges.h"
#include <opencv2/imgproc/types_c.h>//for CvSlice


namespace lvl {


inline int directionToXStep(char direction)
{
	switch(direction)
	{
	case 0: // East
	case 1: // North East
	case 7: // South East
		return 1;
	case 2: // North
	case 6: // South
		return 0;
	case 3: // North West
	case 4: // West
	case 5: // South West
		return -1;
	default:
		assert(direction >= 0 && direction < 8);
		return 0; // should not be called
	}
}

inline int directionToYStep(char direction)
{
	switch(direction)
	{
	case 0: // East
	case 4: // West
		return 0;
	case 1: // North East
	case 2: // North
	case 3: // North West
		return -1;
	case 5: // South West
	case 7: // South East
	case 6: // South
		return 1;	
	default:
		assert(direction >= 0 && direction < 8);
		return 0; // should not be called
	}
}

// Ramer-Douglas-Peucker algorithm for polygon simplification
template<typename T> 
void fastApproxPolyDP(	const ::std::vector<T>& srcPoints, 
						::std::vector<T>& dstPoints, 
						float eps, 
						::std::vector<size_t>& dstIndices )
{
	dstPoints.clear();
	dstIndices.clear();
    
	int count = (int)srcPoints.size();
    bool le_eps = false;

    assert( count >= 2);
    eps *= eps;

	CvSlice         slice = {0, 0}, right_slice = {0, 0};
	right_slice.start_index = count;
	slice.start_index = 0;
	slice.end_index = count - 1;

	static ::std::vector<CvSlice> stack;
	stack.reserve(100);
	stack.clear();
	stack.push_back(slice);

    // Run recursive simplification process
	while( stack.size() != 0 )
    {
		slice = stack.back();
		stack.pop_back();

		const T& end_pt = srcPoints[slice.end_index];
		const T& start_pt = srcPoints[slice.start_index];


        if( slice.end_index > slice.start_index + 1 )
        {
            float dx, dy, dist, max_dist = 0;

            dx = end_pt.x - start_pt.x;
            dy = end_pt.y - start_pt.y;

            assert( dx != 0 || dy != 0 );

            for(int i = slice.start_index + 1; i < slice.end_index; i++ )
            {
				const T& pt = srcPoints[i];
                dist = abs((pt.y - start_pt.y) * dx - (pt.x - start_pt.x) * dy);

                if( dist > max_dist )
                {
                    max_dist = dist;
                    right_slice.start_index = i;
                }
            }

            le_eps = max_dist * max_dist <= eps * (dx * dx + dy * dy);
        }
        else
        {
            assert( slice.end_index > slice.start_index );
            le_eps = true;
        }

        if( le_eps )
        {
			dstPoints.push_back(start_pt);
			dstIndices.push_back(slice.start_index);
        }
        else
        {
            right_slice.end_index = slice.end_index;
            slice.end_index = right_slice.start_index;
			stack.push_back(right_slice);
			stack.push_back(slice);
        }
    }

	dstPoints.push_back(srcPoints.back());
	dstIndices.push_back(srcPoints.size() - 1);
    
    // Do final clean-up of the approximated contour -
    // remove extra points on the [almost] stright lines. 
    
	assert(dstPoints.size() >= 2);
	T start_pt, end_pt, pt;

	start_pt = dstPoints[0];

    int reader2 = 1;
	int reader = 1;
	pt = dstPoints[reader++];

	int new_count = count = (int)dstPoints.size();
    for(int i = 1; i < count - 1 && new_count > 2; i++ )
    {
        float dx, dy, dist;
		end_pt = dstPoints[i+1];

        dx = end_pt.x - start_pt.x;
        dy = end_pt.y - start_pt.y;
        dist = abs((pt.x - start_pt.x)*dy - (pt.y - start_pt.y)*dx);
        if( dist * dist <= 0.5*eps*(dx*dx + dy*dy) && dx != 0 && dy != 0 )
        {
            new_count--;
			dstIndices[reader2] = dstIndices[i+1];
			dstPoints[reader2++] = start_pt = end_pt;
			
            i++;
				if(i+1 > count - 1)
			{
				pt = dstPoints[i];
				break;
			}

			pt = dstPoints[i+1];
            continue;
        }
		dstIndices[reader2] = dstIndices[i];
		dstPoints[reader2++] = start_pt = pt;
        pt = end_pt;
    }

	dstPoints[reader2] = pt;
	dstIndices[reader2] = dstIndices[count - 1];

    if( new_count < count )
	{
		dstPoints.resize(new_count);
		dstIndices.resize(new_count);
	}
}

template<typename T, typename LineVecType>
void approximateLines(	const ::std::vector<T>& points, 
						LineVecType& lines,
						float eps, 
						unsigned int minLinePoints)
{
	typedef typename LineVecType::value_type LineType;
	typedef typename LineType::Scalar _Tp;
	const int lineDim = LineType::Vector::RowsAtCompileTime;


	::std::vector<T> approxPoints, approxPointsSlice;
	::std::vector<size_t> approxIndices;
	int lastLineStartPointIndex = -1;
	fastApproxPolyDP(points, approxPoints, eps, approxIndices);

	const bool noLineMerging = true;
	float curveLength = 0.0f;
	assert(minLinePoints >= 3);

	for(unsigned int j = 0; j < approxIndices.size() - 1; ++j)
	{
		//assert(approxPoints[j] == points[approxIndices[j]]);
		float dx = points[approxIndices[j+1]].x - points[approxIndices[j]].x;
		float dy = points[approxIndices[j+1]].y - points[approxIndices[j]].y;
		float length = cv::sqrt(dx*dx + dy*dy);
		curveLength += length;

		if(approxIndices[j+1] - approxIndices[j] + 1 > minLinePoints)
		{
			cv::Vec<float, 2*lineDim> line;

			// exclude the endpoints from the optimization
			//approxPointsSlice.assign(&points[approxIndices[j] + 1], &points[approxIndices[j+1] - 1] + 1);	
			approxPointsSlice.assign(&points[approxIndices[j]], &points[approxIndices[j+1]] + 1);

			cv::fitLine(cv::Mat(approxPointsSlice), line, CV_DIST_L2, 0.0, 0.001, 0.001);
			//cv::fitLine(cv::Mat(approxPointsSlice), line, CV_DIST_HUBER, 0.2, 0.001, 0.001);
			//lines.push_back(linePool.malloc());
			LineType newLine;

			newLine.dir = (Eigen::Map<Eigen::Matrix<float, lineDim, 1> >(&line[0])).template cast<_Tp>();
			newLine.p1 = (Eigen::Map<Eigen::Matrix<float, lineDim, 1> >(&line[lineDim])).template cast<_Tp>();

			//memcpy((void*)&newLine.dir[0], (void*)&line[0], sizeof(Eigen::Matrix<float, lineDim, 1>));
			//memcpy((void*)&newLine.p1[0], (void*)&line[lineDim], sizeof(Eigen::Matrix<float, lineDim, 1>));

			newLine.length = length;

			// find the closest point on our new line to the previous p0			
			Eigen::Matrix<_Tp, lineDim, 1> p0 = (Eigen::Map<const Eigen::Matrix<float, lineDim, 1> >(&points[approxIndices[j]].x)).template cast<_Tp>();
			newLine.p1 += newLine.dir * newLine.dir.dot(p0 - newLine.p1) * 0.99f;
			if(dx * newLine.dir[0] + dy * newLine.dir[1] < 0.0f)
				newLine.dir *= -1.f;

			//newLine.p2 = newLine.p1 + length * newLine.dir;
			Eigen::Matrix<_Tp, lineDim, 1> p2 = (Eigen::Map<Eigen::Matrix<float, lineDim, 1> >(&approxPointsSlice.back().x)).template cast<_Tp>();
			newLine.p2 = newLine.p1 + newLine.dir * newLine.dir.dot(p2 - newLine.p1) * 0.99f;

			if(	lastLineStartPointIndex == -1 || 
        fabs(linePointIntersect2D(lines.back().p2, lines.back().dir, newLine.p1)) > 3.0f ||
        fabs(linePointIntersect2D(lines.back().p2, lines.back().dir, newLine.p2)) > 3.0f ||
				noLineMerging)
			{
				lines.push_back(newLine);
				lastLineStartPointIndex = (int) approxIndices[j];
			}
			else
			{
				approxPointsSlice.assign(&points[lastLineStartPointIndex], &points[approxIndices[j+1]] + 1);

				cv::fitLine(cv::Mat(approxPointsSlice), line, CV_DIST_L2, 0.0, 0.001, 0.001);
				//cv::fitLine(cv::Mat(approxPointsSlice), line, CV_DIST_HUBER, 0.2, 0.001, 0.001);
				//lines.push_back(linePool.malloc());
				//Line<float, lineDim> newLine;

				//memcpy((void*)&newLine.dir[0], (void*)&line[0], sizeof(Eigen::Matrix<float, lineDim, 1>));
				//memcpy((void*)&newLine.p1[0], (void*)&line[lineDim], sizeof(Eigen::Matrix<float, lineDim, 1>));

				LineType newLine;

				newLine.dir = (Eigen::Map<Eigen::Matrix<float, lineDim, 1> >(&line[0])).template cast<_Tp>();
				newLine.p1 = (Eigen::Map<Eigen::Matrix<float, lineDim, 1> >(&line[lineDim])).template cast<_Tp>();

				newLine.length = length;

				// find the closest point on our new line to the previous p0
				//Eigen::Matrix<_Tp, lineDim, 1> p0 = (Eigen::Map<Eigen::Matrix<float, lineDim, 1> >(&points[lastLineStartPointIndex].x)).template cast<_Tp>();
				newLine.p1 += newLine.dir * newLine.dir.dot(p0 - newLine.p1) * 0.99f;
				if(dx * newLine.dir[0] + dy * newLine.dir[1] < 0.0f)
					newLine.dir *= -1.f;

				//newLine.p2 = newLine.p1 + length * newLine.dir;
				Eigen::Matrix<_Tp, lineDim, 1> p2 = (Eigen::Map<Eigen::Matrix<float, lineDim, 1> >(&approxPointsSlice.back().x)).template cast<_Tp>();
				newLine.p2 = newLine.p1 + newLine.dir * newLine.dir.dot(p2 - newLine.p1) * 0.99f;

				newLine.length = (newLine.p2 - newLine.p1).norm();
				lines.back() = newLine;

				// erase the middle index, so the next line merging happens with all points of this merged line
				approxIndices.erase(approxIndices.begin() + j);
				--j;
			}
			
			

			
			//assert((length - (newLine.p1 - newLine.p2).norm()) < eps / 2.f);
			//assert((Eigen::Matrix<float, lineDim, 1>(&approxPointsSlice.front().x) - newLine.p1).norm() < eps && (Eigen::Matrix<float, lineDim, 1>(&approxPointsSlice.back().x) - newLine.p2).norm() < eps);
						
		}
	}

}


template<typename T, int lineDim>
void approximateEdges(	const ::std::vector<Edge<T>*>& edges, 
						::std::vector<Edge<T>*>& approxLineStrips, 
						::std::vector<Edge<T>*>& approxCurveStrips, 
						::std::vector<Line<float, lineDim> >& lines,
						float eps, 
						float curveThreshold, 
						size_t* pNumPointsBefore, 
						size_t* pNumPointsInLines, 
						size_t* pNumPointsInCurves)
{
	// approximateEdges works only for 2-dimensional or 3-dimensional cases with single precision
	// (float). The Line and Edge must have consistent dimensions.
        /*
       BOOST_STATIC_ASSERT_MSG((lineDim == 2 || lineDim == 3) && (lineDim == sizeof(T) / sizeof(float)),
							"approximateEdges works only for 2D or 3D cases with single precision (float). Also, the Line and Edge must have consistent dimensions.");
*/
	static ObjectPool<Edge<T> > edgePool(500);
	//static ObjectPool<Line<_Tp, 3>> linePool(4000);
	edgePool.freeAll();
	//linePool.freeAll();
	approxLineStrips.resize(edges.size());
	approxCurveStrips.resize(edges.size());
	lines.clear();
	size_t numPointsBefore = 0;
	size_t numPointsInLines = 0;
	size_t numPointsInCurves = 0;
	//float previousDisparity;
	::std::vector<size_t> approxIndices;
	::std::vector<T> approxPointsSlice;
	approxIndices.reserve(30);
	approxPointsSlice.reserve(200);
	
	
	int lineStripCount = 0;
	int curveStripCount = 0;
	int lineCount = 0;

	for(unsigned int i=0; i<edges.size(); ++i)
	{		
		approxCurveStrips[curveStripCount] = edgePool.malloc();

		// TODO: replace this with a solid optimization
		// low-pass filter disparities
		/*if(disparityFilter < 1.0f)
		{
			::std::vector<cv::Point3f>& points = edges[i]->points;
			float complFilter = 1.0f - disparityFilter;
			previousDisparity = points[0].z;
			for(int j=1; j<points.size(); ++j)
			{
				points[j].z = complFilter*previousDisparity +  disparityFilter*points[j].z;
			}
			previousDisparity = points[points.size()-1].z;
			for(int j=points.size()-2; j>=0; --j)
			{
				points[j].z = complFilter*previousDisparity + disparityFilter*points[j].z;
			}
		}*/


 		fastApproxPolyDP(edges[i]->points, approxCurveStrips[curveStripCount]->points, eps, approxIndices);

		//approxPolyDP(cv::Mat(points), approxEdges[i]->points, 1.0, false);
		size_t numApproxPoints = approxCurveStrips[curveStripCount]->points.size();
		float curveLength = 0.0f;

		for(unsigned int j=0; j<numApproxPoints-1; ++j)
		{
			assert(approxCurveStrips[curveStripCount]->points[j] == edges[i]->points[approxIndices[j]]);
			float dx = approxCurveStrips[curveStripCount]->points[j+1].x - approxCurveStrips[curveStripCount]->points[j].x;
			float dy = approxCurveStrips[curveStripCount]->points[j+1].y - approxCurveStrips[curveStripCount]->points[j].y;			
			float length = cv::sqrt(dx*dx + dy*dy);
			curveLength += length;

      if(approxIndices[j+1] - approxIndices[j] > 15)
			{
				cv::Vec<float, 2*lineDim> line;
				approxPointsSlice.assign(&edges[i]->points[approxIndices[j]], &edges[i]->points[approxIndices[j+1]]);	
				
				cv::fitLine(cv::Mat(approxPointsSlice), line, CV_DIST_L2, 0.0, 0.001, 0.001);
				//lines.push_back(linePool.malloc());
				Line<float, lineDim> newLine;
				
				memcpy((void*)&newLine.dir[0], (void*)&line[0], sizeof(Eigen::Matrix<float, lineDim, 1>));
				memcpy((void*)&newLine.p1[0], (void*)&line[lineDim], sizeof(Eigen::Matrix<float, lineDim, 1>));

				newLine.length = length;

				// find the closest point on our new line to the previous p0
				newLine.p1 += newLine.dir * newLine.dir.dot(Eigen::Matrix<float, lineDim, 1>(&approxCurveStrips[curveStripCount]->points[j].x) - newLine.p1);

				if(dx * newLine.dir[0] + dy * newLine.dir[1] < 0.0f)
					newLine.dir *= -1.f;
			
				newLine.p2 = newLine.p1 + length * newLine.dir;

				lines.push_back(newLine);
				//lineCount++;		
			}
		}

		//approxCurveStrips[curveStripCount]->length = curveLength;

		if(curveLength / numApproxPoints < curveThreshold)
		{	
			numPointsInCurves += numApproxPoints;
			++curveStripCount;			
		}
		else
		{
			numPointsInLines += numApproxPoints;
			approxLineStrips[lineStripCount] = approxCurveStrips[curveStripCount];			
			++lineStripCount;			
		}
		numPointsBefore += edges[i]->points.size();
		
	}

	approxCurveStrips.resize(curveStripCount);
	approxLineStrips.resize(lineStripCount);
	//lines.resize(lineCount);

	if(pNumPointsBefore)
		*pNumPointsBefore = numPointsBefore;

	if(pNumPointsInLines)
		*pNumPointsInLines = numPointsInLines;

	if(pNumPointsInCurves)
		*pNumPointsInCurves = numPointsInCurves;

}



template<typename T, int lineDim>
void getTwoPointLinesFromEdges(	const ::std::vector<Edge<T>*>& edges, 
								::std::vector<Line<float, lineDim> >& lines)
{
  /*
	BOOST_STATIC_ASSERT_MSG((lineDim == 2 || lineDim == 3) && (lineDim == sizeof(T) / sizeof(float)), 
							"getTwoPointLinesFromEdges works only for 2D or 3D cases with single precision (float). Also, the Line and Edge must have consistent dimensions.");
*/
	Line<float, lineDim> newLine;
	lines.clear();

	for(unsigned int i=0; i < edges.size(); ++i)
	{
		for(unsigned int j=0; j < edges[i]->points.size() - 1; ++j)
		{
			memcpy((void*)&newLine.p1[0], (void*)&edges[i]->points[j],   sizeof(Eigen::Matrix<float, lineDim, 1>));
			memcpy((void*)&newLine.p2[0], (void*)&edges[i]->points[j+1], sizeof(Eigen::Matrix<float, lineDim, 1>));
			//newLine.p1 = edges[i]->points[j];
			//newLine.p2 = edges[i]->points[j+1];

			newLine.length = (newLine.p2 - newLine.p1).norm();
			newLine.dir = (newLine.p2 - newLine.p1) / newLine.length;

			lines.push_back(newLine);
		}
	}
}



/*inline int directionToXStep(char direction)
{
	switch(direction)
	{
	case 0: // East
	case 1: // North East
	case 7: // South East
		return 1;
	case 2: // North
	case 6: // South
		return 0;
	case 3: // North West
	case 4: // West
	case 5: // South West
		return -1;
	default:
		assert(direction >= 0 && direction < 8);
		return 0; // should not be called
	}
}

inline int directionToYStep(char direction)
{
	switch(direction)
	{
	case 0: // East
	case 4: // West
		return 0;
	case 1: // North East
	case 2: // North
	case 3: // North West
		return -1;
	case 5: // South West
	case 7: // South East
	case 6: // South
		return 1;	
	default:
		assert(direction >= 0 && direction < 8);
		return 0; // should not be called
	}
}

struct NeighbourEdgePixel
{
	int x, y;
	char direction;
};*/

inline unsigned int getStraightEdgeNeighbours(const cv::Mat& binaryEdges, int x, int y, char excludeDirection, NeighbourEdgePixel* neighbours)
{
	const int& width = binaryEdges.cols;
	const int& height = binaryEdges.rows;
	unsigned int foundNeighbours = 0;

	// check neighbours in all straight directions
	for(char dir=0; dir < 8; dir +=2)
	{
		int testX = x + directionToXStep(dir);
		int testY = y + directionToYStep(dir);

		if(dir == excludeDirection || testX < 0 || testX >= width || testY < 0 || testY >= height)
			continue;

		if(binaryEdges.at<unsigned char>(testY, testX) > 0)
		{
			neighbours[foundNeighbours].x = testX;
			neighbours[foundNeighbours].y = testY;
			neighbours[foundNeighbours].direction = dir;
			++foundNeighbours;
		}
	}

	return foundNeighbours;
}

inline unsigned int getDiagonalEdgeNeighbours(const cv::Mat& binaryEdges, int x, int y, char excludeDirection, NeighbourEdgePixel* neighbours, const cv::Mat& visitedPixels = cv::Mat())
{
	const int& width = binaryEdges.cols;
	const int& height = binaryEdges.rows;
	unsigned int foundNeighbours = 0;

	// check neighbours in all diagonal directions
	for(char dir=1; dir < 8; dir +=2)
	{
		int testX = x + directionToXStep(dir);
		int testY = y + directionToYStep(dir);

		if(dir == excludeDirection || testX < 0 || testX >= width || testY < 0 || testY >= height)
			continue;

		if(binaryEdges.at<unsigned char>(testY, testX) > 0 && 
			(visitedPixels.empty() || visitedPixels.at<unsigned char>(testY, testX) == 0) )
		{
			neighbours[foundNeighbours].x = testX;
			neighbours[foundNeighbours].y = testY;
			neighbours[foundNeighbours].direction = dir;
			++foundNeighbours;
		}
	}

	return foundNeighbours;
}


template<typename T> 
bool followEdge(		const cv::Mat& binaryEdges, 
						const NeighbourEdgePixel& initialNeighbour,
						cv::Mat& visitedPixels, 						
						ObjectPool<Edge<T> >& edgePool,
						::std::vector<Edge<T>*>& edges,
						Edge<T>* currentEdgeChain)
{
	NeighbourEdgePixel neighbours[4];
	NeighbourEdgePixel lastNeighbour;
	int numNeighbours = 1;
	neighbours[0] = initialNeighbour;

	while(numNeighbours > 0)
	{
		if(numNeighbours > 1)
		{
			// branch to all neighbours
			for(int i=0; i < numNeighbours; ++i)
			{
				Edge<T>* newBranch = edgePool.malloc();
				newBranch->clear();
				newBranch->startPointConnections.push_back(currentEdgeChain);

				// insert start point of new branch
				newBranch->points.push_back(T(lastNeighbour.x, lastNeighbour.y));
				visitedPixels.at<unsigned char>(lastNeighbour.y, lastNeighbour.x) = 1;
				newBranch->edgeIndex = currentEdgeChain->edgeIndex;

				if(followEdge(binaryEdges, neighbours[i], visitedPixels, edgePool, edges, newBranch))
					currentEdgeChain->endPointConnections.push_back(newBranch);
			}

			if(!currentEdgeChain->endPointConnections.empty())
			{
				edges.push_back(currentEdgeChain);
				return true;
			}
			else
			{
				break;
			}
		}

		// here we have exactly one neighbour

		lastNeighbour = neighbours[0];

		if(visitedPixels.at<unsigned char>(neighbours[0].y, neighbours[0].x) == 0)
		{
			visitedPixels.at<unsigned char>(neighbours[0].y, neighbours[0].x) = 1;
			currentEdgeChain->points.push_back(T(neighbours[0].x, neighbours[0].y));
			char excludeDirection = (neighbours[0].direction + 4) % 8;

			numNeighbours = getStraightEdgeNeighbours(binaryEdges, neighbours[0].x, neighbours[0].y, excludeDirection, neighbours);
			
			if(numNeighbours == 0)
				numNeighbours = getDiagonalEdgeNeighbours(binaryEdges, neighbours[0].x, neighbours[0].y, excludeDirection, neighbours, visitedPixels);
		}
		else
		{
			break;
		}
	}

	if(currentEdgeChain->points.size() > 2)
	{
		edges.push_back(currentEdgeChain);
		return true;
	}
	else
	{
		return false;
	}
}

template<typename T> 
void findEdges(			const cv::Mat& binaryEdges, 
						ObjectPool<Edge<T> >& edgePool,
						::std::vector<Edge<T>*>& edges)
{
	assert(binaryEdges.type() == CV_8UC1);

	cv::Mat visitedPixels(binaryEdges.size(), CV_8UC1);
	visitedPixels = 0.0;

	const int& width = binaryEdges.cols;
	const int& height = binaryEdges.rows;

	NeighbourEdgePixel neighbours[4];
	unsigned int numNeighbours;
	bool includeCircles = false;
	int edgeIndex = 0;

	for(int passes=0; passes < 2; passes++)
	{
		if(passes > 0)
			includeCircles = true;

		for(int y=0; y < height; ++y)
		{
			for(int x=0; x < width; ++x)
			{
				if(binaryEdges.at<unsigned char>(y, x) != 0 && visitedPixels.at<unsigned char>(y,x) == 0)
				{
					numNeighbours  = getStraightEdgeNeighbours(binaryEdges, x, y, -1, neighbours);
					numNeighbours += getDiagonalEdgeNeighbours(binaryEdges, x, y, -1, neighbours);
								
					// Here, we must have one or more neighbours
					if(numNeighbours == 1 || (numNeighbours == 2 && includeCircles))
					{
						visitedPixels.at<unsigned char>(y,x) = 1;

						Edge<T>* newBranch = edgePool.malloc();
						newBranch->clear();

						// insert start point of new branch
						newBranch->points.push_back(T(x, y));
						newBranch->edgeIndex = edgeIndex;

						if(followEdge(binaryEdges, neighbours[0], visitedPixels, edgePool, edges, newBranch))
							edgeIndex++;						
					}
				}
			}
		}
	}

}


inline void cannySubpix(const cv::Mat& image, cv::Mat& edges,
						cv::Mat& edgeXCoords, cv::Mat& edgeYCoords,
						double lowThresh, double highThresh,
						int apertureSize, cv::Mat& dx, cv::Mat& dy)
{
	cv::Size size = image.size();
    edges.create(image.size(), CV_8UC1);
	edgeXCoords.create(image.size(), CV_32FC1);
	edgeYCoords.create(image.size(), CV_32FC1);
	edgeXCoords = 0.0;
	edgeYCoords = 0.0;
	//dx.create(size, CV_16SC1 );
	//dy.create(size, CV_16SC1 );

	cv::Mat src = image;
	cv::Mat dst = edges;

	bool color = (CV_MAT_TYPE( src.type() ) == CV_8UC3);

    //cv::Ptr<CvMat> dx, dy;
    cv::AutoBuffer<char> buffer;
    ::std::vector<uchar*> stack;
    uchar **stack_top = 0, **stack_bottom = 0;

    int low, high;
    int* mag_buf[3];
    uchar* map;
    int mapstep, maxsize;
    int i, j;
    CvMat mag_row;

    if((CV_MAT_TYPE( src.type() ) != CV_8UC1 &&
		CV_MAT_TYPE( src.type() ) != CV_8UC3) ||
        CV_MAT_TYPE( dst.type() ) != CV_8UC1 )
        CV_Error( CV_StsUnsupportedFormat, "" );

    if( !CV_ARE_SIZES_EQ( &src, &dst ))
        CV_Error( CV_StsUnmatchedSizes, "" );

    if( lowThresh > highThresh )
    {
        double t;
        CV_SWAP( lowThresh, highThresh, t );
    }

    apertureSize &= INT_MAX;
    if( (apertureSize & 1) == 0 || apertureSize < 3 || apertureSize > 7 )
        CV_Error( CV_StsBadFlag, "" );

	if(color)
	{
		cv::Mat dxColor, dyColor, magColor;
		//cv::Sobel(image, dxColor, CV_16S, 1, 0, apertureSize);
		//cv::Sobel(image, dyColor, CV_16S, 0, 1, apertureSize);
		if(apertureSize == 3)
		{
			cv::Scharr(image, dxColor, CV_16S, 1, 0);
			cv::Scharr(image, dyColor, CV_16S, 0, 1);
		}
		else
		{
			cv::Sobel(image, dxColor, CV_16S, 1, 0, apertureSize);
			cv::Sobel(image, dyColor, CV_16S, 0, 1, apertureSize);
		}

		//cv::magnitude(dxColor, dyColor, magColor);

		dx.create(size, CV_16SC1 );
		dy.create(size, CV_16SC1 );

		for(int y=0; y < image.rows; y++)
		{
			for(int x=0; x < image.cols; x++)
			{
				const cv::Vec3s& dxCol = dxColor.at<cv::Vec3s>(y, x);
				const cv::Vec3s& dyCol = dyColor.at<cv::Vec3s>(y, x);
				cv::Vec3s magCol;
				magCol[0] = (short) cv::sqrt(dxCol[0]*dxCol[0] + dyCol[0]*dyCol[0]);
				magCol[1] = (short) cv::sqrt(dxCol[1]*dxCol[1] + dyCol[1]*dyCol[1]);
				magCol[2] = (short) cv::sqrt(dxCol[2]*dxCol[2] + dyCol[2]*dyCol[2]);

				if(magCol[0] > magCol[1] && magCol[0] > magCol[2])
				{
					dx.at<short>(y, x) = dxCol[0];
					dy.at<short>(y, x) = dyCol[0];
				}
				else if(magCol[1] > magCol[2])
				{
					dx.at<short>(y, x) = dxCol[1];
					dy.at<short>(y, x) = dyCol[1];
				}
				else
				{
					dx.at<short>(y, x) = dxCol[2];
					dy.at<short>(y, x) = dyCol[2];
				}
			}
		}
	}
	else
	{		
		if(apertureSize == 3)
		{
			cv::Scharr(image, dx, CV_16S, 1, 0);
			cv::Scharr(image, dy, CV_16S, 0, 1);
		}
		else
		{
			cv::Sobel(image, dx, CV_16S, 1, 0, apertureSize);
			cv::Sobel(image, dy, CV_16S, 0, 1, apertureSize);
		}
	}

    /*if( icvCannyGetSize_p && icvCanny_16s8u_C1R_p && !(flags & CV_CANNY_L2_GRADIENT) )
    {
        int buf_size=  0;
        IPPI_CALL( icvCannyGetSize_p( size, &buf_size ));
        CV_CALL( buffer = cvAlloc( buf_size ));
        IPPI_CALL( icvCanny_16s8u_C1R_p( (short*)dx->data.ptr, dx->step,
                                     (short*)dy->data.ptr, dy->step,
                                     dst->data.ptr, dst->step,
                                     size, (float)lowThresh,
                                     (float)highThresh, buffer ));
        EXIT;
    }*/

    
    Cv32suf ul, uh;
    ul.f = (float)lowThresh;
    uh.f = (float)highThresh;
    low = ul.i;
    high = uh.i;
    

    buffer.allocate( (size.width+2)*(size.height+2) + (size.width+2)*3*sizeof(int) );

    mag_buf[0] = (int*)(char*)buffer;
    mag_buf[1] = mag_buf[0] + size.width + 2;
    mag_buf[2] = mag_buf[1] + size.width + 2;
    map = (uchar*)(mag_buf[2] + size.width + 2);
    mapstep = size.width + 2;

    maxsize = MAX( 1 << 10, size.width*size.height/10 );
    stack.resize( maxsize );
    stack_top = stack_bottom = &stack[0];

    memset( mag_buf[0], 0, (size.width+2)*sizeof(int) );
    memset( map, 1, mapstep );
    memset( map + mapstep*(size.height + 1), 1, mapstep );

    /* sector numbers 
       (Top-Left Origin)

        1   2   3
         *  *  * 
          * * *  
        0*******0
          * * *  
         *  *  * 
        3   2   1
    */

    #define CANNY_PUSH(d)    *(d) = (uchar)2, *stack_top++ = (d)
    #define CANNY_POP(d)     (d) = *--stack_top

    mag_row = cvMat( 1, size.width, CV_32F );

    // calculate magnitude and angle of gradient, perform non-maxima supression.
    // fill the map with one of the following values:
    //   0 - the pixel might belong to an edge
    //   1 - the pixel can not belong to an edge
    //   2 - the pixel does belong to an edge
    for( i = 0; i <= size.height; i++ )
    {
        int* _mag = mag_buf[(i > 0) + 1] + 1;
        float* _magf = (float*)_mag;
		const short* _dx = (short*)(dx.data + dx.step*i);
		const short* _dy = (short*)(dy.data + dy.step*i);
        uchar* _map;
        int x, y;
        int magstep1, magstep2;
        int prev_flag = 0;

        if( i < size.height )
        {
            _mag[-1] = _mag[size.width] = 0;

			for( j = 0; j < size.width; j++ )
			{
				x = _dx[j]; y = _dy[j];
				//_magf[j] = (float)::std::sqrt((double)x*x + (double)y*y);
				_magf[j] = cv::sqrt((float)x*x + (float)y*y);
			}
        }
        else
            memset( _mag-1, 0, (size.width + 2)*sizeof(int) );

        // at the very beginning we do not have a complete ring
        // buffer of 3 magnitude rows for non-maxima suppression
        if( i == 0 )
            continue;

        _map = map + mapstep*i + 1;
        _map[-1] = _map[size.width] = 1;
        
        _mag = mag_buf[1] + 1; // take the central row
		_dx = (short*)(dx.data + dx.step*(i-1));
		_dy = (short*)(dy.data + dy.step*(i-1));
        
        magstep1 = (int)(mag_buf[2] - mag_buf[1]);
        magstep2 = (int)(mag_buf[0] - mag_buf[1]);

        if( (stack_top - stack_bottom) + size.width > maxsize )
        {
            int sz = (int)(stack_top - stack_bottom);
            maxsize = MAX( maxsize * 3/2, maxsize + 8 );
            stack.resize(maxsize);
            stack_bottom = &stack[0];
            stack_top = stack_bottom + sz;
        }

        for( j = 0; j < size.width; j++ )
        {
            #define CANNY_SHIFT 15
            #define TG22  (int)(0.4142135623730950488016887242097*(1<<CANNY_SHIFT) + 0.5)

            x = _dx[j];
            y = _dy[j];
            int s = x ^ y;
            int m = _mag[j];
			int ml, mr;
			float subpixOffset;

            x = abs(x);
            y = abs(y);
            if( m > low )
            {
                int tg22x = x * TG22;
                int tg67x = tg22x + ((x + x) << CANNY_SHIFT);

                y <<= CANNY_SHIFT;

                if( y < tg22x ) // vertical edge?
                {
					ml = _mag[j-1]; // left pixel
					mr = _mag[j+1]; // right pixel

                    if( m > ml && m >= mr )
                    {
                        if( m > high && !prev_flag && _map[j-mapstep] != 2 )
                        {
                            CANNY_PUSH( _map + j );
                            prev_flag = 1;
                        }
                        else
                            _map[j] = (uchar)0;

						subpixOffset = ((float)(ml - mr)) / ((float)(2 * (ml - 2*m + mr)));

						edgeXCoords.at<float>(i-1, j) = (float)j + subpixOffset;//  + 0.5f;
						edgeYCoords.at<float>(i-1, j) = (float)(i-1);// + 0.5f;

                        continue;
                    }
                }
                else if( y > tg67x ) // horizontal edge?
                {
					ml = _mag[j+magstep2]; // upper pixel
					mr = _mag[j+magstep1]; // lower pixel

                    if( m > ml && m >= mr )
                    {
                        if( m > high && !prev_flag && _map[j-mapstep] != 2 )
                        {
                            CANNY_PUSH( _map + j );
                            prev_flag = 1;
                        }
                        else
                            _map[j] = (uchar)0;

						subpixOffset = ((float)(ml - mr)) / ((float)(2 * (ml - 2*m + mr)));

						edgeXCoords.at<float>(i-1, j) = (float)j;// + 0.5f;
						edgeYCoords.at<float>(i-1, j) = (float)(i-1) + subpixOffset;// + 0.5f;

                        continue;
                    }
                }
                else // diagonal edge!
                {
                    //s = s < 0 ? -1 : 1;

					if(s < 0)
					{
						mr = _mag[j+magstep2+1]; // upper right
						ml = _mag[j+magstep1-1]; // lower left
					}
					else
					{
						ml = _mag[j+magstep2-1]; // upper left
						mr = _mag[j+magstep1+1]; // lower right
					}

                    if( m > ml && m > mr )
                    {
                        if( m > high && !prev_flag && _map[j-mapstep] != 2 )
                        {
                            CANNY_PUSH( _map + j );
                            prev_flag = 1;
                        }
                        else
                            _map[j] = (uchar)0;

						subpixOffset = ((float)(ml - mr)) / ((float)(2 * (ml - 2*m + mr)));

						edgeXCoords.at<float>(i-1, j) = (float)j + subpixOffset;// + 0.5f;
						edgeYCoords.at<float>(i-1, j) = (float)(i-1) + subpixOffset * (s < 0 ? -1.f : 1.f);//  + 0.5f;
                        continue;
                    }
                }
            }
            prev_flag = 0;
            _map[j] = (uchar)1;
        }

        // scroll the ring buffer
        _mag = mag_buf[0];
        mag_buf[0] = mag_buf[1];
        mag_buf[1] = mag_buf[2];
        mag_buf[2] = _mag;
    }

    // now track the edges (hysteresis thresholding)
    while( stack_top > stack_bottom )
    {
        uchar* m;
        if( (stack_top - stack_bottom) + 8 > maxsize )
        {
            int sz = (int)(stack_top - stack_bottom);
            maxsize = MAX( maxsize * 3/2, maxsize + 8 );
            stack.resize(maxsize);
            stack_bottom = &stack[0];
            stack_top = stack_bottom + sz;
        }

        CANNY_POP(m);
    
        if( !m[-1] )
            CANNY_PUSH( m - 1 );
        if( !m[1] )
            CANNY_PUSH( m + 1 );
        if( !m[-mapstep-1] )
            CANNY_PUSH( m - mapstep - 1 );
        if( !m[-mapstep] )
            CANNY_PUSH( m - mapstep );
        if( !m[-mapstep+1] )
            CANNY_PUSH( m - mapstep + 1 );
        if( !m[mapstep-1] )
            CANNY_PUSH( m + mapstep - 1 );
        if( !m[mapstep] )
            CANNY_PUSH( m + mapstep );
        if( !m[mapstep+1] )
            CANNY_PUSH( m + mapstep + 1 );
    }

    // the final pass, form the final image
    for( i = 0; i < size.height; i++ )
    {
        const uchar* _map = map + mapstep*(i+1) + 1;
        uchar* _dst = dst.data + dst.step*i;
        
        for( j = 0; j < size.width; j++ )
            _dst[j] = (uchar)-(_map[j] >> 1);
    }
}

inline void cannySubpix(	const cv::Mat& image, cv::Mat& edges,
							cv::Mat& edgeXCoords, cv::Mat& edgeYCoords,
							double lowThresh, double highThresh,
							int apertureSize)
{
	cv::Mat dx;
	cv::Mat dy;
	cannySubpix(image, edges, edgeXCoords, edgeYCoords,
				lowThresh, highThresh, apertureSize, dx, dy);
}


template<typename Line2VecType>
void detectLines(	const cv::Mat& image, 
					Line2VecType& lines, 
					LineDetectorParameters& params)
{
	/*static cv::Mat edgeXCoords, edgeYCoords, dx, dy, edgeImage, smoothImage;
	static ::std::vector<Edge<cv::Point>*> edges;
	static ::std::vector<cv::Point2f> edge;
	static ObjectPool<Edge<cv::Point> > edgePool;*/
	
	if(params.preSmoothingGaussianBlurSigma > 0.0f)
		cv::GaussianBlur(image, params.smoothImage, cv::Size(0, 0), params.preSmoothingGaussianBlurSigma, params.preSmoothingGaussianBlurSigma);
	else
		params.smoothImage = image;

	cannySubpix(params.smoothImage, 
				params.edgeImage, 
				params.edgeXCoords, 
				params.edgeYCoords, 
				params.edgeDetectorHighThreshold / 5., 
				params.edgeDetectorHighThreshold, 
				params.edgeFilterKernelSize, 
				params.dx, params.dy);

	//clock_t t = clock();
	params.edges.clear();
	params.edgePool.freeAll();

	findEdges(params.edgeImage, params.edgePool, params.edges);

	lines.clear();
  params.edgeImage = cv::Scalar(0);
			
	for(size_t i = 0; i < params.edges.size(); i++)
	{
		params.edge.resize(params.edges[i]->points.size());
		for(size_t j = 0; j < params.edges[i]->points.size(); j++)
		{
			params.edge[j].x = params.edgeXCoords.at<float>(params.edges[i]->points[j]);
			params.edge[j].y = params.edgeYCoords.at<float>(params.edges[i]->points[j]);

			params.edgeImage.at<unsigned char>(params.edges[i]->points[j]) = params.edges[i]->edgeIndex % 256;
		}
		approximateLines(params.edge, lines, params.douglasPeuckerLineEps, params.minLineLength);
	}	

	//std::cout << "Line Detection Time: " << (clock() - t) << "ms\n";
}


template<typename _Tp>
_Tp computeLineGradient(const Eigen::Matrix<_Tp, 2, 1>& p1, 
						const Eigen::Matrix<_Tp, 2, 1>& p2, 
						const cv::Mat& image)
{
	cv::Point p1i((int)(p1.x() + (_Tp)0.5), (int)(p1.y() + (_Tp)0.5));
	cv::Point p2i((int)(p2.x() + (_Tp)0.5), (int)(p2.y() + (_Tp)0.5));

	const int o = 2; // offset for gradient sampling

	// clip against a rect that is the image size minus a one-pixel border
	if(!clipLine(cv::Rect(o, o, image.cols - 2*o, image.rows - 2*o), p1i, p2i))
		return (_Tp)0.0;

	int xSpan = abs(p1i.x - p2i.x);
	int ySpan = abs(p1i.y - p2i.y);

	_Tp cumulativeGradient = (_Tp)0.0;
	_Tp numGradientPixels;
	// is the line more horizontal or vertical?
	if(xSpan > ySpan)
	{
		int dx = (p1i.x > p2i.x) ? -1 : 1;		
		// we move horizontally along the line
		for(int x = p1i.x;; x += dx)
		{
			int y = (int)((_Tp)(p1i.y * (p2i.x - x) + 
						  p2i.y * (x - p1i.x)) / ((_Tp)(p2i.x - p1i.x)) + (_Tp)0.5);
			// quadratic interpolation to find the maximum gradient
			/*_Tp g1 = (_Tp) gradientImage.at<float>(y-1, x);
			_Tp g2 = (_Tp) gradientImage.at<float>(y, x);
			_Tp g3 = (_Tp) gradientImage.at<float>(y+1, x);

			// compute the maximum gradient and add it to the average
			_Tp g = g2;
			if(g2 > g1 && g2 > g3)
				g = g2 - (g1 - g3) * (g1 - g3) / (((_Tp)8) * (g1 - ((_Tp)2)*g2 + g3));*/

			_Tp g = (_Tp) image.at<unsigned char>(y+o, x) - (_Tp) image.at<unsigned char>(y-o, x);

			cumulativeGradient += g;
			if(x == p2i.x)
				break;
		}

		cumulativeGradient *= (_Tp) dx;
		numGradientPixels = (_Tp)(xSpan + 1);
	}
	else if(ySpan > 0)
	{
		int dy = (p1i.y > p2i.y) ? -1 : 1;
		// we move vertically along the line
		for(int y = p1i.y; ; y += dy)
		{
			int x = (int)((_Tp)(p1i.x * (p2i.y - y) + 
						  p2i.x * (y - p1i.y)) / ((_Tp)(p2i.y - p1i.y)) + (_Tp)0.5);
			// quadratic interpolation to find the maximum gradient
			/*_Tp g1 = (_Tp) gradientImage.at<float>(y, x-1);
			_Tp g2 = (_Tp) gradientImage.at<float>(y, x);
			_Tp g3 = (_Tp) gradientImage.at<float>(y, x+1);

			// compute the maximum gradient and add it to the average
			_Tp g = g2;
			if(g2 > g1 && g2 > g3)
				g = g2 - (g1 - g3) * (g1 - g3) / (((_Tp)8) * (g1 - ((_Tp)2)*g2 + g3));*/

			_Tp g = (_Tp) image.at<unsigned char>(y, x-o) - (_Tp) image.at<unsigned char>(y, x+o);

			cumulativeGradient += g;
			if(y == p2i.y)
				break;
		}

		cumulativeGradient *= (_Tp) dy;
		numGradientPixels = (_Tp)(ySpan + 1);
	}
	else
		return (_Tp)0.0;
	
	return cumulativeGradient / numGradientPixels;
}

template<typename Line3VecType>
void computeLineGradients(	const cv::Mat& leftImage, 
							const cv::Mat& rightImage, 
							Line3VecType& stereoLines)
{
	assert(!leftImage.empty() && !rightImage.empty() && leftImage.size == rightImage.size);

	Eigen::Matrix< typename Line3VecType::value_type::Scalar, 2, 1> p1, p2;
	//_Tp gradientDifference = 0.0f;

	for(size_t i = 0; i < stereoLines.size(); ++i)
	{
		p1 = stereoLines[i].p1.template head<2>();
		p2 = stereoLines[i].p2.template head<2>();
		stereoLines[i].feature = computeLineGradient(p1, p2, leftImage);
		//p1.x() = stereoLines[i].p1[0] - stereoLines[i].p1[2];
		//p2.x() = stereoLines[i].p2[0] - stereoLines[i].p2[2];
		//lineGradients[2*i + 1] = computeLineGradient(p1, p2, rightImage);
		//gradientDifference += fabs(lineGradients[2*i + 0] - lineGradients[2*i + 1]);
	}

	//std::clog << "Gradient Difference: " << gradientDifference << std::endl;
}

template<typename Line2VecType>
void computeLineGradients(	const cv::Mat& leftImage, 
							const cv::Mat& rightImage, 							
							int rightLinesStartIndex,
							Line2VecType& lines)
{
	if(rightLinesStartIndex < 0)
		rightLinesStartIndex = lines.size();

	if(rightLinesStartIndex > 0)
	{
		assert(!leftImage.empty());
		for(size_t i = 0; i < (size_t)rightLinesStartIndex; ++i)
			lines[i].feature = computeLineGradient(lines[i].p1, lines[i].p2, leftImage);
	}
	
	if(rightLinesStartIndex < (int)lines.size())
	{
		assert(!rightImage.empty());
		for(size_t i = (size_t)rightLinesStartIndex; i < lines.size(); ++i)
			lines[i].feature = computeLineGradient(lines[i].p1, lines[i].p2, rightImage);
	}
}

inline
void magnitude(const cv::Mat& xGradient, const cv::Mat& yGradient, cv::Mat& gradientMagnitude)
{
	assert(!xGradient.empty() && !yGradient.empty() && xGradient.size == yGradient.size);
	gradientMagnitude.create(xGradient.size(), CV_32FC1);

	for(int y = 0; y < xGradient.rows; ++y)
		for(int x = 0; x < xGradient.cols; ++x)
		{
			float gx = (float) xGradient.at<short>(y,x);
			float gy = (float) yGradient.at<short>(y,x);
			gradientMagnitude.at<float>(y,x) = (float)cv::sqrt(gx*gx + gy*gy);
		}
}


} // namespace lvl
