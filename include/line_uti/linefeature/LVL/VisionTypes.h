/// \author Jonas Witt
/// \file
/// \brief This file provides definitions of basic vision types.
///
/// Detailed description...


#pragma once

#include <Eigen/Geometry>
#include <string>
#include <exception>
#include <iostream>
#include <vector>

namespace lvl {

class MsgEx: public std::exception
{
  std::string msg;
public:
  virtual const char* what() const throw(){ return msg.c_str(); }
  MsgEx(std::string const& msg) throw() :msg(msg){ std::cerr <<"MsgEx::what(): " << msg << std::endl; }
  virtual ~MsgEx() throw() {};
};


 /// \brief A type to store a chain of points that make up an edge along with connections to adjacent edges.
template <typename _Tp, int reserveSize = 40>
struct Edge
{
	Edge()
	{
		//startPointConnections = NULL;
		startPointConnections.reserve(1);
		endPointConnections.reserve(2);
		points.reserve(reserveSize);
	}

	void clear()
	{
		//startPointConnections = NULL;
		startPointConnections.clear();
		endPointConnections.clear();
		points.clear();
	}

	::std::vector<Edge*> startPointConnections;
	::std::vector<Edge*> endPointConnections;
	::std::vector<_Tp> points;
	unsigned int edgeIndex;
	//float length;
};


/// \brief A struct to store a neighbour pixel along with a confidence
struct NeighbourEdgePixel
{
	int x, y;
	unsigned char confidence;
	char direction;
};

/// \brief This is an empty dummy implemtation of a line feature
struct EmptyLineFeature
{
	/// \brief Allow assignments of anything
	template<typename _AnyType>
	EmptyLineFeature& operator=(_AnyType doesNotMatter) 
	{
		return *this;
	}
};

struct GradientLineFeature
{
	GradientLineFeature() { gradient = 0.0f;}
	GradientLineFeature(float newGradient) { gradient = newGradient; }

	/// \brief The compare function calculates how well two line features match. 1.0 means "100% perfect match", 0.0 means "does not match at all"
	/*float compare(const GradientLineFeature& other, bool linesPointInSameDirection = true) 
	{
		float  gradientDifference = fabs(fabs(other.gradient) - fabs(gradient));
		return (1.0f - std::min(gradientDifference / 30.0f, 0.9f))
		if(linesPointInSameDirection)
			return gradient * other.gradient;
		else
			return -gradient * other.gradient;
	}*/

	GradientLineFeature& operator=(float newGradient) 
	{
		gradient = newGradient;
		return *this;
	}

	float gradient; ///< The average line gradient
};



/// \brief A generic minimal type to store a line
template<typename _Tp, int dim, typename _LineFeature = EmptyLineFeature>
struct BasicLine
{	

	template<typename _Tp2, typename _LineFeature2>
	operator BasicLine<_Tp2, dim, _LineFeature2>() const
	{
		BasicLine<_Tp2, dim, _LineFeature2> result;
		result.p1 = p1.template cast<_Tp2>();//see http://forum.kde.org/viewtopic.php?f=74&t=62606
		result.p2 = p2.template cast<_Tp2>();
		result.feature = feature;
		return result;
	}
	
	typedef _Tp Scalar;
	typedef _LineFeature LineFeature;
	typedef Eigen::Matrix<_Tp, dim, 1> Vector; ///< Line vector type.
	Vector p1;	///< Line point 1.
	Vector p2;	///< Line point 2.
	_LineFeature feature; ///< Line feature (if _LineFeature is not explicitly specified during template instatiation, this is an empty dummy implementation)

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF_VECTORIZABLE_FIXED_SIZE(_Tp, dim)
};

typedef BasicLine<float, 2> BasicLine2f;
typedef BasicLine<double, 2> BasicLine2d;
typedef BasicLine<float, 3> BasicLine3f;
typedef BasicLine<double, 3> BasicLine3d;


/// \brief A generic non-minimal type to store a line along with _LineFeature structure
template<typename _Tp, int dim, typename _LineFeature = EmptyLineFeature>
struct Line : BasicLine<_Tp, dim, _LineFeature>
{
	typename BasicLine<_Tp, dim, _LineFeature>::Vector dir;	///< Line direction.
	_Tp length;	///< Line length.	

	/*template<typename _Tp2>
	Line<_Tp, dim>& operator=( const Line<_Tp2, dim>& other )
	{
		BasicLine<_Tp>::operator=(other);
		dir = other.dir.cast<_Tp>();
		length = static_cast<_Tp>(other.length);
		return *this;
	}*/

	template<typename _Tp2, typename _LineFeature2>
	operator Line<_Tp2, dim, _LineFeature2>() const
	{
		Line<_Tp2, dim, _LineFeature2> result;
		result.p1 = this->p1.template cast<_Tp2>();
		result.p2 = this->p2.template cast<_Tp2>();
		result.dir = this->dir.template cast<_Tp2>();
		result.length = static_cast<_Tp2>(this->length);
		result.feature = this->feature;
		return result;
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF_VECTORIZABLE_FIXED_SIZE(_Tp, dim)
};

typedef Line<float, 2> Line2f;
typedef Line<double, 2> Line2d;
typedef Line<float, 3> Line3f;
typedef Line<double, 3> Line3d;


//template<typename _Tp> 
//struct Line3 : public Line<_Tp, 3>{}; // template alias workaround
//template<typename _Tp>
//using Line3 = Line<_Tp, 3>;

//template<typename _Tp> 
//struct Line2 : public Line<_Tp, 2>{}; // template alias workaround
//template<typename _Tp>
//using Line2 = Line<_Tp, 2>;



/// \brief A type to store a two-dimensional line projection.
/*template<typename _Tp, typename _LineFeature = EmptyLineFeature>
struct LineProjection : Line<_Tp, 2, _LineFeature>
{
	_Tp lineDistance;


	template<typename _Tp2, typename _LineFeature2>
	operator LineProjection<_Tp2, _LineFeature2>() const
	{
		LineProjection<_Tp2, _LineFeature2> result;
		result.p1 = this->p1.template cast<_Tp2>();
		result.p2 = this->p2.template cast<_Tp2>();
		result.dir = this->dir.template cast<_Tp2>();
		result.length = static_cast<_Tp2>(this->length);
		result.lineDistance = static_cast<_Tp2>(lineDistance);
		result.feature = this->feature;
		return result;
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef LineProjection<float> LineProjectionf;

typedef LineProjection<double> LineProjectiond;*/


template<typename _Tp, typename _LineFeature = EmptyLineFeature>
struct StereoLine
{
	typedef _Tp Scalar;
	typedef Eigen::Matrix<_Tp, 2, 1> Vector;
  Line<_Tp, 2, _LineFeature>& operator[](size_t i) { return lines[i]; }
  const Line<_Tp, 2, _LineFeature>& operator[](size_t i) const { return lines[i]; }

	/*template<_Tp2>
	StereoLineProjection<_Tp>& operator=( const StereoLineProjection<_Tp2>& other )
	{
		projection[0] = other.projection[0];
		projection[1] = other.projection[1];
		return *this;
	}*/

	template<typename _Tp2, typename _LineFeature2>
  operator StereoLine<_Tp2, _LineFeature2>() const
	{
    StereoLine<_Tp2, _LineFeature2> result;
    result.lines[0] = lines[0];
    result.lines[1] = lines[1];
		return result;
	}

  Line<_Tp, 2, _LineFeature> lines[2];

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef StereoLine<float> StereoLinef;

typedef StereoLine<double> StereoLined;


/// \brief A type to store a monocular or stereo camera frustum.
template<typename _Tp>
struct Frustum
{
  Frustum() {}

  Frustum(_Tp pixelFocalLength, _Tp imagePixelWidth, _Tp imagePixelHeight, _Tp zNearDistance, _Tp zFarDistance, _Tp baseline = (_Tp)0.0, bool maximizedFrustum = true)
  {
    init(pixelFocalLength, imagePixelWidth, imagePixelHeight, zNearDistance, zFarDistance, baseline, maximizedFrustum);
  }

	typedef Eigen::Hyperplane<_Tp, 3> PlaneType;

	PlaneType planes[6];

	PlaneType& up()		{ return planes[0]; }
	PlaneType& down()	{ return planes[1]; }
	PlaneType& left()	{ return planes[2]; }
	PlaneType& right()	{ return planes[3]; }
	PlaneType& zNear()	{ return planes[4]; }
	PlaneType& zFar()	{ return planes[5]; }

	const PlaneType& up()		const { return planes[0]; }
	const PlaneType& down()	const { return planes[1]; }
	const PlaneType& left()	const { return planes[2]; }
	const PlaneType& right()	const { return planes[3]; }
	const PlaneType& zNear()	const { return planes[4]; }
	const PlaneType& zFar()	const { return planes[5]; }
	
	/// \brief Initialize the frustum with the intrinsic camera parameters. 
	///
	/// The parameters baseline and maximizedFrustum are only relevant for stereo cameras. 
	/// If the union of the left and right camera frustums is desired, maximizedFrustum shall be true.
	/// For the intersection of both, maximizedFrustum shall be false.
	const Frustum& init(_Tp pixelFocalLength, _Tp imagePixelWidth, _Tp imagePixelHeight, _Tp zNearDistance, _Tp zFarDistance, _Tp baseline = (_Tp)0.0, bool maximizedFrustum = true)
	{
		_Tp tanHorizontal = imagePixelWidth  / 2 / pixelFocalLength;
		_Tp tanVertical   = imagePixelHeight / 2 / pixelFocalLength;

		_Tp leftOffset = maximizedFrustum ? (_Tp) 0.0 : baseline;
		_Tp rightOffset = maximizedFrustum ? -baseline : (_Tp) 0.0;

		up()	= PlaneType(typename PlaneType::VectorType((_Tp)  0.0, (_Tp) -1.0, (_Tp) -tanVertical).normalized(), (_Tp) 0.0);
		down()	= PlaneType(typename PlaneType::VectorType((_Tp)  0.0, (_Tp)  1.0, (_Tp) -tanVertical).normalized(), (_Tp) 0.0);
		left()	= PlaneType(typename PlaneType::VectorType((_Tp) -1.0, (_Tp)  0.0, (_Tp) -tanHorizontal).normalized(), leftOffset);
		right()	= PlaneType(typename PlaneType::VectorType((_Tp)  1.0, (_Tp)  0.0, (_Tp) -tanHorizontal).normalized(), rightOffset);
		zNear() = PlaneType(typename PlaneType::VectorType((_Tp)  0.0, (_Tp)  0.0, (_Tp) -1.0), zNearDistance);
		zFar()  = PlaneType(typename PlaneType::VectorType((_Tp)  0.0, (_Tp)  0.0, (_Tp)  1.0), -zFarDistance);

		return *this;
	}

	Frustum transformToWorldCoordinates(const Eigen::Transform<_Tp, 3, Eigen::Isometry>& camToWorldTransform) const
	{
		Frustum result;
		for(int i=0; i < 6; i++)
		{
		    //result.planes[i].transform(camToWorldTransform, Eigen::Isometry);
		    result.planes[i].normal() = camToWorldTransform.rotation() * planes[i].normal();
		    result.planes[i].offset() = planes[i].offset() - result.planes[i].normal().dot(camToWorldTransform.translation());
			//result.planes[i] = PlaneType(	camToWorldTransform.rotation() * planes[i].normal(),
			//								-camToWorldTransform.rotation() * (planes[i].normal() * planes[i].offset()) + camToWorldTransform.translation());
		}

		return result;
	}

	bool contains(	const Eigen::Matrix<_Tp, 3, 1>& point) const
	{
		for(int i=0; i < 6; i++)
		{
			if(planes[i].signedDistance(point) > (_Tp) 0.0)
				return false;
		}
		return true;
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef Frustum<float> Frustumf;
typedef Frustum<double> Frustumd;


/// \brief A type to store a two-dimensional line projection of a stereo camera (left and right projection).
/*template<typename _Tp>
struct StereoLineProjection
{
	typedef Eigen::Matrix<_Tp, 2, 1> Vector; ///< Vector type.

	Vector u1Left;
	Vector u2Left;
	Vector dirLeft;
	_Tp lengthLeft;
	_Tp lineDistanceLeft;
	
	Vector u1Right;
	Vector u2Right;
	Vector dirRight;
	_Tp lengthRight;
	_Tp lineDistanceRight;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};*/



/// \brief A feature that is made up of two three-dimensional lines
template<typename _Tp, typename _LineFeature>
struct LinePairFeature
{
  typedef _Tp Scalar;
  typedef _LineFeature LineFeature;
  typedef Eigen::Matrix<_Tp, 3, 1> Vector3;
  typedef Line<_Tp, 3, LineFeature> Line3;
	// Line 1
  Line3 line1;
  Vector3 p1MinDist;// point on line 1 with minimum distance to line 2
	unsigned int line1Index;

	// Line 2
  Line3 line2;
  Vector3 p2MinDist; // point on line 2 with minimum distance to line 1
	unsigned int line2Index;

	float cosDiffAngle;
	float minDistance;

	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/// \brief A type to hold the line indices of a one-to-many matching along with line matching specific overlap
struct MultiLineMatch
{
	MultiLineMatch()
	{
		matchingLineIndices.reserve(20);
		matchingLineOverlap.reserve(20);
	}

	MultiLineMatch(const MultiLineMatch& other)
	{
		matchingLineIndices.reserve(20);
		matchingLineOverlap.reserve(20);
		matchingLineIndices = other.matchingLineIndices;
		matchingLineOverlap = other.matchingLineOverlap;
		indexOffset = other.indexOffset;
	}

	void clear()
	{
		matchingLineIndices.clear();
		matchingLineOverlap.clear();
	}

	::std::vector<int> matchingLineIndices; ///< The indices of all matching lines are stored in this vector.
	::std::vector<float> matchingLineOverlap; ///< The overlap lengths of all matching lines are stored in this vector.
	int indexOffset;  ///< An index offset value that is relevant for the error vector and jacobian matrix (for absolute addressing during concurrent computation).
};


/// \brief A type to hold the line indices of a one-to-one matching
typedef ::std::pair<int, int> SimpleLineMatch;


/// \brief A structure to hold the information of one camera frame
template<typename _Tp>
struct Frame
{
	Eigen::Matrix<_Tp, 3, 1> cameraTranslation;
	Eigen::Matrix<_Tp, 3, 3> cameraRotation;

	::std::vector<Line<_Tp, 2> > lines;
	::std::vector<Line<_Tp, 3> > stereoLines;
	::std::vector<MultiLineMatch> matches;
	int rightLinesStartIndex;
};

typedef Frame<float> Framef;
typedef Frame<double> Framed;


/// \brief A structure to (stereo-) camera parameters
template<typename _Tp>
struct CameraParameters
{
	CameraParameters()
	{
		this->focalLength = (_Tp) 1.0;
		this->principalPointX = (_Tp) 0.5;
		this->principalPointY = (_Tp) 0.5;
		this->imageWidth = (_Tp) 1.0;
		this->imageHeight = (_Tp) 1.0;
		this->baseline = (_Tp) 1.0;
	}

	CameraParameters(_Tp focalLength, _Tp principalPointX, _Tp principalPointY, _Tp imageWidth, _Tp imageHeight, _Tp baseline)
	{
		this->focalLength = focalLength;
		this->principalPointX = principalPointX;
		this->principalPointY = principalPointY;
		this->imageWidth = imageWidth;
		this->imageHeight = imageHeight;
		this->baseline = baseline;
	}

	_Tp focalLength;
	_Tp principalPointX;
	_Tp principalPointY;
	_Tp imageWidth;
	_Tp imageHeight;
	_Tp baseline;
};

typedef CameraParameters<float> CameraParametersf;
typedef CameraParameters<double> CameraParametersd;

/// \brief A structure to hold the information of a line feature
/*template<typename _Tp>
struct LineFeature
{
	Line<_Tp, 3> lineEstimate;
	::std::map<int, MultiLineMatch> matchingImageLines; ///< the MultiLineMatch references all matching image lines in the frame that corresponds to the index
	int liveCount;
	_Tp cumulativeReprojectionLength;
	_Tp cumulativeMatchedLength;
};

typedef LineFeature<float> LineFeaturef;
typedef LineFeature<double> LineFeatured;*/


/// \brief This dummy implemtation of a compare function always returns 1.0 which means "100% perfect match"
template<typename _Tp, int lineDim>
float compareLineFeatures(const Line<_Tp, lineDim, EmptyLineFeature>& imageLine,
              const Line<_Tp, lineDim, EmptyLineFeature>& worldLineProjection)
{
	return 1.0f;
}

/// \brief This dummy implemtation of a compare function always returns 1.0 which means "100% perfect match"
template<typename _Tp, int lineDim>
float compareLineFeatures(const Line<_Tp, lineDim, EmptyLineFeature>& imageLine, 
              const StereoLine<_Tp, EmptyLineFeature>& stereoWorldLineProjection)
{
	return 1.0f;
}

/// \brief The compare function calculates how well two line features match. 1.0 means "100% perfect match", 0.0 means "does not match at all"
template<typename _Tp, int lineDim>
float compareLineFeatures(const Line<_Tp, lineDim, GradientLineFeature>& imageLine, 
              const StereoLine<_Tp, GradientLineFeature>& stereoWorldLineProjection)
{
	float lineDotProduct = (float) imageLine.dir.template head<2>().dot(stereoWorldLineProjection[0].dir);

	float ig1 = imageLine.feature.gradient;
	//float ig2 =imageLine.feature.gradient;
	float wg1 = stereoWorldLineProjection[0].feature.gradient;
	//float wg2 = stereoWorldLineProjection[1].feature.gradient;

	if(lineDotProduct * ig1 * wg1 < 0.0f)
		return 0.0f;

	float  gradientDifference = fabs(fabs(ig1) - fabs(wg1));
	
	return (1.0f - std::min(gradientDifference / 30.0f, 0.9f));
}

} // namespace lvl

// redefine ::std::vectors with the correct aligned allocators for eigen
#include <Eigen/StdVector>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(lvl::BasicLine2f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(lvl::BasicLine2d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(lvl::Line2f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(lvl::Line2d)
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(lvl::LineProjectionf)
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(lvl::LineProjectiond)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(lvl::StereoLinef)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(lvl::StereoLined)

