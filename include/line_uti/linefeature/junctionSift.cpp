// Line & Junction detection, Matching, and reconstruction
// Author: Ji Zhao
// Date:   12/06/2016
// Email:  zhaoji84@gmail.com
// Shanghai ReadSense Tech., Inc.
// All rights reserved

#include "junctionSift.h"
// This code was adopted from VLFEAT
// http://www.vlfeat.org/

#define VL_INLINE static __inline
#define VL_PI 3.141592653589793
#define VL_EPSILON_F 1.19209290E-07F
#define VL_EPSILON_D 2.220446049250313e-16
#define VL_MIN(x,y) (((x)<(y))?(x):(y))
#define VL_MAX(x,y) (((x)>(y))?(x):(y))
#define EXPN_SZ  256
#define EXPN_MAX 25.0 
double expn_tab[EXPN_SZ + 1];
#define NBO 8
#define NBP 4
typedef float vl_sift_pix;
typedef int                 vl_int32;
typedef long long           vl_int64;
typedef long long unsigned  vl_uint64;
typedef vl_uint64           vl_size; 
typedef vl_int64            vl_index;

VL_INLINE float
vl_mod_2pi_f(float x)
{
	while (x > (float)(2 * VL_PI)) x -= (float)(2 * VL_PI);
	while (x < 0.0F) x += (float)(2 * VL_PI);
	return x;
}

VL_INLINE long int
vl_floor_d(double x)
{
	long int xi = (long int)x;
	if (x >= 0 || (double)xi == x) return xi;
	else return xi - 1;
}

VL_INLINE long int
vl_floor_f(float x)
{
	long int xi = (long int)x;
	if (x >= 0 || (float)xi == x) return xi;
	else return xi - 1;
}

VL_INLINE float
vl_abs_f(float x)
{
	return fabsf(x);
}

VL_INLINE float
vl_fast_resqrt_f(float x)
{
	/* 32-bit version */
	union {
		float x;
		vl_int32  i;
	} u;

	float xhalf = (float) 0.5 * x;

	/* convert floating point value in RAW integer */
	u.x = x;

	/* gives initial guess y0 */
	u.i = 0x5f3759df - (u.i >> 1);
	/*u.i = 0xdf59375f - (u.i>>1);*/

	/* two Newton steps */
	u.x = u.x * ((float) 1.5 - xhalf*u.x*u.x);
	u.x = u.x * ((float) 1.5 - xhalf*u.x*u.x);
	return u.x;
}

VL_INLINE float
vl_fast_sqrt_f(float x)
{
	return (x < 1e-8) ? 0 : x * vl_fast_resqrt_f(x);
}

VL_INLINE float
vl_fast_atan2_f(float y, float x)
{
	float angle, r;
	float const c3 = 0.1821F;
	float const c1 = 0.9675F;
	float abs_y = vl_abs_f(y) + VL_EPSILON_F;

	if (x >= 0) {
		r = (x - abs_y) / (x + abs_y);
		angle = (float)(VL_PI / 4);
	}
	else {
		r = (x + abs_y) / (abs_y - x);
		angle = (float)(3 * VL_PI / 4);
	}
	angle += (c3*r*r - c1) * r;
	return (y < 0) ? -angle : angle;
}

VL_INLINE double
fast_expn(double x)
{
	double a, b, r;
	int i;
	/*assert(0 <= x && x <= EXPN_MAX) ;*/

	if (x > EXPN_MAX) return 0.0;

	x *= EXPN_SZ / EXPN_MAX;
	i = (int)vl_floor_d(x);
	r = x - i;
	a = expn_tab[i];
	b = expn_tab[i + 1];
	return a + r * (b - a);
}

VL_INLINE void
fast_expn_init()
{
	int k;
	for (k = 0; k < EXPN_SZ + 1; ++k) {
		expn_tab[k] = exp(-(double)k * (EXPN_MAX / EXPN_SZ));
	}
}

VL_INLINE vl_sift_pix
normalize_histogram
(vl_sift_pix *begin, vl_sift_pix *end)
{
	vl_sift_pix* iter;
	vl_sift_pix  norm = 0.0;

	for (iter = begin; iter != end; ++iter)
		norm += (*iter) * (*iter);

	norm = vl_fast_sqrt_f(norm) + VL_EPSILON_F;

	for (iter = begin; iter != end; ++iter)
		*iter /= norm;

	return norm;
}

// modified based on imopv.c/vl_imgradient_polar_d()
void vl_imgradient_polar_f(
	float * gradientModulus, float * gradientAngle,
	vl_size gradientHorizontalStride, vl_size gradHeightStride,
	float const* image,
	vl_size imageWidth, vl_size imageHeight, vl_size imageStride)
{
	/* Shortcuts */
	vl_index const xo = 1;
	vl_index const yo = imageStride;
	vl_size const w = imageWidth;
	vl_size const h = imageHeight;

	float const *src, *end;
	float *pgrad_angl, *pgrad_ampl;
	float gx, gy;
	vl_size y;

#define SAVE_BACK                                                    \
	*pgrad_ampl = vl_fast_sqrt_f(gx*gx + gy*gy);                       \
	pgrad_ampl += gradientHorizontalStride;                             \
	*pgrad_angl = vl_mod_2pi_f(vl_fast_atan2_f(gy, gx) + 2 * VL_PI);  \
	pgrad_angl += gradientHorizontalStride;                             \
	++src;                                                              \

	src = image;
	pgrad_angl = gradientAngle;
	pgrad_ampl = gradientModulus;

	/* first pixel of the first row */
	gx = src[+xo] - src[0];
	gy = src[+yo] - src[0];
	SAVE_BACK;

	/* middle pixels of the  first row */
	end = (src - 1) + w - 1;
	while (src < end) {
		gx = 0.5 * (src[+xo] - src[-xo]);
		gy = src[+yo] - src[0];
		SAVE_BACK;
	}

	/* last pixel of the first row */
	gx = src[0] - src[-xo];
	gy = src[+yo] - src[0];
	SAVE_BACK;

	gradientModulus += gradHeightStride;
	pgrad_ampl = gradientModulus;
	gradientAngle += gradHeightStride;
	pgrad_angl = gradientAngle;
	image += imageStride;
	src = image;

	for (y = 1; y < h - 1; ++y) {

		/* first pixel of the middle rows */
		gx = src[+xo] - src[0];
		gy = 0.5 * (src[+yo] - src[-yo]);
		SAVE_BACK;

		/* middle pixels of the middle rows */
		end = (src - 1) + w - 1;
		while (src < end) {
			gx = 0.5 * (src[+xo] - src[-xo]);
			gy = 0.5 * (src[+yo] - src[-yo]);
			SAVE_BACK;
		}

		/* last pixel of the middle row */
		gx = src[0] - src[-xo];
		gy = 0.5 * (src[+yo] - src[-yo]);
		SAVE_BACK;

		gradientModulus += gradHeightStride;
		pgrad_ampl = gradientModulus;
		gradientAngle += gradHeightStride;
		pgrad_angl = gradientAngle;
		image += imageStride;
		src = image;
	}

	/* first pixel of the last row */
	gx = src[+xo] - src[0];
	gy = src[0] - src[-yo];
	SAVE_BACK;

	/* middle pixels of the last row */
	end = (src - 1) + w - 1;
	while (src < end) {
		gx = 0.5 * (src[+xo] - src[-xo]);
		gy = src[0] - src[-yo];
		SAVE_BACK;
	}

	/* last pixel of the last row */
	gx = src[0] - src[-xo];
	gy = src[0] - src[-yo];
	SAVE_BACK;
}

// modified based on sift.c,
// remove input parameter VlSiftFilt
// http://www.vlfeat.org/api/sift.html
void 
vl_sift_calc_raw_descriptor(
	vl_sift_pix const* grad,
	vl_sift_pix *descr,
	int width, int height,
	double x, double y,
	double sigma,
	double angle0)
{
	/*
	The SIFT descriptor is a three dimensional histogram of the
	position and orientation of the gradient.  There are NBP bins for
	each spatial dimension and NBO bins for the orientation dimension,
	for a total of NBP x NBP x NBO bins.

	The support of each spatial bin has an extension of SBP = 3sigma
	pixels, where sigma is the scale of the keypoint.  Thus all the
	bins together have a support SBP x NBP pixels wide. Since
	weighting and interpolation of pixel is used, the support extends
	by another half bin. Therefore, the support is a square window of
	SBP x (NBP + 1) pixels. Finally, since the patch can be
	arbitrarily rotated, we need to consider a window 2W += sqrt(2) x
	SBP x (NBP + 1) pixels wide.
	*/

	double const magnif = 3.0f; //= f->magnif; default is 3.0

	int          w = width;
	int          h = height;

	// 在梯度grad中，按照行优先的顺序依次存放每个位置的梯度幅值和角度，
	// 因此，x的索引增加1，对应的存储位置增加 2
	// y的索引增加1，对应的存储位置增加 2*w
	int const    xo = 2;         /* x-stride */
	int const    yo = 2 * w;     /* y-stride */

	int          xi = (int)(x + 0.5);
	int          yi = (int)(y + 0.5);

	double const st0 = sin(angle0);
	double const ct0 = cos(angle0);
	double const SBP = magnif * sigma + VL_EPSILON_D;
	int    const W = floor
		(sqrt(2.0) * SBP * (NBP + 1) / 2.0 + 0.5);

	// 在描述子的排列中，角度bin第一优先，x第二优先，y第三优先
	// 角度bin的索引加1，位置增加1
	// x的索引加1，位置增加 NBO (8)
	// y的索引加1，位置增加 NBO*NBO (64)
	int const binto = 1;          /* bin theta-stride */
	int const binyo = NBO * NBP;  /* bin y-stride */
	int const binxo = NBO;        /* bin x-stride */

	int bin, dxi, dyi;
	vl_sift_pix const *pt;
	vl_sift_pix       *dpt;

	/* check bounds */
	if (xi    <  0 ||
		xi >= w ||
		yi    <  0 ||
		yi >= h - 1)
		return;

	/* clear descriptor */
	memset(descr, 0, sizeof(vl_sift_pix)* NBO*NBP*NBP);

	/* Center the scale space and the descriptor on the current keypoint.
	* Note that dpt is pointing to the bin of center (SBP/2,SBP/2,0).
	*/
	// keypoint的位置是(xi, yi), pt指向这个点的梯度在grad的位置
	pt = grad + xi*xo + yi*yo; 
	// 构造描述子是一个4*4的格子，中心点算作第2行第2列（起点为0），
	// dpt指向中心点在descr中的位置，此处的值为80
	// dpt意为descriptor pointer
	dpt = descr + (NBP / 2) * binyo + (NBP / 2) * binxo; 

	// dbinx 意为 descriptor bin x-coordinate
	// atd 意为 Descriptor value AT the poision (bin_x, bin_y, bin_t)
#undef atd
#define atd(dbinx,dbiny,dbint) *(dpt + (dbint)*binto + (dbiny)*binyo + (dbinx)*binxo)

	/*
	* Process pixels in the intersection of the image rectangle
	* (1,1)-(M-1,N-1) and the keypoint bounding box.
	*/
	for (dyi = VL_MAX(-W, -yi);
		dyi <= VL_MIN(+W, h - yi - 1); ++dyi) {

		for (dxi = VL_MAX(-W, -xi);
			dxi <= VL_MIN(+W, w - xi - 1); ++dxi) {

			/* retrieve */
			vl_sift_pix mod = *(pt + dxi*xo + dyi*yo + 0);
			vl_sift_pix angle = *(pt + dxi*xo + dyi*yo + 1);
			vl_sift_pix theta = vl_mod_2pi_f(angle - angle0);

			/* fractional displacement */
			vl_sift_pix dx = xi + dxi - x;
			vl_sift_pix dy = yi + dyi - y;

			/* get the displacement normalized w.r.t. the keypoint
			orientation and extension */
			vl_sift_pix nx = (ct0 * dx + st0 * dy) / SBP;
			vl_sift_pix ny = (-st0 * dx + ct0 * dy) / SBP;
			vl_sift_pix nt = NBO * theta / (2 * VL_PI);

			/* Get the Gaussian weight of the sample. The Gaussian window
			* has a standard deviation equal to NBP/2. Note that dx and dy
			* are in the normalized frame, so that -NBP/2 <= dx <=
			* NBP/2. */
			vl_sift_pix const wsigma = 2; // f->windowSize;
			vl_sift_pix win = fast_expn
				((nx*nx + ny*ny) / (2.0 * wsigma * wsigma));

			/* The sample will be distributed in 8 adjacent bins.
			We start from the ``lower-left'' bin. */
			int         binx = (int)vl_floor_f(nx - 0.5);
			int         biny = (int)vl_floor_f(ny - 0.5);
			int         bint = (int)vl_floor_f(nt);
			vl_sift_pix rbinx = nx - (binx + 0.5);
			vl_sift_pix rbiny = ny - (biny + 0.5);
			vl_sift_pix rbint = nt - bint;
			int         dbinx;
			int         dbiny;
			int         dbint;

			/* Distribute the current sample into the 8 adjacent bins*/
			for (dbinx = 0; dbinx < 2; ++dbinx) {
				for (dbiny = 0; dbiny < 2; ++dbiny) {
					for (dbint = 0; dbint < 2; ++dbint) {

						if (binx + dbinx >= -(NBP / 2) &&
							binx + dbinx <    (NBP / 2) &&
							biny + dbiny >= -(NBP / 2) &&
							biny + dbiny <    (NBP / 2)) {
							vl_sift_pix weight = win
								* mod
								* vl_abs_f(1 - dbinx - rbinx)
								* vl_abs_f(1 - dbiny - rbiny)
								* vl_abs_f(1 - dbint - rbint);

							atd(binx + dbinx, biny + dbiny, (bint + dbint) % NBO) += weight;
						}
					}
				}
			}
		}
	}

	/* Standard SIFT descriptors are normalized, truncated and normalized again */
	if (1) {

		/* normalize L2 norm */
		vl_sift_pix norm = normalize_histogram(descr, descr + NBO*NBP*NBP);

		/*
		Set the descriptor to zero if it is lower than our
		norm_threshold.  We divide by the number of samples in the
		descriptor region because the Gaussian window used in the
		calculation of the descriptor is not normalized.
		*/
		int numSamples =
			(VL_MIN(W, w - xi - 1) - VL_MAX(-W, -xi) + 1) *
			(VL_MIN(W, h - yi - 1) - VL_MAX(-W, -yi) + 1);

		float norm_thresh = 0.0f;
//		if (f->norm_thresh && norm < f->norm_thresh * numSamples) {
		if (norm_thresh && norm < norm_thresh * numSamples) {
			for (bin = 0; bin < NBO*NBP*NBP; ++bin)
				descr[bin] = 0;
		}
		else {
			/* truncate at 0.2. */
			for (bin = 0; bin < NBO*NBP*NBP; ++bin) {
				if (descr[bin] > 0.2) descr[bin] = 0.2;
			}

			/* normalize again. */
			normalize_histogram(descr, descr + NBO*NBP*NBP);
		}
	}
}

void siftExtraction(Mat img, vector<junctKeyPt> vecKeyPts, float* descr, float r)
{
	// img should be gray scale image

	fast_expn_init();

	int height = img.rows;
	int width = img.cols;

	float* image = new float[height*width];
	float* grad = new float[height*width*2];
	int cnt = 0;
	for (int i = 0; i < height; i++) {
		unsigned char *p = img.ptr<unsigned char>(i);
		for (int j = 0; j < width; j++) {
			image[cnt] = static_cast<float>(p[j]);
			cnt++;
		}
	}

	// grad是一个2*width*height的连续存储空间
	// 每个像素的梯度幅度和角度占两个连续的float类型，按照行优先的顺序存储
	// 参数1：幅度的第一个存放位置
	// 参数2：角度的第一个存放位置
	// 参数3：水平方向每加1，指针在输出块中移动的位置，通常是2
	// 参数4：垂直方向每加1，指针在输出块要移动的位置，通常是宽度的2倍
	// 参数5：图像
	// 参数6：图像宽度
	// 参数7：图像高度
	// 参数8：图像的列每加1，指针在图像中移动的位置，通常是图像的宽度
	vl_imgradient_polar_f(grad, grad + 1, 2, 2*width,
		image, width, height, width);

#ifdef SIFT_DEBUG
	Mat imgGrad = Mat(height, width, CV_8UC1, cv::Scalar(0));
	cnt = 0;
	for (int i = 0; i < height; i++) {
		uchar *p = imgGrad.ptr<uchar>(i);
		for (int j = 0; j < width; j++) {
			p[j] = static_cast<uchar>(grad[cnt]);
			cnt+=2;
		}
	}
	imshow("amplitude of image gradient", imgGrad);
#endif

	double sigma = r / 3.0f;
	double angle0;
	double x, y;
	int p = 0;
	for (vector<junctKeyPt>::const_iterator it = vecKeyPts.begin();
		it != vecKeyPts.end(); ++it)
	{
		junctKeyPt Pt = *it;
		x = static_cast<double>(Pt.x);
		y = static_cast<double>(Pt.y);
		angle0 = static_cast<double>(Pt.mid_ang1_ang2);
		vl_sift_calc_raw_descriptor(grad, descr+p, width, height, x, y, sigma, angle0);
		p += DIM_DESCRIPTOR;
	}

#ifdef SIFT_DEBUG
	Mat Im = img.clone();
	for (vector<junctKeyPt>::const_iterator it = vecKeyPts.begin();
		it != vecKeyPts.end(); ++it)
	{
		junctKeyPt Pt = *it;
		CvPoint pt1(Pt.x + 0.5f, Pt.y + 0.5f);
		circle(Im, pt1, 10, CV_RGB(255, 255, 255), 2, 8, 0);
	}
	cv::imshow("keypints", Im);
	cv::waitKey(1);
#endif

	delete[] image;
	delete[] grad;
}

