/*
	This code is take from the OpenCV code. These are internal functions that are usefule for going between eigin and cv matrices. 
	I'm not sure why this code is not part of the API, but I need to use it, so here it is.
*/
/* 
License Agreement
For Open Source Computer Vision Library
(3-clause BSD License)
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
Neither the names of the copyright holders nor the names of the contributors may be used to endorse or promote products derived from this software without specific prior written permission.
This software is provided by the copyright holders and contributors “as is” and any express or implied warranties, including, but not limited to, the implied warranties of merchantability and fitness for a particular purpose are disclaimed. In no event shall copyright holders or contributors be liable for any direct, indirect, incidental, special, exemplary, or consequential damages (including, but not limited to, procurement of substitute goods or services; loss of use, data, or profits; or business interruption) however caused and on any theory of liability, whether in contract, strict liability, or tort (including negligence or otherwise) arising in any way out of the use of this software, even if advised of the possibility of such damage.*/

#ifndef OPENSOURCE_EIGEN_CV_CONV_H
#define OPENSOURCE_EIGEN_CV_CONV_H

#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <stdio.h>
#include <assert.h>

namespace opensource{
	template<typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>
	void eigen2cv(const Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols>& src, cv::Mat& dst)
	{

		if (!(src.Flags & Eigen::RowMajorBit))
		{
			cv::Mat _src(src.cols(), src.rows(), cv::DataType<_Tp>::type,
				(void*)src.data(), src.stride() * sizeof(_Tp));
			cv::transpose(_src, dst);
		}
		else
		{
			cv::Mat _src(src.rows(), src.cols(), cv::DataType<_Tp>::type,
				(void*)src.data(), src.stride() * sizeof(_Tp));
			_src.copyTo(dst);
		}
	}
	
	template<typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>
	void cv2eigen( const cv::Mat& src,
				   Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols>& dst )
	{
		CV_DbgAssert(src.rows == _rows && src.cols == _cols);
        dst.resize(src.rows, src.cols);

		if( !(dst.Flags & Eigen::RowMajorBit) )
		{
			cv::Mat _dst(src.cols, src.rows, cv::DataType<_Tp>::type,
					 dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
			if( src.type() == _dst.type() )
				transpose(src, _dst);
			else if( src.cols == src.rows )
			{
				src.convertTo(_dst, _dst.type());
				transpose(_dst, _dst);
			}
			else
				cv::Mat(src.t()).convertTo(_dst, _dst.type());
			CV_DbgAssert(_dst.data == (uchar*)dst.data());
		}
		else
		{
			cv::Mat _dst(src.rows, src.cols, cv::DataType<_Tp>::type,
					 dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
			src.convertTo(_dst, _dst.type());
			CV_DbgAssert(_dst.data == (uchar*)dst.data());
		}
	}

    template<typename _Tp, int _rows, int _cols, int _options >
	void cv2eigen( const cv::Mat& src,
				   Eigen::Matrix<_Tp, Eigen::Dynamic, Eigen::Dynamic>& dst )
	{
		dst.resize(src.rows, src.cols);

		if( !(dst.Flags & Eigen::RowMajorBit) )
		{
			cv::Mat _dst(src.cols, src.rows, cv::DataType<_Tp>::type,
				 dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
			if( src.type() == _dst.type() )
				transpose(src, _dst);
			else if( src.cols == src.rows )
			{
				src.convertTo(_dst, _dst.type());
				transpose(_dst, _dst);
			}
			else
				cv::Mat(src.t()).convertTo(_dst, _dst.type());
			CV_DbgAssert(_dst.data == (uchar*)dst.data());
		}
		else
		{
			cv::Mat _dst(src.rows, src.cols, cv::DataType<_Tp>::type,
					 dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
			src.convertTo(_dst, _dst.type());
			CV_DbgAssert(_dst.data == (uchar*)dst.data());
		}
	}


	template<typename _Tp>
	void cv2eigen( const cv::Mat& src,
				   Eigen::Matrix<_Tp, Eigen::Dynamic, 1>& dst )
	{
		CV_Assert(src.cols == 1);
		dst.resize(src.rows);

		if( !(dst.Flags & Eigen::RowMajorBit) )
		{
			cv::Mat _dst(src.cols, src.rows, cv::DataType<_Tp>::type,
					 dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
			if( src.type() == _dst.type() )
				transpose(src, _dst);
			else
				cv::Mat(src.t()).convertTo(_dst, _dst.type());
			CV_DbgAssert(_dst.data == (uchar*)dst.data());
		}
		else
		{
			cv::Mat _dst(src.rows, src.cols, cv::DataType<_Tp>::type,
					 dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
			src.convertTo(_dst, _dst.type());
			CV_DbgAssert(_dst.data == (uchar*)dst.data());
		}
	}


	template<typename _Tp>
	void cv2eigen( const cv::Mat& src,
				   Eigen::Matrix<_Tp, 1, Eigen::Dynamic>& dst )
	{
		CV_Assert(src.rows == 1);
		dst.resize(src.cols);
		assert( 0 == 1  );
		if( !(dst.Flags & Eigen::RowMajorBit) )
		{
			cv::Mat _dst(src.cols, src.rows, cv::DataType<_Tp>::type,
					 dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
			if( src.type() == _dst.type() )
				transpose(src, _dst);
			else
				cv::Mat(src.t()).convertTo(_dst, _dst.type());
			CV_DbgAssert(_dst.data == (uchar*)dst.data());
		}
		else
		{
			cv::Mat _dst(src.rows, src.cols, cv::DataType<_Tp>::type,
					 dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
			src.convertTo(_dst, _dst.type());
			CV_DbgAssert(_dst.data == (uchar*)dst.data());
		}
	}
	
} //namespace opensource
#endif //OPENSOURCE_EIGEN_CV_CONV_H
