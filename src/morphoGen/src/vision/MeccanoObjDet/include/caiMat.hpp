/*
 * caiMat.hpp
 *
 *  Created on: Nov 16, 2012
 *      Author: caihongp
 */

#ifndef CAIMAT_HPP_
#define CAIMAT_HPP_
#undef __GXX_EXPERIMENTAL_CXX0X__

#include <opencv/highgui.h>

class caiMat:public Mat
{
public:
	template<class T> void convert2MatlabMat(T *pout)
	{
		int ntmp;
		for (int j=0;j<cols;j++)
		{
			ntmp = j*rows;
			for (int i=0;i<rows;i++)
				pout[ntmp+i] = at<T>(i,j);
		}
		return;
	}
	caiMat(int  _rows, int _cols, int _type)
	{
		create(_rows, _cols, _type);
	}

	template<class T>  void fromMat(int _rows, int _cols, int _type, T* _data, size_t _step=AUTO_STEP)
		{
			flags = MAGIC_VAL + (_type & TYPE_MASK);
			dims = 2;
			rows = _rows;
			cols = _cols;
		    refcount = 0;

		    data = (uchar*)_data;
			datastart = (uchar*)data;
			dataend = 0;
		    datalimit = 0;
		    allocator = 0;
		    size = &rows;


		    size_t esz = CV_ELEM_SIZE(_type), minstep = cols*esz;
		    if( _step == AUTO_STEP )
		    {
		        _step = minstep;
		        flags |= CONTINUOUS_FLAG;
		    }
		    else
		    {
		        if( rows == 1 ) _step = minstep;
		        CV_DbgAssert( _step >= minstep );
		        flags |= _step == minstep ? CONTINUOUS_FLAG : 0;
		    }
		    step[0] = _step; step[1] = esz;
		    datalimit = datastart + _step*rows;
		    dataend = datalimit - _step + minstep;
		    return;
		}

	template<class T>  void fromMatlabMat(int _rows, int _cols, int _type, T* _data, size_t _step=AUTO_STEP)
	{
		flags = MAGIC_VAL + (_type & TYPE_MASK);
		dims = 2;
		rows = _rows;
		cols = _cols;
	    refcount = 0;

	    T* dataDst = (T*) data;

		int ntmp;
		for (int i=0;i<rows;i++)
		{
			ntmp = i*cols;
			for (int j=0;j<cols;j++)
				dataDst[ntmp+j] = _data[j*rows+i];
		}

		datastart = (uchar*)data;
		dataend = 0;
	    datalimit = 0;
	    allocator = 0;
	    size = &rows;


	    size_t esz = CV_ELEM_SIZE(_type), minstep = cols*esz;
	    if( _step == AUTO_STEP )
	    {
	        _step = minstep;
	        flags |= CONTINUOUS_FLAG;
	    }
	    else
	    {
	        if( rows == 1 ) _step = minstep;
	        CV_DbgAssert( _step >= minstep );
	        flags |= _step == minstep ? CONTINUOUS_FLAG : 0;
	    }
	    step[0] = _step; step[1] = esz;
	    datalimit = datastart + _step*rows;
	    dataend = datalimit - _step + minstep;
	    return;
	}
	inline caiMat& operator = (const Mat& m)
	{
	    if( this != &m )
	    {
	        if( m.refcount )
	            CV_XADD(m.refcount, 1);
	        release();
	        flags = m.flags;
	        if( dims <= 2 && m.dims <= 2 )
	        {
	            dims = m.dims;
	            rows = m.rows;
	            cols = m.cols;
	            step[0] = m.step[0];
	            step[1] = m.step[1];
	        }
	        else
	            copySize(m);
	        data = m.data;
	        datastart = m.datastart;
	        dataend = m.dataend;
	        datalimit = m.datalimit;
	        refcount = m.refcount;
	        allocator = m.allocator;
	    }
	    return *this;
	}
};


#endif /* CAIMAT_HPP_ */
