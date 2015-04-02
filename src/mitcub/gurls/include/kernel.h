/*
 * The GURLS Package in C++
 *
 * Copyright (C) 2011, IIT@MIT Lab
 * All rights reserved.
 *
 * author:  M. Santoro
 * email:   msantoro@mit.edu
 * website: http://cbcl.mit.edu/IIT@MIT/IIT@MIT.html
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name(s) of the copyright holders nor the names
 *       of its contributors or of the Massacusetts Institute of
 *       Technology or of the Italian Institute of Technology may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _GURLS_KERNEL_H_
#define _GURLS_KERNEL_H_

//#include <cstdio>
//#include <algorithm>
#include <cstdlib>
//#include <string>
//#include <vector>

//#include "gvec.h"
//#include "gmat2d.h"

#include <stdexcept>

#include "optlist.h"

namespace gurls {

//class Kernel
//{
//private:
//	string name;
//	vector<string> parNames;
//	vector<double> parValues;

//public:
//	Kernel(string n = "") : name(n) { }
//	template <typename T>
//	void evaluate( const gMat2D< T >& X1, const gMat2D< T >& X2 , gMat2D< T > Z) {
//		/* Empty method. The method should have been pure virtual	but templates may not be virtual. */
//	} ;

//	virtual string getName() {
//		return this->name;
//	}

//	virtual void setName(string n) {
//		this->name = n;
//	}

//};

template<typename T>
class KernelLinear;

template<typename T>
class KernelRBF;

template<typename T>
class KernelChisquared;

/**
 * \ingroup Kernels
 * \brief Kernel is the class that computes the kernel matrix
 */

template<typename T>
class Kernel
{
public:
    /**
     * Computes (or loads) the kernel matrix for the data matrix passed in the X matrix
     * \param X input data matrix
     * \param Y labels matrix
     * \param opt options with the different required fields based on the sub-class
     *
     * \return adds the field kernel to opt
     */
    virtual void execute(const gMat2D<T>& X, const gMat2D<T>& Y, GurlsOptionsList& opt) = 0;

    /**
     * \ingroup Exceptions
     *
     * \brief BadKernelCreation is thrown when \ref factory tries to generate an unknown kernel
     */
    class BadKernelCreation : public std::logic_error
    {
    public:
		/**
		 * Exception constructor.
		 */
        BadKernelCreation(std::string type)
            : logic_error("Cannot create type " + type) {}
    };

	/**
	 * Factory function returning a pointer to the newly created object.
	 *
	 * \warning The returned pointer is a plain, un-managed pointer. The calling
	 * function is responsible of deallocating the object.
	 */
    static Kernel<T> *factory(const std::string& id) throw(BadKernelCreation)
    {
        if(id == "linear")
            return new KernelLinear<T>;
        if(id == "rbf")
            return new KernelRBF<T>;
        if(id == "chisquared")
            return new KernelChisquared<T>;
        else
            throw BadKernelCreation(id);
    }
};


//class LinearKernel : public Kernel
//{
//private:
//	LinearKernel(string n = "") { };
//	template <typename T>
//	void evaluate ( const gMat2D< T >& X1, const gMat2D< T >& X2 , gMat2D< T > Z ){
//		gMat2D< T >
//		dot(X1, X2, Z);
//	}

//};


}

#endif // _GURLS_KERNEL_H_

