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

#ifndef _GURLS_PREDKERNEL_H_
#define _GURLS_PREDKERNEL_H_

#include <stdexcept>

#include "gmat2d.h"
#include "optlist.h"

namespace gurls {

template<typename T>
class PredKernelTrainTest;

/**
 * \ingroup PredKernels
 * \brief PredKernel is the class that computes the kernel matrix for prediction
 */

template<typename T>
class PredKernel
{
public:
   /**
     * Computes the kernel matrix necessary for predicting the labels associated to X
     *
     * \param X input data matrix
     * \param Y not used
     * \param opt options with the different required fields based on the sub-class
     *
     * \return adds the field predkernel to opt:
     */
    virtual void execute(const gMat2D<T>& X, const gMat2D<T>& Y, GurlsOptionsList& opt) = 0;

    /**
     * \ingroup Exceptions
     *
     * \brief BadPredKernelCreation is thrown when \ref factory tries to generate an unknown prediction kernel
     */
    class BadPredKernelCreation : public std::logic_error
    {
    public:
		/**
		 * Exception constructor.
		 */
        BadPredKernelCreation(std::string type)
            : logic_error("Cannot create type " + type) {}
    };

	/**
	 * Factory function returning a pointer to the newly created object.
	 *
	 * \warning The returned pointer is a plain, un-managed pointer. The calling
	 * function is responsible of deallocating the object.
	 */
    static PredKernel<T> *factory(const std::string& id) throw(BadPredKernelCreation)
    {
        if(id == "traintest")
            return new PredKernelTrainTest<T>;
        else
            throw BadPredKernelCreation(id);
    }
};

}

#endif // _GURLS_PREDKERNEL_H_

