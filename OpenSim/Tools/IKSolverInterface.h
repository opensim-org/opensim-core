#ifndef _IKSolverInterface_h_
#define _IKSolverInterface_h_

#include "osimToolsDLL.h"
#include <iostream>

namespace OpenSim { 

class IKTrial;
class Storage;
class IKTarget;

// IKSolverInterface.h
// Author: Ayman Habib
/* Copyright (c) 2005, Stanford University and Ayman Habib
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

class OSIMTOOLS_API IKSolverInterface 
{
protected: 
	IKTarget&	_ikTarget;
public:
	IKSolverInterface(IKTarget& aOptimizationTarget):
	_ikTarget(aOptimizationTarget)
	{
	}

	virtual ~IKSolverInterface() {}

	// Must call initializeSolver before calling solveFrames

	virtual void initializeSolver(const IKTrial& aIKOptions, Storage& inputData, Storage& outputData)
	{
		std::cout<< "Error, IKSolverInterface::initializeOutputStorage() - not implemented.\n";
	}

	virtual void solveFrames(const IKTrial& aIKOptions, Storage& inputData, Storage& outputData) 
	{
		std::cout<< "Error, IKSolverInterface::solveFrames() - not implemented.\n";
	}

};

}; //namespace

#endif // __IKSolverInterface_h__


