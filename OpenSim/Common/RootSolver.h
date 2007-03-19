#ifndef _RootSolver_h_
#define _RootSolver_h_
// RootSolver.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
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

/*  
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include "osimCommon.h"
#include <string>
#include "Array.h"
#include "VectorFunctionUncoupledNxN.h"


//template class OSIMCOMMON_API Array<double>;


//=============================================================================
//=============================================================================
/**
 * A class for finding the N roots of N one-dimensional non-linear equations.
 * 
 * The rational for making this class solve for the roots of N equations
 * simultaneously is that, for some problems, it is more computationally
 * efficient to evaluate the N equations at the same time, as opposed to one
 * after the other.  That is, the N equations, though decoupled, do share
 * some common terms.
 *
 * This class can always be used for a system where N=1, although there
 * will be some small amount of overhead for this class to function in
 * this way when compared to a class that is dedicated to an N=1.
 *
 * To construct an instance of this class, the user must provide an
 * instance of a VectorFunctionUncoupledNxN.
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
namespace OpenSim { 

class OSIMCOMMON_API RootSolver
{
//=============================================================================
// MEMBER VARIABLES
//=============================================================================
private:
	/** NxN Decoupled Vector Function */
	VectorFunctionUncoupledNxN *_function;


//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	RootSolver(VectorFunctionUncoupledNxN *aFunc);
	virtual ~RootSolver();
private:
	void setNull();

	//--------------------------------------------------------------------------
	// SOLVE
	//--------------------------------------------------------------------------
public:
	Array<double> solve(const Array<double> &ax,const Array<double> &bx,
		const Array<double> &tol);

//=============================================================================
};	// END class RootSolver

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __RootSolver_h__
