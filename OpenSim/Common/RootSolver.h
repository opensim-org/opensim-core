#ifndef _RootSolver_h_
#define _RootSolver_h_
// RootSolver.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*  
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include "osimCommonDLL.h"
#include <string>
#include "Array.h"
#include "VectorFunctionUncoupledNxN.h"
#include "SimTKsimbody.h"


//template class OSIMCOMMON_API Array<double>;

namespace OpenSim { 

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
class OSIMCOMMON_API RootSolver
{
//=============================================================================
// MEMBER VARIABLES
//=============================================================================
private:
	
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
	Array<double> solve(const SimTK::State& s, const Array<double> &ax,const Array<double> &bx,
		const Array<double> &tol);

//=============================================================================
};	// END class RootSolver

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __RootSolver_h__
