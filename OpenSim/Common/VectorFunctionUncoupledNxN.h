#ifndef _VectorFunctionUncoupledNxN_h_
#define _VectorFunctionUncoupledNxN_h_
// VectorFunctionUncoupledNxN.cpp
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
#include "VectorFunction.h"
#include "Array.h"



//=============================================================================
//=============================================================================
/**
 * An abstract class for representing a vector function.
 *
 * A vector function is a relation between some number of independent variables 
 * and some number of dependent values such that for any particular set of
 * independent variables the correct number of dependent variables is returned.
 * Values of the function and its derivatives
 * are obtained by calling the evaluate() method.  The curve may or may not
 * be finite or diferentiable; the evaluate method returns values between
 * rdMath::MINUS_INFINITY and rdMath::PLUS_INFINITY, or it returns rdMath::getNAN()
 * (not a number) if the curve is not defined.
 * Currently, functions of up to 3 variables (x,y,z) are supported.
 *
 * @author Frank C. Anderson and Saryn R. Goldberg
 */
namespace OpenSim { 

class OSIMCOMMON_API VectorFunctionUncoupledNxN : public VectorFunction
{
//=============================================================================
// DATA
//=============================================================================
protected:



//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	VectorFunctionUncoupledNxN();
	VectorFunctionUncoupledNxN(int aN);
	VectorFunctionUncoupledNxN(const VectorFunctionUncoupledNxN &aFunction);
	virtual ~VectorFunctionUncoupledNxN();
	virtual Object* copy() const = 0;
private:
	void setNull();
	void setEqual(const VectorFunctionUncoupledNxN &aVectorFunction);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	VectorFunctionUncoupledNxN&
		operator=(const VectorFunctionUncoupledNxN &aFunction);

	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
	
	//--------------------------------------------------------------------------
	// EVALUATE
	//--------------------------------------------------------------------------

	OPENSIM_DECLARE_DERIVED(VectorFunctionUncoupledNxN, VectorFunction)

//=============================================================================
};	// END class VectorFunctionUncoupledNxN

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __VectorFunctionUncoupledNxN_h__
