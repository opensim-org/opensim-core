#ifndef _ExampleVectorFunctionUncoupledNxN_h_
#define _ExampleVectorFunctionUncoupledNxN_h_
// ExampleVectorFunctionUncoupledNxN.cpp
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

#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Tools/Array.h>
#include <OpenSim/Tools/VectorFunctionUncoupledNxN.h>

//extern template class RDTOOLS_API Array<double>;

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
 * Math::MINUS_INFINITY and Math::PLUS_INFINITY, or it returns Math::NAN
 * (not a number) if the curve is not defined.
 * Currently, functions of up to 3 variables (x,y,z) are supported.
 *
 * @author Frank C. Anderson
 */
namespace OpenSim { 

class ExampleVectorFunctionUncoupledNxN : public VectorFunctionUncoupledNxN
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
	ExampleVectorFunctionUncoupledNxN();
	ExampleVectorFunctionUncoupledNxN(int aN);
	ExampleVectorFunctionUncoupledNxN(const ExampleVectorFunctionUncoupledNxN &aFunction);
	virtual ~ExampleVectorFunctionUncoupledNxN();
	virtual Object* copy() const;
private:
	void setNull();
	void setEqual(const ExampleVectorFunctionUncoupledNxN &aVectorFunction);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	ExampleVectorFunctionUncoupledNxN&
		operator=(const ExampleVectorFunctionUncoupledNxN &aFunction);

	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
public:
	
	//--------------------------------------------------------------------------
	// EVALUATE
	//--------------------------------------------------------------------------
	virtual void evaluate(const double *aX,double *aY);
	virtual void evaluate(const Array<double> &aX,Array<double> &rY);
	virtual void evaluate(const Array<double> &aX,Array<double> &rY,
		const Array<int> &aDerivWRT);

//=============================================================================
};

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __ExampleVectorFunctionUncoupledNxN_h__
