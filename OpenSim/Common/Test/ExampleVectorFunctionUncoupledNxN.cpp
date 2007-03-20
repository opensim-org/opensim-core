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


// INCLUDES
#include "ExampleVectorFunctionUncoupledNxN.h"
#include <math.h>



using namespace OpenSim;
using namespace std;


//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
ExampleVectorFunctionUncoupledNxN::~ExampleVectorFunctionUncoupledNxN()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ExampleVectorFunctionUncoupledNxN::
ExampleVectorFunctionUncoupledNxN() :
	VectorFunctionUncoupledNxN(1)
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ExampleVectorFunctionUncoupledNxN::
ExampleVectorFunctionUncoupledNxN(int aN) :
	VectorFunctionUncoupledNxN(aN)
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aVectorFunction Function to copy.
 */
ExampleVectorFunctionUncoupledNxN::
ExampleVectorFunctionUncoupledNxN(const ExampleVectorFunctionUncoupledNxN &aVectorFunction) :
	VectorFunctionUncoupledNxN(aVectorFunction)
{
	setNull();

	// ASSIGN
	setEqual(aVectorFunction);
}
//_____________________________________________________________________________
/**
 * Copy this object.
 *
 * @return Pointer to a copy of this object.
 */
Object* ExampleVectorFunctionUncoupledNxN::
copy() const
{
	ExampleVectorFunctionUncoupledNxN *func =
		new ExampleVectorFunctionUncoupledNxN(*this);
	return(func);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to NULL values.
 */
void ExampleVectorFunctionUncoupledNxN::
setNull()
{
	setType("ExampleVectorFunctionUncoupledNxN");
}

//_____________________________________________________________________________
/**
 * Set all member variables equal to the members of another object.
 * Note that this method is private.  It is only meant for copying the data
 * members defined in this class.  It does not, for example, make any changes
 * to data members of base classes.
 */
void ExampleVectorFunctionUncoupledNxN::
setEqual(const ExampleVectorFunctionUncoupledNxN &aVectorFunction)
{
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
ExampleVectorFunctionUncoupledNxN& ExampleVectorFunctionUncoupledNxN::
operator=(const ExampleVectorFunctionUncoupledNxN &aVectorFunction)
{
	// BASE CLASS
	VectorFunctionUncoupledNxN::operator=(aVectorFunction);

	// DATA
	setEqual(aVectorFunction);

	return(*this);
}


//=============================================================================
// EVALUATE
//=============================================================================
//_____________________________________________________________________________
/**
 * Evaluate the vector function.
 *
 * @param aX Array of abscissae.
 * @param aY Array of resulting function values.
 */
void ExampleVectorFunctionUncoupledNxN::
evaluate(const double *aX,double *rY)
{
	int N = getNX();

	// COMMON PART
	int i;
	double sum;
	double scale = 0.01;
	for(sum=0.0,i=0;i<N;i++) {
		sum += (double)i;
	}
	sum *= scale;

	// UNIQUE PART
	// Uncoupled-- each aY depends only on its corresponding aX.
	double root;
	for(i=0;i<N;i++) {
		root = scale * (double)i;
		// sin test function
		rY[i] = sum * sin(aX[i] - root);
		// parabolic test function
		//rY[i] = sum *aX[i]*aX[i]*aX[i] - sum*root*root*root; 
	}
}
//_____________________________________________________________________________
/**
 * Evaluate the vector function.
 *
 * @param aX Array of abscissae.
 * @param aY Array of resulting function values.
 */
void ExampleVectorFunctionUncoupledNxN::
evaluate(const Array<double> &aX,Array<double> &rY)
{
	evaluate(&aX[0],&rY[0]);
}
//_____________________________________________________________________________
/**
 * Evaluate this function or a derivative of this function given a value for the
 * independent variable.  
 *
 * @param aX Vector of the independent variables.
 * @param rY Vector of the resulting dependent variables.
 * @param aDerivWRT
 */
void ExampleVectorFunctionUncoupledNxN::
evaluate(const Array<double> &aX,Array<double> &rY,
			const Array<int> &aDerivWRT)
{
	cout<<"\nExampleVectorFunctionUncoupledNxN.evalute(x,y,derivWRT): not implemented.\n";
}


