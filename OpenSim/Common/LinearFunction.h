#ifndef __LinearFunction_h__
#define __LinearFunction_h__

// LinearFunction.h
// Author: Ajay Seth
/*
 * Copyright (c)  2009, Stanford University. All rights reserved. 
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


// INCLUDES
#include <string>
#include "Function.h"
#include "PropertyDblArray.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class for representing a LinearFunction.
 *
 * dependent = slope*independent + intercept
 *
 * This class inherits from Function and so can be used as input to
 * any class requiring a Fuction as input.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMCOMMON_API LinearFunction : public Function {
OpenSim_DECLARE_CONCRETE_OBJECT(LinearFunction, Function);

//=============================================================================
// MEMBER VARIABLES
//=============================================================================
protected:
	PropertyDblArray _coefficientsProp;
	Array<double> &_coefficients;

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	LinearFunction();
	LinearFunction(Array<double> coefficients);
	LinearFunction(const LinearFunction &aSpline);
	virtual ~LinearFunction();

private:
	void setNull();
	void setupProperties();
	void copyData(const LinearFunction &aLinearFunction);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	LinearFunction& operator=(const LinearFunction &aLinearFunction);
#endif

	//--------------------------------------------------------------------------
	// SET AND GET Coefficients
	//--------------------------------------------------------------------------
public:
	/** Set Coefficients for slope and intercept */
	void setCoefficients(Array<double> coefficients);
	/** Get Coefficients */
	const Array<double> getCoefficients() const
	{ return _coefficients; }

	//--------------------------------------------------------------------------
	// EVALUATION
	//--------------------------------------------------------------------------
    virtual SimTK::Function* createSimTKFunction() const;

//=============================================================================
};	// END class LinearFunction
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif  // __LinearFunction_h__
