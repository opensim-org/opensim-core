#ifndef _MultiplierFunction_h_
#define _MultiplierFunction_h_

// MultiplierFunction.h
// Author: Peter Loan
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
#include "osimCommonDLL.h"
#include <string>
#include "PropertyObjPtr.h"
#include "PropertyDbl.h"
#include "Function.h"


//=============================================================================
//=============================================================================
/**
 * A class implementing a Function and a scale factor for the function's value.
 *
 * This class inherits from Function and so can be used as input to
 * any class requiring a Fuction as input.
 *
 * @author Peter Loan
 * @version 1.0
 */
namespace OpenSim { 

class OSIMCOMMON_API MultiplierFunction : public Function
{
//=============================================================================
// MEMBER VARIABLES
//=============================================================================
protected:
	// PROPERTIES
	/** The Function this object operates on. */
	PropertyObjPtr<OpenSim::Function> _osFunctionProp;
	Function *&_osFunction;

	/** Scale factor */
	PropertyDbl _scaleProp;
	double &_scale;

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	MultiplierFunction();
	MultiplierFunction(Function* aFunction);
	MultiplierFunction(Function* aFunction, double aScaleFactor);
	MultiplierFunction(const MultiplierFunction &aFunction);
	virtual ~MultiplierFunction();
	virtual Object* copy() const;
	virtual void init(Function* aFunction);

private:
	void setNull();
	void setupProperties();
	void setEqual(const MultiplierFunction &aFunction);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	MultiplierFunction& operator=(const MultiplierFunction &aFunction);
#endif

public:
	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
	void setFunction(Function* aFunction);
	void setScale(double aScaleFactor);
	Function* getFunction() const { return _osFunction; }
	double getScale() const { return _scale; }

	//--------------------------------------------------------------------------
	// EVALUATION
	//--------------------------------------------------------------------------
	double calcValue(const SimTK::Vector& x) const;
	double calcDerivative(const std::vector<int>& derivComponents, const SimTK::Vector& x) const;
	int getArgumentSize() const;
	int getMaxDerivativeOrder() const;
	SimTK::Function* createSimTKFunction() const;

private:
	OPENSIM_DECLARE_DERIVED(MultiplierFunction, Function)

//=============================================================================
};	// END class MultiplierFunction

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __MultiplierFunction_h__
