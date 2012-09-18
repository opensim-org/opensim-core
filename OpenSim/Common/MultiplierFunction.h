#ifndef _MultiplierFunction_h_
#define _MultiplierFunction_h_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  MultiplierFunction.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


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

class OSIMCOMMON_API MultiplierFunction : public Function {
OpenSim_DECLARE_CONCRETE_OBJECT(MultiplierFunction, Function);

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

//=============================================================================
};	// END class MultiplierFunction

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __MultiplierFunction_h__
