#ifndef __Constant_h__
#define __Constant_h__
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Constant.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Loan, Ajay Seth                                           *
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
#include <string>
#include "Function.h"
#include "PropertyDbl.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class for representing a constant value.
 *
 * This class inherits from Function and so can be used as input to
 * any class requiring a Fuction as input.
 *
 * @author Peter Loan, Ajay Seth
 * @version 1.0
 */
class OSIMCOMMON_API Constant : public Function {
OpenSim_DECLARE_CONCRETE_OBJECT(Constant, Function);

//=============================================================================
// MEMBER VARIABLES
//=============================================================================
protected:
	PropertyDbl _valueProp;
	double &_value;

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	Constant();
	Constant(double value);
	Constant(const Constant &aSpline);
	virtual ~Constant();

private:
	void setNull();
	void setupProperties();
	void copyData(const Constant &aConstant);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	Constant& operator=(const Constant &aConstant);
#endif

	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
public:
	void setValue(double aValue);

	//--------------------------------------------------------------------------
	// EVALUATION
	//--------------------------------------------------------------------------
    virtual double calcValue(const SimTK::Vector& xUnused) const
	{
		return _value;
	}
	const double getValue() const { return _value; }
    SimTK::Function* createSimTKFunction() const;
//=============================================================================
};	// END class Constant
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif  // __Constant_h__
