#ifndef OPENSIM_FUNCTION_SET_H_
#define OPENSIM_FUNCTION_SET_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  FunctionSet.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */




// INCLUDES
#include "osimCommonDLL.h"
#include "Object.h"
#include "PropertyDbl.h"
#include "Function.h"
#include "Set.h"

//=============================================================================
//=============================================================================
/**
 * A class for holding a set of functions.
 *
 * @see Function
 * @author Frank C. Anderson
 */
namespace OpenSim { 

class OSIMCOMMON_API FunctionSet : public Set<Function> {
OpenSim_DECLARE_CONCRETE_OBJECT(FunctionSet, Set<Function>);

//=============================================================================
// DATA
//=============================================================================

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	FunctionSet();
	FunctionSet(const std::string &aFileName);
	virtual ~FunctionSet();

private:
	void setNull();

public:
	//--------------------------------------------------------------------------
	// EVALUATION
	//--------------------------------------------------------------------------
	virtual double
		evaluate(int aIndex,int aDerivOrder,
		double aX=0.0) const;
	virtual void
		evaluate(Array<double> &rValues,int aDerivOrder,
		double aX=0.0) const;

//=============================================================================
};	// END class FunctionSet


}; //namespace
//=============================================================================
//=============================================================================

#endif  // OPENSIM_FUNCTION_SET_H_
