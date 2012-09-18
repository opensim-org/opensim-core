#ifndef __FunctionScaler_h__
#define __FunctionScaler_h__
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  FunctionScaler.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Eastman                                                   *
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
#include "Function.h"



namespace OpenSim { 

//=============================================================================
//=============================================================================
/**
 * This is a SimTK::Function that multiplies the value of another function by
 * a constant.
 *
 * @author Peter Eastman
 */
class OSIMCOMMON_API FunctionScaler : public SimTK::Function
{
//=============================================================================
// DATA
//=============================================================================
protected:
    /** The function to be scaled. */
	const SimTK::Function* _function;
    /** The scale factor. */
    const double _scale;

//=============================================================================
// METHODS
//=============================================================================
public:
	FunctionScaler(const SimTK::Function* function, double scale);
    ~FunctionScaler();
    double calcValue(const SimTK::Vector& x) const;
    double calcDerivative(const std::vector<int>& derivComponents, const SimTK::Vector& x) const;
    int getArgumentSize() const;
    int getMaxDerivativeOrder() const;
private:
    FunctionScaler();
    FunctionScaler& operator=(FunctionScaler& function);

//=============================================================================
};  // END class FunctionScaler

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __FunctionScaler_h__
