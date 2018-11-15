#ifndef OPENSIM_LINEAR_FUNCTION_H_
#define OPENSIM_LINEAR_FUNCTION_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  LinearFunction.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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
 * any class requiring a Function as input.
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
    LinearFunction(double slope, double intercept);
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
    /** %Set Coefficients for slope and intercept */
    void setCoefficients(Array<double> coefficients);
    /** %Set slope */
    void setSlope(double slope) {_coefficients[0] = slope; }
    /** %Set intercept */
    void setIntercept(double intercept) {_coefficients[1] = intercept; }
    /** Get Coefficients */
    const Array<double> getCoefficients() const { return _coefficients; }
    /** Get Slope */
    double getSlope() const { return _coefficients[0]; }
    /** Get Intercept */ 
    double getIntercept() const { return _coefficients[1]; }
    

    //--------------------------------------------------------------------------
    // EVALUATION
    //--------------------------------------------------------------------------
    SimTK::Function* createSimTKFunction() const override;

//=============================================================================
};  // END class LinearFunction
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif  // OPENSIM_LINEAR_FUNCTION_H_
