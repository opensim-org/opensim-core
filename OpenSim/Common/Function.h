#ifndef OPENSIM_FUNCTION_H_
#define OPENSIM_FUNCTION_H_
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Function.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include "Object.h"
#include "SimTKmath.h"


//=============================================================================
//=============================================================================
namespace OpenSim { 

/**
 * An abstract class for representing a function.
 *
 * A function is a relation between independent variables and a dependent
 * value such that for any particular set of independent variables there is
 * only one unique dependent value.  Values of the function and its derivatives
 * are obtained by calling the calcValue() method.  The curve may or may not
 * be finite or differentiable; the calcValue() method returns values between
 * -`SimTK::Infinity` and `SimTK::Infinity`, or it returns `SimTK::NaN`
 * (not a number) if the curve is not defined.
 * Currently, functions of up to 3 variables (x,y,z) are supported.
 *
 * @author Frank C. Anderson
 */
class OSIMCOMMON_API Function : public Object {
OpenSim_DECLARE_ABSTRACT_OBJECT(Function, Object);

//=============================================================================
// DATA
//=============================================================================
protected:
    // The SimTK::Function object implementing this function.
    mutable SimTK::Function* _function;

//=============================================================================
// METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    Function();
    Function(const Function &aFunction);
    virtual ~Function();
    virtual void init(Function* aFunction) { }

private:
    void setNull();

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
#ifndef SWIG
    Function& operator=(const Function &aFunction);
#endif
    //--------------------------------------------------------------------------
    // SET AND GET
    //--------------------------------------------------------------------------
public:
    //--------------------------------------------------------------------------
    // UTILITY
    //--------------------------------------------------------------------------
    static Function* makeFunctionOfType(Function* aFunction, const std::string& aNewTypeName);

    //--------------------------------------------------------------------------
    // EVALUATE
    //--------------------------------------------------------------------------
    /**
     * Calculate the value of this function at a particular point.
     * 
     * @param x the Vector of input arguments.
     *          its size must equal the value returned by getArgumentSize().
     */
    virtual double calcValue(const SimTK::Vector& x) const;
    /**
     * Calculate a partial derivative of this function at a particular point.  Which derivative to take is specified
     * by listing the input components with which to take it.  For example, if derivComponents=={0}, that indicates
     * a first derivative with respective to component 0.  If derivComponents=={0, 0, 0}, that indicates a third
     * derivative with respective to component 0.  If derivComponents=={4, 7}, that indicates a partial second derivative with
     * respect to components 4 and 7.
     * 
     * @param derivComponents  the input components with respect to which the derivative should be taken.  Its size must be
     *                         less than or equal to the value returned by getMaxDerivativeOrder().
     * @param x                the Vector of input arguments.  Its size must equal the value returned by getArgumentSize().
     */
    virtual double calcDerivative(const std::vector<int>& derivComponents, const SimTK::Vector& x) const;
    /**
     * Get the number of components expected in the input vector.
     */
    virtual int getArgumentSize() const;
    /**
     * Get the maximum derivative order this Function object can calculate.
     */
    virtual int getMaxDerivativeOrder() const;
    /**
     * Return a SimTK::Function that can be used natively by the
     * underlying SimTK::System and its elements.
     */
    virtual SimTK::Function* createSimTKFunction() const = 0;

protected:
    /**
     * This should be called whenever this object has been modified.  It clears 
     * the internal SimTK::Function object used to evaluate it.
     */
    void resetFunction();

//=============================================================================
};  // END class Function

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_FUNCTION_H_
