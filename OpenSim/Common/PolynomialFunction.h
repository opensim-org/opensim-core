#ifndef OPENSIM_POLYNOMIAL_FUNCTION_H_
#define OPENSIM_POLYNOMIAL_FUNCTION_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  PolynomialFunction.h                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Carmichael Ong                                                  *
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
#include <OpenSim/Common/Function.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class for representing a Polynomial of x.
 *
 * f(x) = a*x^n + b*x^(n-1) + ... + c
 *
 * Order of the coefficients corresponds to decreasing powers.
 *
 * @author Carmichael Ong
 */
class PolynomialFunction : public Function {
OpenSim_DECLARE_CONCRETE_OBJECT(PolynomialFunction, Function);

//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(coefficients, SimTK::Vector,
        "Coefficients of a polynomial function, from highest to lowest order."
        "Polynomial order is n-1, where n is the number of coefficients.");

//=============================================================================
// METHODS
//=============================================================================
public:

    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    /** Construct a polynomial with default coefficients of {1}
     */
    PolynomialFunction(){   constructProperties();  }

    /** Construct a polynomial with provided Vector of coefficients 
     * {a, b, ..., c} where the polynomial order, n = size-1
    */
    PolynomialFunction(SimTK::Vector coefficients){
        constructProperties();
        set_coefficients(coefficients);
    }

    virtual ~PolynomialFunction() {};

    //--------------------------------------------------------------------------
    // SET AND GET Coefficients
    //--------------------------------------------------------------------------
    /** %Set coefficients for the polynomial f of variable x:
     * f(x) = a*x^n + b*x^(n-1) + ... + c 
     * The size of the coefficient vector determines the order of the polynomial.
     * n = size-1;
     * @param[in] coefficients      Vector of polynomial coefficients
     */
    void setCoefficients(SimTK::Vector coefficients)
    { set_coefficients(coefficients); }
    /** Get the polynomial function coefficients */
    const SimTK::Vector getCoefficients() const
    { return get_coefficients(); }
    

    //--------------------------------------------------------------------------
    // EVALUATION
    //--------------------------------------------------------------------------
    /** Return the underlying SimTK::Function::Polynomial for direct use
     *   at the SimTK::System level 
     * @return   Pointer to the underlying SimTK::Function
     */
    SimTK::Function* createSimTKFunction() const override
    {
        return new SimTK::Function::Polynomial(get_coefficients());
    }

private:
    /**
    * Construct the serializable property member variables and
    * assign their default values
    */
    void constructProperties(){
        setAuthors("Carmichael Ong");
        constructProperty_coefficients(SimTK::Vector(1, 1));
    }

//=============================================================================
};  // END class PolynomialFunction
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif  // OPENSIM_POLYNOMIAL_FUNCTION_H_
