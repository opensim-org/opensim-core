#ifndef OPENSIM_SINE_H_
#define OPENSIM_SINE_H_
/* -------------------------------------------------------------------------- *
 *                              OpenSim:  Sine.h                              *
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
#include <string>
#include "Function.h"
#include "FunctionAdapter.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class for representing a Sine function.
 *
 * This class inherits from Function and can be used as input to
 * any Component requiring a Function as input. Implements:
 *  f(x) = amplitude*sin(omega*x+phase)+offset;
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMCOMMON_API Sine : public Function {
OpenSim_DECLARE_CONCRETE_OBJECT(Sine, Function);
protected:
//==============================================================================
// PROPERTIES
//==============================================================================

    OpenSim_DECLARE_PROPERTY(amplitude, double,
        "The amplitude of the sinusoidal function.");

    OpenSim_DECLARE_PROPERTY(omega, double,
        "The angular frequency (omega) in radians/sec.");

    OpenSim_DECLARE_PROPERTY(phase, double,
        "The phase shift of the sinusoidal function.");

    OpenSim_DECLARE_PROPERTY(offset, double,
        "The DC offset in the sinusoidal function.");

//=============================================================================
// METHODS
//=============================================================================
public:
    //Default construct, copy and assignment
    Sine() {
        constructProperties();
    }

    // Convenience Constructor
    Sine(double amplitude, double omega, double phase, double offset=0) : Sine()
    {
        set_amplitude(amplitude);
        set_omega(omega);
        set_phase(phase);
        set_offset(offset);
    }

    virtual ~Sine() {};

    //--------------------------------------------------------------------------
    // EVALUATION
    //--------------------------------------------------------------------------
    double calcValue(const SimTK::Vector& x) const override {
        return get_amplitude()*sin(get_omega()*x[0] + get_phase())
            + get_offset();
    }
    
    double calcDerivative(const std::vector<int>& derivComponents,
        const SimTK::Vector& x) const override {
        int n = (int)derivComponents.size();
        return get_amplitude()*pow(get_omega(),n) * 
            sin(get_omega()*x[0] + get_phase() + n*SimTK::Pi/2);
    }

    SimTK::Function* createSimTKFunction() const override {
        return new FunctionAdapter(*this);
    }
   
    int getArgumentSize() const override {return 1;}
    int getMaxDerivativeOrder() const override {return 10;}

private:
    void constructProperties() {
        constructProperty_amplitude(1.0);
        constructProperty_omega(1.0);
        constructProperty_phase(0.0);
        constructProperty_offset(0.0);
    }
//=============================================================================
};  // END class Sine
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif  // OPENSIM_SINE_H_
