#ifndef OPENSIM_VECTOR_FUNCTION_FOR_ACTUATORS_H_
#define OPENSIM_VECTOR_FUNCTION_FOR_ACTUATORS_H_
/* -------------------------------------------------------------------------- *
 *                   OpenSim:  VectorFunctionForActuators.h                   *
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

/*  
 * Author: Frank C. Anderson 
 */

#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/VectorFunctionUncoupledNxN.h>

namespace SimTK {
class Integrator;
class System;
}

//=============================================================================
//=============================================================================
namespace OpenSim { 

class CMCActuatorSubsystem;
class Model;

/**
 * An abstract class for representing a vector function.
 *
 * A vector function is a relation between some number of independent variables 
 * and some number of dependent values such that for any particular set of
 * independent variables the correct number of dependent variables is returned.
 * Values of the function and its derivatives
 * are obtained by calling the evaluate() method.  The curve may or may not
 * be finite or differentiable; the evaluate method returns values between
 * - %SimTK::Infinity and %SimTK::Infinity, or it returns SimTK::NaN
 * (not a number) if the curve is not defined.
 * Currently, functions of up to 3 variables (x,y,z) are supported.
 *
 * @author Frank C. Anderson
 */
class VectorFunctionForActuators : public VectorFunctionUncoupledNxN {
OpenSim_DECLARE_CONCRETE_OBJECT(VectorFunctionForActuators, 
                                VectorFunctionUncoupledNxN);

//=============================================================================
// DATA
//=============================================================================
protected:
    /** Initial time for the integration. */
    double _ti;
    /** Final time for the integration. */
    double _tf;
    /** Target actuator forces. */
    Array<double> _f;
    /** Actuator System  */
    SimTK::System* _CMCActuatorSystem;
    /** Actuator SubSystem  */
    CMCActuatorSubsystem* _CMCActuatorSubsystem;
    /** Integrator. */
    SimTK::Integrator* _integrator;
    /** Model */
    Model* _model;


//=============================================================================
// METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    VectorFunctionForActuators(SimTK::System *aActuatorSystem, Model *model, CMCActuatorSubsystem* actSubsys);
    VectorFunctionForActuators(const VectorFunctionForActuators &aFunction);
    VectorFunctionForActuators();
    virtual ~VectorFunctionForActuators();

private:
    void setNull();
    void setEqual(const VectorFunctionForActuators &aVectorFunction);

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
    VectorFunctionForActuators&
        operator=(const VectorFunctionForActuators &aFunction);

    //--------------------------------------------------------------------------
    // SET AND GET
    //--------------------------------------------------------------------------
    void setInitialTime(double aTI);
    double getInitialTime() const;
    void setFinalTime(double aTF);
    double getFinalTime() const;
    void setTargetForces(const double *aF);
    void getTargetForces(double *rF) const;
    CMCActuatorSubsystem* getCMCActSubsys();

    
    //--------------------------------------------------------------------------
    // EVALUATE
    //--------------------------------------------------------------------------

    void calcValue( const double *aX, double *rF, int aSize) override {
        log_warn("Unimplemented evaluate method."); 
//      exit(0);
    }
    void calcValue( const Array<double> &aX, Array<double> &rF) override {
        log_warn("Unimplemented evaluate method."); 
//      exit(0);
    }
    virtual void calcValue( const Array<double> &aX, Array<double> &rF, const Array<int> &aDerivWRT) {
        log_warn("Unimplemented evaluate method."); 
//      exit(0);
    }
    void calcDerivative(const Array<double> &aX,Array<double> &rY,
        const Array<int> &aDerivWRT) override {
        log_warn("Unimplemented calcDerivative method."); 
    }

    void evaluate(const SimTK::State& s, const double *aX, double *rF) override;
    void evaluate(const SimTK::State& s, const Array<double>& aX,
            Array<double>& rF) override;
    void evaluate(const SimTK::State& s, const Array<double>& aX,
            Array<double>& rF, const Array<int>& aDerivWRT) override;
    virtual void evaluate(const double *rY) {}
    virtual void evaluate(const Array<double> &rY) {}
    virtual void evaluate(Array<double> &rY, const Array<int> &aDerivWRT) {}


//=============================================================================
};  // END class VectorFunctionForActuators

}; //namespace
//=============================================================================
//=============================================================================

#endif  // OPENSIM_VECTOR_FUNCTION_FOR_ACTUATORS_H_
