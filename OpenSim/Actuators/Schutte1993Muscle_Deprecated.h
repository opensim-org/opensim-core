#ifndef OPENSIM_SCHUTTE_1993_MUSCLE_DEPRECATED_H_
#define OPENSIM_SCHUTTE_1993_MUSCLE_DEPRECATED_H_
/* -------------------------------------------------------------------------- *
 *                  OpenSim:  Schutte1993Muscle_Deprecated.h                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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


// INCLUDE
#include "osimActuatorsDLL.h"
#include <OpenSim/Common/Function.h>
#include <OpenSim/Simulation/Model/ActivationFiberLengthMuscle_Deprecated.h>

#ifdef SWIG
    #ifdef OSIMACTUATORS_API
        #undef OSIMACTUATORS_API
        #define OSIMACTUATORS_API
    #endif
#endif

namespace OpenSim {

//=============================================================================
//                SCHUTTE 1993 MUSCLE (DEPRECATED)
//=============================================================================
/**
 * A class implementing a SIMM muscle.
 *
 * @author Peter Loan
 */
class OSIMACTUATORS_API Schutte1993Muscle_Deprecated 
:   public ActivationFiberLengthMuscle_Deprecated {
OpenSim_DECLARE_CONCRETE_OBJECT(Schutte1993Muscle_Deprecated, 
                                ActivationFiberLengthMuscle_Deprecated);
public:
//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_PROPERTY(time_scale, double,
        "Scale factor for normalizing time");
    OpenSim_DECLARE_PROPERTY(activation1, double,
        "Parameter used in time constant of ramping up of muscle force");
    OpenSim_DECLARE_PROPERTY(activation2, double,
        "Parameter used in time constant of ramping up and ramping down of muscle force");
    OpenSim_DECLARE_PROPERTY(damping, double,
        "Damping factor related to maximum contraction velocity");
    OpenSim_DECLARE_PROPERTY(tendon_force_length_curve, Function,
        "Function representing force-length behavior of tendon");
    OpenSim_DECLARE_PROPERTY(active_force_length_curve, Function,
        "Function representing active force-length behavior of muscle fibers");
    OpenSim_DECLARE_PROPERTY(passive_force_length_curve, Function,
        "Function representing passive force-length behavior of muscle fibers");

//=============================================================================
// PUBLIC METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    Schutte1993Muscle_Deprecated();
    Schutte1993Muscle_Deprecated(const std::string& aName,
                                 double             aMaxIsometricForce,
                                 double             aOptimalFiberLength,
                                 double             aTendonSlackLength,
                                 double             aPennationAngle);
    
    // default destructor, copy constructor, copy assignment

    //--------------------------------------------------------------------------
    // GET
    //--------------------------------------------------------------------------
    // Properties
    virtual double getTimeScale() const { return get_time_scale(); }
    virtual double getDamping() const { return get_damping(); }
    virtual bool setTimeScale(double aTimeScale);
    virtual bool setActivation1(double aActivation1);
    virtual bool setActivation2(double aActivation2);
    virtual bool setDamping(double aDamping);

    //--------------------------------------------------------------------------
    // COMPUTATION
    //--------------------------------------------------------------------------
    double computeActuation( const SimTK::State& s ) const override;
    double computeIsometricForce(SimTK::State& s, double activation) const override;

    virtual const Function& getActiveForceLengthCurve() const;
    virtual bool setActiveForceLengthCurve(const Function& aActiveForceLengthCurve);
    virtual const Function& getPassiveForceLengthCurve() const;
    virtual bool setPassiveForceLengthCurve(const Function& aPassiveForceLengthCurve);
    virtual const Function& getTendonForceLengthCurve() const;
    virtual bool setTendonForceLengthCurve(const Function& aTendonForceLengthCurve);

protected:
    // Model Component Interface
    void extendConnectToModel(Model& aModel)  override;

    // Super interface
    double calcPassiveForce(const SimTK::State& s, double aNormFiberLength) const override;
    double calcActiveForce(const SimTK::State& s, double aNormFiberLength) const override;

private:
    double calcNonzeroPassiveForce(const SimTK::State& s, double aNormFiberLength, double aNormFiberVelocity) const;
    double calcFiberVelocity(const SimTK::State& s, double aActivation, double aActiveForce, double aVelocityDependentForce) const;
    double calcTendonForce(const SimTK::State& s, double aNormTendonLength) const;

    void constructProperties();
//=============================================================================
};  // END of class Schutte1993Muscle_Deprecated
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_SCHUTTE_1993_MUSCLE_DEPRECATED_H_
