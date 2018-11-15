#ifndef OPENSIM_THELEN_2003_MUSCLE_DEPRECATED_H_
#define OPENSIM_THELEN_2003_MUSCLE_DEPRECATED_H_
/* -------------------------------------------------------------------------- *
 *                  OpenSim:  Thelen2003Muscle_Deprecated.h                   *
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
#include <OpenSim/Simulation/Model/ActivationFiberLengthMuscle_Deprecated.h>

#ifdef SWIG
    #ifdef OSIMACTUATORS_API
        #undef OSIMACTUATORS_API
        #define OSIMACTUATORS_API
    #endif
#endif

namespace OpenSim {

//==============================================================================
//                   THELEN 2003 MUSCLE (DEPRECATED)
//==============================================================================
/**
 * A class implementing a SIMM muscle.
 *
 * @author Peter Loan
 */
class OSIMACTUATORS_API Thelen2003Muscle_Deprecated 
:   public ActivationFiberLengthMuscle_Deprecated {
OpenSim_DECLARE_CONCRETE_OBJECT(Thelen2003Muscle_Deprecated, 
                                ActivationFiberLengthMuscle_Deprecated);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(activation_time_constant, double,
        "time constant for ramping up of muscle activation");
    OpenSim_DECLARE_PROPERTY(deactivation_time_constant, double,
        "time constant for ramping down of muscle activation");
    OpenSim_DECLARE_PROPERTY(Vmax, double,
        "maximum contraction velocity at full activation in fiber lengths/second");
    OpenSim_DECLARE_PROPERTY(Vmax0, double,
        "maximum contraction velocity at low activation in fiber lengths/second");
    OpenSim_DECLARE_PROPERTY(FmaxTendonStrain, double,
        "tendon strain due to maximum isometric muscle force");
    OpenSim_DECLARE_PROPERTY(FmaxMuscleStrain, double,
        "passive muscle strain due to maximum isometric muscle force");
    OpenSim_DECLARE_PROPERTY(KshapeActive, double,
        "shape factor for Gaussian active muscle force-length relationship");
    OpenSim_DECLARE_PROPERTY(KshapePassive, double,
        "exponential shape factor for passive force-length relationship");
    OpenSim_DECLARE_PROPERTY(damping, double,
        "passive damping in the force-velocity relationship");
    OpenSim_DECLARE_PROPERTY(Af, double,
        "force-velocity shape factor");
    OpenSim_DECLARE_PROPERTY(Flen, double,
        "maximum normalized lengthening force");

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    Thelen2003Muscle_Deprecated();
    Thelen2003Muscle_Deprecated(const std::string&  name,
                                double              maxIsometricForce,
                                double              optimalFiberLength,
                                double              tendonSlackLength,
                                double              pennationAngle);
    
    // Properties
    double getActivationTimeConstant() const 
    {   return get_activation_time_constant(); }
    double getDeactivationTimeConstant() const 
    {   return get_deactivation_time_constant(); }
    double getVmax() const {return get_Vmax();}
    double getVmax0() const {return get_Vmax0();}
    double getFmaxTendonStrain() const {return get_FmaxTendonStrain();}
    double getFmaxMuscleStrain() const {return get_FmaxMuscleStrain();}
    double getKshapeActive() const {return get_KshapeActive();}
    double getKshapePassive() const {return get_KshapePassive();}
    double getDamping() const {return get_damping();}
    double getAf() const {return get_Af();}
    double getFlen() const {return get_Flen();}

    void setActivationTimeConstant(double aActivationTimeConstant)
    {   set_activation_time_constant(aActivationTimeConstant); }
    void setDeactivationTimeConstant(double aDeactivationTimeConstant)
    {   set_deactivation_time_constant(aDeactivationTimeConstant); }
    void setVmax(double aVmax)
    {   set_Vmax(aVmax); }
    void setVmax0(double aVmax0)
    {   set_Vmax0(aVmax0); }
    void setFmaxTendonStrain(double aFmaxTendonStrain)
    {   set_FmaxTendonStrain(aFmaxTendonStrain); }
    void setFmaxMuscleStrain(double aFmaxMuscleStrain)
    {   set_FmaxMuscleStrain(aFmaxMuscleStrain); }
    void setKshapeActive(double aKShapeActive)
    {   set_KshapeActive(aKShapeActive); }
    void setKshapePassive(double aKshapePassive)
    {   set_KshapePassive(aKshapePassive); }
    void setDamping(double aDamping)
    {   set_damping(aDamping); }
    void setAf(double aAf)
    {   set_Af(aAf); }
    void setFlen(double aFlen)
    {   set_Flen(aFlen); }

    // Computed quantities
    //--------------------------------------------------------------------------
    // FORCE-LENGTH-VELOCITY PROPERTIES
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    // COMPUTATIONS
    //--------------------------------------------------------------------------
    double computeActuation(const SimTK::State& s) const override;
    double calcTendonForce(const SimTK::State& s, double aNormTendonLength) const;
    double calcPassiveForce(const SimTK::State& s, double aNormFiberLength) const override;
    double calcActiveForce(const SimTK::State& s, double aNormFiberLength) const override;
    double calcFiberVelocity(const SimTK::State& s, double aActivation, double aActiveForce, double aVelocityDependentForce) const;
    double computeIsometricForce(SimTK::State& s, double activation) const override;

private:
    void constructProperties();
//==============================================================================
};  // END of class Thelen2003Muscle_Deprecated
//==============================================================================
//==============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_THELEN_2003_MUSCLE_DEPRECATED_H_
