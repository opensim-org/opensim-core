#ifndef OPENSIM_FATIGABLE_MUSCLE_H_
#define OPENSIM_FATIGABLE_MUSCLE_H_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  FatigableMuscle.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan, Ajay Seth                                           *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


// INCLUDE
#include <OpenSim/OpenSim.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * This class extends a Millard2012EquilibriumMuscle by including three 
 * additional states to model the fatigue and recovery of muscle fibers. 
 * The equations for these states are (loosely) based on the following paper:
 * Liu, Jing Z., Brown, Robert W., Yue, Guang H., "A Dynamical Model of Muscle
 * Activation, Fatigue, and Recovery," Biophysical Journal, Vol. 82, Issue 5,
 * pp. 2344-2359, 2002.
 *
 * NOTE: The primary purpose of this muscle model is to serve as an example
 *       for extending existing OpenSim muscle models and it is not intended 
 *       for research. The implementation of the fatigue dynamics have not 
 *       been tested.
 *
 * @author Ajay Seth (based on Millard2012EquilibriumMuscle)
 * @contributor Peter Loan (originally based on Thelen2003Muscle)
 *
 * The Muscle base class, specifies the interface that must be implemented 
 * by derived muscle classes. The FatigableMuscle derives from
 * Millard2012EquilibriumMuscle, which is a concrete implementation of the 
 * Muscle interface. The dynamics for fatigue are added by overriding methods 
 * extendAddToSystem() which allocates the additional states and 
 * computeStateVariableDerivatives() to specify their dynamics (derivatives). 
 *
 * @see Millard2012EquilibriumMuscle
 * @see Muscle
 */
class FatigableMuscle : public Millard2012EquilibriumMuscle {
OpenSim_DECLARE_CONCRETE_OBJECT(FatigableMuscle, 
                                Millard2012EquilibriumMuscle);
public:
//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_PROPERTY(fatigue_factor, double, 
        "percentage of active motor units that fatigue in unit time");
    OpenSim_DECLARE_PROPERTY(recovery_factor, double, 
        "percentage of fatigued motor units that recover in unit time");
    OpenSim_DECLARE_PROPERTY(default_target_activation, double,
        "default state value used to initialize the muscle's target activation");
    OpenSim_DECLARE_PROPERTY(default_active_motor_units, double, 
        "default state value for the fraction of motor units that are active");
    OpenSim_DECLARE_PROPERTY(default_fatigued_motor_units, double, 
        "default state value for the fraction of motor units that are fatigued");

public:
//=============================================================================
// METHODS
//=============================================================================
    //-------------------------------------------------------------------------
    // CONSTRUCTION
    //-------------------------------------------------------------------------
    FatigableMuscle();
    FatigableMuscle(const std::string &name, double maxIsometricForce, 
                    double optimalFiberLength, double tendonSlackLength,
                    double pennationAngle, double fatigueFactor, 
                    double recoveryFactor);

    // employs the default destructor, copy constructor and copy assignment 
    // that are automatically supplied by the compiler if none are defined

    //-------------------------------------------------------------------------
    // GET & SET Properties
    //-------------------------------------------------------------------------
    /** Fatigue and recovery factors are properties of this muscle */
    double getFatigueFactor() const { return get_fatigue_factor(); }
    void setFatigueFactor(double aFatigueFactor);
    double getRecoveryFactor() const { return get_recovery_factor(); }
    void setRecoveryFactor(double aRecoveryFactor);

    /** default values for states */
    double getDefaultTargetActivation() const {
        return get_default_target_activation();
    }
    void setDefaultTargetActivation(double targetActivation);
    
    double getDefaultActiveMotorUnits() const;
    void setDefaultActiveMotorUnits(double activeMotorUnits);
    
    double getDefaultFatiguedMotorUnits() const;
    void setDefaultFatiguedMotorUnits(double fatiguedMotorUnits);

    //-------------------------------------------------------------------------
    // GET & SET States and their derivatives
    //-------------------------------------------------------------------------
    /** Fatigued activation state and time derivative accessors */
    double getTargetActivation(const SimTK::State& s) const;
    void setTargetActivation(SimTK::State& s, double fatiguedAct) const;
    double getTargetActivationDeriv(const SimTK::State& s) const;
    void setTargetActivationDeriv(const SimTK::State& s,
                                    double fatiguedActDeriv) const;
    /** Active motor units state and time derivative accessors */
    double getActiveMotorUnits(const SimTK::State& s) const;
    void setActiveMotorUnits(SimTK::State& s, double activeMotorUnits) const;
    double getActiveMotorUnitsDeriv(const SimTK::State& s) const;
    void setActiveMotorUnitsDeriv(const SimTK::State& s,
                                  double activeMotorUnitsDeriv) const;
    /** Fatigued motor units state accessors */
    double getFatiguedMotorUnits(const SimTK::State& s) const;
    void setFatiguedMotorUnits(SimTK::State& s, 
                               double fatiguedMotorUnits) const;
    double getFatiguedMotorUnitsDeriv(const SimTK::State& s) const;
    void setFatiguedMotorUnitsDeriv(const SimTK::State& s, 
                                    double fatiguedMotorUnitsDeriv) const;

protected:
    // Model Component Interface
    /** add new dynamical states to the multibody system corresponding
        to this muscle */
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    /** initialize muscle state variables from properties. For example, any 
        properties that contain default state values */
    void extendInitStateFromProperties(SimTK::State& s) const override;
    /** use the current values in the state to update any properties such as 
        default values for state variables */
    void extendSetPropertiesFromState(const SimTK::State& s) override;

    //-------------------------------------------------------------------------
    // COMPUTATIONS
    //-------------------------------------------------------------------------
    /** Compute the derivatives for state variables added by this muscle */
    void computeStateVariableDerivatives(const SimTK::State& s) const override;
private:
    /** construct the new properties and set their default values */
    void constructProperties();

//=============================================================================
};  // END of class FatigableMuscle
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_FATIGABLE_MUSCLE_H_
