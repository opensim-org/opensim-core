#ifndef OPENSIM_LIU_THELEN_2003_MUSCLE_H_
#define OPENSIM_LIU_THELEN_2003_MUSCLE_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  LiuThelen2003Muscle.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Loan, Darryl G. Thelen                                    *
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
 * This class implements a Thelen2003Muscle that includes two states for
 * modeling fatigue and recovery of muscle fibers. The equations for these
 * states are based on the following paper:
 * Liu, Jing Z., Brown, Robert, Yue, Guang H., "A Dynamical Model of Muscle
 * Activation, Fatigue, and Recovery," Biophysical Journal, Vol. 82, Issue 5,
 * pp. 2344-2359, 2002.
 *
 * @author Peter Loan (based on Thelen2003Muscle by Darryl Thelen)
 *
 * The muscle base class, Muscle, contains many pure virtual functions that
 * must be implemented in every derived class. Thelen2003Muscle implements
 * all of them, so LiuThelen2003Muscle, which derives from Thelen2003Muscle,
 * implements only the functions whose behaviors need to change.
 * If you create a new muscle model that derives directly from Muscle, you
 * will need to implement more functions than are declared in this header
 * file. See Muscle.h for a complete list.
 */
class LiuThelen2003Muscle : public Thelen2003Muscle_Deprecated {
OpenSim_DECLARE_CONCRETE_OBJECT(LiuThelen2003Muscle, Thelen2003Muscle_Deprecated);
public:
//=============================================================================
// PROPERTIES
//=============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(fatigue_factor, double, 
        "percentage of active motor units that fatigue in unit time");
    OpenSim_DECLARE_PROPERTY(recovery_factor, double, 
        "percentage of fatigued motor units that recover in unit time");
    /**@}**/

//=============================================================================
// DATA
//=============================================================================
protected:
	// defaults for the state variables
	double _defaultActiveMotorUnits;
	double _defaultFatiguedMotorUnits;

protected:
	static const int STATE_ACTIVE_MOTOR_UNITS;
	static const int STATE_FATIGUED_MOTOR_UNITS;
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	LiuThelen2003Muscle();
	LiuThelen2003Muscle(const std::string &aName, double aMaxIsometricForce, double aOptimalFiberLength,
		double aTendonSlackLength, double aPennationAngle, double aFatigueFactor, double aRecoveryFactor);

    // default destructor, copy constructor, copy assignment

	// Defaults
	virtual double getDefaultActiveMotorUnits() const;
	virtual void setDefaultActiveMotorUnits(double activeMotorUnits);
	virtual double getDefaultFatiguedMotorUnits() const;
	virtual void setDefaultFatiguedMotorUnits(double fatiguedMotorUnits);

	//--------------------------------------------------------------------------
	// GET
	//--------------------------------------------------------------------------
	// Properties
	virtual double getFatigueFactor() const { return get_fatigue_factor(); }
	virtual bool setFatigueFactor(double aFatigueFactor);
	virtual double getRecoveryFactor() const { return get_recovery_factor(); }
	virtual bool setRecoveryFactor(double aRecoveryFactor);

	// Computed quantities
	virtual double getActiveMotorUnits(const SimTK::State& s) const { return getStateVariable(s, "active_motor_units"); }
	virtual void setActiveMotorUnits(SimTK::State& s, double activeMotorUnits) const { setStateVariable(s, "active_motor_units", activeMotorUnits); }
	virtual double getActiveMotorUnitsDeriv(const SimTK::State& s) const { return getStateVariableDeriv(s, "active_motor_units"); }
	virtual void setActiveMotorUnitsDeriv(const SimTK::State& s, double activeMotorUnitsDeriv) const { setStateVariableDeriv(s, "active_motor_units", activeMotorUnitsDeriv); }

	virtual double getFatiguedMotorUnits(const SimTK::State& s) const { return getStateVariable(s, "fatigued_motor_units"); }
	virtual void setFatiguedMotorUnits(SimTK::State& s, double fatiguedMotorUnits) const { setStateVariable(s, "fatigued_motor_units", fatiguedMotorUnits); }
	virtual double getFatiguedMotorUnitsDeriv(const SimTK::State& s) const { return getStateVariableDeriv(s, "fatigued_motor_units"); }
	virtual void setFatiguedMotorUnitsDeriv(const SimTK::State& s, double fatiguedMotorUnitsDeriv) const { setStateVariableDeriv(s, "fatigued_motor_units", fatiguedMotorUnitsDeriv); }

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual SimTK::Vector computeStateVariableDerivatives(const SimTK::State& s) const;
	virtual void computeEquilibrium(SimTK::State& s ) const;
	virtual double computeActuation(const SimTK::State& s) const;
	virtual double computeIsometricForce(SimTK::State& s, double activation) const;
	virtual void equilibrate(SimTK::State& state) const;
	
protected:
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;

private:
	void setNull();
	void constructProperties();

//=============================================================================
};	// END of class LiuThelen2003Muscle
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __LiuThelen2003Muscle_h__
