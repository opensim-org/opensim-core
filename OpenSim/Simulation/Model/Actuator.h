#ifndef OPENSIM_ACTUATOR_H_
#define OPENSIM_ACTUATOR_H_
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Actuator.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Soha Pouya                                           *
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

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include "Force.h"


#ifdef SWIG
    #ifdef OSIMSIMULATION_API
        #undef OSIMSIMULATION_API
        #define OSIMSIMULATION_API
    #endif
#endif

namespace OpenSim {

//=============================================================================
//           ACTUATOR (base class to make a vector of control values)
//=============================================================================
/**
 * Base class for an actuator (e.g., a torque motor, muscle, ...) that requires
 * a generic external input (a vector of controls) to generate force. This class
 * therefore covers scalarActautor as a special case with scalar control value. 
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API Actuator : public Force {
OpenSim_DECLARE_ABSTRACT_OBJECT(Actuator, Force);
//=============================================================================
// NO PROPERTIES
//=============================================================================
//=============================================================================
// OUTPUTS
//=============================================================================
OpenSim_DECLARE_OUTPUT(power, double, getPower, SimTK::Stage::Dynamics);

//=============================================================================
// DATA
//=============================================================================
protected:
 
    // index in Controls Vector shared system cache entry
    int _controlIndex;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    Actuator();

    // default destructor, copy constructor, copy assignment

private:
    void setNull();

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
protected:
    // ModelComponent Interface
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

public:

    //Model building
    virtual int numControls() const = 0;

    /** Actuator default controls are zero */
    virtual const SimTK::Vector getDefaultControls() { return SimTK::Vector(numControls(), 0.0); } 
#ifndef SWIG
    // CONTROLS
    virtual const SimTK::VectorView_<double> getControls( const SimTK::State& s ) const;
#endif
    /** Convenience methods for getting, setting and adding to actuator controls from/into 
        the model controls. These methods have no effect on the realization stage. */
    virtual void getControls(const SimTK::Vector& modelControls, SimTK::Vector& actuatorControls) const;
    /** set actuator controls subvector into the right slot in the system-wide model controls */
    virtual void setControls(const SimTK::Vector& actuatorControls, SimTK::Vector& modelControls) const;
    /** add actuator controls to the values already occupying the slot in the system-wide model controls */
    virtual void addInControls(const SimTK::Vector& actuatorControls, SimTK::Vector& modelControls) const;

    //--------------------------------------------------------------------------
    // COMPUTATIONS
    //--------------------------------------------------------------------------
    virtual double getPower(const SimTK::State& s) const = 0;
    virtual void computeEquilibrium(SimTK::State& s) const { }

//=============================================================================
};  // END of class Actuator
//=============================================================================


//==============================================================================
//                       SCALARACTUATOR (scalar control value)
//==============================================================================

/**
 * This is a derived class from the base class actuator (e.g., a torque motor, 
 * muscle, ...) that requires exactly one external input (control) to generate 
 * a scalar value force, such as a torque/force magnitude or a tension.
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API ScalarActuator : public Actuator {
    OpenSim_DECLARE_ABSTRACT_OBJECT(ScalarActuator, Actuator);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** Default is -Infinity (no limit). **/
    OpenSim_DECLARE_PROPERTY(min_control, double,
        "Minimum allowed value for control signal. Used primarily when solving "
        "for control values.");
    /** Default is Infinity (no limit). **/
    OpenSim_DECLARE_PROPERTY(max_control, double,
        "Maximum allowed value for control signal. Used primarily when solving "
        "for control values.");

//==============================================================================
// OUTPUTS 
//==============================================================================
    OpenSim_DECLARE_OUTPUT(actuation, double, getActuation,
            SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(speed, double, getSpeed, SimTK::Stage::Velocity);


//==============================================================================
// PUBLIC METHODS
//==============================================================================
    ScalarActuator();

    // default destructor, copy constructor, copy assignment

    /** Convenience method to get control given scalar (double) valued control
     */
    virtual double getControl(const SimTK::State& s ) const;

    //Model building
    int numControls() const override {return 1;};

    // Accessing actuation, speed, and power of a scalar valued actuator
    virtual void setActuation(const SimTK::State& s, double aActuation) const;
    virtual double getActuation(const SimTK::State& s) const;
    virtual double getSpeed( const SimTK::State& s) const = 0;
    double getPower(const SimTK::State& s) const override { return getActuation(s)*getSpeed(s); }
    virtual double getStress(const SimTK::State& s) const;
    virtual double getOptimalForce() const;

    /** Methods to manage the bounds on ScalarActuator's control */
    void setMinControl(const double& aMinControl);
    double getMinControl() const;
    void setMaxControl(const double& aMaxControl);
    double getMaxControl() const;

    //--------------------------------------------------------------------------
    // Overriding Actuation
    //--------------------------------------------------------------------------
    /**
    * Enable/disable a ScalarActuator's override actuation.
    *
    * The actuation normally produced by a ScalarActuator can be overridden and
    * When the ScalarActuator's actuation is overridden, the ScalarActuator will
    * by default produce a constant actuation which can be set with
    * setOverrideActuation().
    *
    * @param s    current state
    * @param flag true = override ScalarActuator's output actuation
    *             false = use ScalarActuator's computed force (normal operation)
    */
    void overrideActuation(SimTK::State& s, bool flag) const;

    /**
    *  return ScalarActuator's override status
    */
    bool isActuationOverridden(const SimTK::State& s) const;

    /**
    * set the actuation value used when the override is true 
    * 
    * @param s      current state
    * @param value  value of override actuation   
    */
    void setOverrideActuation(SimTK::State& s, double value) const;

    /**
    * return override actuation 
    */
    double getOverrideActuation(const SimTK::State& s) const;


protected:

    // ScalarActuator interface
    virtual double computeActuation(const SimTK::State& s) const = 0;

    // ModelComponent Interface
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    double computeOverrideActuation(const SimTK::State& s) const;

    //Actuation reporting
    /** 
     * Methods to query a actuation for the value actually applied during 
     * simulation. The names of the quantities (column labels) is returned
     * by this first function getRecordLabels()
     */
    OpenSim::Array<std::string> getRecordLabels() const override {
        OpenSim::Array<std::string> labels("");
        labels.append(getName());
        return labels;
    }
    /**
     * Given SimTK::State object extract all the values necessary to report 
     * actuation, application location frame, etc. used in conjunction 
     * with getRecordLabels and should return same size Array
     */
    OpenSim::Array<double> getRecordValues(const SimTK::State& state) const override {
        OpenSim::Array<double> values(1);
        values.append(getActuation(state));
        return values;
    }

private:
    void constructProperties();

    mutable CacheVariable<double> _actuationCV;

//=============================================================================
};  // END of class ScalarActuator
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_ACTUATOR_H_


