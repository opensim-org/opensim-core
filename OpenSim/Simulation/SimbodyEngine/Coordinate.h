#ifndef OPENSIM_COORDINATE_H_
#define OPENSIM_COORDINATE_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Coordinate.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Michael A. Sherman, Ayman Habib                      *
 * Contributor(s): Frank C. Anderson, Jeffrey A. Reinbolt                     *
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <SimTKsimbody.h>

class ModifiableConstant;

namespace OpenSim {

class Function;
class Joint;
class Model;

//=============================================================================
//=============================================================================
/**
 * A Coordinate is a ModelComponent for managing the access and behavior 
 * of a model's generalized coordinate including its value, speed and 
 * acceleration (once system accelerations have been realized). 
 * As a ModelComponent it provides resources to enable a Coordinate to be
 * locked, prescribed, or clamped (limited to a min-to-max range).
 *
 * @authors Ajay Seth, Ayman Habib, Michael Sherman 
 */
class OSIMSIMULATION_API Coordinate : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(Coordinate, ModelComponent);

public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations 
    These are the serializable properties associated with a Coordinate. **/
    /**@{**/
	OpenSim_DECLARE_PROPERTY(motion_type, std::string, 
		"Coordinate can describe rotational, translational, or coupled motion. "
		"Defaults to rotational.");

	OpenSim_DECLARE_PROPERTY(default_value, double, 
		"The value of this coordinate before any value has been set. "
		"Rotational coordinate value is in radians and Translational in meters.");

	OpenSim_DECLARE_PROPERTY(default_speed_value, double, 
		"The speed value of this coordinate before any value has been set. "
		"Rotational coordinate value is in rad/s and Translational in m/s.");

	OpenSim_DECLARE_LIST_PROPERTY_SIZE(range, double, 2,
		"The minimum and maximum values that the coordinate can range between. "
		"Rotational coordinate range in radians and Translational in meters." );

	OpenSim_DECLARE_PROPERTY(clamped, bool, 
		"Flag indicating whether or not the values of the coordinates should "
		"be limited to the range, above." );

	OpenSim_DECLARE_PROPERTY(locked, bool, 
		"Flag indicating whether or not the values of the coordinates should "
		"be constrained to the current (e.g. default) value, above." );

	OpenSim_DECLARE_OPTIONAL_PROPERTY(prescribed_function, Function, 
		"If specified, the coordinate can be prescribed by a function of time. "
		"It can be any OpenSim Function with valid second order derivatives." );

	OpenSim_DECLARE_PROPERTY(prescribed, bool, 
		"Flag indicating whether or not the values of the coordinates should "
		"be prescribed according to the function above. It is ignored if the "
		"no prescribed function is specified." );

    OpenSim_DECLARE_PROPERTY(is_free_to_satisfy_constraints, bool, 
		"Flag identifies whether or not this coordinate can change freely when "
		"posing the model to satisfy kinematic constraints.  When true, the "
		"coordinate's initial or specified value is ignored when considering "
		"constraints. This allows values for important coordinates, which have "
		"this flag set to false, to dictate the value of unimportant coordinates " 
		"if they are linked via constraints."); 
    /**@}**/

	/** Motion type that describes the motion dictated by the coordinate.
		Types include: Rotational, Translational and Coupled (both) */
    enum MotionType
    {
        Rotational,
		Translational,
		Coupled
    };


//=============================================================================
// PUBLIC METHODS
//=============================================================================
	/** @name Public accessor methods 
		Get and set attributes of the Coordinate **/
    /**@{**/

	/** access to the Coordinate's owning joint */
	const Joint& getJoint() const;

	/** access to the generalized Coordinate's motion type
		This can be Rotational, Translational, or Coupled (both) */
	MotionType getMotionType() const { return _motionType; }
	void setMotionType(MotionType aMotionType);

	/** get the value of the Coordinate from the state */
	double getValue(const SimTK::State& s) const;
	/** set the value of the Coordinate on to the state.
		optional flag to enforce the constraints immediatedly, which may 
		adjust its value in the state. Use getValue(s) to see if/how the
		value was adjusted to satify the kinematic constraints. */
    void setValue(SimTK::State& s, double aValue, bool aEnforceContraints=true) const;

	/** get the speed value of the Coordinate from the state */
	double getSpeedValue(const SimTK::State& s) const;
    void setSpeedValue(SimTK::State& s, double aValue) const;
	const std::string  getSpeedName() const;

	/** get the default value for this coordinate. This is the value 
	    used if no value has been set prior to a simulation. */
	double getDefaultValue() const { return get_default_value(); }
	void setDefaultValue(double aDefaultValue);

	/** get the default speed value for this coordinate. This is the value 
	    used if no value has been set prior to a simulation. */
	double getDefaultSpeedValue() const { return get_default_speed_value(); }
	void setDefaultSpeedValue(double aDefaultSpeedValue) 
		{ upd_default_speed_value() = aDefaultSpeedValue; }

	/** get acceleration of the coordinate is dependent on having 
	    realized the model and state to the acceleration stage */
	double getAccelerationValue(const SimTK::State& s) const;

	/** determine or set whether or not the Coordinate is 
	    "clamped" between a range of values. */
	bool getClamped(const SimTK::State& s) const;
	void setClamped(SimTK::State& s, bool aLocked) const;
	/** get/set whether or not the Coordinate is clamped by default */
	bool getDefaultClamped() const { return get_clamped(); }
    void setDefaultClamped(bool aClamped ) { upd_clamped() = aClamped; }

	/** get the value for the Coordinate's range of motion */ 
	double getRangeMin() const {return get_range(0); }
	double getRangeMax() const {return get_range(1); }
	/** set the range with a double array of length 2 in order of
	    minimum and maximum coordinate values */
	void setRange(double aRange[2]);
	void setRangeMin(double aMin);
	void setRangeMax(double aMax);
	
	/** determine or set whether or not the Coordinate is 
	    "locked" for a given state of the Model. */
	bool getLocked(const SimTK::State& s) const;
	void setLocked(SimTK::State& s, bool aLocked) const;
	/** get/set whether or not the Coordinate is locked by default */
    bool getDefaultLocked() const { return get_locked(); }
    void setDefaultLocked(bool aLocked) { upd_locked() = aLocked; }

	/** determine or set whether or not the Coordinate is 
	    "prescribed" for a given state of the Model. */
	bool isPrescribed(const SimTK::State& s) const;
	void setIsPrescribed(SimTK::State& s, bool isPrescribed ) const;
	/** get/set whether or not the Coordinate is locked by default */
	bool getDefaultIsPrescribed() const {return get_prescribed();}
    void setDefaultIsPrescribed(bool isPrescribed ) {upd_prescribed() = isPrescribed;}
	/** Specify an OpenSim Function specifies the prescribed motion for this 
	    Coordinate as a function of time. Note, this function must provide
		valid first and second order derivatives. */       
	void setPrescribedFunction(const Function& function);
	const Function& getPrescribedFunction() const;
	
	/** Return true if coordinate is dependent on other coordinates via a coupler
	    constraint OR it has been flagged as free to change when satisfying 
		the model's kinematic constraints in general. */
	bool isDependent(const SimTK::State& s) const;

	/** Return true if coordinate is locked, prescribed, or dependent on other coordinates */
	bool isConstrained(const SimTK::State& s) const; 

	/** @name Advanced Access to underlying Simbody system resources */
	/**@{**/
	int getMobilizerQIndex() const { return _mobilizerQIndex; };
	SimTK::MobilizedBodyIndex getBodyIndex() const { return _bodyIndex; };


    // ModelComponent interface.
	Array<std::string> getStateVariableNames() const OVERRIDE_11;
	SimTK::SystemYIndex getStateVariableSystemIndex(const std::string &stateVariableName) const OVERRIDE_11;
	/**@}**/

	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	/** default contructor*/
	Coordinate();

	/** Convenience constructor */	
	Coordinate(const std::string &aName, MotionType aMotionType, 
		double defualtValue, double aRangeMin, double aRangeMax);	
	
	// Uses default (compiler-generated) destructor, copy constructor and copy 
    // assignment operator.

protected:
	// Only model should be invoking these ModelComponent interface methods.
    // Also see connectToModel() above.
	void connectToModel(Model& aModel) OVERRIDE_11;
    void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;
    void initStateFromProperties(SimTK::State& s) const OVERRIDE_11;
    void setPropertiesFromState(const SimTK::State& state) OVERRIDE_11;
	int getNumStateVariables() const  OVERRIDE_11 { return 2; }; // value and speed

	// Only the coordinate or the joint itself can specify the owner
	// of Coordinate
	void setJoint(const Joint& aOwningJoint);

//=============================================================================
// MODEL DATA
//=============================================================================
private:
	// All coordinates (Simbody mobility) have associated constraints that
	// perform joint locking, prescribed motion and range of motion.
	// Constraints are created upon setup: locked, precribedFunction
	// and range must be set.
	// NOTE: Changing the prescribed motion function requires topology to be realized
	//       so state is invalidated
	//       Enabling/disabling locking, prescribed motion or clamping is allowable 
	//       during a simulation.
	//       The last constraint to be set takes precedence.
	/** Indices for the constraint in Simbody. */
	SimTK::ConstraintIndex _prescribedConstraintIndex;
	SimTK::ConstraintIndex _lockedConstraintIndex;
	SimTK::ConstraintIndex _clampedConstraintIndex;

	/** MobilizedBodyIndex of the body which this coordinate serves.  */
	SimTK::MobilizedBodyIndex _bodyIndex;

	/** Mobilizer Q (i.e. genearlized coordinate) index for this Coordinate. */
	SimTK::MobilizerQIndex _mobilizerQIndex;

	/** Motion type (translational, rotational or combination). */
	MotionType _motionType;

	/** Simbody joint that owns this coordinate. */
	SimTK::ReferencePtr<const Joint> _joint;

	/** SimTK function used to lock the joint at a fixed value */
	mutable SimTK::ReferencePtr<ModifiableConstant> _lockFunction;

	mutable bool _lockedWarningGiven;

	// PRIVATE METHODS
	void constructProperties();

	friend class CoordinateCouplerConstraint; 
	friend class Joint; 

//=============================================================================
};	// END of class Coordinate
//=============================================================================
//=============================================================================
} // end of namespace OpenSim


#endif // OPENSIM_COORDINATE_H_


