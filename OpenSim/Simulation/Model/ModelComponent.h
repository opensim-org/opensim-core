#ifndef OPENSIM_MODEL_COMPONENT_H_
#define OPENSIM_MODEL_COMPONENT_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  ModelComponent.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2013 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Ayman Habib, Michael Sherman                         *
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

/** @file
 * This defines the abstract ModelComponent class, which is used to add 
 * computational components to the underlying SimTK::System (MultibodySystem). 
 * It specifies the interface that components must satisfy in order to be part 
 * of the system and provides a series of helper methods for adding variables 
 * (state, discrete, cache, ...) to the underlying system. As such, 
 * ModelComponent handles all of the bookkeeping for variable indices and  
 * provides convenience access via variable names.
 *
 * All OpenSim components: Bodies, Joints, Coordinates, Constraints, Forces,   
 * Actuators, Controllers, etc. and Model itself, are ModelComponents. Each  
 * component is composed of one or more underlying Simbody multibody system 
 * elements (MobilizedBody, Constraint, Force, or Measure) which are part of 
 * a SimTK::Subsystem and by default this is the System's DefaultSubsystem.
 * The SimTK::Subsystem is where new variables are allocated.
 */


// INCLUDES
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Component.h>
#include "Simbody.h"

namespace OpenSim {

class Model;
class ModelDisplayHints;


//==============================================================================
//                            MODEL COMPONENT
//==============================================================================
/**
 * This defines the abstract ModelComponent class, which is used to add computational
 * components to the underlying SimTK::System (MultibodySystem). It specifies
 * the interface that components must satisfy in order to be part of the system
 * and provides a series of helper methods for adding variables (state, discrete,
 * cache, ...) to the underlying system. As such, ModelComponent handles all of
 * the bookkeeping of system indices and provides convenience access to variable 
 * values via their names as strings. 
 *
 * The MultibodySystem and its State are defined by Simbody (ref ...). Briefly,
 * a System can be thought of the equations that define the mathematical behavior
 * of a model. The State is a collection of all the variables that uniquely
 * define the unknowns in the system equations. These would be joint coordinates,
 * their speeds, accelerations as well other variables that govern system dynamics 
 * (e.g. muscle activation and fiber-length variables that dictate muscle force). 
 * These variables are called continuous state variables in Simbody, but  are more
 * simply referred to as <em>StateVariables</em> in OpenSim.  ModelComponent provides 
 * services to define and access its StateVariables and specify their dynamics
 * (derivatives with respect to time) that are automatically and simultaneously
 * integrated with the MultibodySystem dynamics.
 *
 * There are other types of "State" variables such as a flag (or option) that 
 * enables a component to be disabled or for a muscle force to be overridden and 
 * and these are identified as <em>ModelingOptions</em> since they fundamentally 
 * change the modeled dynamics of the component. ModelComponent provides services
 * that enable developers of components to define additional ModelingOptions.
 *
 * Often a component requires input from an outside source (precomputed data from 
 * a file, another program, or interaction from a user) in which case these
 * variables do not have dynamics (differential eqns.) known to the component, but 
 * are necessary to describe the dynamical "state" of the system. An example, is
 * a throttle component (a "controller" that provides an actuator (e.g. a motor) with
 * a control signal (a voltage or current)) which it gets direct input from the user 
 * (via a joystick, key press, etc..). The throttle controls the motor torque output 
 * and therefore the behavior of the model. The input by the user to the throttle
 * the motor (the controls) is necessary to specify the model dynamics at any instant
 * and therefore are considered part of the State in Simbody as Discrete State Variables.  
 * In OpenSim they are simplify referred to as \e DiscreteVariables. The ModelComponent 
 * provides services to enable developers of components to define and access its 
 * DiscreteVariables.
 *
 * Fast and efficient simulations also require computationally expensive calculations
 * to be performed only when necessary. Often the result of a expensive calculation can
 * be reused many times over, while the variables it is dependent on remain fixed. The
 * concept of holding onto these values is called caching and the variables that hold
 * these values are call <em>CacheVariables</em>. It is important to note, that cache 
 * variables are not state variables. Cache variables can always be recomputed excactly
 * from the State. OpenSim uses the Simbody infrastructure to manage cache variables and
 * their validity. ModelComponent provides a simplified interface to define and 
 * access CacheVariables.
 *
 * Many modeling and simulation codes put the onus on users and component creators to
 * manage the validity of cache variables, which is likely to lead to undetectable 
 * errors where cache values are stale (calculated based on past state variable values).
 * Simbody, on the other hand, provides a more strict infrastructure to make it easy to
 * exploit the efficiencies of caching while reducing the risks of validity errors. 
 * To do this, Simbody employs the concept of computational stages. To "realize" a model's
 * system to a particular stage is to perform all the computations necessary to evaluate the 
 * cached quantities up to and including the stage specified. Simbody utilizes 
 * nine realization stages (<tt>SimTK::Stage::</tt>)
 *
 * -# \c Topology       finalize System with "slots" for most variables (above)
 * -# \c %Model         specify modeling choices
 * -# \c Instance       specify modifiable model parameters
 * -# \c Time           compute time dependent quantities
 * -# \c Position       compute position dependent quantities	
 * -# \c Velocity       compute velocity dependent quantities
 * -# \c Dynamics       compute system applied forces and dependent quantities	
 * -# \c Acceleration   compute system accelerations and all other derivatives
 * -# \c Report         compute quantities for reporting/output
 *  
 * The ModelComponent interface is automatically invoked by the System and its realizations.
 * Component users and most developers need not concern themselves with
 * \c Topology, \c %Model or \c Instance stages. That interaction is managed by ModelComponent 
 * when component creators implement addToSystem() and use the services provided ModelComponent.
 * Component creators do need to determine and specify stage dependencies for Discrete  
 * and CacheVariables that they add to their components. For example, the throttle 
 * controller reads its value from user input and it is valid for all calculations as
 * long as time does not change. If the simulation (via numerical integration) steps
 * forward (or backward for a trial step) and updates the state, the control from
 * a previous state (time) should be invalid and an error generated for trying to access
 * the DiscreteVariable for the control value. To do this one specifies the "invalidates" stage
 * (e.g. <tt>SimTK::Stage::Time</tt>) for a DiscreteVariable when the variable is added to
 * the ModelComponent. A subsequent change to that variable will invalidate all
 * state cache entries at that stage or higher. For example, if a DiscreteVariable is
 * declared to invalidate <tt>Stage::Position</tt> then changing it will 
 * invalidate cache entries that depend on positions, velocities, forces, and
 * accelerations.
 *
 * Similar principles apply to CacheVariables, which requires a "dependsOn" stage to 
 * be specified when a CacheVariable is added to the component. In this case, the cache
 * variable "shadows" the State (unlike a DiscreteVariable, which is a part
 * of the State) holding already-computed state-dependent values so that they
 * do not need to be recomputed until the state changes.
 * Accessing the CacheVariable in a State whose current stage is lower than 
 * that CacheVariable's specified dependsOn stage will trigger
 * an exception. It is up to the component to update the value of the cache variable.
 * ModelComponent provides methods to check if the cache is valid, update its value and 
 * mark as valid. 

 * All components: Bodies, Joints, Coordinates, Constraints, Forces, Controllers, 
 * and even Model itself, are ModelComponents. Each component is "connected" to a
 * SimTK::Subsystem and by default this is the System's DefaultSubsystem.
 *
 * The primary responsibility of a ModelComponent is to add its computational 
 * representation(s) to the underlying SimTK::System by implementing
 * addToSystem().
 *
 * Additional methods provide support for adding modeling options, state and
 * cache variables.
 *
 * Public methods enable access to component variables via their names.
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API ModelComponent : public Component {
OpenSim_DECLARE_ABSTRACT_OBJECT(ModelComponent, Component);

//==============================================================================
// METHODS
//==============================================================================
public:
    /** Default constructor **/
    ModelComponent();
    /** Construct ModelComponent from an XML file. **/
    ModelComponent(const std::string& aFileName, 
                   bool aUpdateFromXMLNode = true) SWIG_DECLARE_EXCEPTION;
    /** Construct ModelComponent from a specific node in an XML document. **/
    explicit ModelComponent(SimTK::Xml::Element& aNode);
    /** Construct ModelComponent with its contents copied from another 
    ModelComponent; this is a deep copy so nothing is shared with the 
    source after the copy. **/
    ModelComponent(const ModelComponent& source);
    /** Destructor is virtual to allow concrete model component cleanup. **/
    virtual ~ModelComponent() {}

#ifndef SWIG
    /** Assignment operator to copy contents of an existing component */
    ModelComponent& operator=(const ModelComponent &aModelComponent);
#endif

    /**
     * Get a const reference to the Model this component is part of.
     */
    const Model& getModel() const;
    /**
     * Get a modifiable reference to the Model this component is part of.
     */
    Model& updModel();

    /**
     * In case the ModelComponent has a visual representation (VisualObject), override this method  
     * to update it. This is typically done by recomputing anchor points and positions based 
     * on transforms obtained from current state.
     */
    virtual void updateDisplayer(const SimTK::State& s) const {};


    // End of Model Component State Accessors.
    //@} 

protected:
template <class T> friend class ModelComponentSet;
// Give the ModelComponentMeasure access to the realize() methods.
template <class T> friend class ModelComponentMeasure;

    /** @name           ModelComponent Basic Interface
    The interface ensures that deserialization, resolution of inter-connections,
    and handling of dependencies are performed systematically and prior to 
    system creation, followed by allocation of necessary System resources. These 
    methods are virtual and may be implemented by subclasses of 
    ModelComponents. 
    
    @note Every implementation of virtual method xxx(args) must begin
    with the line "Super::xxx(args);" to ensure that the parent class methods
    execute before the child class method, starting with ModelComponent::xxx()
    and going down. 
    
    The base class implementations here do two things: (1) take care of any
    needs of the %ModelComponent base class itself, and then (2) ensure that the 
    corresponding calls are made to any subcomponents that have been specified 
    by derived %ModelComponent objects, via calls to the 
    includeAsSubComponent() method. So assuming that your concrete 
    %ModelComponent and all intermediate classes from which it derives properly 
    follow the requirement of calling the Super class method first, the order of
    operations enforced here for a call to a single method will be
      -# %ModelComponent base class computations
      -# calls to that same method for \e all subcomponents
      -# calls to that same method for intermediate %ModelComponent-derived 
         objects' computations, in order down from %ModelComponent, and
      -# finally a call to that method for the bottom-level concrete class. 

    You should consider this ordering when designing a %ModelComponent. In 
    particular the fact that all your subcomponents will be invoked before you
    are may be surprising. **/ 
    //@{

    /** Perform any necessary initializations required to connect the 
    component into the Model, and check for error conditions. connectToModel() 
    is invoked on all components to complete construction of a Model, prior to
    creating a Simbody System to represent it computationally. It may also be
    invoked at times just for its error-checking side effects.
    
    If you override this method, be sure to invoke the base class method first, 
    using code like this:
    @code
    void MyComponent::connectToModel(Model& model) {
        Super::connectToModel(model); // invoke parent class method
        // ... your code goes here
    }
    @endcode

    Note that this method is expected to check for modeling errors and should
    throw an OpenSim::Exception if there is something wrong. For example, if
    your model component references another object by name, you should verify
    that it exists in the supplied Model, which is not guaranteed since 
    components may be independently instantiated or constructed from XML files.

    @param[in,out]  model   The Model currently being constructed to which this
                            %ModelComponent should be connected. **/
    virtual void connectToModel(Model& model);



    /** Optional method for generating arbitrary display geometry that reflects
    this %ModelComponent at the specified \a state. This will be called once to 
    obtain ground- and body-fixed geometry (with \a fixed=\c true), and then 
    once per frame (with \a fixed=\c false) to generate on-the-fly geometry such
    as rubber band lines, force arrows, labels, or debugging aids.
  
    If you override this method, be sure to invoke the base class method first, 
    using code like this:
    @code
    void MyComponent::generateDecorations
       (bool                                        fixed, 
        const ModelDisplayHints&                    hints,
        const SimTK::State&                         state,
        SimTK::Array_<SimTK::DecorativeGeometry>&   appendToThis) const
    {
        // invoke parent class method
        Super::generateDecorations(fixed,hints,state,appendToThis); 
        // ... your code goes here
    }
    @endcode

    @param[in]      fixed   
        If \c true, generate only geometry that is independent of time, 
        configuration, and velocity. Otherwise generate only such dependent 
        geometry.
    @param[in]      hints   
        See documentation for ModelDisplayHints; you may want to alter the 
        geometry you generate depending on what you find there. For example, 
        you can determine whether the user wants to see debug geometry.
    @param[in]      state
        The State for which geometry should be produced. See below for more
        information.
    @param[in,out]  appendToThis
        %Array to which generated geometry should be \e appended via the
        \c push_back() method.

    When called with \a fixed=\c true only modeling options and parameters 
    (Instance variables) should affect geometry; time, position, and velocity
    should not. In that case OpenSim will already have realized the \a state
    through Instance stage. When called with \a fixed=\c false, you may 
    consult any relevant value in \a state. However, to avoid unnecessary
    computation, OpenSim guarantees only that \a state will have been realized
    through Position stage; if you need anything higher than that (reaction 
    forces, for example) you should make sure the \a state is realized through 
    Acceleration stage. **/
    virtual void generateDecorations
       (bool                                        fixed, 
        const ModelDisplayHints&                    hints,
        const SimTK::State&                         state,
        SimTK::Array_<SimTK::DecorativeGeometry>&   appendToThis) const;

    // End of Model Component Basic Interface (protected virtuals).
    //@} 

    /** @name     ModelComponent System Creation and Access Methods
     * These methods support implementing concrete ModelComponents. Add methods
     * can only be called inside of addToSystem() and are useful for creating
     * the underlying SimTK::System level variables that are used for computing
     * values of interest.
     * @warning Accessors for System indices are intended for component internal use only.
     **/

    //@{



    // End of System Creation and Access Methods.
    //@} 

private:

	// Satisfy the general Component interface, but this is not part of the
	// ModelComponent interface. connectToModel() ensures that any connect()
	// operations on the parent component are invoked.
	void ModelComponent::connect() OVERRIDE_11;

    const SimTK::DefaultSystemSubsystem& getDefaultSubsystem() const;
    const SimTK::DefaultSystemSubsystem& updDefaultSubsystem();


    // Clear out all the data fields in the base class. There should be one
    // line here for each data member below.
    void setNull() {
        _model = NULL;
    }
    
protected:
    /** The model this component belongs to. */
    // TODO: this should be private; all components should use getModel()
    // and updModel() to get access. This is just a reference; don't delete!
    Model* _model;


//==============================================================================
};	// END of class ModelComponent
//==============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_MODEL_COMPONENT_H_

