#ifndef OPENSIM_MODEL_COMPONENT_H_
#define OPENSIM_MODEL_COMPONENT_H_

// ModelComponent.h
// Authors: Ajay Seth, Peter Eastman, Ayman Habib, Michael Sherman
/*
 * Copyright (c) 2009-12, Stanford University. All rights reserved.
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** @file
 * This defines the abstract ModelComponent class, which is used to add computational
 * components to the underlying SimTK::System (MultibodySystem). It specifies
 * the interface that components must satisfy in order to be part of the system
 * and provides a series of helper methods for adding variables (state, discrete,
 * cache, ...) to the underlying system. As such, ModelComponent handles all of
 * the bookkeeping for variable indices and provides conveniece access via 
 * variable names.
 *
 * All components, Bodies, Joints, Coordinates, Constraints, Forces, Controllers, .. 
 * and even Model itself, are ModelComponents. Each component is "connected" to a
 * SimTK::Subsystem and by default this is the System's DefaultSubsystem.
 */


// INCLUDES
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include "OpenSim/Common/Object.h"
#include "SimTKsimbody.h"

namespace OpenSim {

class Model;
class ModelDisplayHints;
class ModelComponent;


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
 * the motor (the controls) is necesary to specify the model dynamics at any instant
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
 * system a particular stage is to perform all the computations necessary to evaluate the 
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
 * when component creators implement createSystem() and use the services provided ModelComponent.
 * Component creators do need to determine and specify stage dependencies for Discrete  
 * and CacheVariables that they add to their components. For example, the throttle 
 * controller reads its value from user input and it is valid for all calculations as
 * long as time does not change. If the simulation (via numerical integration) steps
 * forward (or backward for a trial step) and updates the state, the control from
 * a previous state (time) should be invalid and an error generated for trying to access
 * the DiscreteVariable for the control value. To do this one specifies the "dependsOn" stage
 * (e.g. <tt>SimTK::Stage::Time</tt>) for a DiscreteVariable when the variable is added to
 * the ModelComponent. If the control is fixed (not changing with time) then the lowest
 * valid stage could just be Instance.
 *
 * Similar principles apply to CacheVariables, which requires a "lowestValid" stage to 
 * be specified when a CacheVariable is added to the component. In this case, the cache
 * variable augments the State (unlike a discrete state variable, which is a part
 * of the State) for the convenience of not recomputing a value unless necesary.
 * Accessing the cache variable at a stage lower than the lowestValid specified will trigger
 * an exception. It is up to the component to update the value of the cache variable.
 * ModelComponent provides methods to check if the cache is valid, update its value and 
 * mark as valid. 

 * All components: Bodies, Joints, Coordinates, Constraints, Forces, Controllers, .. 
 * and even Model itself, are ModelComponents. Each component is "connected" to a
 * SimTK::Subsystem and by default this is the System's DefaultSubsystem.
 *
 * The primary responsibility of a ModelComponent is to add its computational 
 * representation(s) to the underlying SimTK::System by implementing
 * createSystem().
 *
 * Additional methods provide support for adding modeling options, state and
 * cache variables.
 *
 * Public methods enable access to component variables via their names.
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API ModelComponent : public Object {
OpenSim_DECLARE_ABSTRACT_OBJECT(ModelComponent, Object);

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
    virtual void updateDisplayer(const SimTK::State& s) {};

    /**
     * Get the number of "Continuous" state variables maintained by the ModelComponent
     * and its specified subcomponents
     */
    virtual int getNumStateVariables() const;

    /**
     * Get the names of "continuous" state variables maintained by the ModelComponent
     * and its subcomponents
     */
    virtual Array<std::string> getStateVariableNames() const;

   /**
     * Get the System Index of a state variable allocated by this ModelComponent.  
     * Returns an InvalidIndex if no state variable with the name provided is found.
     * @param stateVariableName   the name of the state variable 
     */
    virtual SimTK::SystemYIndex getStateVariableSystemIndex(const std::string& stateVariableName) const;

   /** @name ModelComponent State Access methods
     * Get and set modeling option, state, discrete and/or cache variables in the State
     */ 
    //@{
    
    /**
     * Get a ModelingOption flag for this ModelComponent by name.
     * The flag is an integer corresponding to the index of modelingOptionNames used 
     * add the modeling option to the component. @see addModelingOption
     *
     * @param state  the State in which to set the modeling option
     * @param name   the name (string) of the modeling option of interest
     * @return flag  integer value for modeling option
     */
    int getModelingOption(const SimTK::State& state, const std::string& name) const;

    /**
     * Set the value of a ModelingOption flag for this ModelComponent.
     * if the integer value exceeds the number of option names used to
     * define the options, an exception is thrown.
     *
     * @param state  the State in which to set the flag
     * @param name   the name (string) of the modeling option of interest
     * @param flag   the desired flag (integer) value specifying the modeling option
     */
    void setModelingOption(SimTK::State& state, const std::string& name, int flag) const;

    /**
     * Get the value of a state variable allocated by this ModelComponent.
     *
     * @param state   the State for which to get the value
     * @param name    the name (string) of the state variable of interest
     */
    double getStateVariable(const SimTK::State& state, const std::string& name) const;

    /**
     * Set the value of a state variable allocated by this ModelComponent by name.
     *
     * @param state  the State for which to set the value
     * @param name   the name of the state variable
     * @param value  the value to set
     */
    void setStateVariable(SimTK::State& state, const std::string& name, double value) const;

    /**
     * Get the value of a discrete variable allocated by this ModelComponent by name.
     *
     * @param state   the State for which to set the value
     * @param name    the name of the state variable
     * @return value  the discrete variable value
     */
    double getDiscreteVariable(const SimTK::State& state, const std::string& name) const;

    /**
     * Set the value of a discrete variable allocated by this ModelComponent by name.
     *
     * @param state  the State for which to set the value
     * @param name   the name of the dsicrete variable
     * @param value  the value to set
     */
    void setDiscreteVariable(SimTK::State& state, const std::string& name, double value) const;

    /**
     * Get the value of a cache variable allocated by this ModelComponent by name.
     *
     * @param state  the State for which to set the value
     * @param name   the name of the cache variable
     * @return T	 const reference to the cache variable's value
     */
    template<typename T> const T& 
    getCacheVariable(const SimTK::State& state, const std::string& name) const
    {
        std::map<std::string, CacheInfo>::const_iterator it;
        it = _namedCacheVariableInfo.find(name);

        if(it != _namedCacheVariableInfo.end()) {
            SimTK::CacheEntryIndex ceIndex = it->second.index;
            return SimTK::Value<T>::downcast(
                getDefaultSubsystem().getCacheEntry(state, ceIndex)).get();
        } else {
            std::stringstream msg;
            msg << "ModelComponent::getCacheVariable: ERR- name not found.\n "
                << "for component '"<< getName() << "' of type " 
                << getConcreteClassName();
            throw Exception(msg.str(),__FILE__,__LINE__);
        }
    }
    /**
     * Obtain a writable cache variable value allocated by this ModelComponent by name.
     * Do not forget to mark the cache value as valid after updating, otherwise it will
     * force a reevaluation if the evaluation method is monitoring the validity of the
     * cache value.
     *
     * @param state  the State for which to set the value
     * @param name   the name of the state variable
     * @return value modifiable reference to the cache variable's value
     */
    template<typename T> T& 
    updCacheVariable(const SimTK::State& state, const std::string& name) const
    {
        std::map<std::string, CacheInfo>::const_iterator it;
        it = _namedCacheVariableInfo.find(name);

        if(it != _namedCacheVariableInfo.end()) {
            SimTK::CacheEntryIndex ceIndex = it->second.index;
            return SimTK::Value<T>::downcast(
                getDefaultSubsystem().updCacheEntry(state, ceIndex)).upd();
        }
        else{
            std::stringstream msg;
            msg << "ModelComponent::updCacheVariable: ERR- '" << name 
                << "' name not found.\n "
                << "for component '"<< getName() << "' of type " 
                << getConcreteClassName();
            throw Exception(msg.str(),__FILE__,__LINE__);
        }
    }

    /**
     * After updating a cache variable value allocated by this ModelComponent, you can
     * mark its value as valid, which will not change until the realization stage falls
     * below the minimum set at the time the cache variable was created. If not marked
     * as valid, the evaluation method monitoring this flag will force a re-evaluation
     * rather that just reading the value from the cache.
     *
     * @param state  the State containing the cache variable
     * @param name   the name of the cache variable
     */
    void markCacheVariableValid(const SimTK::State& state, const std::string& name) const
    {
        std::map<std::string, CacheInfo>::const_iterator it;
        it = _namedCacheVariableInfo.find(name);

        if(it != _namedCacheVariableInfo.end()) {
            SimTK::CacheEntryIndex ceIndex = it->second.index;
            getDefaultSubsystem().markCacheValueRealized(state, ceIndex);
        }
        else{
            std::stringstream msg;
            msg << "ModelComponent::markCacheVariableValid: ERR- name not found.\n "
                << "for component '"<< getName() << "' of type " 
                << getConcreteClassName();
            throw Exception(msg.str(),__FILE__,__LINE__);
        }
    }

    /**
     * Mark a cache variable value allocated by this ModelComponent as invalid.
     * When the system realization drops to below the lowest valid stage, cache 
     * variables are automatically marked as invalid. There are instances when
     * component added state variables require invalidating a cache at a lower stage.
     * For example, a component may have a length state variable which should 
     * invalidate calculations involving it and other positions when the state
     * variable is set. Changing the component state variable automatically
     * invalidates Dynamics and higher realizations, but to force realizations
     * at Position and Velocity requires setting the lowest valid stage to 
     * Position and marking the cache variable as invalid whenver the length
     * state variable value is set/changed.
     *
     * @param state  the State containing the cache variable
     * @param name   the name of the cache variable
     */
    void markCacheVariableInvalid(const SimTK::State& state, const std::string& name) const
    {
        std::map<std::string, CacheInfo>::const_iterator it;
        it = _namedCacheVariableInfo.find(name);

        if(it != _namedCacheVariableInfo.end()) {
            SimTK::CacheEntryIndex ceIndex = it->second.index;
            getDefaultSubsystem().markCacheValueNotRealized(state, ceIndex);
        }
        else{
            std::stringstream msg;
            msg << "ModelComponent::markCacheVariableInvalid: ERR- name not found.\n "
                << "for component '"<< getName() << "' of type " 
                << getConcreteClassName();
            throw Exception(msg.str(),__FILE__,__LINE__);
        }
    }

    /**
     * Enables the to monitor the validity of the cache variable value using the
     * returned flag. For components performing a costly evaluation, use this method to 
     * force a re-evaluation cache variable value only when necessary (returns false).
     *
     * @param state  the State in which the cache value resides
     * @param name   the name of the cache variable
     * @return bool  whether the cache variable value is valid or not
     */
    bool isCacheVariableValid(const SimTK::State& state, const std::string& name) const
    {
        std::map<std::string, CacheInfo>::const_iterator it;
        it = _namedCacheVariableInfo.find(name);

        if(it != _namedCacheVariableInfo.end()) {
            SimTK::CacheEntryIndex ceIndex = it->second.index;
            return getDefaultSubsystem().isCacheValueRealized(state, ceIndex);
        }
        else{
            std::stringstream msg;
            msg << "ModelComponent::isCacheVariableValid: ERR- name not found.\n "
                << "for component '"<< getName() << "' of type " 
                << getConcreteClassName();
            throw Exception(msg.str(),__FILE__,__LINE__);
        }
    }

    /**
     *  Set cache variable value allocated by this ModelComponent by name.
     *  All cache entries are lazily evaluated (on a need basis) so a set
     *  also marks the cache as valid.
     *
     * @param state  the State for which to set the value
     * @param name   the name of the state variable
     * @param value  the variable's cache new value to set
     */
    template<typename T> void 
    setCacheVariable(const SimTK::State& state, const std::string& name, T &value) const
    {
        std::map<std::string, CacheInfo>::const_iterator it;
        it = _namedCacheVariableInfo.find(name);

        if(it != _namedCacheVariableInfo.end()) {
            SimTK::CacheEntryIndex ceIndex = it->second.index;
            SimTK::Value<T>::downcast(
                getDefaultSubsystem().updCacheEntry( state, ceIndex)).upd() 
                = value;
            getDefaultSubsystem().markCacheValueRealized(state, ceIndex);
        }
        else{
            std::stringstream msg;
            msg << "ModelComponent::setCacheVariable: ERR- name not found.\n "
                << "for component '"<< getName() << "' of type " 
                << getConcreteClassName();
            throw Exception(msg.str(),__FILE__,__LINE__);
        }	
    }
    // End of Model Component State Accessors.
    //@} 

protected:
template <class T> friend class ModelComponentSet;

    /** @name               ModelComponent Protected Interface
    The interface ensures that deserialization, resolution of inter-connections 
    and dependencies, are performed systematically and prior to system 
    creation, followed by allocation of necessary System resouces. These 
    methods are virtual and must be implemented by subclasses of 
    ModelComponents. Every implementation of virtual method xxx(args) must begin
    with the line "Super::xxx(args);" to ensure that the parent class methods
    execute before the child class method, starting with ModelComponent::xxx()
    and going down. **/ 
    //@{

    /**
     * setup() is automatically called on all components prior to system creation. 
     * Set the Model this component is part of and then perform any necessary initialization,
     * such as looking up references to other objects in the Model (to verify that they exist,
     * which is not guaranteed when a component is instantiated or constructed from XML).
     * Override this method as necessary but always call the parent class setup to
     * ensure all members (including those defined by the parent class) are initialized.
     * @param model   the model this component is a part of
     */
    virtual void setup(Model& model);

    /**
     * createSystem() is called when the SimTK::System is being created for the Model.  It must be 
     * implemented in order to add appropriate SimTK elements to the System corresponding to this component. 
     * Helper methods for adding modeling option, state variables and their derivatives, discrete variables,
     * etc., are available and can be called within createSystem() only.
     * @see addModelingOption()
     * @see addStateVariable()
     * @see addDiscreteVariables()
     * @see addCacheVariable()
     *
     * @param system   the System being created
     */
    virtual void createSystem(SimTK::MultibodySystem& system) const;

    /**
     * This is called after a SimTK::System and State have been created for the Model.  It is
     * implemented to set initial values of state variables from defaults (typical) or by  
     * any calculation that is not dependent on the state (i.e. function of property values).
     * @param state    the State to initialize
     */
    virtual void initState(SimTK::State& state) const {
        for(unsigned int i=0; i < _subComponents.size(); i++)
            _subComponents[i]->initState(state);
    };

    /**
     * Assign new default values for this component to match those in a specified State.  It must be
     * implemented/overriden to set default values to states defined by each subclass. @note Defaults 
     * are usually properties of the component, and these properties can be updated to match an
     * existing state. Thus, state variable values can persist as part of the model component and be
     * serialized as a property.
     *
     * @param state    the State from which the default values are obtained (calculated) for this component
     */
    virtual void setDefaultsFromState(const SimTK::State& state) {
        for(unsigned int i=0; i < _subComponents.size(); i++)
            _subComponents[i]->setDefaultsFromState(state);
    };

    /** If a model component has allocated any continuous state variables
    using the addStateVariable() method, then computeStateVariableDerivatives()
    must be implemented to provide time derivatives for those states.
    Override to return a Vector of the same size as the number of state 
    variables defined and in order of getStateVariableNames(). Default returns 
    empty (no derivatives are defined). 
    TODO: this API does not work in general; need a way to incorporate variables
    from anywhere in the class hierarchy without coordinating the 
    implementations.
    **/
    virtual SimTK::Vector computeStateVariableDerivatives(const SimTK::State& s) const
    {   return SimTK::Vector(0); };

    /** Optional method for generating arbitrary display geometry that reflects
    this %ModelComponent at the specified \a state. This will be called once to 
    obtain ground- and body-fixed geometry (with \a fixed=\c true), and then 
    once per frame (with \a fixed=\c false) to generate on-the-fly geometry such
    as rubber band lines, force arrows, labels, or debugging aids.

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
        SimTK::Array_<SimTK::DecorativeGeometry>&   appendToThis) const {}

    // End of Model Component Interface (protected virtuals).
    //@} 

    /** @name ModelComponent System Creation and Access Methods
     * These methods support implementing concrete ModelComponents. Add methods
     * can only be called inside of createSystem() and are useful for creating
     * the underlying SimTK::System level variables that are used for computing
     * values of interest.
     * @warning Accessors for System indices are intended for component internal use only.
     **/

    //@{

    /**
     * Include another ModelComponent as a Subcomponent of this ModelComponent.
     * ModelComponent methods (e.g. createSystem(), initState(), ...) are therefore
     * invoked on Subcomponents when called on the parent. Realization is also
     * performed automatically on subcomponents. This ModelComponent does not
     * take ownership of the subcomponent.
     */
    void includeAsSubComponent(ModelComponent *aComponent);

    /** Add a modeling option (integer flag stored in the State) for use by 
    this ModelComponent. Each modeling option is identified by its own 
    \a optionName, specified here. Modeling options enable the model
    component to be configured differently in order to represent different 
    operating modes. For example, if two modes of operation are necessary (mode
    off and mode on) then specify optionName, "mode" with maxFlagValue = 1. 
    Subsequent gets will return 0 or 1 and set will only accept 0 and 1 as 
    acceptable values. Changing the value of a model option invalidates 
    Stage::Instance and above in the State, meaning all calculations involving
    time, positions, velocity, and forces are invalidated. **/
    void addModelingOption(const std::string&  optionName, 
                           int                 maxFlagValue) const;


    /** Add a continuous system state variable belonging to this ModelComponent,
    and assign a name by which to refer to it. Changing the value of this state 
    variable will automatically invalidate everything at and above its
    \a invalidatesStage, which is normally Stage::Dynamics meaning that there
    are forces that depend on this variable. If you define one or more
    of these variables you must also override computeStateVariableDerivatives()
    to provide time derivatives for them. **/
    void addStateVariable
       (const std::string&  stateVariableName,
        SimTK::Stage        invalidatesStage=SimTK::Stage::Dynamics) const;


    /** Add a system discrete variable belonging to this ModelComponent, give
    it a name by which it can be referenced, and declare the lowest Stage that
    should be invalidated if this variable's value is changed. **/
    void addDiscreteVariable(const std::string& discreteVariableName,
                             SimTK::Stage       invalidatesStage) const;

    /** Add a state cache entry belonging to this ModelComponent to hold
    calculated values that must be automatically invalidated when certain 
    state values change. Cache entries contain values whose computations depend
    on state variables and provide convenience and/or efficiency by holding on 
    to them in memory (cache) to avoid recomputation. Once the state changes, 
    the cache values automatically become invalid and has to be
    recomputed based on the current state before it can be referenced again.
    Any attempt to reference an invalid cache entry results in an exception
    being thrown.

    Cache entry validity is managed by computation Stage, rather than by 
    dependence on individual state variables. Changing a variables whose
    "invalidates" stage is the same or lower as the one specified as the
    "depends on" stage here cause the cache entry to be invalidated. For 
    example, a body's momementum, which is dependent on position and velocity 
    states, should have Stage::Velocity as its \a dependsOnStage. Then if a
    Velocity stage variable or lower (e.g. Position stage) changes, then the 
    cache is invalidated. But, if a Dynamics stage variable (or above) is 
    changed, the velocity remains valid so the cache entry does not have to be 
    recomputed.

    @param[in]      cacheVariableName
        The name you are assigning to this cache entry. Must be unique within
        this model component.
    @param[in]      variablePrototype	
        An object defining the type of value, and a default value of that type,
        to be held in this cache entry. Can be a simple int or an elaborate
        class, as long as it has deep copy semantics.
    @param[in]      dependsOnStage		
        This is the highest computational stage on which this cache entry's
        value computation depends. State changes at this level or lower will
        invalidate the cache entry. **/ 
    template <class T> void 
    addCacheVariable(const std::string&     cacheVariableName,
                     const T&               variablePrototype, 
                     SimTK::Stage           dependsOnStage) const
    {
        // Note, cache index is invalid until the actual allocation occurs 
        // during realizeTopology.
        _namedCacheVariableInfo[cacheVariableName] = 
            CacheInfo(new SimTK::Value<T>(variablePrototype), dependsOnStage);
    }

    /** Get the index of a ModelComponent's continuous state variable in the Subsystem for
        allocations. This method is intended for derived ModelComponents that may need direct
        access to its underlying Subsystem.*/
    const int getStateIndex(const std::string& name) const;

    /** Get the index of a ModelComponent's discrete variable in the Subsystem for allocations.
        This method is intended for derived ModelComponents that may need direct access
        to its underlying Subsystem.*/
    const SimTK::DiscreteVariableIndex 
    getDiscreteVariableIndex(const std::string& name) const;

    /** Get the index of a ModelComponent's cache variable in the Subsystem for allocations.
        This method is intended for derived ModelComponents that may need direct access
        to its underlying Subsystem.*/
    const SimTK::CacheEntryIndex 
    getCacheVariableIndex(const std::string& name) const;

    // End of System Creation and Access Methods.
    //@} 

private:
// Give the ModelComponentMeasure access to realizeTopology() and
// realizeAcceleration();
template <class T> friend class ModelComponentMeasure;

    // Get the number of continuous states that the ModelComponent added to 
    // the underlying computational system. It does not include the number of 
    // states already built-in by the SimTK::System level component. Should 
    // query it and add to this to obtain the total number of states managed by 
    // this ModelComponent.
    int getNumStateVariablesAddedByModelComponent() const 
    {   return _namedStateVariableInfo.size(); }
    Array<std::string> getStateVariablesNamesAddedByModelComponent() const;

    const SimTK::DefaultSystemSubsystem& getDefaultSubsystem() const;
    const SimTK::DefaultSystemSubsystem& updDefaultSubsystem();

    // We only need a call at Topology stage (to allocate state variables
    // and cache entries) and Acceleration stage (to ask the component to
    // calculate state derivatives). These are invoked by the model component's
    // measure.
    void realizeTopology(SimTK::State& s) const;
    void realizeAcceleration(const SimTK::State& s) const;

    void clearStateAllocations() {
        _namedModelingOptionInfo.clear();
        _namedStateVariableInfo.clear();
        _namedDiscreteVariableInfo.clear();
        _namedCacheVariableInfo.clear();
    }

    // Clear out all the data fields in the base class. There should be one
    // line here for each data member below.
    void setNull() {
        _model = NULL;
        _subComponents.clear();
        _simTKcomponentIndex.invalidate();
        clearStateAllocations();
    }
    
protected:
    /** The model this component belongs to. */
    // TODO: this should be private; all components should use getModel()
    // and updModel() to get access. This is just a reference; don't delete!
    Model* _model;

private:
    // Maintain pointers to subcomponents so we can invoke them automatically.
    // These are just references, don't delete them!
    // TODO: this feature is not completely implemented; should it 
    // exist at all? (sherm)
    SimTK::Array_<ModelComponent *>     _subComponents;

    // Underlying SimTK custom measure ModelComponentMeasure, which implements
    // the realizations in the subsystem by calling private concrete methods on
    // the ModelComponent. Every model component has one of these, allocated
    // in its addToSystem() method, and placed in the System's default subsystem.
    SimTK::MeasureIndex                 _simTKcomponentIndex;

    // Structure to hold modeling option information. Modeling options are
    // integers 0..maxOptionValue. At run time we keep them in a Simbody
    // discrete state variable that invalidates Model stage if changed.
    struct ModelingOptionInfo {
        ModelingOptionInfo() : maxOptionValue(-1) {}
        explicit ModelingOptionInfo(int maxOptVal) 
        :   maxOptionValue(maxOptVal) {}
        // Model
        int                             maxOptionValue;
        // System
        SimTK::DiscreteVariableIndex    index;
    };

    // Structure to hold related info about state variables 
    struct StateVariableInfo {
        StateVariableInfo() : index(SimTK::InvalidIndex) {}
        explicit StateVariableInfo(SimTK::Stage invalidates) 
        :   invalidatesStage(invalidates), index(SimTK::InvalidIndex) {}
        // Model
        SimTK::Stage    invalidatesStage;
        // System (can be QIndex, UIndex, or Zindex type)
        int             index;
    };

    // Structure to hold related info about discrete variables 
    struct DiscreteVariableInfo {
        DiscreteVariableInfo() {}
        explicit DiscreteVariableInfo(SimTK::Stage invalidates)
        :   invalidatesStage(invalidates) {}
        // Model
        SimTK::Stage                    invalidatesStage;
        // System
        SimTK::DiscreteVariableIndex    index;
    };

    // Structure to hold related info about cache variables 
    struct CacheInfo {
        CacheInfo() {}
        CacheInfo(SimTK::AbstractValue* proto,
                  SimTK::Stage          dependsOn)
        :   prototype(proto), dependsOnStage(dependsOn) {}
        // Model
        SimTK::ClonePtr<SimTK::AbstractValue>   prototype;
        SimTK::Stage                            dependsOnStage;
        // System
        SimTK::CacheEntryIndex                  index;
    };


    // Map names of modeling options for the ModelComponent to their underlying
    // SimTK indices.
    // TODO: these are mutable here so they can be modified in addToSystem()
    // but that is an API bug -- these are just part of the Model and should
    // be added much earlier (like constructProperties()?).
    mutable std::map<std::string, ModelingOptionInfo>   _namedModelingOptionInfo;
    // Map names of continuous state variables of the ModelComponent to their 
    // underlying SimTK indices.
    mutable std::map<std::string, StateVariableInfo>    _namedStateVariableInfo;
    // Map names of discrete variables of the ModelComponent to their underlying
    // SimTK indices.
    mutable std::map<std::string, DiscreteVariableInfo> _namedDiscreteVariableInfo;
    // Map names of cache entries of the ModelComponent to their individual 
    // cache information.
    mutable std::map<std::string, CacheInfo>            _namedCacheVariableInfo;

//==============================================================================
};	// END of class ModelComponent
//==============================================================================
//=============================================================================



//==============================================================================
//                         MODEL COMPONENT MEASURE
//==============================================================================
/** @cond **/ // hide from Doxygen

// Every OpenSim::ModelComponent is associated with a Simbody Measure of type
// ModelComponentMeasure defined here. This provides a full set of realize()
// methods for performing computations with System resources that are maintained
// at the ModelComponent base level, such as calculating state derivatives.

template <class T>
class ModelComponentMeasure : public SimTK::Measure_<T> {
public:
    SimTK_MEASURE_HANDLE_PREAMBLE(ModelComponentMeasure, SimTK::Measure_<T>);

    ModelComponentMeasure(SimTK::Subsystem& sub, 
                          const OpenSim::ModelComponent& mc)
    :   Measure_<T>(sub, new Implementation(mc), 
                    AbstractMeasure::SetHandle()) {}

    SimTK_MEASURE_HANDLE_POSTSCRIPT(ModelComponentMeasure, SimTK::Measure_<T>);
};

template <class T>
class ModelComponentMeasure<T>::Implementation 
:   public SimTK::Measure_<T>::Implementation {
public:
    // Don't allocate a value cache entry since this measure's value is
    // just a dummy.
    explicit Implementation(const ModelComponent& mc)
    :   SimTK::Measure_<T>::Implementation(0), _modelComponent(mc) {}

    // Implementations of Measure_<T>::Implementation virtual methods.

    Implementation* cloneVirtual() const FINAL_11
    {   return new Implementation(*this); }

    int getNumTimeDerivativesVirtual() const FINAL_11 {return 0;}
    SimTK::Stage getDependsOnStageVirtual(int order) const FINAL_11
    {   return SimTK::Stage::Empty; }
       
    const T& getUncachedValueVirtual
       (const SimTK::State& s, int derivOrder) const FINAL_11
    {   return this->getValueZero(); }

    void realizeMeasureTopologyVirtual(SimTK::State& s) const FINAL_11
    {   _modelComponent.realizeTopology(s); }
    void realizeMeasureAccelerationVirtual(const SimTK::State& s) const FINAL_11
    {   _modelComponent.realizeAcceleration(s); }

private:
    const ModelComponent& _modelComponent;
};

/** @endcond **/


} // end of namespace OpenSim

#endif // OPENSIM_MODEL_COMPONENT_H_

