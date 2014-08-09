#ifndef OPENSIM_COMPONENT_H_
#define OPENSIM_COMPONENT_H_
/* -------------------------------------------------------------------------- *
 *                             OpenSim: Component.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2014 Stanford University and the Authors                *
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

/** @file
 * This file defines the abstract Component class, which is used to add 
 * computational components to the underlying SimTK::System (MultibodySystem). 
 * It specifies the interface that components must satisfy in order to be part 
 * of the system and provides a series of helper methods for adding variables 
 * (state, discrete, cache, ...) to the underlying system. As such, 
 * Component handles all of the bookkeeping for variable indices and  
 * provides convenience access via variable names. Furthermore, a component
 * embodies constructs of inputs, outputs and connections that are useful in
 * composing computational systems.
 *
 * A component may contain one or more underlying Simbody multibody system 
 * elements (MobilizedBody, Constraint, Force, or Measure) which are part of 
 * a SimTK::Subsystem. A SimTK::Subsystem is where new variables are 
 * allocated. A Component, by default, uses the System's DefaultSubsystem.
 * for allocating new variables.
 */

// INCLUDES
#include <OpenSim/Common/osimCommonDLL.h>
#include "OpenSim/Common/Object.h"
#include "OpenSim/Common/ComponentConnector.h"
#include "OpenSim/Common/ComponentOutput.h"
#include "Simbody.h"
#include <functional>
#include <memory>

namespace OpenSim {

//==============================================================================
//                            OPENSIM COMPONENT
//==============================================================================
/**
 * The abstract Component class defines the interface used to add computational
 * elements to the underlying SimTK::System (MultibodySystem). It specifies
 * the interface that components must satisfy in order to be part of the system
 * and provides a series of helper methods for adding variables 
 * (state, discrete, cache, ...) to the underlying system. As such, Component
 * handles all of the bookkeeping of system indices and provides convenience 
 * access to variable values (incl. derivatives) via their names as strings. 
 *
 * The MultibodySystem and its State are defined by Simbody (ref ...). Briefly,
 * a System represents the mathematical equations that specify the behavior
 * of a computational model. The State is a collection of all the variables 
 * that uniquely define the unknowns in the system equations. Consider a single 
 * differential equation as a system, while a single set of variable values that  
 * satisfy the equation is a state of that system. These could be values for 
 * joint coordinates, their speeds, as well other variables that govern
 * the system dynamics (e.g. muscle activation and fiber-length variables 
 * that dictate muscle force). These variables are called continuous state 
 * variables in Simbody, but are more simply referred to as <em>StateVariables</em>
 * in OpenSim. Component provides services to define and access its 
 * StateVariables and specify their dynamics (derivatives with respect to time) 
 * that are automatically and simultaneously integrated with the MultibodySystem
 * dynamics. Common operations to integrate, take the max or min, or to delay a 
 * signal, etc. require internal variables to perform their calculations and 
 * these are also held in the State. Simbody provides the infrastructure to
 * ensure that calculations are kept up-to-date with the state variable values.
 *
 * There are other types of "State" variables such as a flag (or options) that 
 * enables a component to be disabled or for a muscle force to be overridden and 
 * and these are identified as <em>ModelingOptions</em> since they may change 
 * the modeled dynamics of the component. Component provides services
 * that enable developers of components to define additional ModelingOptions.
 *
 * Often a component requires input from an outside source (precomputed data 
 * from a file, another program, or interaction from a user) in which case these
 * variables do not have dynamics (differential eqns.) known to the component, 
 * but are necessary to describe the dynamical "state" of the system. An example,
 * is a throttle component (a "controller" that provides an actuator, e.g. a 
 * motor, with a control signal like a voltage or current) which it gets as direct 
 * input from the user (via a joystick, key press, etc..). The throttle controls
 * the motor torque output and therefore the behavior of the model. The input by
 * the user to the throttle the motor (the controls) is necessary to specify the
 * model dynamics at any instant and therefore are considered part of the State.
 * In OpenSim they are simply referred to as DiscreteVariables. The Component
 * provides services to enable developers of components to define and access its
 * DiscreteVariables.
 *
 * Fast and efficient simulations also require computationally expensive 
 * calculations to be performed only when necessary. Often the result of an
 * expensive calculation can be reused many times over, while the variables it
 * is dependent on remain fixed. The concept of holding onto these values is 
 * called caching and the variables that hold these values are call 
 * <em>CacheVariables</em>. It is important to note, that cache variables are
 * not state variables. Cache variables can always be recomputed excactly
 * from the State. OpenSim uses the Simbody infrastructure to manage cache 
 * variables and their validity. Component provides a simplified interface to
 * define and access CacheVariables.
 *
 * Many modeling and simulation codes put the onus on users and component 
 * creators to manage the validity of cache variables, which is likely to lead 
 * to undetectable errors where cache values are stale (calculated based on past
 * state variable values). Simbody, on the other hand, provides a more strict
 * infrastructure to make it easy to exploit the efficiencies of caching while 
 * reducing the risks of validity errors. To do this, Simbody employs the concept
 * of computational stages to "realize" (or compute) a model's system to a 
 * particular stage requires cached quantities up to and including the stage to 
 * to computed/specified. Simbody utilizes nine realization stages 
 * (<tt>SimTK::Stage::</tt>)
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
 * The Component interface is automatically invoked by the System and its 
 * realizations. Component users and most developers need not concern themselves
 * with \c Topology, \c %Model or \c Instance stages. That interaction is managed
 * by Component when component creators implement addToSystem() and use the 
 * services provided by Component. Component creators do need to determine and 
 * specify stage dependencies for Discrete and CacheVariables that they add to 
 * their components. For example, the throttle controller reads its value from
 * user input and it is valid for all calculations as long as time does not 
 * change. If the simulation (via numerical integration) steps forward (or 
 * backward for a trial step) and updates the state, the control from a previous
 * state (time) should be invalid and an error generated for trying to access
 * the DiscreteVariable for the control value. To do this one specifies the 
 * "invalidates" stage (e.g. <tt>SimTK::Stage::Time</tt>) for a DiscreteVariable
 * when the variable is added to the Component. A subsequent change to that 
 * variable will invalidate all state cache entries at that stage or higher. For
 * example, if a DiscreteVariable is declared to invalidate <tt>Stage::Position</tt>
 * then changing it will invalidate cache entries that depend on positions, 
 * velocities, forces, and accelerations.
 *
 * Similar principles apply to CacheVariables, which requires a "dependsOn" stage to 
 * be specified when a CacheVariable is added to the component. In this case, 
 * the cache variable "shadows" the State (unlike a DiscreteVariable, which is a
 * part of the State) holding already-computed state-dependent values so that 
 * they do not need to be recomputed until the state changes.
 * Accessing the CacheVariable in a State whose current stage is lower than 
 * that CacheVariable's specified dependsOn stage will trigger an exception. 
 * It is up to the component to update the value of the cache variable.
 * Component provides methods to check if the cache is valid, update its value
 * and then to mark it as valid. 
 *
 * The primary responsibility of a Component is to add its computational 
 * representation(s) to the underlying SimTK::System by implementing
 * addToSystem().
 *
 * Additional methods provide support for adding modeling options, state and
 * cache variables.
 *
 * Public methods enable access to component variables via their names.
 *
 * @author Ajay Seth, Michael Sherman
 */
class OSIMCOMMON_API Component : public Object {
OpenSim_DECLARE_ABSTRACT_OBJECT(Component, Object);

protected:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with a Connector. **/
    /**@{**/
    OpenSim_DECLARE_LIST_PROPERTY(connectors, AbstractConnector,
        "List of connectors (structural dependencies) that this component has.");
    /**@}**/

public:
//==============================================================================
// METHODS
//==============================================================================
    /** Default constructor **/
    Component();

    /** Construct Component from an XML file. **/
    Component(const std::string& aFileName,
        bool aUpdateFromXMLNode = true) SWIG_DECLARE_EXCEPTION;

    /** Construct Component from a specific node in an XML document. **/
    explicit Component(SimTK::Xml::Element& aNode);

    /** Copy Constructor. Required to perform custom handling of 
        internal references to subcomonents and system indices.
        The copy has to be connected in order to function. */
    Component(const Component& source);

    /** Copy assignment.  Required to disconnect Connectors.
        and reset indices. Musct call connect() on Component
        after is has been assigned to another. */
    Component& operator=(const Component &component);
    
    /** Destructor is virtual to allow concrete Component to cleanup. **/
    virtual ~Component() {}

    /**
     * Get the underlying MultibodySystem that this component is connected to.
     */
    const SimTK::MultibodySystem& getSystem() const
        { return *_system; } 

    /**
     * Get an iterator through the underlying components that this component 
     * is composed of.
     */
    //const ComponentIterator&  getComponentsIterator();

    /**
     * Get a subcomponent of this Component by its name. 
     * Note using a component's full "path" name is faster and will provide a
     * unique result. Otherwise, the first component to satisfy the name match 
     * will be returned.
     * For example right_elbow/elbow_flexion will return a Coordinate 
     * Component that is a member of the model's right elbow joint Component.
     *
     * @param name       the name (string) of the Component of interest
     * @return Component the component of interest
     */
    const Component& getComponent(const std::string& name) const;
    Component& updComponent(const std::string& name) const;

    /**
     * Get the number of "Continuous" state variables maintained by the Component
     * and its subcomponents
     */
    int getNumStateVariables() const;

    /**
     * Get the names of "continuous" state variables maintained by the Component
     * and its subcomponents
     */
    Array<std::string> getStateVariableNames() const;


   /** @name Component State Access methods
     * Get and set modeling option, state, discrete and/or cache variables in the State
     */ 
    //@{
    
    /**
     * Get a ModelingOption flag for this Component by name.
     * The flag is an integer corresponding to the index of modelingOptionNames used 
     * add the modeling option to the component. @see addModelingOption
     *
     * @param state  the State in which to set the modeling option
     * @param name   the name (string) of the modeling option of interest
     * @return flag  integer value for modeling option
     */
    int getModelingOption(const SimTK::State& state, const std::string& name) const;

    /**
     * Set the value of a ModelingOption flag for this Component.
     * if the integer value exceeds the number of option names used to
     * define the options, an exception is thrown. The SimTK::State 
     * Stage will be reverted back to Stage::Instance.
     *
     * @param state  the State in which to set the flag
     * @param name   the name (string) of the modeling option of interest
     * @param flag   the desired flag (int) value specifying the modeling option
     */
    void setModelingOption(SimTK::State& state, const std::string& name, int flag) const;

    /**
    * Get the Connector provided by this Component by name.
    *
    * @param name       the name of the Connector
    * @return const reference to the (Abstract)Connector
    */
    template<typename T> Connector<T>& 
        updConnector(const std::string& name) 
    {
        return *const_cast<Connector<T>*>(&getConnector<T>(name));
    }

    template<typename T> 
    const Connector<T>& getConnector(const std::string& name) const
    {
        const AbstractConnector* found = findConnector(name);

        if (!found){
            std::stringstream msg;
            msg << "Component::getConnector: ERR- no Connector '" << name << "' found.\n "
                << "for component '" << getName() << "' of type "
                << getConcreteClassName();
            throw Exception(msg.str(), __FILE__, __LINE__);
        }

        return (Connector<T>::downcast(*found));
    }

    /**
    * Get the "connectee" object that the Component's Connector
    * is bound to. Gauranteed to be valid only after the Component
    * has been connected (that is connect() has been invoked).
    * If Connector has not been connected an exception is thrown.
    *
    * @param name       the name of the connector
    * @return T         const reference to object that satisfies
    *                   the Connector
    */
    template<typename T>
    const T& getConnectee(const std::string& name) const    {
        // get the Connector and check if it is connected.
        const AbstractConnector& connector = getConnector<T>(name);
        if (connector.isConnected()){
            return (Connector<T>::downcast(connector)).getConectee();
        }
        else{
            std::stringstream msg;
            msg << "Component::getConnection() ERR- Connector '" << name << "' not connected.\n "
                << "for component '" << getName() << "' of type " << getConcreteClassName();
            throw Exception(msg.str(), __FILE__, __LINE__);
        }
    }

    /**
    * Get the Input provided by this Component by name.
    *
    * @param name       the name of the input
    * @return input const reference to the AbstractInput
    */
    const AbstractInput& getInput(const std::string& name) const
    {
        auto it = _inputsTable.find(name);

        if (it != _inputsTable.end()) {
            return *it->second;
        }
        else {
            std::string::size_type back = name.rfind("/");
            std::string prefix = name.substr(0, back);
            std::string inName = name.substr(back + 1, name.length() - back);

            const Component* found = findComponent(prefix);
            if (found)
                return found->getInput(inName);
        }

        std::stringstream msg;
        msg << "Component::getInput: ERR- no input '" << name <<"' found.\n "
                << "for component '" << getName() << "' of type "
                << getConcreteClassName();
        throw Exception(msg.str(), __FILE__, __LINE__);
    }

    /**
    * Get the Input value that this component is dependent on.
    * Check if Input is connected, otherwise it will throw an
    * exception.
    *
    * @param state      the State for which to set the value
    * @param name       the name of the input
    * @return T         const Input value
    */
    template<typename T> const T&
        getInputValue(const SimTK::State& state, const std::string& name) const {
        // get the input and check if it is connected.
        const AbstractInput& in = getInput(name);
        if (in.isConnected()){
            return (Input<T>::downcast(in)).getValue(state);
        }
        else{
            std::stringstream msg;
            msg << "Component::getInputValue: ERR- input '" << name << "' not connected.\n "
                << "for component '" << getName() << "' of type "<< getConcreteClassName();
            throw Exception(msg.str(), __FILE__, __LINE__);
        }
    }

    /** An iterator for the map of Outputs of this component, pointing at the
     * beginning of the map. This can be used in a loop as such:
     *
     *  @code
     *  std::map<std::string, const AbstractOutput*>::const_iterator it;
     *  for (it = myComp.getOutputsBegin(); it != myComp.getOutputsEnd(); it++)
     *  { ... }
     *  @endcode
     *
     * @see getOutputsEnd()
     */
    std::map<std::string, std::unique_ptr<const AbstractOutput>
        >::const_iterator
        getOutputsBegin() const
    {
        return _outputsTable.begin();
    }

    /** An iterator for the map of Outputs of this component, pointing at the
     * end of the map. This can be used in a loop as such:
     *
     * @see getOutputsBegin()
     */
    std::map<std::string, std::unique_ptr<const AbstractOutput>
        >::const_iterator
        getOutputsEnd() const
    {
        return _outputsTable.end();
    }


    /**
    * Get the Output provided by this Component by name.
    *
    * @param name   the name of the cache variable
    * @return       const reference to the AbstractOutput
    */
    const AbstractOutput& getOutput(const std::string& name) const
    {
        auto it = _outputsTable.find(name);

        if (it != _outputsTable.end()) {
            return *it->second;
        }
        else {
            std::string::size_type back = name.rfind("/");
            std::string prefix = name.substr(0, back);
            std::string outName = name.substr(back+1, name.length()-back);

            const Component* found = findComponent(prefix);
            // if found is this component again, no point trying to find
            // output again, otherwise we would not have reached here 
            if (found && (found != this)) { 
                return found->getOutput(outName);
            }
        }

        std::stringstream msg;
        msg << "Component::getOutput: ERR-  no output '" << name << "' found.\n "
            << "for component '" << getName() << "' of type "
            << getConcreteClassName();
        throw Exception(msg.str(), __FILE__, __LINE__);
    }

    /**
    * Get the Output value provided by this Component by name.
    *
    * @param state      the State for which to set the value
    * @param name       the name of the cache variable
    * @return T         const Output value
    */
    template<typename T> const T&
        getOutputValue(const SimTK::State& state, const std::string& name) const
    {
        return (Output<T>::downcast(getOutput(name))).getValue(state);
    }
    
    
    /**
     * Get the value of a state variable allocated by this Component.
     *
     * To connect this StateVariable as an input to another component (such as
     * a Reporter), use getOutput(name); each state variable has a
     * corresponding Output:
     *  @code
     *  foo.getInput("input1").connect(bar.getOutput(name));
     *  @endcode
     *
     * @param state   the State for which to get the value
     * @param name    the name (string) of the state variable of interest
     */
    double getStateVariable(const SimTK::State& state, const std::string& name) const;

    /**
     * Set the value of a state variable allocated by this Component by name.
     *
     * @param state  the State for which to set the value
     * @param name   the name of the state variable
     * @param value  the value to set
     */
    void setStateVariable(SimTK::State& state, const std::string& name, double value) const;


    /**
     * Get all values of the state variables allocated by this Component.
     * Includes state variables allocated by its subcomponents.
     *
     * @param state   the State for which to get the value
     * @return Vector of state variable values of length getNumStateVariables()
     *                in the order returned by getStateVariableNames()
     */
    SimTK::Vector getStateVariableValues(const SimTK::State& state) const;

    /**
     * Set all values of the state variables allocated by this Component.
     * Includes state variables allocated by its subcomponents.
     *
     * @param state   the State for which to get the value
     * @param values  Vector of state variable values of length getNumStateVariables()
     *                in the order returned by getStateVariableNames()
     */
    void setStateVariableValues(SimTK::State& state, const SimTK::Vector& values);

    /**
     * Get the value of a state variable derivative computed by this Component.
     *
     * @param state   the State for which to get the derivative value
     * @param name    the name (string) of the state variable of interest
     */
    double getStateVariableDerivative(const SimTK::State& state, 
        const std::string& name) const;

    /**
     * Get the value of a discrete variable allocated by this Component by name.
     *
     * @param state   the State from which to get the value
     * @param name    the name of the state variable
     * @return value  the discrete variable value
     */
    double getDiscreteVariable(const SimTK::State& state, const std::string& name) const;

    /**
     * Set the value of a discrete variable allocated by this Component by name.
     *
     * @param state  the State for which to set the value
     * @param name   the name of the dsicrete variable
     * @param value  the value to set
     */
    void setDiscreteVariable(SimTK::State& state, const std::string& name, double value) const;

    /**
     * Get the value of a cache variable allocated by this Component by name.
     *
     * @param state  the State from which to get the value
     * @param name   the name of the cache variable
     * @return T     const reference to the cache variable's value
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
            msg << "Component::getCacheVariable: ERR- name not found.\n "
                << "for component '"<< getName() << "' of type " 
                << getConcreteClassName();
            throw Exception(msg.str(),__FILE__,__LINE__);
        }
    }
    /**
     * Obtain a writable cache variable value allocated by this Component by name.
     * Do not forget to mark the cache value as valid after updating, otherwise it
     * will force a reevaluation if the evaluation method is monitoring the 
     * validity of the cache value.
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
            msg << "Component::updCacheVariable: ERR- '" << name 
                << "' name not found.\n "
                << "for component '"<< getName() << "' of type " 
                << getConcreteClassName();
            throw Exception(msg.str(),__FILE__,__LINE__);
        }
    }

    /**
     * After updating a cache variable value allocated by this Component, you can
     * mark its value as valid, which will not change until the realization stage 
     * falls below the minimum set at the time the cache variable was created. If 
     * not marked as valid, the evaluation method monitoring this flag will force 
     * a re-evaluation rather that just reading the value from the cache.
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
            msg << "Component::markCacheVariableValid: ERR- name not found.\n "
                << "for component '"<< getName() << "' of type " 
                << getConcreteClassName();
            throw Exception(msg.str(),__FILE__,__LINE__);
        }
    }

    /**
     * Mark a cache variable value allocated by this Component as invalid.
     * When the system realization drops to below the lowest valid stage, cache 
     * variables are automatically marked as invalid. There are instances when
     * component added state variables require invalidating a cache at a lower 
     * stage. For example, a component may have a length state variable which 
     * should invalidate calculations involving it and other positions when the 
     * state variable is set. Changing the component state variable automatically
     * invalidates Dynamics and higher realizations, but to force realizations
     * at Position and Velocity requires setting the lowest valid stage to 
     * Position and marking the cache variable as invalid whenver the length
     * state variable value is set/changed.
     *
     * @param state  the State containing the cache variable
     * @param name   the name of the cache variable
     */
    void markCacheVariableInvalid(const SimTK::State& state, 
                                  const std::string& name) const
    {
        std::map<std::string, CacheInfo>::const_iterator it;
        it = _namedCacheVariableInfo.find(name);

        if(it != _namedCacheVariableInfo.end()) {
            SimTK::CacheEntryIndex ceIndex = it->second.index;
            getDefaultSubsystem().markCacheValueNotRealized(state, ceIndex);
        }
        else{
            std::stringstream msg;
            msg << "Component::markCacheVariableInvalid: ERR- name not found.\n"
                << "for component '"<< getName() << "' of type " 
                << getConcreteClassName();
            throw Exception(msg.str(),__FILE__,__LINE__);
        }
    }

    /**
     * Enables the user to monitor the validity of the cache variable value using the
     * returned flag. For components performing a costly evaluation, use this 
     * method to force a re-evaluation cache variable value only when necessary 
     * (returns false).
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
            msg << "Component::isCacheVariableValid: ERR- name not found.\n "
                << "for component '"<< getName() << "' of type " 
                << getConcreteClassName();
            throw Exception(msg.str(),__FILE__,__LINE__);
        }
    }

    /**
     *  Set cache variable value allocated by this Component by name.
     *  All cache entries are lazily evaluated (on a need basis) so a set
     *  also marks the cache as valid.
     *
     * @param state  the State in which to store the new value
     * @param name   the name of the cache variable
     * @param value  the new value for this cache variable
     */
    template<typename T> void 
    setCacheVariable(const SimTK::State& state, const std::string& name, 
                     const T& value) const
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
            msg << "Component::setCacheVariable: ERR- name not found.\n "
                << "for component '"<< getName() << "' of type " 
                << getConcreteClassName();
            throw Exception(msg.str(),__FILE__,__LINE__);
        }   
    }
    // End of Model Component State Accessors.
    //@} 

protected:

class StateVariable;
//template <class T> friend class ComponentSet;
// Give the ComponentMeasure access to the realize() methods.
template <class T> friend class ComponentMeasure;

  /** Single call to construct the underlying infastructure of a Component, which
     include: 1) its properties, 2) its structural connectors (to other components),
     3) its Inputs (slots) for expected Output(s) of other components and, 4) its 
     own Outputs (wires) that it provides for other components to access its values.
     Override the corresponding private virtual method to customize any of them. */ 
    void constructInfrastructure() {
        constructProperties();
        constructStructuralConnectors();
        constructInputs();
        constructOutputs();
    }

    /** @name           Component Basic Interface
    The interface ensures that deserialization, resolution of inter-connections,
    and handling of dependencies are performed systematically and prior to 
    system creation, followed by allocation of necessary System resources. These 
    methods are virtual and may be implemented by subclasses of 
    Components. 
    
    @note Every implementation of virtual method xxx(args) must begin
    with the line "Super::xxx(args);" to ensure that the parent class methods
    execute before the child class method, starting with Component::xxx()
    and going down. 
    
    The base class implementations here do two things: (1) take care of any
    needs of the %Component base class itself, and then (2) ensure that the 
    corresponding calls are made to any subcomponents that have been specified 
    by derived %Component objects, via calls to the addComponent() method. 
    So assuming that your concrete %Component and all intermediate classes from
    which it derives properly follow the requirement of calling the Super class 
    method first, the order of operations enforced here for a call to a single 
    method will be
      -# %Component base class computations
      -# calls to that same method for \e all subcomponents
      -# calls to that same method for intermediate %Component-derived 
         objects' computations, in order down from %Component, and
      -# finally a call to that method for the bottom-level concrete class. 

    You should consider this ordering when designing a %Component. In 
    particular the fact that all your subcomponents will be invoked before you
    are may be surprising. **/ 
    //@{

    /** Perform any time invariant calculation, data structure initializations or
    other component configuration based on its properties necessary to form a  
    functioning, yet not connected component. It also marks the Component
    as up-to-date with its properties when compete.

    If you override this method, be sure to invoke the base class method LAST,
        using code like this :
        @code
        void MyComponent::finalizeFromProperties() {
            // ... your code goes here
            // ... addComponent(...) that are listed in or formed from properties
            // ... initialize any internal data structures 
            Super::finalizeFromProperties(); // invoke parent class method
        }
        @endcode   */
    virtual void finalizeFromProperties();

    /** Perform any necessary initializations required to connect the component
    (including it subcomponents) to other components and mark the connection status.
    Provides a check for error conditions. connect() is invoked on all components 
    to form a directed acyclic graph of the multibody system, prior to creating the
    Simbody MultibodySystem to represent it computationally. It may also be invoked
    at times just for its error-checking side effects.

    The "root" Component argument is the root node of the directed graph composed
    of all the subcomponents (and their subcomponents and so on ...) and their
    interconnections. This should yield a fully connected root component. For 
    ModelComponents this is the Model component. But a Model can be connected to
    an environment or world component with several other models, by choosing the
    environment/world as the root.
    
    If you override this method, be sure to invoke the base class method first, 
    using code like this:
    @code
    void MyComponent::connect(Component& root) {
        Super::connect(root); // invoke parent class method
        // ... your code goes here
    }
    @endcode   */
    virtual void connect(Component &root);

    /** Opportunity to remove connection related information. 
    If you override this method, be sure to invoke the base class method first,
        using code like this :
        @code
        void MyComponent::disconnect(Component& root) {
            // disconnect your subcomponents first
            Super::disconnect(); 
            //your code to wipeout your connection related information
    }
    @endcode  */
    virtual void disconnect();


    /** Add appropriate Simbody elements (if needed) to the System 
    corresponding to this component and specify needed state resources. 
    addToSystem() is called when the Simbody System is being created to 
    represent a completed system (model) for computation. That is, connect()
    will already have been invoked on all components before any addToSystem()
    call is made. Helper methods for adding modeling options, state variables 
    and their derivatives, discrete variables, and cache entries are available 
    and can be called within addToSystem() only.

    Note that this method is const; you must not modify your model component
    or the containing model during this call. Any modifications you need should
    instead be performed in finalizeFromProperties() or at the latest connect(),
    which are non-const. The only exception is that you may need to record access 
    information for resources you create in the \a system, such as an index number.
    You should declare those data members mutable so that you can set them here.
    For common Components, OpenSim base classes either provide convenience methods
    or handle indices automatically. 
   
    If you override this method, be sure to invoke the base class method at the
    end, using code like this:
    @code
    void MyComponent::addToSystem(SimTK::MultibodySystem& system) const {
        // ... your code goes here
        // call Super class to invoke method on subcomponents
        Super::addToSystem(system);       
    }
    @endcode

    @param[in,out] system   The System being created.

    @see addModelingOption(), addStateVariable(), addDiscreteVariables(), 
         addCacheVariable() **/
    virtual void addToSystem(SimTK::MultibodySystem& system) const;


    /** Transfer property values or other state-independent initial values
    into this component's state variables in the passed-in \a state argument.
    This is called after a SimTK::System and State have been created for the 
    Model (that is, after addToSystem() has been called on all components). 
    You should override this method if your component has properties
    (serializable values) that can affect initial values for your state
    variables. You can also perform any other state-independent calculations
    here that result in state initial conditions.
   
    If you override this method, be sure to invoke the base class method first, 
    using code like this:
    @code
    void MyComponent::initStateFromProperties(SimTK::State& state) const {
        Super::initStateFromProperties(state); // invoke parent class method
        // ... your code goes here
    }
    @endcode

    @param      state
        The state that will receive the new initial conditions.

    @see setPropertiesFromState() **/
    virtual void initStateFromProperties(SimTK::State& state) const;

    /** Update this component's property values to match the specified State,
    if the component has created any state variable that is intended to
    correspond to a property. Thus, state variable values can persist as part 
    of the model component and be serialized as a property.
   
    If you override this method, be sure to invoke the base class method first, 
    using code like this:
    @code
    void MyComponent::setPropertiesFromState(const SimTK::State& state) {
        Super::setPropertiesFromState(state); // invoke parent class method
        // ... your code goes here
    }
    @endcode

    @param      state    
        The State from which values may be extracted to set persistent
        property values.

    @see initStateFromProperties() **/
    virtual void setPropertiesFromState(const SimTK::State& state);

    /** If a model component has allocated any continuous state variables
    using the addStateVariable() method, then %computeStateVariableDerivatives()
    must be implemented to provide time derivatives for those states.
    Override to set the derivatives of state variables added to the system 
    by this component. (also see addToSystem()). If the component adds states
    and computeStateVariableDerivatives is not implemented by the component,
    an exception is thrown when the system tries to evaluate its derivates.

    Implement like this:
    @code
    void computeStateVariableDerivatives(const SimTK::State& state) const {
        
        // Let the parent class set the derivative values for the 
        // the state variables that it added.
        Super::computeStateVariableDerivatives(state)

        // Compute derivative values for states allocated by this component
        // as a function of the state.
        double deriv = ... 

        // Then set the derivative value by state variable name
        setStateVariableDerivative(state, "<state_variable_name>", deriv);
    }
    @endcode

    For subclasses, it is highly recommended that you first call
    Super::computeStateVariableDerivatives(state) to preserve the derivative
    computation of the parent class and to only specify the derivatives of the state
    variables added by name. One does have the option to override all the derivative 
    values for the parent by accessing the derivatives by their state variable name.
    This is necessary, for example, if a newly added state variable is coupled to the
    dynamics (derivatives) of the states variables that were added by the parent.
    **/
    virtual void computeStateVariableDerivatives(const SimTK::State& s) const;

    /**
     * Set the derivative of a state variable by name when computed inside of  
     * this Component's computeStateVariableDerivatives() method.
     *
     * @param state  the State for which to set the value
     * @param name   the name of the state variable
     * @param deriv  the derivative value to set
     */
    void setStateVariableDerivative(const SimTK::State& state, 
                            const std::string& name, double deriv) const;


    // End of Model Component Basic Interface (protected virtuals).
    //@} 

    /** @name           Component Advanced Interface
    You probably won't need to override methods in this section. These provide
    a way for you to perform computations ("realizations") that must be 
    scheduled in carefully-ordered stages as described in the class description
    above. The typical operation will be that the given SimTK::State provides
    you with the inputs you need for your computation, which you will then 
    write into some element of the state cache, where later computations can
    pick it up.

    @note Once again it is crucial that, if you override a method here,
    you invoke the superclass method as the <em>first line</em> in your
    implementation, via a call like "Super::realizePosition(state);". This 
    will ensure that all necessary base class computations are performed, and
    that subcomponents are handled properly.

    @warning Currently the realize() methods here are invoked early in the
    sequence of realizations at a given stage, meaning that you will not be
    able to access other computations at that same stage.
    @bug Should defer calls to these until at least kinematic realizations
    at the same stage have been performed.

    @see Simbody documentation for more information about realization.
    **/
    //@{
    /** Obtain state resources that are needed unconditionally, and perform
    computations that depend only on the system topology. **/
    virtual void realizeTopology(SimTK::State& state) const;
    /** Obtain and name state resources (like state variables allocated by
    an underlying Simbody component) that may be needed, depending on modeling
    options. Also, perform any computations that depend only on topology and 
    selected modeling options. **/
    virtual void realizeModel(SimTK::State& state) const;
    /** Perform computations that depend only on instance variables, like
    lengths and masses. **/
    virtual void realizeInstance(const SimTK::State& state) const;
    /** Perform computations that depend only on time and earlier stages. **/
    virtual void realizeTime(const SimTK::State& state) const;
    /** Perform computations that depend only on position-level state
    variables and computations performed in earlier stages (including time). **/
    virtual void realizePosition(const SimTK::State& state) const;
    /** Perform computations that depend only on velocity-level state 
    variables and computations performed in earlier stages (including position, 
    and time). **/
    virtual void realizeVelocity(const SimTK::State& state) const;
    /** Perform computations (typically forces) that may depend on 
    dynamics-stage state variables, and on computations performed in earlier
    stages (including velocity, position, and time), but not on other forces,
    accelerations, constraint multipliers, or reaction forces. **/
    virtual void realizeDynamics(const SimTK::State& state) const;
    /** Perform computations that may depend on applied forces. **/
    virtual void realizeAcceleration(const SimTK::State& state) const;
    /** Perform computations that may depend on anything but are only used
    for reporting and cannot affect subsequent simulation behavior. **/
    virtual void realizeReport(const SimTK::State& state) const;
    //@}


    /** @name     Component System Creation and Access Methods
     * These methods support implementing concrete Components. Add methods
     * can only be called inside of addToSystem() and are useful for creating
     * the underlying SimTK::System level variables that are used for computing
     * values of interest.
     * @warning Accessors for System indices are intended for component internal use only.
     **/

    //@{
    /**
    * Construct a specialized Connector for this Component's dependence on an another
    * Component. It serves as a placeholder for the Component and its type and enables
    * the Component to automatically traverse its dependencies and provide a meaningful 
    * message if the provided Component is incompatible or non-existant.
    */
    template <typename T>
    void constructStructuralConnector(const std::string& name) {
        int ix = updProperty_connectors().adoptAndAppendValue(
            new Connector<T>(name, SimTK::Stage::Topology));
        //add pointer to connectorsTable so we can access connectors easily by name
        _connectorsTable[name] = ix;
    }

    /**
    * Construct an Input (socket) for this Component's dependence on an Output signal.
    * It is a placeholder for the Output and its type and enables the Component
    * to automatically traverse its dependencies and provide a meaningful message
    * if the provided Output is incompatible or non-existant. The also specifies at what
    * stage the output must be valid for the the component to consume it as an input.
    * if the Output's dependsOnStage is above the Input's requiredAtStage, an Exception
    * is thrown because the output cannot satisfy the Input's requirement.
    */
    template <typename T>
    void constructInput(const std::string& name,
        const SimTK::Stage& requiredAtStage = SimTK::Stage::Instance) {
        _inputsTable[name] = std::unique_ptr<AbstractInput>(new Input<T>(name, requiredAtStage));
    }

    /**
     * A convenient way to construct an Output.  Here, we assume the following
     * about componentMemberFunction, the function that returns the output:
     *
     *  1. It is a member function of \a this Component.
     *  2. The member function is const.
     *  3. It takes only one input, which is const SimTK::State&
     *
     * If these are not true for your case, then use the more general method
     * Component::constructOutput(const std::string&, const std::function<T(const SimTK::State&)>, const SimTK::Stage&).
     *
     * Here's an example. Say your Component has a method calcForce:
     *  @code
     *  constructOutput<SimTK::Vec3>("force", &MyComponent::calcForce,
     *          SimTK::Stage::Velocity);
     *  @endcode
     */
#ifndef SWIG // SWIG can't parse the const at the end of the second argument.
    template <typename T, typename Class>
    void constructOutput(const std::string& name,
            T(Class::*componentMemberFunction)(const SimTK::State&) const,
            const SimTK::Stage& dependsOn = SimTK::Stage::Acceleration) {
        constructOutput<T>(name, std::bind(componentMemberFunction,
                    static_cast<Class*>(this),
                    std::placeholders::_1), dependsOn);
    }
#endif

    /**
    * Construct an Output (wire) for the Component as function of the State.
    * Specifiy a (member) function of the state implemented by this component to
    * be an Output and include the Stage that output is dependent on. If no
    * Stage is specified it defaults to Acceleration. Here's an example. Say you have a class Markers that manages markers, you have an instance of this class as a member variable in your Component, and Markers has a method `Vec3 Markers\:\:calcMarkerPos(const SimTK\:\:State& s, std\:\:string marker);` to compute
    * motion capture marker positions, given the name of a marker.
     *  @code
     *  constructOutput<SimTK::Vec3>("ankleMarkerPos",
     *          std::bind(&Markers::calcMarkerPos, _markers,
     *          std::placeholders::_1, "ankle"),
     *          SimTK::Stage::Position);
     *  @endcode
    */
    template <typename T>
    void constructOutput(const std::string& name, 
        const std::function<T(const SimTK::State&)> outputFunction, 
        const SimTK::Stage& dependsOn = SimTK::Stage::Acceleration) {
        _outputsTable[name] = std::unique_ptr<const AbstractOutput>(new
                Output<T>(name, outputFunction, dependsOn));
    }
    
    /**
     * Add another Component as a subcomponent of this Component.
     * Component methods (e.g. addToSystem(), initStateFromProperties(), ...) are 
     * therefore invoked on subcomponents when called on this Component. Realization is 
     * also performed automatically on subcomponents. This Component does not take 
     * ownership of designated subcomponents and does not destroy them when the Component.
     */
    void addComponent(Component *aComponent);

    /** Clear all designations of (sub)components for this Component. 
      * Components are not deleted- the list of references to its components is cleared. */
    void clearComponents() {
        _components.clear();
    }

    /** Add a modeling option (integer flag stored in the State) for use by 
    this Component. Each modeling option is identified by its own 
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


    /** Add a continuous system state variable belonging to this Component,
    and assign a name by which to refer to it. Changing the value of this state 
    variable will automatically invalidate everything at and above its
    \a invalidatesStage, which is normally Stage::Dynamics meaning that there
    are forces that depend on this variable. If you define one or more
    of these variables you must also override computeStateVariableDerivatives()
    to provide time derivatives for them. Note, all corresponding system
    indices are automatically determined using this interface. As an advanced
    option you may choose to hide the state variable from being accessed outside
    of this component, in which case it is considered to be "hidden". 
    @param[in] stateVariableName     string value to access variable by name
    @param[in] invalidatesStage      the system realization stage that is
                                     invalidated when variable value is changed
    @param[in] isHidden              flag (bool) to optionally hide this state
                                     variable from being accessed outside this
                                     component as an Output
    */
    void addStateVariable(const std::string&  stateVariableName,
         const SimTK::Stage& invalidatesStage=SimTK::Stage::Dynamics,
         bool isHidden = false) const;

    /** The above method provides a convenient interface to this method, which
    automatically creates an 'AddedStateVariable' and allocates resources in the
    SimTK::State for this variable.  This interface allows the creator to
    add/expose state variables that are allocated by underlying Simbody
    components and specify how the state variable value is accessed by
    implementing a concrete StateVariable and adding it to the component using
    this method. If the StateVariable is NOT hidden, this also creates an
    Output in this Component with the same name as the StateVariable. Reporters
    should use this Output to get the StateVariable's value (instead of using
    getStateVariable()). */
    void addStateVariable(Component::StateVariable*  stateVariable) const;

    /** Add a system discrete variable belonging to this Component, give
    it a name by which it can be referenced, and declare the lowest Stage that
    should be invalidated if this variable's value is changed. **/
    void addDiscreteVariable(const std::string& discreteVariableName,
                             SimTK::Stage       invalidatesStage) const;

    /** Add a state cache entry belonging to this Component to hold
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

    
    /**
     * Get writeable reference to the MultibodySystem that this component is
     * connected to.
     */
    SimTK::MultibodySystem& updSystem() const
        { return *_system; } 

    /** Get the index of a Component's continuous state variable in the Subsystem for
        allocations. This method is intended for derived Components that may need direct
        access to its underlying Subsystem.*/
    const int getStateIndex(const std::string& name) const;

   /**
     * Get the System Index of a state variable allocated by this Component.  
     * Returns an InvalidIndex if no state variable with the name provided is
     * found.
     * @param stateVariableName   the name of the state variable 
     */
    SimTK::SystemYIndex 
        getStateVariableSystemIndex(const std::string& stateVariableName) const;

    /** Get the index of a Component's discrete variable in the Subsystem for allocations.
        This method is intended for derived Components that may need direct access
        to its underlying Subsystem.*/
    const SimTK::DiscreteVariableIndex 
    getDiscreteVariableIndex(const std::string& name) const;

    /** Get the index of a Component's cache variable in the Subsystem for allocations.
        This method is intended for derived Components that may need direct access
        to its underlying Subsystem.*/
    const SimTK::CacheEntryIndex 
    getCacheVariableIndex(const std::string& name) const;

    // End of System Creation and Access Methods.

    /** Utility method to find a component in the list of sub components of this
        component and any of their sub components, etc..., by name or state variable name.
        The search can be sped up considerably if the "path" or even partial path name
        is known. For example name = "forearm/elbow/elbow_flexion" will find the 
        Coordinate component of the elbow joint that connects the forearm body in 
        linear time (linear search for name at each component level. Whereas
        supplying "elbow_flexion" requires a tree search.
        Returns NULL if Component of that specified name cannot be found. 
        If the name provided is a component's state variable name and a pointer to
        a StateVariable pointer is provided, the pointer will be set to the 
        StateVariable object that was found. This facilitates the getting and setting
        of StateVariables by name. 
        
        NOTE: If the component name or the state variable name is ambiguous, the 
         first instance found is returned. To disambiguate use the full name provided
         by owning component(s). */
    const Component* findComponent(const std::string& name, 
                                   const StateVariable** rsv = nullptr) const;
    
    /** Similarly find a Connector of this Component (also amongst its subcomponents) */
    const AbstractConnector* findConnector(const std::string& name) const;

    const StateVariable* findStateVariable(const std::string& name) const;

    //@} 

private:
    // Construct the table of serializable properties for a Component.
    // Base constructs property that contains the structural connectors.
    virtual void constructProperties() {}

    //Construct the table of structural Connectors this component requires to
    //hookup to other components in order to function. For example, a Joint needs 
    //a parent body in order to join its owning body to the model. A Connector
    //formalizes this dependendency. The Component is inoperable until the Connector
    //is satisfied. Connectors are not to be confused with subcomponents, with the key 
    //difference being that a subcomponent is part of and owned by the component, 
    //whereas a Connector is a requirement or a "slot" that must be satisfied by
    //the time the system is ready to simulate. 
    //Connectors are resolved in Component's connect().
    //constructStructuralDependencies is a series of calls to constrcuctConnector()
    //which adds a component by name and type to a dependency Connectors table.
    virtual void constructStructuralConnectors() {}

    //Construct the table of Inputs for this component. A Component::Input is a
    //dependency on the Output of another Component. Unlike a structural 
    //connector, an input specifies the required flow of data into the component.
    //@see Component::Input
    virtual void constructInputs() {}

    //Construct the table of Outputs provided by this component. An Output is
    //a data signal generated by this Component. It can be any response or 
    //calculation made by the Component as a function of the state. Specifically,
    //an Output is a redirect to a method on the Component and a specification of 
    //the return type, @see addOutput()
    virtual void constructOutputs() {}

    // Get the number of continuous states that the Component added to the 
    // underlying computational system. It includes the number of built-in states  
    // exposed by this component. It represents the number of state variables  
    // managed by this Component.
    int getNumStateVariablesAddedByComponent() const 
    {   return (int)_namedStateVariableInfo.size(); }
    Array<std::string> getStateVariablesNamesAddedByComponent() const;

    const SimTK::DefaultSystemSubsystem& getDefaultSubsystem() const
        {   return getSystem().getDefaultSubsystem(); }
    SimTK::DefaultSystemSubsystem& updDefaultSubsystem() const
        {   return updSystem().updDefaultSubsystem(); }

    void clearStateAllocations() {
        _namedModelingOptionInfo.clear();
        _namedStateVariableInfo.clear();
        _namedDiscreteVariableInfo.clear();
        _namedCacheVariableInfo.clear();    
    }

    // Reset by clearing underlying system indices, disconnecting connectors and
    // creating a fresh connectorsTable.
    void reset() {
        _simTKcomponentIndex.invalidate();
        clearStateAllocations();

        _connectorsTable.clear();
        for (int ix = 0; ix < getProperty_connectors().size(); ++ix){
            AbstractConnector& connector = upd_connectors(ix);
            _connectorsTable[connector.getName()] = ix;
        }
    }
    
protected:
    //Derived Components must create concrete StateVariables to expose their state 
    //variables. When exposing state variables allocated by the underlying Simbody
    //component (MobilizedBody, Constraint, Force, etc...) use its interface to 
    //implement the virtual methods below. Otherwise, if the Component is adding its
    //own state variables using the addStateVariable() helper, then an 
    //AddedStateVariable implements the interface and automatically handles state
    //variable access.
    class StateVariable {
        friend void Component::addStateVariable(StateVariable* sv) const;
    public:
        StateVariable() : name(""), owner(NULL),
            subsysIndex(SimTK::InvalidIndex), varIndex(SimTK::InvalidIndex),
            sysYIndex(SimTK::InvalidIndex), hidden(true) {}
        explicit StateVariable(const std::string& name, //state var name
            const Component& owner,     //owning component
            SimTK::SubsystemIndex sbsix,//subsystem for allocation
            int varIndex,               //variable's index in subsystem
            bool hide = false)          //state variable is hidden or not
            : name(name), owner(&owner),
            subsysIndex(sbsix), varIndex(varIndex),
            sysYIndex(SimTK::InvalidIndex), hidden(hide)  {}

        virtual ~StateVariable() {}

        const std::string& getName() const { return name; }
        const Component& getOwner() const { return *owner; }

        const int& getVarIndex() const { return varIndex; }
        // return the index of the subsystem used to make resource allocations 
        const SimTK::SubsystemIndex& getSubsysIndex() const { return subsysIndex; }
        // return the index of the subsystem used to make resource allocations 
        const SimTK::SystemYIndex& getSystemYIndex() const { return sysYIndex; }

        bool isHidden() const { return hidden; }
        void hide()  { hidden = true; }
        void show()  { hidden = false; }

        void setVarIndex(int index) { varIndex = index; }
        void setSubsystemIndex(const SimTK::SubsystemIndex& sbsysix)
        {
            subsysIndex = sbsysix;
        }

        //Concrete Components implement how the state variable value is evaluated
        virtual double getValue(const SimTK::State& state) const = 0;
        virtual void setValue(SimTK::State& state, double value) const = 0;
        virtual double getDerivative(const SimTK::State& state) const = 0;
        // The derivative a state should be a cache entry and thus does not
        // change the state
        virtual void setDerivative(const SimTK::State& state, double deriv) const = 0;

    private:
        std::string name;
        SimTK::ReferencePtr<const Component> owner;

        // Identify which subsystem this state variable belongs to, which should 
        // be determined and set at creation time
        SimTK::SubsystemIndex subsysIndex;
        // The local variable index in the subsystem also provided at creation
        // (e.g. can be QIndex, UIndex, or Zindex type)
        int  varIndex;
        // Once allocated a state will in the system will have a global index
        // and that can be stored here as well
        SimTK::SystemYIndex sysYIndex;

        // flag indicating if state variable is hidden to the outside world
        bool hidden;
    };



    // Maintain pointers to subcomponents so we can invoke them automatically.
    // These are just references, don't delete them!
    // TODO: subcomponents should not be exposed to derived classes to trash.
    //       Need to provide universal access via const iterators -aseth
    SimTK::Array_<Component *>  _components;

private:
    class Connection;
    
    // Reference pointer to the system that this component belongs to.
    SimTK::ReferencePtr<SimTK::MultibodySystem> _system;

    // propertiesTable maintained by Object

    // Table of Component's structural Connectors indexed by name.
    // Index is the slot in the connectors property where the concrete
    // Connector lives.
    std::map<std::string, int> _connectorsTable;

    // Table of Component's Inputs indexed by name.
    std::map<std::string, std::unique_ptr<const AbstractInput> > _inputsTable;

    // Table of Component's Outputs indexed by name.
    std::map<std::string, std::unique_ptr<const AbstractOutput> >
        _outputsTable;

    // Underlying SimTK custom measure ComponentMeasure, which implements
    // the realizations in the subsystem by calling private concrete methods on
    // the Component. Every model component has one of these, allocated
    // in its addToSystem() method, and placed in the System's default subsystem.
    SimTK::MeasureIndex  _simTKcomponentIndex;

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

    // Class for handling state variable added (allocated) by this Component
    class AddedStateVariable : public StateVariable {
        public:
        // Constructors
        AddedStateVariable() : StateVariable(),
            invalidatesStage(SimTK::Stage::Empty)  {}

        /** Convience constructor for defining a Component added state variable */ 
        explicit AddedStateVariable(const std::string& name, //state var name
                        const Component& owner,       //owning component
                        SimTK::Stage invalidatesStage,//stage this variable invalidates
                        bool hide=false) : 
                    StateVariable(name, owner,
                            SimTK::SubsystemIndex(SimTK::InvalidIndex),
                            SimTK::InvalidIndex, hide), 
                        invalidatesStage(SimTK::Stage::Empty) {}

        //override virtual methods
        double getValue(const SimTK::State& state) const override;
        void setValue(SimTK::State& state, double value) const override;

        double getDerivative(const SimTK::State& state) const override;
        void setDerivative(const SimTK::State& state, double deriv) const override;

        private: // DATA
        // Changes in state variables trigger recalculation of appropriate cache 
        // variables by automatically invalidating the realization stage specified
        // upon allocation of the state variable.
        SimTK::Stage    invalidatesStage;
    };

    // Structure to hold related info about discrete variables 
    struct StateVariableInfo {
        StateVariableInfo() {}
        explicit StateVariableInfo(Component::StateVariable* sv, int order) :
        stateVariable(sv), order(order) {}

        // Need empty copy constructor because default compiler generated
        // will fail since it cannot copy a unique_ptr!
        StateVariableInfo(const StateVariableInfo&) {}
        // Now handle assignment by moving ownership of the unique pointer
        StateVariableInfo& operator=(const StateVariableInfo& svi) {
            if(this != &svi){
                //assignment has to be const but cannot swap const
                //want to keep unique pointer to guarantee no multiple reference
                //so use const_cast to swap under the covers
                StateVariableInfo* mutableSvi = const_cast<StateVariableInfo *>(&svi);
                stateVariable.swap(mutableSvi->stateVariable);
            }
            order = svi.order;
            return *this;
        }

        // State variable
        std::unique_ptr<Component::StateVariable> stateVariable;
        // order of allocation
        int order;
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

    // Map names of modeling options for the Component to their underlying
    // SimTK indices.
    // These are mutable here so they can ONLY be modified in addToSystem().
    // This is not an API bug. The purpose of these maps is to automate the 
    // bookkeeping of component variables (state variables and cache entries) with 
    // their index in the computational system. The earliest time we have a valid 
    // index is when we ask the system to allocate the resources and that only
    // happens in addToSystem. Furthermore, addToSystem may not alter the Component
    // in any way that would effect its behavior- that is why it it const!
    // The setting of the variable indices is not in the public interface and is 
    // not polymorphic.

    mutable std::map<std::string, ModelingOptionInfo> _namedModelingOptionInfo;
    // Map names of continuous state variables of the Component to their 
    // underlying SimTK indices.
    mutable std::map<std::string, StateVariableInfo> _namedStateVariableInfo;
    // Map names of discrete variables of the Component to their underlying
    // SimTK indices.
    mutable std::map<std::string, DiscreteVariableInfo> _namedDiscreteVariableInfo;
    // Map names of cache entries of the Component to their individual 
    // cache information.
    mutable std::map<std::string, CacheInfo>            _namedCacheVariableInfo;
//==============================================================================
};  // END of class Component
//==============================================================================
//==============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_COMPONENT_H_

