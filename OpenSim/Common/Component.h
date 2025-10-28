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
 * Copyright (c) 2005-2024 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
 * Contributor(s): F. C. Anderson                                             *
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
#include "ComponentList.h"
#include "ComponentPath.h"
#include "Logger.h"
#include "OpenSim/Common/Array.h"
#include "OpenSim/Common/ComponentOutput.h"
#include "OpenSim/Common/ComponentSocket.h"
#include "OpenSim/Common/Object.h"
#include "simbody/internal/MultibodySystem.h"

#include <OpenSim/Common/osimCommonDLL.h>

#include <functional>
#include <memory>
#include <type_traits>
#include <unordered_map>
#include <utility>

namespace OpenSim {

class Model;
class ModelDisplayHints;

//==============================================================================
/// Component Exceptions
//==============================================================================
class EmptyComponentPath : public Exception {
public:
    EmptyComponentPath(const std::string& file,
        size_t line,
        const std::string& methodName,
        const std::string& componentName) :
        Exception(file, line, methodName) {
        std::string msg = componentName;
        msg += "." + methodName + "() called with empty component path.\n";
        msg += "Please assign a valid path and try again.";
        addMessage(msg);
    }
};

class ComponentHasNoName : public Exception {
public:
    ComponentHasNoName(const std::string& file,
        size_t line,
        const std::string& methodName,
        const std::string& componentConcreteClassName) :
        Exception(file, line, methodName) {
        std::string msg = componentConcreteClassName;
        msg += " was constructed with no name.\n";
        msg += "Please assign a valid name and try again.";
        addMessage(msg);
    }
};

class InvalidComponentName : public Exception {
public:
    InvalidComponentName(const std::string& file,
        size_t line,
        const std::string& methodName,
        const std::string& thisName,
        const std::string& invalidChars,
        const std::string& componentConcreteClassName) :
        Exception(file, line, methodName) {
        std::string msg = "Component '" + thisName + "' of type " +
            componentConcreteClassName + " contains invalid characters of: '" +
            invalidChars + "'.";
        addMessage(msg);
    }
};

class ComponentNotFoundOnSpecifiedPath : public ComponentNotFound {
public:
    ComponentNotFoundOnSpecifiedPath(const std::string& file,
        size_t line,
        const std::string& methodName,
        const std::string& toFindName,
        const std::string& toFindClassName,
        const std::string& thisName) :
        ComponentNotFound(file, line, methodName) {
        std::string msg = "Component '" + thisName;
        msg += "' could not find '" + toFindName;
        msg += "' of type " + toFindClassName + ". ";
        msg += "Make sure a component exists at this path and that it is of ";
        msg += "the correct type.";
        addMessage(msg);
    }
};

class VariableOwnerNotFoundOnSpecifiedPath : public ComponentNotFound {
public:
    VariableOwnerNotFoundOnSpecifiedPath(const std::string& file,
        size_t line,
        const std::string& methodName,
        const std::string& componentName,
        const std::string& varName,
        const std::string& ownerPath) :
        ComponentNotFound(file, line, methodName) {
        std::string msg = componentName + "." + methodName;
        msg += "(): No component found at path = '" + ownerPath;
        msg += "' while searching for owner of variable = '" + varName + "'.";
        addMessage(msg);
    }
};

class ModelingOptionMaxExceeded : public Exception {
public:
    ModelingOptionMaxExceeded(const std::string& file,
        size_t line,
        const std::string& methodName,
        const std::string& componentName,
        const std::string& moName,
        const int flag,
        const int max) :
        Exception(file, line, methodName) {
        std::string msg = componentName + "." + methodName;
        msg += "(): called with flag = ";
        msg += std::to_string(flag) += ".\n";
        msg += "Value of modeling option '" + moName + "' cannot exceed ";
        msg += std::to_string(max) + ".";
        addMessage(msg);
    }
};

class VariableNotFound : public Exception {
public:
    VariableNotFound(const std::string& file,
        size_t line,
        const std::string& methodName,
        const std::string& componentName,
        const std::string& varName) :
        Exception(file, line, methodName) {
        std::string msg = componentName + "." + methodName;
        msg += "(): discrete variable or modeling option '";
        msg += varName + "' not found.";
        addMessage(msg);
    }
};

class ComponentIsAnOrphan : public Exception {
public:
    ComponentIsAnOrphan(const std::string& file,
        size_t line,
        const std::string& methodName,
        const std::string& thisName,
        const std::string& componentConcreteClassName) :
        Exception(file, line, methodName) {
        std::string msg = "Component '" + thisName + "' of type " +
            componentConcreteClassName + " has no owner and is not the root.\n" +
            "Verify that finalizeFromProperties() has been invoked on the " +
            "root Component or that this Component is not a clone, which has " +
            "not been added to another Component.";
        addMessage(msg);
    }
};

class SubcomponentsWithDuplicateName : public Exception {
public:
    SubcomponentsWithDuplicateName(const std::string& file,
        size_t line,
        const std::string& methodName,
        const std::string& thisName,
        const std::string& duplicateName) :
        Exception(file, line, methodName) {
        std::string msg = "Component '" + thisName + "' has subcomponents " +
            "with duplicate name '" + duplicateName + "'. "
            "Please supply unique names for immediate subcomponents.";
        addMessage(msg);
    }
};

class ComponentIsRootWithNoSubcomponents : public Exception {
public:
    ComponentIsRootWithNoSubcomponents(const std::string& file,
        size_t line,
        const std::string& methodName,
        const std::string& thisName,
        const std::string& componentConcreteClassName) :
        Exception(file, line, methodName) {
        std::string msg = "Component '" + thisName + "' of type " +
            componentConcreteClassName + " is the root but has no " +
            "subcomponents listed.\n" +
            "Verify that finalizeFromProperties() was called on this "
            "Component to identify its subcomponents.";
        addMessage(msg);
    }
};

class ComponentAlreadyPartOfOwnershipTree : public Exception {
public:
    ComponentAlreadyPartOfOwnershipTree(const std::string& file,
                                        size_t line,
                                        const std::string& methodName,
                                        const std::string& compName,
                                        const std::string& thisName) :
        Exception(file, line, methodName) {
        std::string msg = "Component '" + compName;
        msg += "' already owned by tree to which '" + thisName;
        msg += "' belongs. Clone the component to adopt a fresh copy.";
        addMessage(msg);
    }
};

class ComponentHasNoSystem : public Exception {
public:
    ComponentHasNoSystem(const std::string& file,
                         size_t line,
                         const std::string& methodName,
                         const Object& obj) :
        Exception(file, line, methodName, obj) {
        std::string msg = "Component has no underlying System.\n";
        msg += "You must call initSystem() on the top-level Component ";
        msg += "(i.e. Model) first.";
        addMessage(msg);
    }
};

class SocketNotFound : public Exception {
public:
    SocketNotFound(const std::string& file,
                   size_t line,
                   const std::string& methodName,
                   const Object& obj,
                   const std::string& socketName) :
        Exception(file, line, methodName, obj) {
        std::string msg = "no Socket '" + socketName;
        msg += "' found for this Component.";
        addMessage(msg);
    }
};

class InputNotFound : public Exception {
public:
    InputNotFound(const std::string& file,
                  size_t line,
                  const std::string& methodName,
                  const Object& obj,
                  const std::string& inputName) :
        Exception(file, line, methodName, obj) {
        std::string msg = "no Input '" + inputName;
        msg += "' found for this Component.";
        addMessage(msg);
    }
};

class OutputNotFound : public Exception {
public:
    OutputNotFound(const std::string& file,
                   size_t line,
                   const std::string& methodName,
                   const Object& obj,
                   const std::string& outputName) :
        Exception(file, line, methodName, obj) {
        std::string msg = "no Output '" + outputName;
        msg += "' found for this Component.";
        addMessage(msg);
    }
};

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
 * # Component's Interfaces
 * ## System and State
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
 * ## Discrete variables
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
 * ## Cache variables
 *
 * Fast and efficient simulations also require computationally expensive
 * calculations to be performed only when necessary. Often the result of an
 * expensive calculation can be reused many times over, while the variables it
 * is dependent on remain fixed. The concept of holding onto these values is
 * called caching and the variables that hold these values are call
 * <em>CacheVariables</em>. It is important to note, that cache variables are
 * not state variables. Cache variables can always be recomputed exactly
 * from the State. OpenSim uses the Simbody infrastructure to manage cache
 * variables and their validity. Component provides a simplified interface to
 * define and access CacheVariables.
 *
 * ## Stages
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
 * by Component when component creators implement extendAddToSystem() and use the
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
 * ## The interface of this class
 *
 * The primary responsibility of a Component is to add its computational
 * representation(s) to the underlying SimTK::System by implementing
 * extendAddToSystem().
 *
 * Additional methods provide support for adding modeling options, state and
 * cache variables.
 *
 * Public methods enable access to component variables via their names.
 *
 * ## Subcomponents
 *
 * A %Component can have any number of %Components within it; we call these
 * subcomponents. Subcomponents can also contain their own subcomponents as
 * well. There are three categories of subcomponents, which vary in whether
 * they are *configurable* and *fixed in number*:
 *
 * - **property subcomponents** Any Property in a Component that is of type
 *   Component is a subcomponent. This includes list properties and Set%s. This
 *   is the most common category of subcomponent, and its distinguishing
 *   feature is that these subcomponents are *configurable* by the user of this
 *   component. These subcomponents appear in the XML for this component, and
 *   can be modified in XML or through the API. They are also not fixed in
 *   number; users can add more property subcomponents to an existing
 *   component (though it is possible to enforce a fixed number by using
 *   one-value properties or limiting the size of a list property). The bodies,
 *   joints, forces, etc. in a Model's BodySet, JointSet, ForceSet, etc. are
 *   all examples of property subcomponents. This category of subcomponent is
 *   the most similar to what was available pre-v4.0.
 * - **member subcomponents** These are *not* configurable by the user of this
 *   Component, and can only be modified by this Component. You can
 *   still access member subcomponents through the API, but only the component
 *   containing the subcomponents can modify them. Any Component class can have
 *   any number of member subcomponents, but this number is *fixed* for every
 *   instance of the component.
 * - **adopted subcomponents** These are *not* configurable (does not appear in
 *   XML) and *not* fixed in number. For example, a component can decide,
 *   based on other aspects of the model, that it needs to create a new
 *   subcomponent. This can be done using adopted subcomponents.
 *
 * Also, any specific Component can end up in any of these three categories.
 * That is, if you have a MySpecialForce Component, any other Component can
 * have it as a property subcomponent, a member subcomponent, or as an adopted
 * subcomponent.
 *
 * @author Ajay Seth, Michael Sherman, Chris Dembia
 */
class OSIMCOMMON_API Component : public Object {
OpenSim_DECLARE_ABSTRACT_OBJECT(Component, Object);

protected:
//==============================================================================
// PROPERTIES
//==============================================================================

    OpenSim_DECLARE_LIST_PROPERTY(components, Component,
        "List of components that this component owns and serializes.");

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

    /** Use default copy constructor and assignment operator. */
    Component(const Component&) = default;
    Component& operator=(const Component&) = default;

    /** Destructor is virtual to allow concrete Component to cleanup. **/
    virtual ~Component() = default;

    /** @name Component Structural Interface
    The structural interface ensures that deserialization, resolution of
    inter-connections, and handling of dependencies are performed systematically
    and prior to system creation, followed by allocation of necessary System
    resources. These methods can be extended by virtual methods that form the
    Component Extension Interface (e.g. #extendFinalizeFromProperties)
    that can be implemented by subclasses of Components.

    Component ensures that the corresponding calls are propagated to all of its
    subcomponents.*/

    ///@{

    /** Define a Component's internal data members and structure according to
        its properties. This includes its subcomponents as part of the component
        ownership tree and identifies its owner (if present) in the tree.
        finalizeFromProperties propagates to all of the component's subcomponents
        prior to invoking the virtual extendFinalizeFromProperties() on itself.
        Note that if the Component has already been added to a System (result of
        addToSystem(); e.g., Model::initSystem()) when finalizeFromProperties()
        is called, then finalizeFromProperties() disassociates the component from
        that System.*/
    void finalizeFromProperties();

    /** Satisfy the Component's connections specified by its Sockets and Inputs.
        Locate Components and their Outputs to satisfy the connections in an
        aggregate Component (e.g. Model), which is the root of a tree of
        Components. */
    void finalizeConnections(Component& root);

    /** Disconnect/clear this Component from its aggregate component. Empties
        all component's sockets and sets them as disconnected.*/
    void clearConnections();

    /** Have the Component add itself to the underlying computational System */
    void addToSystem(SimTK::MultibodySystem& system) const;

    /** Initialize Component's state variable values from its properties */
    void initStateFromProperties(SimTK::State& state) const;

    /** %Set Component's properties given a state. */
    void setPropertiesFromState(const SimTK::State& state);

    // End of Component Structural Interface (public non-virtual).
    ///@}

    /** Optional method for generating arbitrary display geometry that reflects
    this %Component at the specified \a state. This will be called once to
    obtain ground- and body-fixed geometry (with \a fixed=\c true), and then
    once per frame (with \a fixed=\c false) to generate on-the-fly geometry such
    as rubber band lines, force arrows, labels, or debugging aids.

    Please note that there is a precondition that the state passed in to
    generateDecorations be realized to Stage::Position. If your component can
    visualize quantities realized at Velocity, Dynamics or Acceleration stages,
    then you must check that the stage has been realized before using/requesting
    stage dependent values. It is forbidden to realize the model to a higher
    stage within generateDecorations, because this can trigger costly side-
    effects such as evaluating all model forces even when performing a purely
    kinematic study.

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
        // can render velocity dependent quanities if stage is Velocity or higher
        if(state.getSystemStage() >= Stage::Velocity) {
            // draw velocity vector for model COM
        }
        // can render computed forces if stage is Dynamics or higher
        if(state.getSystemStage() >= Stage::Dynamics) {
            // change the length of a force arrow based on the force in N
        }
    }
    @endcode

    @param[in]      fixed
        If \c true, generate only geometry that is fixed to a PhysicalFrame,
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
            (bool                                       fixed,
            const ModelDisplayHints&                    hints,
            const SimTK::State&                         state,
            SimTK::Array_<SimTK::DecorativeGeometry>&   appendToThis) const {};

    /**
     * Get the underlying MultibodySystem that this component is connected to.
     * Make sure you have called Model::initSystem() prior to accessing the System.
     * Throws an Exception if the System has not been created or the Component
     * has not added itself to the System.
     * @see hasSystem().  */
    const SimTK::MultibodySystem& getSystem() const
    {
        // This method is inlined because it is called *a lot* at runtime

        OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);
        return _system.getRef();
    }

    /**
    * Check if this component has an underlying MultibodySystem.
    * Returns false if the System has not been created OR if this
    * Component has not added itself to the System.  */
    bool hasSystem() const { return !_system.empty(); }

    /** Does the provided component already exist anywhere in the ownership
     * tree (not just subcomponents of this component)? */
    bool isComponentInOwnershipTree(const Component* component) const;

    /**
    * Add a Component (as a subcomponent) of this component.
    * This component takes ownership of the subcomponent and it will be
    * serialized (appear in XML) as part of this component. Specifically,
    * it will appear in the `<components>` list for this Component.
    * If the subcomponent is already owned by this component or exists
    * in the same hierarchy (tree) as this component, an Exception
    * is thrown.
    * @note addComponent is intended to replace existing addBody(), addJoint,
    *       ... on Model or the requirement for specific add###() methods to
    *       subcomponents to a Component.
    *
    * Typical usage is:
    @code
        // Start with an empty Model (which is a Component)
        Model myModel;
        // Create any Component type on the heap
        Body* newBody = new Body();
        // Customize the Component by setting its properties
        newBody->setName("newBody");
        newBody->setMass(10.0);
        newBody->setMassCenter(SimTK::Vec3(0));
        // ...
        // Now add it to your model, which will take ownership of it
        myModel.addComponent(newBody);
        //
        // Keep creating and adding new components, like Joints, Forces, etc..
    @endcode
    *
    * @throws ComponentAlreadyPartOfOwnershipTree
    * @param subcomponent is the Component to be added. */
    void addComponent(Component* subcomponent);

#ifndef SWIG
    /**
     * Returns `subcomponent` if `subcomponent` was successfully extracted from
     * this `Component`.
     *
     * Specifically, this tries to extract a direct sub-`Component` that was
     * previously added via `addComponent`, or existed in the `<components>` list
     * XML of this `Component`.
     */
    std::unique_ptr<Component> extractComponent(Component* subcomponent);

    template<
        typename T,
        std::enable_if_t<std::is_base_of_v<Component, T>, bool> = true
    >
    std::unique_ptr<T> extractComponent(T* subcomponent)
    {
        // If the type of `subcomponent` is known at compile-time then it is also
        // known that the return value will be `subcomponent`, which will have the
        // same type.
        std::unique_ptr<Component> typeErased = extractComponent(static_cast<Component*>(subcomponent));
        return std::unique_ptr<T>{static_cast<T*>(typeErased.release())};
    }
#endif

    /**
     * Returns `true` if `subcomponent`, which should be a direct subcomponent
     * of this component, was successfully removed from this component.
     *
     * Specifically, this tries to remove a direct component that was
     * previously added via `addComponent`, or existed in the `<components>`
     * list XML for this component.
     */
    bool removeComponent(Component* subcomponent);

    /**
     * Get an iterator through the underlying subcomponents that this component is
     * composed of. The hierarchy of Components/subComponents forms a tree.
     * The order of the Components is that of tree preorder traversal so that a
     * component is traversed before its subcomponents.
     *
     * @code{.cpp}
     * for (const auto& muscle : model.getComponentList<Muscle>()) {
     *     muscle.get_max_isometric_force();
     * }
     * @endcode
     *
     * The returned ComponentList does not permit modifying any components; if
     * you want to modify the components, see updComponentList().
     *
     * @tparam T A subclass of Component (e.g., Body, Muscle).
     */
    template <typename T = Component>
    ComponentList<const T> getComponentList() const {
        static_assert(std::is_base_of<Component, T>::value,
                "Template argument must be Component or a derived class.");
        initComponentTreeTraversal(*this);
        return ComponentList<const T>(*this);
    }

    /** Similar to getComponentList(), except the resulting list allows one to
    modify the components. For example, you could use this method to change
    the max isometric force of all muscles:

    @code{.cpp}
    for (auto& muscle : model.updComponentList<Muscle>()) {
        muscle.set_max_isometric_force(...);
    }
    @endcode

    @note Do NOT use this method to add (or remove) (sub)components from any
    component. The tree structure of the components should not be altered
    through this ComponentList.

    @tparam T A subclass of Component (e.g., Body, Muscle). */
    template <typename T = Component>
    ComponentList<T> updComponentList() {
        static_assert(std::is_base_of<Component, T>::value,
                "Template argument must be Component or a derived class.");
        initComponentTreeTraversal(*this);
        clearObjectIsUpToDateWithProperties();
        return ComponentList<T>(*this);
    }

    /**
     * Uses getComponentList<T>() to count the number of underlying
     * subcomponents of the specified type.
     *
     * @tparam T A subclass of Component (e.g., Body, Muscle).
     */
    template <typename T = Component>
    unsigned countNumComponents() const {
        unsigned count = 0u;
        const auto compList = getComponentList<T>();
        auto it = compList.begin();
        while (it != compList.end()) {
            ++count;
            ++it;
        }
        return count;
    }

    /** Class that permits iterating over components/subcomponents (but does
     * not actually contain the components themselves). */
    template <typename T>
    friend class ComponentList;
    /** Class to iterate over ComponentList returned by getComponentList(). */
    template <typename T>
    friend class ComponentListIterator;


    /** Get the complete (absolute) pathname for this Component to its ancestral
     * Component, which is the root of the tree to which this Component belongs.
     * For example: a Coordinate Component would have an absolute path name
     * like: `/arm26/elbow_r/flexion`. Accessing a Component by its
     * absolutePathName from root is guaranteed to be unique. The
     * absolutePathName is generated on-the-fly by traversing the ownership tree
     * and, therefore, calling this method is not "free". */
    std::string getAbsolutePathString() const;

    /** Return a ComponentPath of the absolute path of this Component.
     * Note that this has more overhead than calling `getName()` because
     * it traverses up the tree to generate the absolute pathname (and its
     * computational cost is thus a function of depth). Consider other
     * options if this is repeatedly called and efficiency is important.
     * For instance, `getAbsolutePathString()` is faster if you only
     * need the path as a string. */
    ComponentPath getAbsolutePath() const;

    /** Get the relative path of this Component with respect to another
     * Component, as a string. */
    std::string getRelativePathString(const Component& wrt) const;

    /** Get the relative path of this Component with respect to another
     * Component. */
    ComponentPath getRelativePath(const Component& wrt) const;

    /** Query if there is a component (of any type) at the specified
     * path name. For example,
     * @code
     * bool exists = model.hasComponent("right_elbow/elbow_flexion");
     * @endcode
     * checks if `model` has a subcomponent "right_elbow," which has a
     * subcomponent "elbow_flexion." */
    bool hasComponent(const std::string& pathname) const {
        return hasComponent<Component>(pathname);
    }

    /** Query if there is a component of a given type at the specified
     * path name. For example,
     * @code
     * bool exists = model.hasComponent<Coordinate>("right_elbow/elbow_flexion");
     * @endcode
     * checks if `model` has a subcomponent "right_elbow," which has a
     * subcomponent "elbow_flexion," and that "elbow_flexion" is of type
     * Coordinate. This method cannot be used from scripting; see the
     * non-templatized hasComponent(). */
    template <class C = Component>
    bool hasComponent(const std::string& pathname) const {
        static_assert(std::is_base_of<Component, C>::value,
            "Template parameter 'C' must be derived from Component.");
        const C* comp = this->template traversePathToComponent<C>({pathname});
        return comp != nullptr;
    }

    /**
     * Get a unique subcomponent of this Component by its path name and type 'C'.
     * Throws ComponentNotFoundOnSpecifiedPath exception if the component at
     * that path name location does not exist OR it is not of the correct type.
     * For example,
     * @code
     *    auto& coord = model.getComponent<Coordinate>("right_elbow/elbow_flexion");
     * @endcode
     * returns coord which is a Coordinate named "elbow_flexion" from a Joint
     * named "right_elbow" given it is a child of the Component (Model) model.
     * If unsure of a Component's path or whether or not it exists in the model,
     * use printComponentsMatching() or hasComponent().
     *
     * This template function cannot be used in Python/Java/MATLAB; see the
     * non-templatized getComponent().
     *
     * @param  pathname        a pathname of a Component of interest
     * @return const reference to component of type C at
     * @throws ComponentNotFoundOnSpecifiedPath if no component exists
     */
    template <class C = Component>
    const C& getComponent(const std::string& pathname) const {
        return getComponent<C>(ComponentPath(pathname));
    }
    template <class C = Component>
    const C& getComponent(const ComponentPath& pathname) const {
        static_assert(std::is_base_of<Component, C>::value,
            "Template parameter 'CompType' must be derived from Component.");

        const C* comp = this->template traversePathToComponent<C>(pathname);
        if (comp) {
            return *comp;
        }

        // Only error cases remain
        OPENSIM_THROW(ComponentNotFoundOnSpecifiedPath, pathname.toString(),
                                                       C::getClassName(),
                                                       getName());
    }

    /** Similar to the templatized getComponent(), except this returns the
     * component as the generic Component type. This can be used in
     * Python/Java/MATLAB. Here is an example of using this in MATLAB:
     * @code
     * coord = model.getComponent('right_elbow/elbow_flexion')
     * coord.getNumConnectees() % okay; this is a Component method.
     * coord.getDefaultClamped() % inaccessible; method on Coordinate.
     * Coordinate.safeDownCast(coord).getDefaultClamped() % now accessible.
     * @endcode
     *
     * %Exception: in Python, you will get the concrete type (in most cases):
     * @code{.py}
     * coord = model.getComponent('right_elbow/elbow_flexion')
     * coord.getDefaultClamped() # works; no downcasting necessary.
     * @endcode
     */
    const Component& getComponent(const std::string& pathname) const {
        return getComponent<Component>(pathname);
    }

    /** Get a writable reference to a subcomponent. Use this method
    * to edit the properties and connections of the subcomponent.
    * Note: the method will mark this Component as out-of-date with
    * its properties and will require finalizeFromProperties() to be
    * invoked directly or indirectly (by finalizeConnections() or
    * Model::initSystem())
    * @param name       the pathname of the Component of interest
    * @return Component the component of interest
    * @throws ComponentNotFoundOnSpecifiedPath if no component exists
    * @see getComponent()
    */
    template <class C = Component>
    C& updComponent(const std::string& name) {
        return updComponent<C>(ComponentPath(name));
    }
    template <class C = Component>
    C& updComponent(const ComponentPath& name) {
        clearObjectIsUpToDateWithProperties();
        return *const_cast<C*>(&(this->template getComponent<C>(name)));
    }

    /** Similar to the templatized updComponent(), except this returns the
     * component as the generic Component type. As with the non-templatized
     * getComponent(), though, this will give the concrete type in Python in
     * most cases.
     * @see getComponent()
     */
    Component& updComponent(const std::string& pathname) {
        return updComponent<Component>(pathname);
    }


    /** Print a list to the console of all components whose absolute path name
     * contains the given string. You might use this if (a) you know the name
     * of a component in your model but don't know its absolute path, (b) if
     * you want to find all components with a given name, or (c) to get a list
     * of all components on the right leg of a model (if all components on the
     * right side have "_r" in their name).
     *
     * A function call like:
     * @code{.cpp}
     * unsigned num = comp.printComponentsMatching("rotation");
     * @endcode
     * may produce output like:
     * @verbatim
     * /leg_model/right_hip/rotation
     * /leg_model/left_hip/rotation
     * @endverbatim
     *
     * @returns The number of matches. */
    unsigned printComponentsMatching(const std::string& substring) const;

    /**
     * Get the number of "continuous" state variables maintained by the
     * Component and its subcomponents.
     * @throws ComponentHasNoSystem if this Component has not been added to a
     *         System (i.e., if initSystem has not been called)
     */
    int getNumStateVariables() const;

    /**
     * Get the names of continuous state variables maintained by the
     * Component and its subcomponents. Each variable's name is prepended
     * by its path in the component hierarchy.
     * @throws ComponentHasNoSystem if this Component has not been added to a
     *         System (i.e., if initSystem has not been called)
     */
    Array<std::string> getStateVariableNames() const;

    /**
    * Get the names of discrete state variables maintained by the Component
    * and its subcomponents. Each variable's name is prepended by its path in
    * the component hierarchy.
    * @throws ComponentHasNoSystem if this Component has not been added to a
    *         System (i.e., if initSystem has not been called)
    */
    Array<std::string> getDiscreteVariableNames() const;

    /**
    * Get the names of the modeling options maintained by the Component
    * and its subcomponents. Each options's name is prepended by its path in
    * the component hierarchy.
    * @throws ComponentHasNoSystem if this Component has not been added to a
    *         System (i.e., if initSystem has not been called)
    */
    Array<std::string> getModelingOptionNames() const;

    /** @name Component Socket Access methods
        Access Sockets of this component by name. */
    //@{
    /** Get the number of Sockets in this Component. */
    int getNumSockets() const {
        return int(_socketsTable.size());
    }

    /** Collect and return the names of the sockets in this component. You
     * can use this to iterate through the sockets:
     * @code
     * for (std::string name : comp.getSocketNames()) {
     *     const AbstractSocket& socket = getSocket(name);
     * }
     * @endcode */
    std::vector<std::string> getSocketNames() const {
        std::vector<std::string> names;
        for (const auto& it : _socketsTable) {
            names.push_back(it.first);
        }
        return names;
    }

    /**
    * Get the "connectee" object that the Component's Socket
    * is bound to. Guaranteed to be valid only after the Component
    * has been connected (that is connect() has been invoked).
    * If the Socket has not been connected, an exception is thrown.
    *
    * This method is for getting the concrete connectee object, and is not
    * available in scripting. If you want generic access to the connectee as an
    * Object, use the non-templated version.
    *
    * @tparam T         the type of the Connectee (e.g., PhysicalFrame).
    * @param name       the name of the socket
    * @return T         const reference to object that satisfies
    *                   the Socket
    *
    * Example:
    * @code
    * const PhysicalFrame& frame = joint.getConnectee<PhysicalFrame>("parent_frame");
    * frame.getMobilizedBody();
    * @endcode
    */
    template<typename T>
    const T& getConnectee(const std::string& name) const {
        // get the Socket and check if it is connected.
        const Socket<T>& socket = getSocket<T>(name);
        OPENSIM_ASSERT_FRMOBJ(socket.isConnected());
        return socket.getConnectee();
    }

    /**
    * Get the "connectee" object at the provided index that the Component's
    * Socket is bound to. Guaranteed to be valid only after the Component
    * has been connected (that is, connect() has been invoked).
    * If the Socket has not been connected, an exception is thrown.
    *
    * This method is for getting the concrete connectee object, and is not
    * available in scripting. If you want generic access to the connectee as an
    * Object, use the non-templated version.
    *
    * @tparam T         the type of the Connectee (e.g., Actuator).
    * @param name       the name of the socket
    * @param index      the index of the connectee
    * @return T         const reference to object that satisfies
    *                   the Socket
    *
    * Example:
    * @code
    * const Actuator& actu = controller.getConnectee<Actuator>("actuators", 1);
    * actu.getDefaultControls();
    * @endcode
    */
    template<typename T>
    const T& getConnectee(const std::string& name, int index) const {
        // get the Socket and check if it is connected.
        const Socket<T>& socket = getSocket<T>(name);
        OPENSIM_ASSERT_FRMOBJ(socket.isConnected());
        return socket.getConnectee(index);
    }

    /** Get the connectee as an Object. This means you will not have
    * access to the methods on the concrete connectee. This is the method you
    * must use in MATLAB to access the connectee.
    *
    * Example:
    * @code{.cpp}
    * const Object& obj = joint.getConnectee("parent_frame");
    * obj.getName(); // method on Object works.
    * obj.getMobilizedBody(); // error: not available.
    * @endcode
    *
    * In MATLAB, if you want the concrete type, you need to downcast the
    * Object. Here is an example where you know the "parent_frame" is a Body:
    * @code
    * f = joint.getConnectee('parent_frame');
    * m = Body.safeDownCast(f).getMass();
    * @endcode
    *
    * Exception: in Python, you will get the concrete type (in most cases):
    * @code{.py}
    * f = joint.getConnectee("parent_frame");
    * m = f.getMass() # works (if the parent frame is a body)
    * @endcode
    */
    const Object& getConnectee(const std::string& name) const {
        const AbstractSocket& socket = getSocket(name);
        OPENSIM_ASSERT_FRMOBJ(socket.isConnected());
        return socket.getConnecteeAsObject();
    }

    /** Get the connectee at the provided index as an Object. This means you
    * will not have access to the methods on the concrete connectee. This is the
    * method you must use in scripts to access the connectee.
    *
    * Example:
    * @code{.cpp}
    * const Object& obj = controller.getConnectee("actuators", 1);
    * obj.getName(); // method on Object works.
    * obj.getDefaultControls(); // error: not available.
    * @endcode
    *
    * In MATLAB, if you want the concrete type, you need to downcast the
    * Object. Here is an example where you know the "actuators" are Actuators:
    * @code
    * actu = controller.getConnectee('actuators', 1);
    * controls = Actuator.safeDownCast(f).getDefaultControls();
    * @endcode
    *
    * Exception: in Python, you will get the concrete type (in most cases):
    * @code{.py}
    * actu = controller.getConnectee("actuators", 1);
    * controls = actu.getDefaultControls() # works (if 'actu' is an Actuator)
    * @endcode
    */
    const Object& getConnectee(const std::string& name, int index) const {
        const AbstractSocket& socket = getSocket(name);
        OPENSIM_ASSERT_FRMOBJ(socket.isConnected());
        return socket.getConnecteeAsObject(index);
    }

    /**
     * Returns a pointer to the AbstractSocket with a given socket name, or `nullptr`
     * if the socket with the given name does not exist on the component.
     *
     * See `getSocket()` for more details about how the socket is looked up.
     *
     * <b>C++ example</b>
     * @code{.cpp}
     * if (const AbstractSocket* s = component.tryGetSocket("frame")) {
     *     // do something with *s
     * }
     * else {
     *     // handle the no-socket-by-that-name case
     * }
     * @endcode
     */
    const AbstractSocket* tryGetSocket(const std::string& name) const {
        auto it = _socketsTable.find(name);

        if (it != _socketsTable.end()) {
            // The following allows one to use a Socket immediately after
            // copying the component;
            // e.g., myComponent.clone().getSocket("a").getConnecteePath().
            // Since we use the default copy constructor for Component,
            // the copied AbstractSocket cannot know its new owner
            // immediately after copying.
            if (!it->second->hasOwner()) {
                // The `this` pointer must be non-const because the Socket
                // will want to be able to modify the connectee path property.
                const_cast<AbstractSocket*>(it->second.get())->setOwner(
                    const_cast<Self&>(*this));
            }
            return it->second.get();
        }
        else {
            return nullptr;
        }
    }

    /**
     * Get an AbstractSocket for the given socket name. This
     * lets you get information about the connection (like if the socket is
     * connected), but does not give you access to the socket's connectee.
     * For that, use getConnectee().
     *
     * @internal If you have not yet called finalizeFromProperties() on this
     * component, this function will update the Socket (to tell it which
     * component it's in) before providing it to you.
     *
     * <b>C++ example</b>
     * @code{.cpp}
     * model.getComponent("/path/to/component").getSocket("socketName");
     * @endcode
     */
    const AbstractSocket& getSocket(const std::string& name) const {
        const AbstractSocket* ptr = tryGetSocket(name);

        if (ptr) {
            return *ptr;
        }
        else {
            OPENSIM_THROW_FRMOBJ(SocketNotFound, name);
        }
    }

    /**
     * Returns a writable pointer to the AbstractSocket with a given socket
     * name, or `nullptr` if the socket with the given name does not exist
     * on the component.
     *
     * See `tryGetSocket` for usage example
     * See `getSocket`/`updSocket` for other internal details
     */
    const AbstractSocket* tryUpdSocket(const std::string& name) {
        return const_cast<AbstractSocket*>(tryGetSocket(name));
    }

    /** Get a writable reference to the AbstractSocket for the given
     * socket name. Use this method to connect the Socket to something.
     *
     * <b>C++ example</b>
     * @code
     * joint.updSocket("parent_frame").connect(model.getGround());
     * @endcode
     *
     * @internal If you have not yet called finalizeFromProperties() on this
     * component, this function will update the Socket (to tell it which
     * component it's in) before providing it to you.
     */
    AbstractSocket& updSocket(const std::string& name) {
        return const_cast<AbstractSocket&>(getSocket(name));
    }

    /**
    * Get a const reference to the concrete Socket provided by this
    * Component by name.
    *
    * @internal If you have not yet called finalizeFromProperties() on this
    * component, this function will update the Socket (to tell it which
    * component it's in) before providing it to you.
    *
    * @param name       the name of the Socket
    * @return const reference to the (Abstract)Socket
    */
    template<typename T>
    const Socket<T>& getSocket(const std::string& name) const {
        return Socket<T>::downcast(getSocket(name));
    }

    /**
    * Get a writable reference to the concrete Socket provided by this
    * Component by name.
    *
    * @internal If you have not yet called finalizeFromProperties() on this
    * component, this function will update the Socket (to tell it which
    * component it's in) before providing it to you.
    *
    * @param name       the name of the Socket
    * @return const reference to the (Abstract)Socket
    */
    template<typename T> Socket<T>& updSocket(const std::string& name) {
        return const_cast<Socket<T>&>(getSocket<T>(name));
    }
    //@} end of Component Socket Access methods

    /** @name Component Inputs and Outputs Access methods
        Access inputs and outputs by name and iterate over all outputs.
    */
    //@{

    /** Access the number of Inputs that this component has. */
    int getNumInputs() const {
        return int(_inputsTable.size());
    }

    /** Access the number of Outputs that this component has. */
    int getNumOutputs() const {
        return int(_outputsTable.size());
    }

    /** Collect and return the names of Inputs in this component as an
     * std::vector. */
    std::vector<std::string> getInputNames() const {
        std::vector<std::string> names;
        for (const auto& it : _inputsTable) {
            names.push_back(it.first);
        }
        return names;
    }

    /** Collect and return the names of Outputs in this component as an
     * std::vector. */
    std::vector<std::string> getOutputNames() const {
        std::vector<std::string> names;
        for (const auto& entry : getOutputs()) {
            names.push_back(entry.first);
        }
        return names;
    }

    /**
    * Get an Input provided by this Component by name.
    *
    * <b>C++ example:</b> get an Input from a Component in the model
    * @code{.cpp}
    * model.getComponent("/path/to/component").getInput("inputName");
    * @endcode
    *
    * @internal If you have not yet called finalizeFromProperties() on this
    * component, this function will update the Input (to tell it which
    * component it's in) before providing it to you.
    *
    * @param name   the name of the Input
    * @return       const reference to the AbstractInput
    */
    const AbstractInput& getInput(const std::string& name) const
    {
        auto it = _inputsTable.find(name);

        if (it != _inputsTable.end()) {
            // The following allows one to use an Input immediately after
            // copying the component;
            // e.g., myComponent.clone().getInput("a").getConnecteePath().
            // Since we use the default copy constructor for Component,
            // the copied AbstractSocket (base class of AbstractInput)
            // cannot know its new owner immediately after copying.
            if (!it->second->hasOwner()) {

                // The `this` pointer must be non-const because the Socket
                // will want to be able to modify the connectee_name property.
                const_cast<AbstractInput*>(it->second.get())->setOwner(
                        const_cast<Self&>(*this));
            }
            return it->second.getRef();
        }

        OPENSIM_THROW_FRMOBJ(InputNotFound, name);
    }

    /**
    * Get a writable reference to an Input provided by this Component by name.
    *
    * <b>C++ example:</b> get a writable reference to an Input of a
    * Component in a model
    * @code{.cpp}
    * model.updComponent("/path/to/component").updInput("inputName");
    * @endcode
    *
    * @internal If you have not yet called finalizeFromProperties() on this
    * component, this function will update the Input (to tell it which
    * component it's in) before providing it to you.

    * @param name   the name of the Input
    * @return       reference to the AbstractInput
    */
    AbstractInput& updInput(const std::string& name)
    {
        return *const_cast<AbstractInput *>(&getInput(name));
    }


    /**
    * Get a concrete Input that you can directly ask for its values.
    *
    * @internal If you have not yet called finalizeFromProperties() on this
    * component, this function will update the Input (to tell it which
    * component it's in) before providing it to you.
    *
    * @param name   the name of the Input
    * @throws Exception if an Input with the given name does not exist.
    * @throws std::bad_cast if the provided type T is incorrect for the given name.
    */
    template<typename T>
    const Input<T>& getInput(const std::string& name) const {
        return dynamic_cast<const Input<T>&>(getInput(name));
    }

    /**
     * If it exists on the component, returns a pointer to the named `Output`; otherwise,
     * returns a `nullptr`.
     *
     * Related: `getOutput`
     *
     * @param name  the name the `Output` to find
     * @return      if it exists, a pointer to the `Output`; otherwise, `nullptr`
     */
    const AbstractOutput* tryGetOutput(const std::string& name) const
    {
        auto it = _outputsTable.find(name);
        if (it != _outputsTable.end()) {
            return it->second.get();
        }
        else {
            return nullptr;
        }
    }

    /**
    * Get the Output provided by this Component by name.
    *
    * <b>C++ example:</b> get an Output from a Component in a model
    * @code{.cpp}
    * model.getComponent("/path/to/component").getOutput("outputName");
    * @endcode
    *
    * @param name   the name of the Output
    * @return       const reference to the AbstractOutput
    */
    const AbstractOutput& getOutput(const std::string& name) const
    {
        if (auto ptr = tryGetOutput(name)) {
            return *ptr;
        }
        else {
            OPENSIM_THROW_FRMOBJ(OutputNotFound, name);
        }
    }

    /**
     * If it exists on the component returns a writable pointer to the named `Output`; otherwise,
     * returns a `nullptr`
     *
     * Related: `updOutput`
     *
     * @param name  the name of the `Output` to find
     * @return      if it exists, a writable pointer to the `Output`; otherwise, `nullptr`
     */
    AbstractOutput* tryUpdOutput(const std::string& name)
    {
        return const_cast<AbstractOutput*>(tryGetOutput(name));
    }

    /**
    * Get a writable reference to an Output provided by this Component by name.
    *
    * <b>C++ example:</b> get a writable reference to an Output of a
    * Component in a model
    * @code{.cpp}
    * model.updComponent("/path/to/component").updOutput("outputName");
    * @endcode

    * @param name   the name of the Output
    * @return       reference to the AbstractOutput
    */
    AbstractOutput& updOutput(const std::string& name)
    {
        return *const_cast<AbstractOutput *>(&getOutput(name));
    }

    /** Define OutputConstIterator for convenience */
    typedef std::map<std::string, SimTK::ClonePtr<AbstractOutput>>::
        const_iterator OutputConstIterator;

    /** Iterate through all Outputs of this component. The intent is to use
     * this in a loop as such:
     * @code
     * for (const auto& entry : comp.getOutputs()) {
     *     const std::string& name = entry.first;
     *     const AbstractOutput* output = entry.second.get();
     *     std::cout << output->getTypeName() << std::endl;
     * }
     * @endcode
     * This provides access to the outputs as AbstractOutput%s, not as the
     * concrete type. This also does not permit modifying the outputs.
     *
     * Not available in Python/Java/MATLAB; use getOutputNames() and
     * getOutput() instead.
     */
    SimTK::IteratorRange<OutputConstIterator> getOutputs() const {
        return {_outputsTable.cbegin(), _outputsTable.cend()};
    }
    //@} end of Component Inputs and Outputs Access methods



    /** @name Component State Access methods
        Get and set modeling option, input and output values, state variable,
        discrete and/or cache variables in the State.
     */
    //@{

    /**
     * Based on a specified path, get the value of a modeling option.
     *
     * The specified path consists of the name of the modeling option
     * prepended by its absolute or relative path in the component hierarchy.
     *
     * If this component is the owner of the modeling option, the specified
     * path should simply be the name of the modeling option.
     *
     * @param state State in which to set the modeling option.
     * @param path Path of the modeling option in the component hierarchy.
     * @return flag Value of the modeling option.
     * @throws EmptyComponentPath if the specified path is an empty string
     * (i.e., path == "").
     * @throws VariableOwnerNotFoundOnSpecifiedPath if the candidate owner
     * of the modeling option cannot be found at the specified path.
     * @throws VariableNotFound if the specified modeling option cannot be
     * found in the candidate owner.
     * @see Component::resolveVariableNameAndOwner()
     */
    int getModelingOption(const SimTK::State& state,
        const std::string& path) const;
    int getModelingOption(const SimTK::State& state,
        const ComponentPath& path) const;

    /**
     * Based on a specified path, set the value of a modeling option.
     *
     * The specified path consists of the name of the modeling option
     * prepended by its absolute or relative path in the component hierarchy.
     *
     * If this component is the owner of the modeling option, the specified
     * path should simply be the name of the modeling option.
     *
     * @note Successfully setting the value of the modeling option will revert
     * the realization stage back to SimTK::Stage::Instance.
     *
     * @param state State in which to set the modeling option.
     * @param path Path of the modeling option in the component hierarchy.
     * @param flag Value to which to set the modeling option.
     * @throws ModelingOptionMaxExceeded if the flag is greater that the
     * maximum allowed for the specified modeling option.
     * @throws EmptyComponentPath if the specified path is an empty string
     * (i.e., path == "").
     * @throws VariableOwnerNotFoundOnSpecifiedPath if the candidate owner
     * of the modeling option cannot be found at the specified path.
     * @throws VariableNotFound if the specified modeling option cannot be
     * found in the candidate owner.
     * @see Component::resolveVariableNameAndOwner()
     */
    void setModelingOption(SimTK::State& state, const std::string& path,
        int flag) const;
    void setModelingOption(SimTK::State& state, const ComponentPath& path,
        int flag) const;

    /**
    * Get the Input value that this component is dependent on.
    * Checks if Input is connected, otherwise it will throw an
    * exception. You can only call this method for non-list inputs.
    * For list inputs, you must get the input using getInput(),
    * from which you can ask for its values.
    *
    * @param state      the State for which to set the value
    * @param name       the name of the input
    * @return T         const Input value
    */
    template<typename T> const T&
        getInputValue(const SimTK::State& state, const std::string& name) const {
        // get the input and check if it is connected.
        const AbstractInput& in = getInput(name);
        // TODO could maybe remove this check and have the Input do it. Or,
        // here, we could catch Input's exception and give a different message.
        if (in.isConnected()) {
            return (Input<T>::downcast(in)).getValue(state);
        }

        else {
        std::stringstream msg;
            msg << "Component::getInputValue: ERR- Input '" << name << "' not connected.\n "
                << "for component '" << getName() << "' of type "<< getConcreteClassName();
        throw Exception(msg.str(), __FILE__, __LINE__);
        }
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
     * @throws ComponentHasNoSystem if this Component has not been added to a
     *         System (i.e., if initSystem has not been called)
     */
    double getStateVariableValue(const SimTK::State& state, const std::string& name) const;

    /**
     * Get the value of a state variable allocated by this Component.
     *
     * To connect this StateVariable as an input to another component (such as
     * a Reporter), use getOutput(name); each state variable has a
     * corresponding Output:
     *
     *  @code
     *  foo.getInput("input1").connect(bar.getOutput(name));
     *  @endcode
     *
     * @param state   the State for which to get the value
     * @param path    path to the state variable of interest
     * @throws ComponentHasNoSystem if this Component has not been added to a
     *         System (i.e., if initSystem has not been called)
     */
    double getStateVariableValue(const SimTK::State& state, const ComponentPath& path) const;

    /**
     * %Set the value of a state variable allocated by this Component by name.
     *
     * @param state  the State for which to set the value
     * @param name   the name of the state variable
     * @param value  the value to set
     * @throws ComponentHasNoSystem if this Component has not been added to a
     *         System (i.e., if initSystem has not been called)
     */
    void setStateVariableValue(SimTK::State& state, const std::string& name, double value) const;


    /**
     * Get all values of the state variables allocated by this Component.
     * Includes state variables allocated by its subcomponents.
     *
     * @param state   the State for which to get the value
     * @return Vector of state variable values of length getNumStateVariables()
     *                in the order returned by getStateVariableNames()
     * @throws ComponentHasNoSystem if this Component has not been added to a
     *         System (i.e., if initSystem has not been called)
     */
    SimTK::Vector getStateVariableValues(const SimTK::State& state) const;

    /**
     * %Set all values of the state variables allocated by this Component.
     * Includes state variables allocated by its subcomponents. Note, this
     * method simply sets the values on the input State. If other conditions
     * must be met (such as satisfying kinematic constraints for Coordinates,
     * or fiber and tendon equilibrium for muscles) you must invoke the
     * appropriate methods on Model (e.g. assemble() to satisfy constraints or
     * equilibrateMuscles()) to satisfy these conditions starting from the
     * State values provided by setStateVariableValues.
     *
     * @param state   the State whose values are set
     * @param values  Vector of state variable values of length
     *                getNumStateVariables() in the order returned by
     *                getStateVariableNames()
     * @throws ComponentHasNoSystem if this Component has not been added to a
     *         System (i.e., if initSystem has not been called)
     */
    void setStateVariableValues(SimTK::State& state,
                                const SimTK::Vector& values) const;

    /**
     * Get the value of a state variable derivative computed by this Component.
     *
     * @param state   the State for which to get the derivative value
     * @param name    the name (string) of the state variable of interest
     * @throws ComponentHasNoSystem if this Component has not been added to a
     *         System (i.e., if initSystem has not been called)
     */
    double getStateVariableDerivativeValue(const SimTK::State& state,
        const std::string& name) const;

    /**
     * Get the value of a state variable derivative computed by this Component.
     *
     * @param state   the State for which to get the derivative value
     * @param path    path to the state variable of interest
     * @throws ComponentHasNoSystem if this Component has not been added to a
     *         System (i.e., if initSystem has not been called)
     */
    double getStateVariableDerivativeValue(const SimTK::State& state,
        const ComponentPath& path) const;

    /**
    * Based on a specified path, resolve the name of a state variable,
    * discrete variable, or modeling option and resolve the component that
    * owns it (i.e., its parent).
    *
    * The path consists of the name of the state variable, discrete variable,
    * or modeling option prepended by its absolute or relative path in the
    * component hierarchy.
    *
    * This method does not verify that the variable or option can actually be
    * found at the spcified path. It simply parses the path, returning the
    * variable name and candidate owner.
    *
    * @note Calling Component::traversPathToComponent<Component>() will not
    * work for state variables, discrete variables, and modeling options
    * because these obects are not themselves Components. However, a pointer
    * to a StateVariable can be obtained in a single step by calling
    * Component::traverseToStateVariable(). A similar dedicated method is not
    * available for discrete variables or for modeling options.
    *
    * #### Example Paths
    *
    * A relative path in which this component is the owner:
    *   ```variable_name```
    *
    * An absolute path from the root of the component hierarchy:
    *   ```/grandparent_name/parent_name/variable_name```
    *
    * A relative path in which a sibling of this component is the owner:
    *   ```../sibling_name/variable_name```
    *
    * @param path Path of the state variable, discrete variable, or modeling
    * option in the component heirarchy.
    * @param varName The name of the state variable, discrete variable,
    * or modeling option is returned in this parameter.
    * @return Pointer to the component that, according to the path, owns the
    * state variable, discrete variable, or modeling option. If the specified
    * path consists only of the name of the state variable, discrete variable,
    * or modeling option, a pointer to this component is returned.
    * @throws EmptyComponentPath if the specified path is an empty string
    * (i.e., path == "").
    * @throws VariableOwnerNotFoundOnSpecifiedPath if the candidate owner
    * cannot be found at the specified path.
    */
    const Component* resolveVariableNameAndOwner(
        const ComponentPath& path, std::string& varName) const;

    /**
    * Based on a specified path, get the value of a discrete variable.
    *
    * The specified path consists of the name of the discrete variable
    * prepended by its absolute or relative path in the component hierarchy.
    *
    * If this component is the owner of the discrete variable, the specified
    * path should simply be the name of the discrete variable.
    *
    * @param state State from which to get the value.
    * @param path Path of the discrete variable in the component hierarchy.
    * @return Value of the discrete variable.
    * @throws ComponentHasNoSystem if this Component has not been added to a
    * System (i.e., if initSystem has not been called).
    * @throws EmptyComponentPath if the specified path is an empty string
    * (i.e., path == "").
    * @throws VariableOwnerNotFoundOnSpecifiedPath if the candidate owner
    * of the variable cannot be found at the specified path.
    * @throws VariableNotFound if the specified variable cannot be found in
    * the candidate owner.
    * @see Component::resolveVariableNameAndOwner()
    */
    double getDiscreteVariableValue(const SimTK::State& state,
        const std::string& path) const
    {
        double value{SimTK::NaN};
        value = getDiscreteVariableValue<double>(state, path);
        return value;
    }

    /**
    * Based on a specified path, get the value (type T) of a discrete variable.
    *
    * The specified path consists of the name of the discrete variable
    * prepended by its absolute or relative path in the component hierarchy.
    *
    * If this component is the owner of the discrete variable, the specified
    * path should simply be the name of the discrete variable.
    *
    * @param state State from which to get the value.
    * @param path Path of the discrete variable in the component hierarchy.
    * @return value Value of the discrete variable.
    * @throws ComponentHasNoSystem if this Component has not been added to a
    *         System (i.e., if initSystem has not been called).
    * @throws EmptyComponentPath if the specified path is an empty string
    * (i.e., path == "").
    * @throws VariableOwnerNotFoundOnSpecifiedPath if the candidate owner
    * of the variable cannot be found at the specified path.
    * @throws VariableNotFound if the specified variable cannot be found in
    * the candidate owner.
    * @see Component::resolveVariableNameAndOwner()
    */
    template<class T>
    T getDiscreteVariableValue(const SimTK::State& state,
        const std::string& path) const
    {
        T value = SimTK::Value<T>::downcast(
            getDiscreteVariableAbstractValue(state, path));
        return value;
    }

    /**
    * Based on a specified path, retrieve a read-only reference to the
    * abstract value of the discrete variable at a specified path. This method
    * provides a more general interface that is not limited to values of type
    * double.
    *
    * The specified path consists of the name of the discrete variable
    * prepended by its absolute or relative path in the component hierarchy.
    *
    * If this component is the owner of the discrete variable, the specified
    * path should simply be the name of the discrete variable.
    *
    * To obtain the type-specific value of a discrete variable from an
    * AbstractValue, perform a cast using the templated methods provided in
    * class SimTK::Value<T>. When the type is unknown, it can be queried using
    * the SimTK::Value<T>::isA() method. For example,
    *
    * ```
    *      const SimTK::AbstractValue& valAbstract =
    *          getDiscreteVariableAbstractValue(state, pathName);
    *
    *      if (SimTK::Value<double>::isA(valAbstract)) {
    *          const SimTK::Value<double>& valDbl =
    *              SimTK::Value<double>::downcast(valAbstract);
    *          double x = valDbl + 0.4;
    *
    *      } else if (SimTK::Value<Vec3>::isA(valAbstract)) {
    *          const SimTK::Value<Vec3>& valVec3 =
    *              SimTK::Value<Vec3>::downcast(valAbstract);
    *          Vec3 x = valVec3 + Vec3(0.4);
    *      }
    * ```
    * For convenience, a templated method that implements basic downcasting
    * internally is available. See getDiscreteVariableValue<T>().
    *
    * @param state State from which to get the value.
    * @param path Specified path of the variable in the component heirarchy.
    * @return Read-only reference to the discrete variable's AbstractValue.
    * @throws ComponentHasNoSystem if this Component has not been added to a
    *         System (i.e., if initSystem has not been called).
    * @throws EmptyComponentPath if the specified path is an empty string
    * (i.e., path == "").
    * @throws VariableOwnerNotFoundOnSpecifiedPath if the candidate owner
    * of the variable cannot be found at the specified path.
    * @throws VariableNotFound if the specified variable cannot be found in
    * the candidate owner.
    * @see Component::resolveVariableNameAndOwner
    */
    const SimTK::AbstractValue& getDiscreteVariableAbstractValue(
        const SimTK::State& state, const std::string& path) const;

    /**
    * Based on a specified path, set the value of a discrete variable.
    *
    * The specified path consists of the name of the discrete variable
    * prepended by its absolute or relative path in the component hierarchy.
    *
    * If this component is the owner of the discrete variable, the specified
    * path should simply be the name of the discrete variable.
    *
    * @param state State in which to set the discrete variable.
    * @param path Path of the discrete variable in the component hierarchy.
    * @param value Value to which to set the discrete variable.
    * @throws ComponentHasNoSystem if this Component has not been added to a
    *         System (i.e., if initSystem has not been called).
    * @throws EmptyComponentPath if the specified path is an empty string
    * (i.e., path == "").
    * @throws VariableOwnerNotFoundOnSpecifiedPath if the candidate owner
    * of the variable cannot be found at the specified path.
    * @throws VariableNotFound if the specified variable cannot be found in
    * the candidate owner.
    * @see Component::resolveVariableNameAndOwner()
    */
    void setDiscreteVariableValue(SimTK::State& state,
        const std::string& path, double value) const
    {
        setDiscreteVariableValue<double>(state, path, value);
    }

    /**
    * Based on a specified path, set the value (type T) of a discrete variable.
    *
    * The specified path consists of the name of the discrete variable
    * prepended by its absolute or relative path in the component hierarchy.
    *
    * If this component is the owner of the discrete variable, the specified
    * path should simply be the name of the discrete variable.
    *
    * @param state State for which to set the value.
    * @param path Path of the discrete variable in the component hierarchy.
    * The last name in the path should be the name of the discrete variable.
    * A path can consist of just the name of the discrete variable, in which
    * case the calling component is assumed to be the owner of the variable.
    * @param value Value to which to set the discrete variable.
    * @throws ComponentHasNoSystem if this Component has not been added to a
    * System (i.e., if initSystem has not been called).
    * @throws EmptyComponentPath if the specified path is an empty string
    * (i.e., path == "").
    * @throws VariableOwnerNotFoundOnSpecifiedPath if the candidate owner
    * of the variable cannot be found at the specified path.
    * @throws VariableNotFound if the specified variable cannot be found in
    * this Component.
    * @see Component::resolveVariableNameAndOwner()
    */
    template <class T>
    void setDiscreteVariableValue(SimTK::State& state, const std::string& path,
        const T& value) const
    {
        SimTK::Value<T>::downcast(
            updDiscreteVariableAbstractValue(state, path)) = value;
    }

    /**
    * Based on a specified path, retrieve a writable reference to the abstract
    * value of the discrete variable. This method provides a more general
    * interface that is not limited to values of type double.
    *
    * The specified path consists of the name of the discrete variable
    * prepended by its absolute or relative path in the component hierarchy.
    *
    * If this component is the owner of the discrete variable, the specified
    * path should simply be the name of the discrete variable.
    *
    * To obtain the type-specific value of a discrete variable, perform
    * a cast using the template methods provided in class SimTK::Value<T>.
    * When the type is unknown, it can be queried using the
    * SimTK::Value<T>::isA() method. For example,
    *
    * ```
    *      SimTK::AbstractValue& valAbstract =
    *          updDiscreteVariableAbstractValue(state, pathName);
    *
    *      if (SimTK::Value<double>::isA(valAbstract)) {
    *          SimTK::Value<double>& valDbl =
    *              SimTK::Value<double>::updDowncast(valAbstract);
    *          valDbl = 0.4;
    *
    *      } else if (SimTK::Value<Vec3>::isA(valAbstract)) {
    *          SimTK::Value<Vec3>& valVec3 =
    *              SimTK::Value<Vec3>::updDowncast(valAbstract);
    *          valVec3 = Vec3(0.4);
    *      }
    * ```
    * For convenience, a templated method that implements basic downcasting
    * internally is available. See setDiscreteVariableValue<T>().
    *
    * @param state State from which to get the value.
    * @param path Path of the discrete variable in the component hierarchy.
    * @return Writable reference to the discrete variable's AbstractValue.
    * @throws ComponentHasNoSystem if this Component has not been added to a
    *         System (i.e., if initSystem has not been called).
    * @throws EmptyComponentPath if the specified path is an empty string
    * (i.e., path == "").
    * @throws VariableOwnerNotFoundOnSpecifiedPath if the candidate owner
    * of the variable cannot be found at the specified path.
    * @throws VariableNotFound if the specified variable cannot be found in
    * this Component.
    */
    SimTK::AbstractValue& updDiscreteVariableAbstractValue(
        SimTK::State& state, const std::string& path) const;


    /**
     * A cache variable containing a value of type T.
     *
     * - A `CacheVariable` is a handle to the cache variable's value and
     *   validity
     *
     * - Derived classes may declare `CacheVariable` members (ideally,
     *   as `private` members), which grants them strongly-typed access
     *   to cache variable values
     *
     * - `CacheVariable` should be initialized with
     *   `Component::addCacheVariable`; usually, in the
     *   `Component::extendAddToSystem` method. A cache variable's value
     *   will not be valid until after `Component::realizeTopology` is
     *   called
     *
     * @tparam T
     *   Type of data held in the cache variable
     */
    template<class T>
    class CacheVariable {
    private:
        std::string name;

        // this is initialized in Component::getCacheVariableIndex. It enhances
        // performance by skipping using `name` to perform runtime map lookups.
        mutable SimTK::ResetOnCopy<SimTK::CacheEntryIndex> maybeUninitIndex{
            SimTK::InvalidIndex
        };

        friend class Component;

        explicit CacheVariable(std::string _name) : name{std::move(_name)} {
        }

    public:
        // A default constructor is required for user-facing impl. because
        // derived classes might not be able to fully initialize an instance
        // at construction time (e.g. via member initialization).
        //
        // Usage of a partially-initialized CacheVariable is invalid. The way
        // that this is dealt with is by checking for errors at runtime:
        //
        // - CacheVariable will default-initialize with a name of "", which is
        //   disallowed by `Component::addCacheVariable`. Therefore, if a
        //   CacheVariable instance has an empty name, it was not initialized.
        //
        // - The index is default-initialized, and copy constructed, with an
        //   invalid (erroring) index, so derived classes can't alias
        //   cache variables by copying this class around.
        CacheVariable() = default;
    };

protected:
    /**
     * Add a state cache entry belonging to this Component to hold
     * calculated values that must be automatically invalidated when certain
     * state values change. Cache entries contain values whose computations depend
     * on state variables and provide convenience and/or efficiency by holding on
     * to them in memory (cache) to avoid recomputation. Once the state changes,
     * the cache values automatically become invalid and has to be
     * recomputed based on the current state before it can be referenced again.
     * Any attempt to reference an invalid cache entry results in an exception
     * being thrown.
     *
     * Cache entry validity is managed by computation Stage, rather than by
     * dependence on individual state variables. Changing a variables whose
     * "invalidates" stage is the same or lower as the one specified as the
     * "depends on" stage here cause the cache entry to be invalidated. For
     * example, a body's momentum, which is dependent on position and velocity
     * states, should have Stage::Velocity as its \a dependsOnStage. Then if a
     * Velocity stage variable or lower (e.g. Position stage) changes, then the
     * cache is invalidated. But, if a Dynamics stage variable (or above) is
     * changed, the velocity remains valid so the cache entry does not have to be
     * recomputed.
     *
     * @param[in]      name
     *   The name you are assigning to this cache entry. Must be unique within
     *   this model component.
     *
     * @param[in]      variablePrototype
     *   An object defining the type of value, and a default value of that type,
     *   to be held in this cache entry. Can be a simple int or an elaborate
     *   class, as long as it has deep copy semantics.
     * @param[in]      dependsOnStage
     *   This is the highest computational stage on which this cache entry's
     *   value computation depends. State changes at this level or lower will
     *   invalidate the cache entry.
     */
    template <class T>
    CacheVariable<T> addCacheVariable(std::string name,
                                      T variablePrototype,
                                      SimTK::Stage dependsOnStage) const
    {
        if (name.empty()) {
            OPENSIM_THROW_FRMOBJ(Exception, "Cannot create a cache variable with an empty name");
        }

        // edge-case: there is already a cache variable with the same name allocated.
        //            This is disallowed--and probably a development error--because it
        //            might result in horrible edge cases such as two cachevars indirectly
        //            aliasing eachother at run-time.
        if (this->_namedCacheVariables.find(name) != this->_namedCacheVariables.end()) {
            std::stringstream msg;
            msg << "Cannot create a cache variable with the name '" << name << "' because another cache variable with that name already exists";
            OPENSIM_THROW_FRMOBJ(Exception, msg.str());
        }

        this->_namedCacheVariables.emplace(
                name,
                StoredCacheVariable{
                    new SimTK::Value<T>(std::move(variablePrototype)),
                    dependsOnStage
                });

        return CacheVariable<T>{std::move(name)};
    }

public:
    /**
     * Get the index of a Component's cache variable in the Subsystem for allocations.
     *
     * @tparam T
     *   Type of value held in the cache variable
     * @param cv
     *   A CacheVariable<T>, as allocated by Component::addCacheVariable
     * @return
     *   A valid SimTK::CacheEntryIndex, which callers can use with Simbody methods
     *   (e.g. markCacheValueRealized)
     */
    template<class T>
    SimTK::CacheEntryIndex getCacheVariableIndex(const CacheVariable<T>& cv) const {
        // cheap: index previously initialized, just return that
        if (cv.maybeUninitIndex.isValid()) {
            return cv.maybeUninitIndex;
        }

        // expensive: perform index lookup and initialize it

        if (cv.name.empty()) {
            OPENSIM_THROW_FRMOBJ(Exception, "Cannot get cache variable index: the cache variable has no name: has it been initialized with Component::addCacheVariable?");
        }

        // getCacheVariableIndex asserts whether the returned index is valid or not,
        // so this assignment will set the index to something valid, making subsequent
        // calls use the cheap path (above).
        cv.maybeUninitIndex = this->getCacheVariableIndex(cv.name);

        return cv.maybeUninitIndex;
    }

    /**
     * Get the index of a Component's cache variable in the Subsystem for allocations.
     *
     * @tparam T
     *   Type of value held in the cache variable
     * @param name
     *   Name of the cache variable, as provided to Component::addCacheVariable
     * @return
     *   A valid SimTK::CacheEntryIndex, which callers can use with simbody methods
     *   (e.g. markCacheValueRealized)
     */
    SimTK::CacheEntryIndex getCacheVariableIndex(const std::string& name) const;

private:
    template<class T, class K>
    const T& getCacheVariableValueGeneric(const SimTK::State& state, const K& key) const
    {
        const SimTK::DefaultSystemSubsystem& subsystem = this->getDefaultSubsystem();
        const SimTK::CacheEntryIndex idx = this->getCacheVariableIndex(key);
        const SimTK::AbstractValue& v = subsystem.getCacheEntry(state, idx);
        return SimTK::Value<T>::downcast(v).get();
    }

public:
    /**
     * Get the value of a cache variable allocated by this Component by name.
     *
     * @param state
     *     the State from which to get the value
     * @param name
     *     the name of the cache variable
     * @return T
     *     A const reference to the cache variable's value
     * @throws ComponentHasNoSystem
     *     if this Component has not been added to a System (i.e., if initSystem has not been called)
     */
    template<class T>
    const T& getCacheVariableValue(const SimTK::State& state, const std::string& name) const
    {
        return getCacheVariableValueGeneric<T>(state, name);
    }

    /**
     * Get the value of a cache variable allocated by this Component.
     *
     * @param state
     *     the State from which to get the value
     * @param cv
     *     the cache variable
     * @return T
     *     A const reference to the cache variable's value
     * @throws ComponentHasNoSystem
     *     if this Component has not been added to a System (i.e., if initSystem has not been called)
     */
    template<class T>
    const T& getCacheVariableValue(const SimTK::State& state, const CacheVariable<T>& cv) const
    {
        return getCacheVariableValueGeneric<T>(state, cv);
    }

private:
    template<typename T, typename K>
    void setCacheVariableValueGeneric(const SimTK::State& state, const K& key, T value) const
    {
        const SimTK::DefaultSystemSubsystem& subsystem = this->getDefaultSubsystem();
        const SimTK::CacheEntryIndex idx = this->getCacheVariableIndex(key);
        SimTK::AbstractValue& valWrapper = subsystem.updCacheEntry(state, idx);

        T& currentVal = SimTK::Value<T>::downcast(valWrapper).upd();
        currentVal = std::move(value);
        subsystem.markCacheValueRealized(state, idx);
    }

public:
    /**
     * Set the value of a cache variable, identified by `name`, to a new value
     * and mark the cache variable as valid.
     *
     * @param state
     *     the State in which to store the new value
     * @param k
     *     the name of the cache variable
     * @param value
     *     the new value for this cache variable
     * @return T
     *     A const reference to the cache variable's new value
     * @throws ComponentHasNoSystem
     *     if this Component has not been added to a System (i.e., if initSystem has not been called)
     */
    template<typename T>
    void setCacheVariableValue(const SimTK::State& state, const std::string& k, T value) const
    {
      setCacheVariableValueGeneric<T>(state, k, std::move(value));
    }

    /**
     * Set the value of a cache variable to a new value and mark the cache variable
     * as valid.
     *
     * @param state
     *     the State in which to store the new value
     * @param cv
     *     the cache variable to update
     * @param value
     *     the new value for the cache variable
     * @return T
     *     a const reference to the cache variable's new value
     * @throws ComponentHasNoSystem
     *     if this Component has not been added to a System (i.e., if initSystem has not been called)
     */
    template<typename T>
    void setCacheVariableValue(const SimTK::State& state, const CacheVariable<T>& cv, T value) const
    {
        setCacheVariableValueGeneric<T>(state, cv, std::move(value));
    }

private:
    template<typename T, typename K>
    T& updCacheVariableValueGeneric(const SimTK::State& state, const K& key) const {
        const SimTK::DefaultSystemSubsystem& subsystem = this->getDefaultSubsystem();
        const SimTK::CacheEntryIndex idx = this->getCacheVariableIndex(key);
        SimTK::AbstractValue& valWrapper = subsystem.updCacheEntry(state, idx);
        return SimTK::Value<T>::downcast(valWrapper).upd();
    }

public:
    /**
     * Returns a mutable reference to the value of a cache variable identified by `name`.
     *
     * Note: do not forget to mark the cache variable as valid after updating.
     *       Otherwise, it will force a re-computation of the value if the
     *       computation method is monitoring the validity of the cache value.
     *
     * @param state
     *     the State in which to set the value
     * @param name
     *     the name of the cache variable
     * @return value
     *     modifiable reference to the cache variable's value
     * @throws ComponentHasNoSystem
     *     if this Component has not been added to a System (i.e., if initSystem has not been called)
     */
    template<typename T>
    T& updCacheVariableValue(const SimTK::State& state, const std::string& name) const
    {
        return updCacheVariableValueGeneric<T>(state, name);
    }

    /**
     * Returns a mutable reference to the value of a cache variable.
     *
     * Note: do not forget to mark the cache variable as valid after updating.
     *       Otherwise, it will force a re-computation of the value if the
     *       computation method is monitoring the validity of the cache value.
     *
     * @param state
     *     the State in which to set the value
     * @param cv
     *     the cache variable
     * @return value
     *     modifiable reference to the cache variable's value
     * @throws ComponentHasNoSystem
     *     if this Component has not been added to the System (i.e., if initSystem has not been called)
     */
    template<typename T>
    T& updCacheVariableValue(const SimTK::State& state, const CacheVariable<T>& cv) const
    {
        return updCacheVariableValueGeneric<T>(state, cv);
    }

    /**
     * Returns true if the cache variable, identified by `name`, is valid.
     *
     * This method enables callers to monitor the validity of the cache variable,
     * which enables the caller to decide whether to update the cache variable's
     * value (or not). When computing an update is costly, use this method to check
     * whether computing the value is necessary.
     *
     * @param state
     *     the State in which the cache variable's value resides
     * @param name
     *     the name of the cache variable
     * @return bool
     *     whether the cache variable's value is valid or not
     * @throws ComponentHasNoSystem
     *     if this Component has not been added to a System (i.e., if initSystem has not been called)
     */
    bool isCacheVariableValid(const SimTK::State& state, const std::string& name) const;

    /**
     * Returns true if the cache variable is valid.
     *
     * This method enables callers to monitor the validity of the cache variable,
     * which enables the caller to decide whether to update the cache variable's
     * value (or not). When computing an update is costly, use this method to check
     * whether computing the value is necessary.
     *
     * @param state
     *     the State in which the cache variable's value resides
     * @param cv
     *     the cache variable
     * @return bool
     *     whether the cache variable's value is valid or not
     * @throws ComponentHasNoSystem
     *     if this Component has not been added to a System (i.e., if initSystem has not been called)
     */
    template<class T>
    bool isCacheVariableValid(const SimTK::State& state, const CacheVariable<T>& cv) const {
        const SimTK::DefaultSystemSubsystem& subsystem = this->getDefaultSubsystem();
        const SimTK::CacheEntryIndex idx = this->getCacheVariableIndex(cv);
        return subsystem.isCacheValueRealized(state, idx);
    }

    /**
     * Marks the value of a cache variable, identified by `name`, as valid.
     *
     * Upon marking a cache variable's value as valid, the cache variable will remain
     * valid until either:
     *
     * - the realization stage falls below the minimum realization stage set
     *   when the cache variable was initialized with `Component::addCacheVariable`
     *
     * - the cache variable is explicitly invalidated by calling
     *   `Component::markCacheVariableInvalid`
     *
     * This method causes `Component::isCacheVariableValid` to return true.
     * `Component::isCacheVariableValid` is commonly used by value-getting
     * methods to decide on whether to return the value as-is or recompute the
     * value. Therefore, if a cache variable is not marked as valid then the
     * cache variable's value may be recomputed more than necessary, which may
     * be costly.
     *
     * @param state
     *     the State in which the cache variable's value resides
     * @param name
     *     the name of the cache variable
     * @throws ComponentHasNoSystem
     *     if this Component has not been added to a System (i.e., if initSystem has not been called)
     */
    void markCacheVariableValid(const SimTK::State& state, const std::string& name) const;

    /**
     * Marks the value of a cache variable as valid.
     *
     * Upon marking a cache variable's value as valid, the cache variable will remain
     * valid until either:
     *
     * - the realization stage falls below the minimum realization stage set
     *   when the cache variable was initialized with `Component::addCacheVariable`
     *
     * - the cache variable is explicitly invalidated by calling
     *   `Component::markCacheVariableInvalid`
     *
     * This method causes `Component::isCacheVariableValid` to return true.
     * `Component::isCacheVariableValid` is commonly used by value-getting
     * methods to decide on whether to return the value as-is or recompute the
     * value. Therefore, if a cache variable is not marked as valid then the
     * cache variable's value may be recomputed more than necessary, which may
     * be costly.
     *
     * @param state
     *     the State in which the cache variable's value resides
     * @param cv
     *     the cache variable
     * @throws ComponentHasNoSystem
     *     if this Component has not been added to a System (i.e., if initSystem
     *     has not been called)
     */
    template<typename T>
    void markCacheVariableValid(const SimTK::State& state, const CacheVariable<T>& cv) const {
        const SimTK::DefaultSystemSubsystem& subsystem = this->getDefaultSubsystem();
        const SimTK::CacheEntryIndex idx = this->getCacheVariableIndex(cv);
        subsystem.markCacheValueRealized(state, idx);
    }

    /**
     * Marks the value of a cache variable, identified by `name`, as invalid.
     *
     * Upon marking a cache variable's value as invalid, it will remain invalid
     * until `Component::markCacheVariableValid` is called (or a method which
     * uses that, such as `Component::setCacheVariableValue`, is called).
     *
     * - Cache variables are automatically marked as invalid when the realization stage
     *   falls below the minimum realization stage set when the cache variable was
     *   initialized with `Component::addCacheVariable`.
     *
     * - Cache variables *may* be indirectly marked as invalid by other methods. For
     *   example, a component-added state variable may invalidate a cache variable at
     *   a lower stage. Concretely:
     *
     *   - A (hypothetical) component has a `length` state variable
     *   - There are cache variables that are computed from `length` (e.g.
     *   `strain`)
     *   - So changing the `length` may invalidate the `strain` indirectly
     *     (depending on how the state variable is handled)
     *
     * @param state
     *     the State in which the cache variable's value resides
     * @param name
     *     the name of the cache variable
     * @throws ComponentHasNoSystem
     *     if this Component has not been added to a System (i.e., if initSystem has not been called)
     */
    void markCacheVariableInvalid(const SimTK::State& state, const std::string& name) const;

    /**
     * Marks the value of a cache variable as invalid.
     *
     * Upon marking a cache variable's value as invalid, it will remain invalid
     * until `Component::markCacheVariableValid` is called (or a method which
     * uses that, such as `Component::setCacheVariableValue`, is called).
     *
     * - Cache variables are automatically marked as invalid when the realization stage
     *   falls below the minimum realization stage set when the cache variable was
     *   initialized with `Component::addCacheVariable`.
     *
     * - Cache variables *may* be indirectly marked as invalid by other methods. For
     *   example, a component-added state variable may invalidate a cache variable at
     *   a lower stage. Concretely:
     *
     *   - A (hypothetical) component has a `length` state variable
     *   - There are cache variables that are computed from `length` (e.g.
     *   `strain`)
     *   - So changing the `length` may invalidate the `strain` indirectly (depending on
     *     how the state variable is handled)
     *
     * @param state
     *     the State in which the cache variable's value resides
     * @param cv
     *     the cache variable
     * @throws ComponentHasNoSystem
     *     if this Component has not been added to a System (i.e., if initSystem has not been called)
     */
    template<class T>
    void markCacheVariableInvalid(const SimTK::State& state, const CacheVariable<T>& cv) const {
        const SimTK::DefaultSystemSubsystem& subsystem = this->getDefaultSubsystem();
        const SimTK::CacheEntryIndex idx = this->getCacheVariableIndex(cv);
        subsystem.markCacheValueNotRealized(state, idx);
    }

    // End of Model Component State Accessors.
    //@}

    /** @name Component State Trajectory Methods
    Methods that support comprehensive de/serialization of a time ordered
    ordered sequence (i.e., a "trajectory") of SimTK::State objects. Such
    trajectories are typically gathered during the course of a simulation.
    - The methods are comprehensive in that all categories of the variables
    held in the SimTK::State (ModelingOption%s, StateVariable%s, and
    DiscreteVariable%s) are supported.
    - Because the methods are templatized, they have the flexibility to
    handle variables of different types (e.g., bool, int, double, Vec3,
    Quaternion, etc.).
    - Because each method performs only a single string-based path lookup,
    they are reasonably efficient.
    - SimTK::Array_<T> is used to contain a SimTK::State trajectory, as
    opposed to a SimTK::Vector_<T>. This is because SimTK::State does not
    possess all of the computational characteristics required by
    SimTK::Vector_<T>. Note that SimTK::Array_<T> is essentially equivalent
    to std::vector<T> but with a number of advantages in terms of performance
    and binary compatibility.
    */
    //@{

    //=========================================================================
    /// GET TRAJECTORIES
    //=========================================================================

    /** From a trajectory of SimTK::State objects, get the corresponding
    trajectory of a specified state variable of type T.
    @param path Path of the specified variable in the component heirarchy.
    @param input State trajectory.
    @param output Trajectory of the specified variable. */
    template<class T>
    void getStateVariableTrajectory(const std::string& path,
        const SimTK::Array_<SimTK::State>& input,
        SimTK::Array_<T>& output) const
    {
        // Prepare the output Vector
        output.clear();
        int n = input.size();
        if(n<=0) return;
        output.reserve(n);

        // Find the state variable
        // This path traversal only needs to be done once.
        const StateVariable* var = traverseToStateVariable(path);

        // Loop over input and set the output
        for (int i = 0; i < n; ++i) {
            output.emplace_back(var->getValue(input[i]));
        }
    }

    /** From a trajectory of SimTK::State objects, get the trajectory of
    a specified discrete variable.

    The word "trajectory" connotes that the State and variable values are
    expected to be time-ordered, which will most commonly be the case.
    To be clear, however, this charicteristic is not essential here and is not
    checked during execution of this method.

    This method performs only a single string-based path lookup, so it is
    reasonably efficient.

    @param path Path of the specified variable in the component heirarchy.
    @param input Trajectory of SimTK::State objects.
    @param output Trajectory of the specified variable.
    @throws EmptyComponentPath if the specified path is an empty string
    (i.e., path == "").
    @throws VariableOwnerNotFoundOnSpecifiedPath if the candidate owner
    of the discrete variable cannot be found at the specified path.
    @throws VariableNotFound if the specified discrete variable cannot be found
    in the candidate owner. */
    template<class T>
    void getDiscreteVariableTrajectory(const std::string& path,
        const SimTK::Array_<SimTK::State>& input,
        SimTK::Array_<T>& output) const
    {
        // Prepare the output Vector
        output.clear();
        int n = input.size();
        if(n<=0) return;
        output.reserve(n);

        // Find the info struct for the discrete variable.
        // This path traversal only needs to be done once.
        std::string dvName{""};
        const Component* owner =
            resolveVariableNameAndOwner(path, dvName);
        std::map<std::string, DiscreteVariableInfo>::const_iterator it;
        it = owner->_namedDiscreteVariableInfo.find(dvName);

        // Not Found. Throw an exception.
        if(it == owner->_namedDiscreteVariableInfo.end()) {
            OPENSIM_THROW(VariableNotFound, getName(), dvName);
        }

        // Found. Loop over the input and set the output.
        const SimTK::SubsystemIndex ssIndex = it->second.ssIndex;
        const SimTK::DiscreteVariableIndex dvIndex = it->second.dvIndex;
        for (int i = 0; i < n; ++i) {
            SimTK::Value<T> vVal = SimTK::Value<T>::downcast(
                input[i].getDiscreteVariable(ssIndex, dvIndex));
            output.emplace_back(vVal.template getValue<T>());
        }
    }

    /** From a trajectory of SimTK::State objects, get the corresponding
    trajectory of a specified modeling option of type T.

    The word "trajectory" connotes that the State and option values are
    expected to be time-ordered, which will most commonly be the case.
    To be clear, however, this charicteristic is not essential here and is not
    checked during execution of this method.

    This method performs only a single string-based path lookup, so it is
    reasonably efficient.

    @param path Path of the specified variable in the component heirarchy.
    @param input States trajectory.
    @param output Corresponding trajectory of the specified option.
    @throws EmptyComponentPath if the specified path is an empty string
    (i.e., path == "").
    @throws VariableOwnerNotFoundOnSpecifiedPath if the candidate owner
    of the modeling option cannot be found at the specified path.
    @throws VariableNotFound if the specified modeling option cannot be found
    in the candidate owner.
    */
    template<class T>
    void getModelingOptionTrajectory(const std::string& path,
        const SimTK::Array_<SimTK::State>& input,
        SimTK::Array_<T>& output) const
    {
        // Prepare the output Vector
        output.clear();
        int n = input.size();
        if(n<=0) return;
        output.reserve(n);

        // Find the info struct for the modeling option.
        // This path traversal only needs to be done once.
        std::string moName{""};
        const Component* owner =
            resolveVariableNameAndOwner(path, moName);
        std::map<std::string, ModelingOptionInfo>::const_iterator it;
        it = owner->_namedModelingOptionInfo.find(moName);

        // Not Found. Throw an exception.
        if( it == owner->_namedModelingOptionInfo.end()) {
            OPENSIM_THROW(VariableNotFound, getName(), moName);
        }

        // Found. Loop over the input and set the output.
        // TODO: account for variable types other than int. This will
        // likely not be necessary, as modeling options are almost always
        // type bool or int and an int can be used to represent either.
        // Again, for an example, see how discrete variables do this.
        const SimTK::SubsystemIndex ssIndex = it->second.ssIndex;
        const SimTK::DiscreteVariableIndex moIndex = it->second.moIndex;
        for (int i = 0; i < n; ++i) {
            SimTK::Value<T> vVal = SimTK::Value<T>::downcast(
                input[i].getDiscreteVariable(ssIndex, moIndex));
            output.emplace_back(vVal.template getValue<T>());
        }
    }

    //=========================================================================
    /// SET TRAJECTORIES
    //=========================================================================

    /** From the trajectory of a specified state variable, set its
    corresponding values in a trajectory of SimTK::State objects.

    The word "trajectory" connotes that the State and variable values are
    expected to be time-ordered, which will most commonly be the case.
    To be clear, however, this charicteristic is not essential here and is not
    checked during execution of this method.

    This method performs only a single string-based path lookup, so it is
    reasonably efficient.

    @param path Path of the specified variable in the component heirarchy.
    @param input Trajectory of the specified variable.
    @param output Trajectory of SimTK::State objects with updated values for
    the specified variable. */
    template<class T>
    void setStateVariableTrajectory(const std::string& path,
        const SimTK::Array_<T>& input,
        SimTK::Array_<SimTK::State>& output) const
    {
        // Check that the input and output sizes are the same.
        // If not, throw an exception.
        int ni = input.size();
        int no = output.size();
        SimTK_ASSERT2_ALWAYS(no == ni,
            "Variable and State arrays are not the same size (%d != %d).",
            ni, no);

        // Find the state variable
        // This path traversal only needs to be done once.
        const StateVariable* var = traverseToStateVariable(path);

        // Loop over input values
        for (int i = 0; i < ni; ++i) {
            var->setValue(output[i], input[i]);
        }
    }

    /** From the trajectory of a specified discrete variable, set its
    corresponding values in a trajectory of SimTK::State objects.

    The word "trajectory" connotes that the State and variable values are
    expected to be time-ordered, which will most commonly be the case.
    To be clear, however, this charicteristic is not essential here and is not
    checked during execution of this method.

    This method performs only a single string-based path lookup, so it is
    reasonably efficient.

    @param path Path name of the specified variable in the component heirarchy.
    @param input Trajectory of the specified variable.
    @param output Trajectory of SimTK::State objects with updated values for
    the specified variable.
    @throws EmptyComponentPath if the specified path is an empty string
    (i.e., path == "").
    @throws VariableOwnerNotFoundOnSpecifiedPath if the candidate owner
    of the discrete variable cannot be found at the specified path.
    @throws VariableNotFound if the specified discrete variable cannot be found
    in the candidate owner. */
    template<class T>
    void setDiscreteVariableTrajectory(const std::string& path,
        const SimTK::Array_<T>& input,
        SimTK::Array_<SimTK::State>& output) const
    {
        // Check that the input and output sizes are the same.
        // If not, throw an exception.
        int ni = input.size();
        int no = output.size();
        SimTK_ASSERT2_ALWAYS(no == ni,
            "Variable and State arrays are not the same size (%d != %d).",
            ni, no);

        // Find the info struct for the discrete variable.
        // This path traversal only needs to be done once.
        std::string dvName{""};
        const Component* owner =
            resolveVariableNameAndOwner(path, dvName);
        std::map<std::string, DiscreteVariableInfo>::const_iterator it;
        it = owner->_namedDiscreteVariableInfo.find(dvName);

        // Not Found. Throw an exception.
        if( it == owner->_namedDiscreteVariableInfo.end()) {
            OPENSIM_THROW(VariableNotFound, getName(), dvName);
        }

        // Found. Loop over the input and set the output.
        const SimTK::SubsystemIndex ssIndex = it->second.ssIndex;
        const SimTK::DiscreteVariableIndex dvIndex = it->second.dvIndex;
        for (int i = 0; i < ni; ++i) {
            SimTK::Value<T>::downcast(
                output[i].updDiscreteVariable(ssIndex, dvIndex)).upd()
                = input[i];
        }
    }

    /** From the trajectory of a specified modeling option, set its
    corresponding values in a trajectory of SimTK::State objects.

    The word "trajectory" connotes that the State and option values are
    expected to be time-ordered, which will most commonly be the case.
    To be clear, however, this charicteristic is not essential here and is not
    checked during execution of this method.

    This method performs only a single string-based path lookup, so it is
    reasonably efficient.

    @param path Path of the specified variable in the component heirarchy.
    @param input Trajectory of the specified option.
    @param output Trajectory of SimTK::State objects with updated values for
    the specified modeling option.
    @throws EmptyComponentPath if the specified path is an empty string
    (i.e., path == "").
    @throws VariableOwnerNotFoundOnSpecifiedPath if the candidate owner
    of the modeling option cannot be found at the specified path.
    @throws VariableNotFound if the specified modeling option cannot be found
    in the candidate owner. */
    template<class T>
    void setModelingOptionTrajectory(const std::string& path,
        const SimTK::Array_<T>& input,
        SimTK::Array_<SimTK::State>& output) const
    {
        // Check that the input and output sizes are the same.
        // If not, throw an exception.
        int ni = input.size();
        int no = output.size();
        SimTK_ASSERT2_ALWAYS(no == ni,
            "Option and State arrays are not the same size (%d != %d).",
            ni, no);

        // Find the info struct for the modeling option.
        // This path traversal only needs to be done once.
        std::string moName{""};
        const Component* owner =
            resolveVariableNameAndOwner(path, moName);
        std::map<std::string, ModelingOptionInfo>::const_iterator it;
        it = owner->_namedModelingOptionInfo.find(moName);

        // Not Found. Throw an exception.
        if( it == owner->_namedModelingOptionInfo.end()) {
            OPENSIM_THROW(VariableNotFound, getName(), moName);
        }

        // Found. Loop over the input and set the output.
        const SimTK::SubsystemIndex ssIndex = it->second.ssIndex;
        const SimTK::DiscreteVariableIndex moIndex = it->second.moIndex;
        for (int i = 0; i < ni; ++i) {
            SimTK::Value<T>::downcast(
                output[i].updDiscreteVariable(ssIndex, moIndex)).upd()
                = input[i];
        }
    }

    // End of Component State Trajectory Methods.
    //@}


    /** @name Print information to the console */
    /// @{
    /** List all subcomponents by name and recurse into these components to
    list their subcomponents, and so on.                                      */
    void printSubcomponentInfo() const;

    /** List all the Sockets of this component and whether or not they are
    connected. Also list the connectee paths for sockets that are connected. */
    void printSocketInfo() const;

    /** List all the inputs of this component and whether or not they are
    connected. Also list the (desired) connectee paths for the inputs.       */
    void printInputInfo() const;

    template<typename C>
    void printSubcomponentInfo() const {

        ComponentList<const C> compList = getComponentList<C>();

        // Step through compList once to determine if there are any
        // subcomponents and to find the longest concrete class name.
        const std::string concreteClassName = this->getConcreteClassName();
        unsigned numSubcomponents = 0;
        size_t maxlen = concreteClassName.length();
        for (const C& thisComp : compList) {
            ++numSubcomponents;
            auto len = thisComp.getConcreteClassName().length();
            maxlen = std::max(maxlen, len);
        }

        if (numSubcomponents == 0) {
            log_cout("Component '{}' has no subcomponents.", getName());
            return;
        }
        maxlen += 6; //padding

        std::string className = SimTK::NiceTypeName<C>::namestr();
        // Remove "OpenSim::", etc. if it exists.
        const std::size_t colonPos = className.rfind(":");
        if (colonPos != std::string::npos)
            className = className.substr(colonPos+1,
                                         className.length()-colonPos);

        log_cout("Class name and absolute path name for descendants of '{}'"
                 "that are of type {}:\n", getName(), className);

        log_cout("{:>{}}  {}", concreteClassName, maxlen,
                getAbsolutePathString());

        // Step through compList again to print.
        for (const C& thisComp : compList) {
            const std::string thisClass = thisComp.getConcreteClassName();
            auto path = thisComp.getAbsolutePath();
            log_cout(fmt::format(
                    "{:>{}}  {}/{}",
                    fmt::format("[{}]", thisClass),
                    maxlen,
                    std::string((path.getNumPathLevels() - 1) * 4, ' '),
                    path.getComponentName()));
        }
        log_cout("");
    }

    /** Print outputs of this component and optionally, those of all
    subcomponents.                                                            */
    void printOutputInfo(const bool includeDescendants = true) const;
    /// @}

protected:
    class StateVariable;
    //template <class T> friend class ComponentSet;
    // Give the ComponentMeasure access to the realize() methods.
    template <class T> friend class ComponentMeasure;

#ifndef SWIG
    /// @class MemberSubcomponentIndex
    /// Unique integer type for local member subcomponent indexing
    SimTK_DEFINE_UNIQUE_INDEX_TYPE(MemberSubcomponentIndex);

    /** Construct a subcomponent as a data member of this Component. All Component
        interface calls are automatically invoked on its subcomponents. */
    template<class C=Component>
    MemberSubcomponentIndex constructSubcomponent(const std::string& name) {
        C* component = new C();
        component->setName(name);
        component->setOwner(*this);
        _memberSubcomponents.push_back(SimTK::ClonePtr<Component>(component));
        return MemberSubcomponentIndex(_memberSubcomponents.size()-1);
    }
    template<class C = Component>
    const C& getMemberSubcomponent(MemberSubcomponentIndex ix) const {
        const C* comp = dynamic_cast<const C*>(_memberSubcomponents[ix].get());
        if(comp)
            return *comp;

        throw Exception("Component::getMemberSubcomponent() - Incorrect type requested.");
    }
    template<class C = Component>
    C& updMemberSubcomponent(MemberSubcomponentIndex ix) {
        C* comp = dynamic_cast<C*>(_memberSubcomponents[ix].upd());
        if (comp)
            return *comp;

        throw Exception("Component::updMemberSubcomponent() - Incorrect type requested.");
    }
#endif //SWIG

    /**
    * Adopt a component as a subcomponent of this Component. Component
    * methods (e.g. addToSystem(), initStateFromProperties(), ...) are
    * automatically invoked on subcomponents when called on this Component.
    * Realization is also performed automatically on subcomponents. All
    * subcomponents are owned, therefore this Component also takes ownership.
    */
    void adoptSubcomponent(Component* subcomponent);

    /** Get the number of Subcomponents immediately owned by this Component */
    size_t getNumImmediateSubcomponents() const {
        return getNumMemberSubcomponents() + getNumPropertySubcomponents()
            + getNumAdoptedSubcomponents();
    }

    /** Get the number of Subcomponents that are data members of this Component */
    size_t getNumMemberSubcomponents() const;
    /** Get the number of Subcomponents that are properties of this Component */
    size_t getNumPropertySubcomponents() const;
    /** Get the number of Subcomponents adopted by this Component */
    size_t getNumAdoptedSubcomponents() const;

    /** Access this Component's immediate subcomponents (not those owned by
        subcomponents) */
    std::vector<SimTK::ReferencePtr<const Component>>
        getImmediateSubcomponents() const;

private:
    /**
     * Calls `callback` with a reference to each immediate subcomponent of this `Component`.
     *
     * @param callback A function that's called with a reference to each immediate subcomponent
     *                 of this `Component`.
     */
    void forEachImmediateSubcomponent(const std::function<void(const Component&)> callback) const;
    void forEachImmediateSubcomponent(const std::function<void(Component&)> callback);

    /**
     * Returns the first immediate subcomponent of this component with the given
     * name, or `nullptr` if no component could be found.
     *
     * @param name The name of the immediate subcomponent to search for.
     */
    const Component* findImmediateSubcomponentByName(const std::string& name) const;
protected:

    /** @name  Component Extension Interface
    The interface ensures that deserialization, resolution of inter-connections,
    and handling of dependencies are performed systematically and prior to
    system creation, followed by allocation of necessary System resources. These
    methods are virtual and may be implemented by subclasses of
    Components.

    @note Every implementation of virtual extend method xxx(args) must begin
    with the line "Super::extend<xxx>(args);" to ensure that the parent class
    is called before the child class method.

    The base class implementations ensures that the corresponding calls are made
    to any subcomponents which are owned by this Component. Ownership is
    established by the subcomponent being a data member (not serialized), a
    property (serialized), or created and adopted based on other settings
    or options that arise from the properties. For example, a Model (Component)
    may have to split a body and add a Weld constraint to handle a closed
    loop specified by Joints that are properties of the Model. The new Body and
    Weld (components) are created and adopted as part of connecting the model to
    form a valid multibody tree.

    So assuming that your concrete %Component and all intermediate classes from
    which it derives properly follow the requirement of calling the Super class
    method first, the order of operations enforced here for a call to a single
    method will be
      -# %Component base class computations
      -# calls to that same method for intermediate %Component-derived
         objects' computations, in order down from %Component, and
      -# call to that method for the bottom-level concrete class.
      -# finally calls to that same method for \e all subcomponents
    You should consider this ordering when designing a %Component.  **/

    ///@{

    /** Perform any secondary operations, e.g. to investigate the component or
    to insert it into a particular internal list (for grouping), after adding
    the subcomponent to this component. This is intended primarily for composites
    like Model to have more control over the handling of a component being added
    to it.

    If you override this method, be sure to invoke the base class method first,
    using code like this :
    @code
    void MyComponent::extendAddComponent(Component* subcomponent) {
        Super::extendAddComponent(); // invoke parent class method
        // ... your code goes here
        // ... initialize any internal data structures
    }
    @endcode   */
    virtual void extendAddComponent(Component* subcomponent) {};

    /** Perform any time-invariant calculations, data structure initializations,
    or other configuration based on the component's properties to form a
    functioning (but not yet connected) component. For example, each property
    should be checked to ensure that its value is within an acceptable range.
    When this method returns, the component will be marked as being up-to-date
    with its properties. Do not perform any configuration that depends on the
    SimTK::MultibodySystem; it is not available at this point.

    If you override this method, be sure to invoke the base class method first,
    using code like this:
        @code
        void MyComponent::extendFinalizeFromProperties() {
            Super::extendFinalizeFromProperties(); // invoke parent class method
            // ... your code goes here
            // ... catch invalid property values
            // ... initialize any internal data structures
        }
        @endcode   */
    virtual void extendFinalizeFromProperties() {};

    /** Perform any necessary initializations required to connect the component
    (and it subcomponents) to other components and mark the connection status.
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
    void MyComponent::extendFinalizeConnections(Component& root) {
        Super::extendFinalizeConnections(root); // invoke parent class method
        // ... your code goes here
    }
    @endcode   */
    virtual void extendFinalizeConnections(Component& root) {};

    /** Build the tree of Components from this component through its descendants.
    This method is invoked whenever a ComponentList<C> is requested. Note that
    all components must have been added to the model (or its subcomponents),
    otherwise it will not be included in the tree and will not be found for
    iteration or for connection. The implementation populates the _nextComponent
    ReferencePtr with a pointer to the next Component in tree pre-order traversal.

    @throws ComponentIsRootWithNoSubcomponents if the Component is the root and
            yet has no subcomponents.
    */
    void initComponentTreeTraversal(const Component &root) const;

    ///@cond
    /** Opportunity to remove connection-related information.
    If you override this method, be sure to invoke the base class method first,
        using code like this:
        @code
        void MyComponent::disconnect(Component& root) {
        // disconnect your subcomponents and your Super first
        Super::extendDisconnect();
            //your code to wipe out your connection-related information
    }
    @endcode  */
    //virtual void extendDisconnect() {};
    ///@endcond

    /** Add appropriate Simbody elements (if needed) to the System
    corresponding to this component and specify needed state resources.
    extendAddToSystem() is called when the Simbody System is being created to
    represent a completed system (model) for computation. That is, connect()
    will already have been invoked on all components before any addToSystem()
    call is made. Helper methods for adding modeling options, state variables
    and their derivatives, discrete variables, and cache entries are available
    and can be called within extendAddToSystem() only.

    Note that this method is const; you may not modify your model component
    or the containing model during this call. Any modifications you need should
    instead be performed in finalizeFromProperties() or at the latest connect(),
    which are non-const. The only exception is that you may need to record access
    information for resources you create in the \a system, such as an index number.
    For most Components, OpenSim base classes either provide convenience methods
    or handle indices automatically. Otherwise, you must declare indices as mutable
    data members so that you can set them here.

    If you override this method, be sure to invoke the base class method at the
    beginning, using code like this:
    @code
    void MyComponent::extendAddToSystem(SimTK::MultibodySystem& system) const {
        // Perform any additions to the system required by your Super
        Super::extendAddToSystem(system);
        // ... your code goes here
    }
    @endcode

    This method assumes that this Component's addToSystem will be invoked before
    its subcomponents. If you need your subcomponents to be added to the system,
    first (e.g. require of a Force to be anchored to a SimTK::MobilizedBody
    specified by subcomponents) then you must implement:
        extendAddToSystemAfterSubcomponents().
    It is possible to implement both method to add system elements before and then
    after your subcomponents have added themselves. Caution is required that
    Simbody elements are not added twice especially when order is unimportant.

    @param[in,out] system   The MultibodySystem being added to.

    @see addModelingOption(), addStateVariable(), addDiscreteVariables(),
         addCacheVariable() **/
    virtual void extendAddToSystem(SimTK::MultibodySystem& system) const {};

    /** Add appropriate Simbody elements (if needed) to the System after your
    component's subcomponents have had a chance to add themselves to the system.

    If you override this method, be sure to invoke the base class method at the
    beginning, using code like this:
    @code
    void MyComponent::
        extendAddToSystemAfterSubcomponents(SimTK::MultibodySystem& system) const {
        // Perform any additions to the system required by your Super
        Super::extendAddToSystemAfterSubcomponents(system);
        // ... your code goes here
    }
    @endcode

    @param[in,out] system   The MultibodySystem being added to.

    @see extendAddToSystem() **/
    virtual void
        extendAddToSystemAfterSubcomponents(SimTK::MultibodySystem& system)
                                                                      const {};

    /** Transfer property values or other state-independent initial values
    into this component's state variables in the passed-in \a state argument.
    This is called after a SimTK::System and State have been created for the
    Model (that is, after extendAddToSystem() has been called on all components).
    You should override this method if your component has properties
    (serializable values) that can affect initial values for your state
    variables. You can also perform any other state-independent calculations
    here that result in state initial conditions.

    If you override this method, be sure to invoke the base class method first,
    using code like this:
    @code
    void MyComponent::extendInitStateFromProperties(SimTK::State& state) const {
        Super::extendInitStateFromProperties(state); // invoke parent class method
        // ... your code goes here
    }
    @endcode

    @param      state
        The state that will receive the new initial conditions.

    @see extendSetPropertiesFromState() **/
    virtual void extendInitStateFromProperties(SimTK::State& state) const {};

    /** Update this component's property values to match the specified State,
    if the component has created any state variable that is intended to
    correspond to a property. Thus, state variable values can persist as part
    of the model component and be serialized as a property.

    If you override this method, be sure to invoke the base class method first,
    using code like this:
    @code
    void MyComponent::extendSetPropertiesFromState(const SimTK::State& state) {
        Super::extendSetPropertiesFromState(state); // invoke parent class method
        // ... your code goes here
    }
    @endcode

    @param      state
        The State from which values may be extracted to set persistent
        property values.

    @see extendInitStateFromProperties() **/
    virtual void extendSetPropertiesFromState(const SimTK::State& state) {};

    /** If a model component has allocated any continuous state variables
    using the addStateVariable() method, then %computeStateVariableDerivatives()
    must be implemented to provide time derivatives for those states.
    Override to set the derivatives of state variables added to the system
    by this component. (also see extendAddToSystem()). If the component adds states
    and computeStateVariableDerivatives is not implemented by the component,
    an exception is thrown when the system tries to evaluate its derivatives.

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
        setStateVariableDerivativeValue(state, "<state_variable_name>", deriv);
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
     * %Set the derivative of a state variable by name when computed inside of
     * this Component's computeStateVariableDerivatives() method.
     *
     * @param state  the State for which to set the value
     * @param name   the name of the state variable
     * @param deriv  the derivative value to set
     */
    void setStateVariableDerivativeValue(const SimTK::State& state,
                            const std::string& name, double deriv) const;


    // End of Component Extension Interface (protected virtuals).
    ///@}

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
    implementation, via a call like "Super::extendRealizePosition(state);". This
    will ensure that all necessary base class computations are performed, and
    that subcomponents are handled properly.

    @warning Currently the realize() methods here are invoked early in the
    sequence of realizations at a given stage, meaning that you will not be
    able to access other computations at that same stage.
    TODO: Should defer calls to these until at least kinematic realizations
    at the same stage have been performed.

    @see Simbody documentation for more information about realization.
    **/
    //@{
    /** Obtain state resources that are needed unconditionally, and perform
    computations that depend only on the system topology. **/
    virtual void extendRealizeTopology(SimTK::State& state) const;
    /** Obtain and name state resources (like state variables allocated by
    an underlying Simbody component) that may be needed, depending on modeling
    options. Also, perform any computations that depend only on topology and
    selected modeling options. **/
    virtual void extendRealizeModel(SimTK::State& state) const;
    /** Perform computations that depend only on instance variables, like
    lengths and masses. **/
    virtual void extendRealizeInstance(const SimTK::State& state) const;
    /** Perform computations that depend only on time and earlier stages. **/
    virtual void extendRealizeTime(const SimTK::State& state) const;
    /** Perform computations that depend only on position-level state
    variables and computations performed in earlier stages (including time). **/
    virtual void extendRealizePosition(const SimTK::State& state) const;
    /** Perform computations that depend only on velocity-level state
    variables and computations performed in earlier stages (including position,
    and time). **/
    virtual void extendRealizeVelocity(const SimTK::State& state) const;
    /** Perform computations (typically forces) that may depend on
    dynamics-stage state variables, and on computations performed in earlier
    stages (including velocity, position, and time), but not on other forces,
    accelerations, constraint multipliers, or reaction forces. **/
    virtual void extendRealizeDynamics(const SimTK::State& state) const;
    /** Perform computations that may depend on applied forces. **/
    virtual void extendRealizeAcceleration(const SimTK::State& state) const;
    /** Perform computations that may depend on anything but are only used
    for reporting and cannot affect subsequent simulation behavior. **/
    virtual void extendRealizeReport(const SimTK::State& state) const;
    //@} end of Component Advanced Interface


    /** @name     Component System Creation and Access Methods
     * These methods support implementing concrete Components. Add methods
     * can only be called inside of extendAddToSystem() and are useful for creating
     * the underlying SimTK::System level variables that are used for computing
     * values of interest.
     * @warning Accessors for System indices are intended for component internal use only.
     **/

    //@{

    /** Add a modeling option (integer flag stored in the State) for use by
    this Component. Each modeling option is identified by its own
    \a optionName, specified here. Modeling options enable the model
    component to be configured differently in order to represent different
    operating modes. For example, if two modes of operation are necessary (mode
    off and mode on) then specify optionName, "mode" with maxFlagValue = 1.
    Subsequent gets will return 0 or 1 and set will only accept 0 and 1 as
    acceptable values. Changing the value of a model option invalidates
    Stage::Instance and above in the State, meaning all calculations involving
    time, positions, velocity, and forces are invalidated.

    The `allocate` flag accommodates modeling options that are allocated
    external to OpenSim (e.g., in Simbody).
    - If `allocate == true`, the modeling option will be allocated normally
    in the base implementation of Component::extendRealizeTopology().
    - If `allocate == false`, the modeling option is assumed to have been
    allocated externally, in which case the derived Component (i.e., the
    wrapping Component) is responsible for initializing the subsystem index
    and modeling option index in its own overriding implementation of
    extendRealizeTopology() by calling initializeModelingOptionIndexes().

    @param moName Name of the modeling option.
    @param maxFlagValue The maximum allowed value of the modeling option.
    @param allocate A flag that, if true, specifies that the option should
    be allocated normally. If false, it specifies that normal allocation
    should be bypassed, in which case the derived Component is responsible for
    index initialization.
    @see Component::initializeModelingOptionIndexes().
    **/
    void addModelingOption(const std::string&  moName,
        int maxFlagValue, bool allocate = true) const;

    /**
    * Get the indexes for a Component's modeling option. This method is
    * intended for derived Components that may need direct access to its
    * underlying Subsystem.
    *
    * Two indexes are needed to properly access a modeling option in a
    * SimTK::State. The first identifies the SimTK::Subystem to which the
    * modeling option belongs; the second is the index into that subsystem
    * for the modeling option itself.
    *
    * @param[in] moName Name of the modeling option.
    * @param[out] ssIndex Reference that returns the index of the
    * SimTK::Subsystem to which the modeling options belongs.
    * @param[out] moIndex Reference that returns the index of the modeling
    * option within its SimTK::Subsytem.
    * @throws VariableNotFound if the specified modeling option is not found
    * in this Component.
    */
    void
    getModelingOptionIndexes(const std::string& moName,
        SimTK::SubsystemIndex& ssIndex,
        SimTK::DiscreteVariableIndex& moIndex) const;

    /**
    * Initialize the index and associated subsystem index of a Component's
    * modeling option.
    *
    * This method is intended to be used by a derived Component that possesses
    * a modeling option that was allocated external to OpenSim. Such a
    * situation can occur, for example, when a Simbody class is wrapped and
    * brought into OpenSim.
    *
    * To initialize the indexes of an externally allocated modeling option,
    * the derived Component should implement an overriding
    * `extendRealizeTopology()` and call this method from within that
    * overriding method.
    *
    * @param moName Name of the modeling option.
    * @param ssIndex Index of the SimTK::Subsystem to which the modeling
    * option belongs.
    * @param moIndex Index of the modeling option within its SimTK::Subystem.
    * @note A modeling option is a special kind of discrete variable.
    * As such, its index type is SimTK::DiscreteVariableIndex.
    * @throws VariableNotFound if the specified modeling option was not
    * found in this Component.
    */
    void initializeModelingOptionIndexes(const std::string& name,
        SimTK::SubsystemIndex ssIndex,
        const SimTK::DiscreteVariableIndex& moIndex) const;


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
    You may also want to create an Output for this state variable; see
    #OpenSim_DECLARE_OUTPUT_FOR_STATE_VARIABLE for more information. Reporters
    should use such an Output to get the StateVariable's value (instead of using
    getStateVariableValue()).

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
    this method.
    You may also want to create an Output for this state variable; see
    #OpenSim_DECLARE_OUTPUT_FOR_STATE_VARIABLE for more information. Reporters
    should use such an Output to get the StateVariable's value (instead of
    using getStateVariableValue()).
    */
    void addStateVariable(Component::StateVariable*  stateVariable) const;


    /** Add a system discrete variable that belongs to this Component.

    The `allocate` flag accommodates discrete variables that are allocated
    external to OpenSim (e.g., in Simbody).
    - If `allocate == true`, the discrete variable will be allocated normally
    in the base implementation of Component::extendRealizeTopology().
    - If `allocate == false`, the discrete variable is assumed to have been
    allocated externally, in which case the derived Component (i.e., the
    wrapping Component) is responsible for initializing the subsystem index
    and discrete variable index in its own overriding implementation of
    extendRealizeTopology() by calling initializeDiscreteVariableIndexes().

    @param dvName Name of the discrete variable.
    @param invalidatesStage The lowest SimTK realization stage that should be
    invalidated if the variables value is changed.
    @param allocate A flag that, if true, specifies that the variable should
    be allocated normally. If false, it specifies that normal allocation
    should be bypassed, in which case the derived Component is responsible for
    index initialization.
    @see Component::initializeDiscreteVariableIndexes().
    */
    void addDiscreteVariable(const std::string& dvName,
        SimTK::Stage invalidatesStage, bool allocate = true) const;

    /**
     * Get writable reference to the MultibodySystem that this component is
     * connected to.
     */
    SimTK::MultibodySystem& updSystem() const;

    /** Get the index of a Component's continuous state variable in the Subsystem for
        allocations. This method is intended for derived Components that may need direct
        access to its underlying Subsystem.*/
    int getStateIndex(const std::string& name) const;

   /**
     * Get the System Index of a state variable allocated by this Component.
     * Returns an InvalidIndex if no state variable with the name provided is
     * found.
     * @param stateVariableName   the name of the state variable
     */
    SimTK::SystemYIndex
        getStateVariableSystemIndex(const std::string& stateVariableName) const;


   /**
     * Get the indexes for a Component's discrete variable. This method is
     * intended for derived Components that may need direct access to its
     * underlying Subsystem.
     *
     * Two indexes are needed to properly access a discrete variable in a
     * SimTK::State. The first identifies the SimTK::Subystem to which the
     * discrete variable belongs; the second is the index into that subsystem
     * for the discrete variable itself.
     *
     * @param[in] dvName Name of the discrete variable.
     * @param[out] ssIndex Reference that returns the index of the
     * SimTK::Subsystem to which the discrete variable belongs.
     * @param[out] dvIndex Reference that returns the index of the discrete
     * variable within its SimTK::Subsytem.
     * @throws VariableNotFound if the specified discrete variable is not found
     * in this Component.
     */
    void
    getDiscreteVariableIndexes(const std::string& dvName,
        SimTK::SubsystemIndex& ssIndex,
        SimTK::DiscreteVariableIndex& dvIndex) const;

    /**
    * Initialize the index and associated subsystem index of a Component's
    * discrete variable.
    *
    * This method is intended to be used by a derived Component that possesses
    * a discrete variable that was allocated external to OpenSim. Such a
    * situation can occur, for example, when a Simbody class is wrapped and
    * brought into OpenSim.
    *
    * To initialize the indexes of an externally allocated discrete variable,
    * the derived Component should implement an overriding
    * `extendRealizeTopology()` and call this method from within that
    * overriding method.
    *
    * @param dvName Name of the discrete variable.
    * @param ssIndex Index of the SimTK::Subsystem to which the discrete
    * variable belongs.
    * @param dvIndex Index of the discrete variable within its SimTK::Subystem.
    * @throws VariableNotFound if the specified discrete variable was not
    * found in this Component.
    */
    void initializeDiscreteVariableIndexes(const std::string& dvName,
        SimTK::SubsystemIndex ssIndex,
        const SimTK::DiscreteVariableIndex& dvIndex) const;


    // End of System Creation and Access Methods.
    //@}

public:

    /** Find a Component to which this Component is an ancestor---in other
    words, a Component that is directly owned by this Component or is owned
    by one of its sub-components, sub-sub-components, etc. The Component can
    be found by type (by specifying a template argument) and either path or
    name.

    Here is an example of searching for a component of any type with the name
    'elbow_flexion':
    @code{.cpp}
    if (const Component* found =
            model.findComponent(ComponentPath("elbow_flexion"))) {
        std::cout << found.getName() << std::endl;
    }
    @endcode

    Here, we require that 'elbow_flexion' is of type Coordinate.
    @code{.cpp}
    if (const Coordinate* found =
            model.findComponent<Coordinate>(ComponentPath("elbow_flexion"))) {
        std::cout << "Coordinate " << found.getName() << std::endl;
    }
    @endcode

    The search can be sped up considerably if the path or even partial path
    name is known. For example, "forearm/elbow/elbow_flexion" will find
    the Coordinate component of the elbow joint that connects the forearm body
    in linear time (linear search for name at each component level). Whereas
    supplying "elbow_flexion" requires a tree search. Returns nullptr (None in
    Python, empty array in Matlab) if Component of that specified name cannot
    be found.

    NOTE: If the component name is ambiguous, an exception is thrown. To
    disambiguate, more information must be provided, such as the template
    argument to specify the type and/or a path rather than just the name. */
    template<class C = Component>
    const C* findComponent(const ComponentPath& pathToFind) const {
        const std::string name = pathToFind.toString();
        std::string msg = getConcreteClassName() + "'" + getName() +
                          "'::findComponent() ";
        if (name.empty()) {
            msg += "cannot find a nameless subcomponent.";
            throw Exception(msg);
        }

        ComponentPath thisAbsPath = getAbsolutePath();

        const C* found = NULL;
        if (thisAbsPath == pathToFind) {
            found = dynamic_cast<const C*>(this);
            if (found)
                return found;
        }

        std::vector<const C*> foundCs;

        std::string subname = pathToFind.getComponentName();
        std::string thisName = this->getName();
        if (thisName == subname) {
            found = dynamic_cast<const C*>(this);
            if (found)
                foundCs.push_back(found);
        }

        ComponentList<const C> compsList = this->template getComponentList<C>();

        for (const C& comp : compsList) {
            // if a child of this Component, one should not need
            // to specify this Component's absolute path name
            ComponentPath compAbsPath = comp.getAbsolutePath();
            ComponentPath thisAbsPathPlusSubname = getAbsolutePath();
            thisAbsPathPlusSubname.pushBack(subname);
            if (compAbsPath == thisAbsPathPlusSubname) {
                foundCs.push_back(&comp);
                break;
            }

            // otherwise, we just have a type and name match
            // which we may need to support for compatibility with older models
            // where only names were used (not path or type)
            // TODO replace with an exception -aseth
            std::string compName = comp.getName();
            if (compName == subname) {
                foundCs.push_back(&comp);
                // TODO Revisit why the exact match isn't found when
                // when what appears to be the complete path.
                log_debug("{} Found '{}' as a match for: Component '{}', "
                          "but it is not on the specified path.",
                        msg, compAbsPath.toString(),
                        comp.getConcreteClassName());
                //throw Exception(details, __FILE__, __LINE__);
            }
        }

        if (foundCs.size() == 1) {
            //unique type and name match!
            return foundCs[0];
        }

        // Only error cases remain
        // too many components of the right type with the same name
        if (foundCs.size() > 1) {
            msg += "Found multiple '" + name + "'s of type " +
                foundCs[0]->getConcreteClassName() + ".";
            throw Exception(msg, __FILE__, __LINE__);
        }

        // Not found
        return nullptr;
    }

    /** Same as findComponent(const ComponentPath&), but accepting a string (a
    path or just a name) as input. */
    template<class C = Component>
    const C* findComponent(const std::string& pathToFind) const {
        return findComponent<C>(ComponentPath(pathToFind));
    }

protected:

    const Component* traversePathToComponent(const ComponentPath&) const;

    template<class C>
    const C* traversePathToComponent(const ComponentPath& path) const
    {
        return dynamic_cast<const C*>(traversePathToComponent(path));
    }

public:
#ifndef SWIG // StateVariable is protected.
    /**
     * Get a StateVariable anywhere in the Component tree, given a
     * StateVariable path.
     *
     * The StateVariable doesn't need to be in a subcomponent of this
     * component; it could be located in a different branch of the Component
     * tree (in such a case, the specified path might begin with "../").
     *
     * This returns nullptr if a StateVariable does not exist at the specified
     * path or if the path is invalid.
     *
     * @throws ComponentHasNoSystem if this Component has not been added to a
     *         System (i.e., if initSystem has not been called)
     */
    const StateVariable* traverseToStateVariable(
            const std::string& pathName) const;

    /**
     * Get a StateVariable anywhere in the Component tree, given a
     * StateVariable path.
     *
     * The StateVariable doesn't need to be in a subcomponent of this
     * component; it could be located in a different branch of the Component
     * tree (in such a case, the specified path might begin with "../").
     *
     * This returns nullptr if a StateVariable does not exist at the specified
     * path or if the path is invalid.
     *
     * @throws ComponentHasNoSystem if this Component has not been added to a
     *         System (i.e., if initSystem has not been called)
     */
    const StateVariable* traverseToStateVariable(
            const ComponentPath& path) const;
#endif

    /// @name Access to the owning component (advanced).
    /// @{
    /** Access the owner of this Component.
     * An exception is thrown if the %Component has no owner; in this case, the
     * component is the root component, or is orphaned.
     * @see hasOwner() */
    const Component& getOwner() const;

    /** (For advanced users) Check if this %Component has an owner.
     * A component may not have an owner if it:
     * (1) is the root component, or
     * (2) has not been added to another component */
    bool hasOwner() const;

    /** Obtain the root %Component, which is this component if it is orphaned.
     */
    const Component& getRoot() const;

protected:
    /** %Set this %Component's reference to its owning %Component */
    void setOwner(const Component& owner);

    /// @}

    /** @name Internal methods for constructing Sockets, Outputs, Inputs
     * To declare Socket%s, Output%s, and Input%s for your component,
     * use the following macros within your class declaration (ideally at
     * the top near property declarations):
     *
     *  - #OpenSim_DECLARE_SOCKET
     *  - #OpenSim_DECLARE_LIST_SOCKET
     *  - #OpenSim_DECLARE_OUTPUT
     *  - #OpenSim_DECLARE_LIST_OUTPUT
     *  - #OpenSim_DECLARE_OUTPUT_FOR_STATE_VARIABLE
     *  - #OpenSim_DECLARE_INPUT
     *  - #OpenSim_DECLARE_LIST_INPUT
     *
     * The methods below are used in those macros, and you should not use these
     * methods yourself.
     */
    /// @{
    /**
    * Construct a specialized Socket for this Component's dependence on
    * another Component. It serves as a placeholder for the Component and its
    * type and enables the Component to automatically traverse its dependencies
    * and provide a meaningful message if the provided Component is
    * incompatible or non-existent. This function also creates a Property in
    * this component to store the connectee path for this socket; the
    * propertyComment argument is the comment to use for that Property. */
    template <typename T>
    PropertyIndex constructSocket(const std::string& name, bool isList,
                                  const std::string& propertyComment) {
        OPENSIM_THROW_IF(_socketsTable.count(name), Exception,
            getConcreteClassName() + " already has a socket named '"
            + name + "'.");

        PropertyIndex propIndex;
        // This property is accessed / edited by the Socket class. It is
        // not easily accessible to users.
        // TODO does putting the addProperty here break the ability to
        // create a custom-copy-ctor version of all of this?
        // TODO property type should be ComponentPath or something like that.
        if (isList) {
            propIndex = this->template addListProperty<std::string>(
                    "socket_" + name, propertyComment,
                    0, std::numeric_limits<int>::max());
        } else {
            propIndex = this->template addProperty<std::string>(
                    "socket_" + name , propertyComment, "");
        }
        // We must create the Property first: the Socket needs the property's
        // index in order to access the property later on.
        _socketsTable[name].reset(
            new Socket<T>(name, propIndex, SimTK::Stage::Topology, *this));
        return propIndex;
    }

#ifndef SWIG // SWIG can't parse the const at the end of the second argument.
    /** Construct an output for a member function of the same component.
        The following must be true about componentMemberFunction, the function
        that returns the output:

           -# It is a member function of \a this component.
           -# The member function is const.
           -# It takes only one input, which is `const SimTK::State&`
           -# The function returns the computed quantity *by value* (e.g.,
              `double computeQuantity(const SimTK::State&) const`).

        You must also provide the stage on which the output depends.

        You can ask outputs for their value only after you call
        `finalizeFromProperties()`.

       @see constructOutputForStateVariable()
     */
    template <typename T, typename CompType = Component>
    bool constructOutput(const std::string& name,
            T (CompType::*const memFunc)(const SimTK::State&) const,
            const SimTK::Stage& dependsOn = SimTK::Stage::Acceleration) {
        // The `const` in `CompType::*const componentMemberFunction` means this
        // function can't assign componentMemberFunction to some other function
        // pointer. This is unlikely, since that function would have to match
        // the same template parameters (T and CompType).
        static_assert(std::is_base_of<Component, CompType>::value,
            "Template parameter 'CompType' must be derived from Component.");

        // This lambda takes a pointer to a component, downcasts it to the
        // appropriate derived type, then calls the member function of the
        // derived type. Thank you, klshrinidhi!
        // TODO right now, the assignment to result within the lambda is
        // making a copy! We can fix this using a reference pointer.
        auto outputFunc = [memFunc] (const Component* comp,
                const SimTK::State& s, const std::string&, T& result) -> void {
            result = std::mem_fn(memFunc)(dynamic_cast<const CompType*>(comp), s);
        };
        return constructOutput<T>(name, outputFunc, dependsOn);
    }
    /** This variant handles component member functions that return the
     * output value by const reference (const T&).
     * @warning ONLY use this with member functions that fetch quantities that
     * are stored within the passed-in SimTK::State. The function cannot return
     * local variables. */
    template <typename T, typename CompType = Component>
    bool constructOutput(const std::string& name,
            const T& (CompType::*const memFunc)(const SimTK::State&) const,
            const SimTK::Stage& dependsOn = SimTK::Stage::Acceleration) {
        static_assert(std::is_base_of<Component, CompType>::value,
            "Template parameter 'CompType' must be derived from Component.");

        // This lambda takes a pointer to a component, downcasts it to the
        // appropriate derived type, then calls the member function of the
        // derived type. Thank you, klshrinidhi!
        auto outputFunc = [memFunc] (const Component* comp,
                const SimTK::State& s, const std::string&, T& result) -> void {
            result = std::mem_fn(memFunc)(dynamic_cast<const CompType*>(comp), s);
        };
        return constructOutput<T>(name, outputFunc, dependsOn);
    }
    /** Construct an output that can have multiple channels. You add Channels
    to this Output in extendFinalizeFromProperties() using
    AbstractOutput::addChannel(). The member function
    you provide must take the name of the channel whose value is requested. */
    template <typename T, typename CompType>
    bool constructListOutput(const std::string& name,
             T (CompType::*const memFunc)(const SimTK::State&,
                                          const std::string& channel) const,
            const SimTK::Stage& dependsOn = SimTK::Stage::Acceleration) {
        // The `const` in `CompType::*const componentMemberFunction` means this
        // function can't assign componentMemberFunction to some other function
        // pointer. This is unlikely, since that function would have to match
        // the same template parameters (T and CompType).
        static_assert(std::is_base_of<Component, CompType>::value,
            "Template parameter 'CompType' must be derived from Component.");

        // This lambda takes a pointer to a component, downcasts it to the
        // appropriate derived type, then calls the member function of the
        // derived type. Thank you, klshrinidhi!
        auto outputFunc = [memFunc] (const Component* comp,
                const SimTK::State& s, const std::string& channel, T& result) -> void {
            result = std::mem_fn(memFunc)(
                    dynamic_cast<const CompType*>(comp), s, channel);
        };
        return constructOutput<T>(name, outputFunc, dependsOn, true);
    }
#endif

    /** Construct an Output for a StateVariable. While this method is a
     * convenient way to construct an Output for a StateVariable, it is
     * inefficient because it uses a string lookup. To create a more efficient
     * Output, create a member variable that returns the state variable
     * directly; see the implementations of Coordinate::getValue() or
     * Muscle::getActivation() for examples.
     *
     * @param name Name of the output, which must be the same as the name of
     * the corresponding state variable. */
    bool constructOutputForStateVariable(const std::string& name);

    /** Construct an Input (socket) for this Component's dependence on an
     * Output signal.  It is a placeholder for the Output and its type and
     * enables the Component to automatically traverse its dependencies and
     * provide a meaningful message if the provided Output is incompatible or
     * non-existent. This also specifies at what stage the output must be valid
     * for the component to consume it as an input.  If the Output's
     * dependsOnStage is above the Input's requiredAtStage, an Exception is
     * thrown because the output cannot satisfy the Input's requirement.
     * This function also creates a Property in this component to store the
     * connectee paths for this input; the
     * propertyComment argument is the comment to use for that Property. */
    template <typename T>
    PropertyIndex constructInput(const std::string& name, bool isList,
            const std::string& propertyComment,
            const SimTK::Stage& requiredAtStage = SimTK::Stage::Instance) {

        OPENSIM_THROW_IF(_inputsTable.count(name), Exception,
            getConcreteClassName() + " already has an input named '"
            + name + "'.");

        PropertyIndex propIndex;
        // This property is accessed / edited by the AbstractSocket class.
        // It is not easily accessible to users.
        // TODO property type should be OutputPath or ChannelPath.
        if (isList) {
            propIndex = this->template addListProperty<std::string>(
                    "input_" + name, propertyComment,
                    0, std::numeric_limits<int>::max());
        } else {
            propIndex = this->template addProperty<std::string>(
                    "input_" + name, propertyComment, "");
        }
        // We must create the Property first: the Input needs the property's
        // index in order to access the property later on.
        _inputsTable[name].reset(
                new Input<T>(name, propIndex, requiredAtStage, *this));
        return propIndex;
    }
    /// @}

    /// For internal use. Update absolute connectee paths in all sockets and
    /// inputs in the subcomponent by prepending the absolute path of the
    /// subcomponent. To be used when adding subcomponent to another component.
    static void prependComponentPathToConnecteePath(Component& subcomponent);

private:
    //Mark components that are properties of this Component as subcomponents of
    //this Component. This happens automatically upon construction of the
    //component. If a Component property is added programmatically, then one must
    //also mark it by calling markAsPropertySubcomponent() with that component.
    void markPropertiesAsSubcomponents();

    // Internal use: mark as a subcomponent, a component that is owned by this
    // Component by virtue of being one of its properties.
    void markAsPropertySubcomponent(const Component* subcomponent);

    /// Invoke finalizeFromProperties() on the (sub)components of this Component.
    void componentsFinalizeFromProperties() const;

    /// Invoke connect() on the (sub)components of this Component.
    void componentsFinalizeConnections(Component& root);

    /// Base Component must create underlying resources in computational System.
    void baseAddToSystem(SimTK::MultibodySystem& system) const;

    /// Invoke addToSystem() on the (sub)components of this Component.
    void componentsAddToSystem(SimTK::MultibodySystem& system) const;

    /// Invoke initStateFromProperties() on (sub)components of this Component
    void componentsInitStateFromProperties(SimTK::State& state) const;

    /// Invoke setPropertiesFromState() on (sub)components of this Component
    void componentsSetPropertiesFromState(const SimTK::State& state);

    /** Used internally to construct outputs. Creating the functions for
     outputs is potentially error-prone if the function binds to (or
     captures) a pointer to a class. When the component is copied, the
     pointer bound to the function is also copied and points to the original
     object. This is unlikely to be the intended behavior. For this reason,
     this variant of constructOutput should be used with care.
    */
    template <typename T>
    bool constructOutput(const std::string& name,
            const std::function<void (const Component*,
                                      const SimTK::State&,
                                      const std::string& channel, T&)> outputFunction,
            const SimTK::Stage& dependsOn = SimTK::Stage::Acceleration,
            bool isList = false) {

        OPENSIM_THROW_IF(_outputsTable.count(name), Exception,
            getConcreteClassName() + " already has an output named '"
            + name + "'.");

        _outputsTable[name].reset(
                new Output<T>(name, outputFunction, dependsOn, isList));
        return true;
    }

    // Get the number of continuous states that the Component added to the
    // underlying computational system. It includes the number of built-in
    // states exposed by this component. It represents the number of state
    // variables managed by this Component.
    int getNumStateVariablesAddedByComponent() const
    {   return (int)_namedStateVariableInfo.size(); }
    Array<std::string> getStateVariableNamesAddedByComponent() const;

    // Get the number of variables states that the Component added to the
    // underlying computational system. The number of built-in discrete
    // states exposed by this component is included. It represents the number
    // of discrete variables managed by this Component. This may be useful
    // when de/serializing discrete variables.
    int getNumDiscreteVariablesAddedByComponent() const {
        return (int)_namedDiscreteVariableInfo.size();
    }
    Array<std::string> getDiscreteVariableNamesAddedByComponent() const;

    // Get the number of modeling options that the Component added to the
    // underlying computational system. The number of built-in modeling
    // options exposed by this component is included. It represents the number
    // of modeling options managed by this Component. This may be useful
    // when de/serializing modeling options.
    int getNumModelingOptionsAddedByComponent() const {
        return (int)_namedModelingOptionInfo.size();
    }
    Array<std::string> getModelingOptionNamesAddedByComponent() const;

    const SimTK::DefaultSystemSubsystem& getDefaultSubsystem() const
        {   return getSystem().getDefaultSubsystem(); }
    SimTK::DefaultSystemSubsystem& updDefaultSubsystem() const
        {   return updSystem().updDefaultSubsystem(); }

    // Clear all modeling options, continuous and discrete state variables,
    // and cache variable allocated by this Component
    void clearStateAllocations();

    // Reset by clearing underlying system indices.
    void reset();

    void warnBeforePrint() const override;

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
        StateVariable() : name(""), owner(nullptr),
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
        // return the index in the global list of continuous state variables, Y
        const SimTK::SystemYIndex& getSystemYIndex() const { return sysYIndex; }

        bool isHidden() const { return hidden; }
        void hide()  { hidden = true; }
        void show()  { hidden = false; }

        void setVarIndex(int index) { varIndex = index; }
        void setSubsystemIndex(const SimTK::SubsystemIndex& sbsysix) {
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
        // Once allocated a state in the system will have a global index
        // and that can be stored here as well
        SimTK::SystemYIndex sysYIndex;

        // flag indicating if state variable is hidden to the outside world
        bool hidden;
    };

    /// Helper method to enable Component makers to specify the order of their
    /// subcomponents to be added to the System during addToSystem(). It is
    /// highly unlikely that you will need to reorder the subcomponents of your
    /// custom component. This ability is primarily intended for Model (and
    /// other top-level) components that have the responsibility of creating a
    /// valid SimTK::MultibodySystem. MultibodySystem (Simbody) elements such as
    /// MobilizedBodies must be added sequentially to form a Multibody tree.
    /// SimTK::Constraints and SimTK::Forces must be applied to MobilizedBodies
    /// that are already present in the MultibodySystem. The Model component
    /// handles this order for you and should handle user-defined Components
    /// without any issues. You should rarely need to use this method yourself.
    /// If needed, use this method in extendFinalizeConnections() of your
    /// Component (or within your extendConnectToModel() for ModelComponents) to
    /// set the order of your subcomponents. For example, Model orders
    /// subcomponents according to the Multibody tree and adds bodies and joints
    /// in order starting from Ground and growing outward. If the subcomponent
    /// already appears in the ordered list setting it later in the list has no
    /// effect. The list remains unique. NOTE: If you do need to set the order
    /// of your subcomponents, you must do so for all your immediate
    /// subcomponents, otherwise those components not in the ordered list will
    /// not be added to the System.
    void setNextSubcomponentInSystem(const Component& sub) const;

    /// resetSubcomponentOrder clears this Component's list of ordered
    /// subcomponents (but otherwise leaves subcomponents untouched). You can
    /// form the ordered list using setNextSubcomponentInSystem() above.
    void resetSubcomponentOrder() {
        _orderedSubcomponents.clear();
    }

    /// Handle a change in XML syntax for Sockets.
    void updateFromXMLNode(SimTK::Xml::Element& node, int versionNumber)
            override;

private:

    // Reference to the owning Component of this Component. It is not the
    // previous in the tree, but is the Component one level up that owns this
    // one.
    SimTK::ReferencePtr<const Component> _owner;

    // Reference pointer to the successor of the current Component in Pre-order traversal
    mutable SimTK::ReferencePtr<const Component> _nextComponent;

    // Reference pointer to the system that this component belongs to.
    SimTK::ReferencePtr<SimTK::MultibodySystem> _system;

    // propertiesTable maintained by Object

    // Table of Component's structural Sockets indexed by name.
    std::map<std::string, SimTK::ClonePtr<AbstractSocket>> _socketsTable;

    // Table of Component's Inputs indexed by name.
    std::map<std::string, SimTK::ClonePtr<AbstractInput>> _inputsTable;

    // Table of Component's Outputs indexed by name.
    std::map<std::string, SimTK::ClonePtr<AbstractOutput> > _outputsTable;

    // Underlying SimTK custom measure ComponentMeasure, which implements
    // the realizations in the subsystem by calling private concrete methods on
    // the Component. Every model component has one of these, allocated
    // in its extendAddToSystem() method, and placed in the System's default
    // subsystem.
    SimTK::ResetOnCopy<SimTK::MeasureIndex> _simTKcomponentIndex;

    // list of subcomponents that are contained in this Component's properties
    SimTK::ResetOnCopy<SimTK::Array_<SimTK::ReferencePtr<Component>>>
        _propertySubcomponents;
    // Keep fixed list of data member Components upon construction
    SimTK::Array_<SimTK::ClonePtr<Component> > _memberSubcomponents;
    // Hold onto adopted components
    SimTK::Array_<SimTK::ClonePtr<Component> > _adoptedSubcomponents;

    // A flat list of subcomponents (immediate and otherwise) under this
    // Component. This list must be populated prior to addToSystem(), and is
    // used strictly to specify the order in which addToSystem() is invoked
    // on its subcomponents. The order is necessary for the construction
    // of Simbody elements (e.g. MobilizedBodies, Constraints, Forces,
    // Measures, ...) which cannot be added to the SimTK::MultibodySystem in
    // arbitrary order. In the case of MobilizedBodies, for example, the parent
    // MobilizedBody must be part of the system before the child can be added.
    // OpenSim::Model performs the mapping from User specifications to the
    // system order required by the SimTK::MultibodySystem.
    // If the Component does not reset the list, it is by default the ownership
    // tree order of its subcomponents.
    mutable std::vector<SimTK::ReferencePtr<const Component> > _orderedSubcomponents;

    // Structure to hold modeling option information. Modeling options are
    // integers 0..maxOptionValue. At run time we keep them in a Simbody
    // discrete state variable that invalidates Model stage if changed.
    struct ModelingOptionInfo {
        ModelingOptionInfo() : maxOptionValue(-1), allocate(true) {}
        explicit ModelingOptionInfo(int maxOptVal, bool allocate = true)
        :   maxOptionValue(maxOptVal), allocate(allocate) {}

        // Maximum allowed value of this modeling option
        int                             maxOptionValue;

        // Index of the SimTK::Subsystem to which this modeling option belongs
        SimTK::SubsystemIndex           ssIndex;

        // Index of this modeling option within its SimTK::Subsystem
        SimTK::DiscreteVariableIndex    moIndex;

        // Allocate Flag
        // If true, the modeling option will be allocated normally in the
        // base implementation of Componente::extendRealizeTopology().
        // If false, the modeling option is assumed to have been allocated
        // external to OpenSim (e.g., in Simbody), in which case the derived
        // Component (i.e., the wrapping Component) is responsible for
        // initializing ssIndex and moIndex in its own overriding
        // implementation of extendRealizeTopology() by calling
        // Component::initializeModelingOptionIndexes().
        bool                            allocate{true};
    };

    // Class for handling state variable added (allocated) by this Component
    class AddedStateVariable : public StateVariable {
        public:
        // Constructors
        AddedStateVariable() : StateVariable(),
            invalidatesStage(SimTK::Stage::Empty)  {}

        /** Convenience constructor for defining a Component added state variable */
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

    // Structure to hold essential info for discrete variables.
    struct DiscreteVariableInfo {
        DiscreteVariableInfo() {}
        explicit DiscreteVariableInfo(SimTK::Stage invalidates,
            bool allocate = true) :
            invalidatesStage(invalidates), allocate(allocate) {}

        // Realization stage at which the SimTK::State is invalidated when a
        // change is made to the value of this discrete variable
        SimTK::Stage                    invalidatesStage;

        // Index of the SimTK::Subsystem to which this discrete variable
        // belongs
        SimTK::SubsystemIndex           ssIndex;

        // Index of this discrete variable within its SimTK::Subsystem
        SimTK::DiscreteVariableIndex    dvIndex;

        // Allocate Flag
        // If true, the discrete variable will be allocated normally in the
        // base implementation of Component::extendRealizeTopology().
        // If false, the discrete variable is assumed to have been allocated
        // external to OpenSim (e.g., in Simbody), in which case the derived
        // Component (i.e., the wrapping Component) is responsible for
        // initializing ssIndex and dvIndex in its own overriding
        // implementation of extendRealizeTopology() by calling
        // Component::initializeDiscreteVariableIndexes().
        bool                            allocate{true};
    };

    /**
    * A cache variable, as stored internally by Component.
    */
    struct StoredCacheVariable {
        SimTK::ClonePtr<SimTK::AbstractValue> value;
        SimTK::Stage dependsOnStage;

        // initialized by Component::extendRealizeTopology
        SimTK::ResetOnCopy<SimTK::CacheEntryIndex> maybeUninitIndex{
            SimTK::InvalidIndex
        };

        StoredCacheVariable(SimTK::AbstractValue* _value,
                            SimTK::Stage _dependsOnStage) :
            value{_value},
            dependsOnStage{_dependsOnStage} {
       }

       SimTK::CacheEntryIndex index() const {
           if (this->maybeUninitIndex.isValid()) {
               return this->maybeUninitIndex;
           } else {
               OPENSIM_THROW(Exception, "StoredCacheVariable::get: failed because this->index == SimTK::InvalidIndex: this can happen if Component::extendRealizeTopology has not been called");
           }
       }
    };

    // Map names of modeling options for the Component to their underlying
    // SimTK indices.
    // These are mutable here so they can ONLY be modified in extendAddToSystem().
    // This is not an API bug. The purpose of these maps is to automate the
    // bookkeeping of component variables (state variables and cache entries) with
    // their index in the computational system. The earliest time we have a valid
    // index is when we ask the system to allocate the resources and that only
    // happens in extendAddToSystem. Furthermore, extendAddToSystem may not
    // alter the Component in any way that would affect its behavior- that is
    // why it is const!
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
    mutable SimTK::ResetOnCopy<std::unordered_map<std::string, StoredCacheVariable>> _namedCacheVariables;

    // Check that the list of _allStateVariables is valid
    bool isAllStatesVariablesListValid() const;

    // Array of all state variables for fast access during simulation
    mutable SimTK::Array_<SimTK::ReferencePtr<const StateVariable> >
                                                            _allStateVariables;
    // A handle the System associated with the above state variables
    mutable SimTK::ReferencePtr<const SimTK::System> _statesAssociatedSystem;

//==============================================================================
};  // END of class Component
//==============================================================================
//==============================================================================

// Implement methods for ComponentListIterator
/// ComponentListIterator<T> pre-increment operator, advances the iterator to
/// the next valid entry.
template <typename T>
ComponentListIterator<T>& ComponentListIterator<T>::operator++() {
    if (_node==nullptr)
        return *this;
    // If _node has children then successor is first child
    // move _node to point to it
    if (_node->_memberSubcomponents.size() > 0) {
        _node = _node->_memberSubcomponents[0].get();
    }
    else if (_node->_propertySubcomponents.size() > 0) {
        _node = _node->_propertySubcomponents[0].get();
    }
    else if (_node->_adoptedSubcomponents.size() > 0) {
        _node = _node->_adoptedSubcomponents[0].get();
    }
    // If processing a subtree under _root we stop when our successor is the same
    // as the successor of _root as this indicates we're leaving the _root's subtree.
    else if (_node->_nextComponent.get() == _root->_nextComponent.get())
        _node = nullptr;
    else // move on to the next component we computed earlier for the full tree
        _node = _node->_nextComponent.get();
    advanceToNextValidComponent(); // make sure we have a _node of type T after advancing
    return *this;
}

/// Internal method to advance iterator to next valid component.
template <typename T>
void ComponentListIterator<T>::advanceToNextValidComponent() {
    // Advance _node to next valid (of type T) if needed
    // Similar logic to operator++ but applies _filter->isMatch()
    while (_node != nullptr && (dynamic_cast<const T*>(_node) == nullptr ||
                                !(!_filter || _filter->isMatch(*_node)) ||
                                (_node == _root))) {

        if (_node->_memberSubcomponents.size() > 0) {
            _node = _node->_memberSubcomponents[0].get();
        }
        else if (_node->_propertySubcomponents.size() > 0) {
            _node = _node->_propertySubcomponents[0].get();
        }
        else if (_node->_adoptedSubcomponents.size() > 0) {
            _node = _node->_adoptedSubcomponents[0].get();
        }
        else {
            if (_node->_nextComponent.get() == _root->_nextComponent.get()){ // end of subtree under _root
                _node = nullptr;
                continue;
            }
            _node = _node->_nextComponent.get();
        }
    }
    return;
}


class ConnecteeNotSpecified : public Exception {
public:
    ConnecteeNotSpecified(const std::string& file,
                          size_t line,
                          const std::string& func,
                          const AbstractSocket& socket,
                          const Component& owner) :
    Exception(file, line, func) {
        std::string msg = "Connectee for Socket '" + socket.getName() +
                "' of type " + socket.getConnecteeTypeName() + " in " +
                owner.getConcreteClassName() + " at " +
                owner.getAbsolutePathString() + " is unspecified. "
                "If this model was built programmatically, perhaps "
                "finalizeConnections() was not called before "
                "printing.";
        addMessage(msg);
    }
};

template<class C>
const C& Socket<C>::getConnectee(int index) const {
    if (index < 0) {
        if (!isListSocket()) { index = 0; }
        else {
            std::stringstream msg;
            msg << "Socket<T>::getConnectee(): an index must be "
                << "provided for a socket that takes a list "
                << "of values.";
            OPENSIM_THROW(Exception, msg.str());
        }
    }
    return getConnecteeInternal(index);
}

template<class C>
const C& Socket<C>::getConnecteeInternal(int index) const {
    if (0 <= index && index < static_cast<int>(_connectees.size()) && _connectees[index]) {
        // fast case: use the cached connectee pointer
        //
        // it's usually cached via a direct call to `connect`, or through
        // a call to `finalizeConnection`
        return *_connectees[index];
    }

    // else: slow case: use the connectee path property to perform the lookup
    //
    // this is slower, but runtime-validated. It's necessary when (e.g.) a
    // component wants to use sockets during an `extendFinalizeConnections`
    // override (i.e. midway through recursively caching `_connectees`)
    return getOwner().template getComponent<C>(getConnecteePath(index));
}

template<class C>
void Socket<C>::findAndConnect(const ComponentPath& connectee) {
    const auto* comp =
            getOwner().getRoot().template findComponent<C>(connectee);
    OPENSIM_THROW_IF(!comp, ComponentNotFound, connectee.toString(),
                     getConnecteeTypeName(),
                     getOwner().getAbsolutePathString());
    connect(*comp);
}

template<class C>
void Socket<C>::finalizeConnection(const Component& root) {

    // If the reference to the connectee is set, use that. Otherwise, use the
    // connectee path property.
    if (isConnected()) {
        clearConnecteePath();
        for (auto& connectee : _connectees) {
            const auto& comp = *connectee;
            const auto& rootOfConnectee = comp.getRoot();
            const auto& myRoot = getOwner().getRoot();
            if (&myRoot != &rootOfConnectee) {
                std::stringstream msg;
                msg << "Socket<" << getConnecteeTypeName() << "> '" << getName()
                    << "' in " << getOwner().getConcreteClassName() << " at "
                    << getOwner().getAbsolutePathString()
                    << " cannot connect to " << comp.getConcreteClassName()
                    << " at " << comp.getAbsolutePathString()
                    << ": components do not have the same root component. "
                    << "Did you intend to add '" << rootOfConnectee.getName()
                    << "' to '" << myRoot.getName() << "'?";
                OPENSIM_THROW(Exception, msg.str());
            }

            ComponentPath connecteePath = connectee->getRelativePath(getOwner());
            // If the relative path starts with ".." then use an absolute path
            // instead.
            if (connecteePath.getNumPathLevels() > 1 &&
                    connecteePath.getSubcomponentNameAtLevel(0) == "..")
                connecteePath = connectee->getAbsolutePath();

            // Assign the connectee path to this Socket.
            assignConnecteePath(connecteePath.toString());
        }
    } else {
        for (int i = 0; i < static_cast<int>(getNumConnectees()); ++i) {
            const auto connecteePath = getConnecteePath(i);
            OPENSIM_THROW_IF(connecteePath.empty(), ConnecteeNotSpecified,
                             *this, getOwner());
            ComponentPath path(connecteePath);
            const C* comp = nullptr;
            if (path.isAbsolute()) {
                comp = &root.template getComponent<C>(path);
            } else {
                comp = &getOwner().template getComponent<C>(path);
            }
            connectInternal(*comp);
        }
    }
}

template<class T>
void Input<T>::connect(const AbstractOutput& output,
                       const std::string& alias) {
    const auto* outT = dynamic_cast<const Output<T>*>(&output);
    if (!outT) {
        std::stringstream msg;
        msg << "Type mismatch between Input and Output: Input '" << getName()
            << "' of type " << getConnecteeTypeName()
            << " cannot connect to Output '" << output.getPathName()
            << "' of type " << output.getTypeName() << ".";
        OPENSIM_THROW(Exception, msg.str());
    }

    if (!isListSocket() && outT->getChannels().size() > 1) {
        OPENSIM_THROW(Exception,
            "Non-list input '" + getName() +
            "' cannot connect to output '" + output.getPathName() +
            " with more than 1 channel");
    }

    // For a non-list socket, there will only be one channel.
    for (const auto& chan : outT->getChannels()) {
        registerChannel(chan.second, alias);
    }
}

template<class T>
void Input<T>::connect(const AbstractChannel& channel,
                       const std::string& alias) {
    registerChannel(channel, alias);
}

template<class T>
void Input<T>::finalizeConnection(const Component& root) {

    _connectees.clear();
    _aliases.clear();
    if (!_registeredChannels.empty()) {
        clearConnecteePath();
        OPENSIM_THROW_IF(!isListSocket() && getChannels().size() > 1,
                         Exception,
                         "Cannot connect single-value input to multiple channels.");
        for (const auto& reg : _registeredChannels) {
            const Output<T>& output = std::get<0>(reg).getRef();
            std::string channelName = std::get<1>(reg);
            const AbstractChannel& channel = output.getChannel(channelName);
            const std::string& alias = std::get<2>(reg);
            connectInternal(channel, std::get<2>(reg));
        }

        int i = -1;
        for (const auto& chan : getChannels()) {

            const auto& rootOfConnectee =
                    chan->getOutput().getOwner().getRoot();
            const auto& myRoot = getOwner().getRoot();
            OPENSIM_THROW_IF(&myRoot != &rootOfConnectee, Exception,
                "Input<" + getConnecteeTypeName() + "> '" + getName() +
                "' in " + getOwner().getConcreteClassName() + " at " +
                getOwner().getAbsolutePathString() + " cannot connect to " +
                "Channel " + chan->getPathName() + ": components do not have "
                "the same root component. Did you intend to add '" +
                rootOfConnectee.getName() + "' to '" + myRoot.getName() + "'?");

            ++i;
            // Update the connectee path as
            // <OwnerPath>/<Output><:Channel><(annotation)>
            const auto& outputOwner = chan->getOutput().getOwner();
            ComponentPath path = outputOwner.getRelativePath(getOwner());
            // If the relative path starts with ".." then use an absolute path
            // instead.
            if (path.getNumPathLevels() > 1 &&
                    path.getSubcomponentNameAtLevel(0) == "..")
                path = outputOwner.getAbsolutePath();

            auto pathStr = composeConnecteePath(path.toString(),
                                                chan->getOutput().getName(),
                                                chan->getOutput().isListOutput()
                                                ?
                                                chan->getChannelName() :
                                                "",
                                                _aliases[i]);

            // Assign the connectee path to this Input.
            assignConnecteePath(pathStr);
        }
    } else {
        // If the connectee path is empty, then we have nothing to connect to.
        if (isConnecteePathEmpty()) return;
        std::string compPathStr, outputName, channelName, alias;
        for (int i = 0; i < static_cast<int>(getNumConnectees()); ++i) {
            parseConnecteePath(getConnecteePath(i),
                               compPathStr, outputName, channelName, alias);
            ComponentPath compPath(compPathStr);
            const AbstractOutput* output = nullptr;

            if (compPath.isAbsolute()) { //absolute path string
                if (compPathStr.empty()) {
                    output = &root.getOutput(outputName);
                } else {
                    output = &root.getComponent(compPathStr).getOutput(
                            outputName);
                }
            } else { // relative path string
                const Component* comp = nullptr;
                if (compPathStr.empty()) {
                    comp = &getOwner();
                } else {
                    comp = &getOwner().getComponent(compPathStr);
                }
                // comp should never be null at this point.
                OPENSIM_THROW_IF(!comp, Exception, "Internal error: "
                                 "could not find component '" +
                                 compPathStr + ".");
                output = &comp->getOutput(outputName);
            }
            const auto& channel = output->getChannel(channelName);
            connectInternal(channel, alias);
        }
    }
}


} // end of namespace OpenSim

#endif // OPENSIM_COMPONENT_H_

