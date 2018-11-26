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
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include "OpenSim/Common/ComponentSocket.h"
#include "OpenSim/Common/ComponentOutput.h"
#include "OpenSim/Common/Array.h"
#include "ComponentList.h"
#include "ComponentPath.h"
#include <functional>

#include "simbody/internal/MultibodySystem.h"

namespace OpenSim {

class Model;
class ModelDisplayHints;

//==============================================================================
/// Component Exceptions
//==============================================================================
class ComponentHasNoName : public Exception {
public:
    ComponentHasNoName(const std::string& file,
        size_t line,
        const std::string& func,
        const std::string& componentConcreteClassName) :
        Exception(file, line, func) {
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
        const std::string& func,
        const std::string& thisName,
        const std::string& invalidChars,
        const std::string& componentConcreteClassName) :
        Exception(file, line, func) {
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
        const std::string& func,
        const std::string& toFindName,
        const std::string& toFindClassName,
        const std::string& thisName) :
        ComponentNotFound(file, line, func) {
        std::string msg = "Component '" + thisName;
        msg += "' could not find '" + toFindName;
        msg += "' of type " + toFindClassName + ". ";
        msg += "Make sure a component exists at this path and that it is of ";
        msg += "the correct type.";
        addMessage(msg);
    }
};

class ComponentIsAnOrphan : public Exception {
public:
    ComponentIsAnOrphan(const std::string& file,
        size_t line,
        const std::string& func,
        const std::string& thisName,
        const std::string& componentConcreteClassName) :
        Exception(file, line, func) {
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
        const std::string& func,
        const std::string& thisName,
        const std::string& duplicateName) :
        Exception(file, line, func) {
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
        const std::string& func,
        const std::string& thisName,
        const std::string& componentConcreteClassName) :
        Exception(file, line, func) {
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
                                        const std::string& func,
                                        const std::string& compName,
                                        const std::string& thisName) :
        Exception(file, line, func) {
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
                         const std::string& func,
                         const Object& obj) :
        Exception(file, line, func, obj) {
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
                   const std::string& func,
                   const Object& obj,
                   const std::string& socketName) :
        Exception(file, line, func, obj) {
        std::string msg = "no Socket '" + socketName;
        msg += "' found for this Component.";
        addMessage(msg);
    }
};

class InputNotFound : public Exception {
public:
    InputNotFound(const std::string& file,
                  size_t line,
                  const std::string& func,
                  const Object& obj,
                  const std::string& inputName) :
        Exception(file, line, func, obj) {
        std::string msg = "no Input '" + inputName;
        msg += "' found for this Component.";
        addMessage(msg);
    }
};

class OutputNotFound : public Exception {
public:
    OutputNotFound(const std::string& file,
                   size_t line,
                   const std::string& func,
                   const Object& obj,
                   const std::string& outputName) :
        Exception(file, line, func, obj) {
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
 * ### System and State
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
 * ### Discrete variables
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
 * ### Cache variables
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
 * ### Stages
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
 * ### The interface of this class
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
 * ### Subcomponents
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
    const SimTK::MultibodySystem& getSystem() const;

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
     * Get the names of "continuous" state variables maintained by the Component
     * and its subcomponents.
     * @throws ComponentHasNoSystem if this Component has not been added to a
     *         System (i.e., if initSystem has not been called)
     */
    Array<std::string> getStateVariableNames() const;


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
    std::vector<std::string> getSocketNames() {
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
        OPENSIM_THROW_IF_FRMOBJ(!socket.isConnected(), Exception,
                "Socket '" + name + "' not connected.");
        return socket.getConnectee();
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
        OPENSIM_THROW_IF_FRMOBJ(!socket.isConnected(), Exception,
                "Socket '" + name + "' not connected.");
        return socket.getConnecteeAsObject();
    }

    /** Get an AbstractSocket for the given socket name. This
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
            return it->second.getRef();
        }

        OPENSIM_THROW_FRMOBJ(SocketNotFound, name);
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
        auto it = _outputsTable.find(name);
        
        if (it != _outputsTable.end()) {
            return it->second.getRef();
        }

        OPENSIM_THROW_FRMOBJ(OutputNotFound, name);
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
     * %Set the value of a ModelingOption flag for this Component.
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
     * Get the value of a discrete variable allocated by this Component by name.
     *
     * @param state   the State from which to get the value
     * @param name    the name of the state variable
     * @return value  the discrete variable value
     * @throws ComponentHasNoSystem if this Component has not been added to a
     *         System (i.e., if initSystem has not been called)
     */
    double getDiscreteVariableValue(const SimTK::State& state,
                                    const std::string& name) const;

    /**
     * %Set the value of a discrete variable allocated by this Component by name.
     *
     * @param state  the State for which to set the value
     * @param name   the name of the discrete variable
     * @param value  the value to set
     * @throws ComponentHasNoSystem if this Component has not been added to a
     *         System (i.e., if initSystem has not been called)
     */
    void setDiscreteVariableValue(SimTK::State& state, const std::string& name,
                                  double value) const;

    /**
     * Get the value of a cache variable allocated by this Component by name.
     *
     * @param state  the State from which to get the value
     * @param name   the name of the cache variable
     * @return T     const reference to the cache variable's value
     * @throws ComponentHasNoSystem if this Component has not been added to a
     *         System (i.e., if initSystem has not been called)
     */
    template<typename T> const T& 
    getCacheVariableValue(const SimTK::State& state, const std::string& name) const
    {
        // Must have already called initSystem.
        OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

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
     * Obtain a writable cache variable value allocated by this Component by
     * name. Do not forget to mark the cache value as valid after updating,
     * otherwise it will force a re-evaluation if the evaluation method is
     * monitoring the validity of the cache value.
     *
     * @param state  the State for which to set the value
     * @param name   the name of the state variable
     * @return value modifiable reference to the cache variable's value
     * @throws ComponentHasNoSystem if this Component has not been added to a
     *         System (i.e., if initSystem has not been called)
     */
    template<typename T> T& 
    updCacheVariableValue(const SimTK::State& state, const std::string& name) const
    {
        // Must have already called initSystem.
        OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

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
     * After updating a cache variable value allocated by this Component, you
     * can mark its value as valid, which will not change until the realization
     * stage falls below the minimum set at the time the cache variable was
     * created. If not marked as valid, the evaluation method monitoring this
     * flag will force a re-evaluation rather that just reading the value from
     * the cache.
     *
     * @param state  the State containing the cache variable
     * @param name   the name of the cache variable
     * @throws ComponentHasNoSystem if this Component has not been added to a
     *         System (i.e., if initSystem has not been called)
     */
    void markCacheVariableValid(const SimTK::State& state, const std::string& name) const
    {
        // Must have already called initSystem.
        OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

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
     * Mark a cache variable value allocated by this Component as invalid. When
     * the system realization drops to below the lowest valid stage, cache
     * variables are automatically marked as invalid. There are instances when
     * component-added state variables require invalidating a cache at a lower
     * stage. For example, a component may have a "length" state variable which
     * should invalidate calculations involving it and other positions when the
     * state variable is set. Changing the component state variable
     * automatically invalidates Dynamics and higher realizations, but to force
     * realizations at Position and Velocity requires setting the lowest valid
     * stage to Position and marking the cache variable as invalid whenever the
     * "length" state variable value is set/changed.
     *
     * @param state  the State containing the cache variable
     * @param name   the name of the cache variable
     * @throws ComponentHasNoSystem if this Component has not been added to a
     *         System (i.e., if initSystem has not been called)
     */
    void markCacheVariableInvalid(const SimTK::State& state, 
                                  const std::string& name) const
    {
        // Must have already called initSystem.
        OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

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
     * Enables the user to monitor the validity of the cache variable value
     * using the returned flag. For components performing a costly evaluation,
     * use this method to force a re-evaluation of a cache variable value only
     * when necessary (i.e., returns false).
     *
     * @param state  the State in which the cache value resides
     * @param name   the name of the cache variable
     * @return bool  whether the cache variable value is valid or not
     * @throws ComponentHasNoSystem if this Component has not been added to a
     *         System (i.e., if initSystem has not been called)
     */
    bool isCacheVariableValid(const SimTK::State& state, const std::string& name) const
    {
        // Must have already called initSystem.
        OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

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
     * %Set cache variable value allocated by this Component by name. All cache
     * entries are lazily evaluated (on a need basis) so a set also marks the
     * cache as valid.
     *
     * @param state  the State in which to store the new value
     * @param name   the name of the cache variable
     * @param value  the new value for this cache variable
     * @throws ComponentHasNoSystem if this Component has not been added to a
     *         System (i.e., if initSystem has not been called)
     */
    template<typename T> void 
    setCacheVariableValue(const SimTK::State& state, const std::string& name, 
                     const T& value) const
    {
        // Must have already called initSystem.
        OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

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
            std::cout << "Component '" << getName()
                      << "' has no subcomponents." << std::endl;
            return;
        }
        maxlen += 4; //padding

        std::string className = SimTK::NiceTypeName<C>::namestr();
        // Remove "OpenSim::", etc. if it exists.
        const std::size_t colonPos = className.rfind(":");
        if (colonPos != std::string::npos)
            className = className.substr(colonPos+1,
                                         className.length()-colonPos);

        std::cout << "Class name and absolute path name for descendants of '"
                  << getName() << "' that are of type " << className << ":\n"
                  << std::endl;

        std::cout << std::string(maxlen-concreteClassName.length(), ' ')
                  << "[" << concreteClassName << "]"
                  << "  " << getAbsolutePathString() << std::endl;

        // Step through compList again to print.
        for (const C& thisComp : compList) {
            const std::string thisClass = thisComp.getConcreteClassName();
            std::cout << std::string(maxlen-thisClass.length(), ' ') << "["
                      << thisClass << "]  ";
            auto path = thisComp.getAbsolutePath();
            std::cout << std::string((path.getNumPathLevels() - 1) * 4, ' ')
                      << "/" << path.getComponentName() << std::endl;
        }
        std::cout << std::endl;
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
    @bug Should defer calls to these until at least kinematic realizations
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
    example, a body's momentum, which is dependent on position and velocity 
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
     * Get writable reference to the MultibodySystem that this component is
     * connected to.
     */
    SimTK::MultibodySystem& updSystem() const;

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

   /**
     * Get the index of a Component's discrete variable in the Subsystem for
     * allocations. This method is intended for derived Components that may need
     * direct access to its underlying Subsystem.
     */
    const SimTK::DiscreteVariableIndex 
    getDiscreteVariableIndex(const std::string& name) const;

    /** Get the index of a Component's cache variable in the Subsystem for allocations.
        This method is intended for derived Components that may need direct access
        to its underlying Subsystem.*/
    const SimTK::CacheEntryIndex 
    getCacheVariableIndex(const std::string& name) const;

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
            if ( (found = dynamic_cast<const C*>(this)) )
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
                if (comp.getDebugLevel() > 0) {
                    std::string details = msg + " Found '" + compAbsPath.toString() +
                        "' as a match for:\n Component '" + name + "' of type " + 
                        comp.getConcreteClassName() + ", but it "
                        "is not on specified path.\n";
                    //throw Exception(details, __FILE__, __LINE__);
                    std::cout << details << std::endl;
                }
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

    template<class C>
    const C* traversePathToComponent(ComponentPath path) const
    {
        // Get rid of all the ".."'s that are not at the front of the path.
        path.trimDotAndDotDotElements();

        // Move up either to the root component or just enough to resolve all
        // the ".."'s.
        size_t iPathEltStart = 0u;
        const Component* current = this;
        if (path.isAbsolute()) {
            current = &current->getRoot();
        } else {
            while (iPathEltStart < path.getNumPathLevels() &&
                    path.getSubcomponentNameAtLevel(iPathEltStart) == "..") {
                // The path sends us up farther than the root.
                if (!current->hasOwner()) return nullptr;
                current = &current->getOwner();
                ++iPathEltStart;
            }
        }
        
        using RefComp = SimTK::ReferencePtr<const Component>;

        // Skip over the root component name.
        for (size_t i = iPathEltStart; i < path.getNumPathLevels(); ++i) {
            // At this depth in the tree, is there a component whose name
            // matches the corresponding path element?
            const auto& currentPathElement =
                path.getSubcomponentNameAtLevel(i);
            const auto& currentSubs = current->getImmediateSubcomponents();
            const auto it = std::find_if(currentSubs.begin(), currentSubs.end(),
                    [currentPathElement](const RefComp& sub)
                    { return sub->getName() == currentPathElement; });
            if (it != currentSubs.end())
                current = it->get();
            else
                return nullptr;
        }
        if (const C* comp = dynamic_cast<const C*>(current))
            return comp;
        return nullptr;
    }

public:
#ifndef SWIG // StateVariable is protected.
    /**
     * Get a StateVariable anywhere in the Component tree, given a
     * StateVariable path. The StateVariable doesn't need to be in a
     * subcomponent of this compoonent; it could be located in a different
     * branch of the Component tree (in such a case, the specified path might
     * begin with "../").
     * This returns nullptr if a StateVariable does not exist at the specified
     * path or if the path is invalid.
     * @throws ComponentHasNoSystem if this Component has not been added to a
     *         System (i.e., if initSystem has not been called)
     */
    const StateVariable* traverseToStateVariable(
            const std::string& pathName) const;
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
    PropertyIndex constructSocket(const std::string& name,
                                     const std::string& propertyComment) {
        OPENSIM_THROW_IF(_socketsTable.count(name), Exception,
            getConcreteClassName() + " already has a socket named '"
            + name + "'.");

        // This property is accessed / edited by the Socket class. It is
        // not easily accessible to users.
        // TODO does putting the addProperty here break the ability to
        // create a custom-copy-ctor version of all of this?
        // TODO property type should be ComponentPath or something like that.
        PropertyIndex propIndex = this->template addProperty<std::string>(
                "socket_" + name , propertyComment, "");
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
    // underlying computational system. It includes the number of built-in states  
    // exposed by this component. It represents the number of state variables  
    // managed by this Component.
    int getNumStateVariablesAddedByComponent() const 
    {   return (int)_namedStateVariableInfo.size(); }
    Array<std::string> getStateVariableNamesAddedByComponent() const;

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
    mutable std::map<std::string, CacheInfo>            _namedCacheVariableInfo;

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
                                !_filter.isMatch(*_node) || 
                                (_node == _root))){
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
void Socket<C>::finalizeConnection(const Component& root) {

    // If the reference to the connectee is set, use that. Otherwise, use the
    // connectee path property.
    if (isConnected()) {
        const auto& comp = *connectee;
        const auto& rootOfConnectee = comp.getRoot();
        const auto& myRoot = getOwner().getRoot();
        OPENSIM_THROW_IF(&myRoot != &rootOfConnectee, Exception,
            "Socket<" + getConnecteeTypeName() + "> '" + getName() +
            "' in " + getOwner().getConcreteClassName() + " at " +
            getOwner().getAbsolutePathString() + " cannot connect to " +
            comp.getConcreteClassName() + " at " +
            comp.getAbsolutePathString() + ": components do not have the same "
            "root component. Did you intend to add '" +
            rootOfConnectee.getName() + "' to '" + myRoot.getName() + "'?");

        ComponentPath connecteePath = connectee->getRelativePath(getOwner());
        // If the relative path starts with ".." then use an absolute path
        // instead.
        if (connecteePath.getNumPathLevels() > 1 &&
                connecteePath.getSubcomponentNameAtLevel(0) == "..")
            connecteePath = connectee->getAbsolutePath();
        updConnecteePathProp().setValue(0, connecteePath.toString());
        
    } else {
        const auto connecteePath = getConnecteePath();
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

            if (isListSocket())
                updConnecteePathProp().appendValue(pathStr);
            else
                updConnecteePathProp().setValue(pathStr);
        }
    } else {
        if (!isListSocket() && getConnecteePath().empty()) return;
        std::string compPathStr, outputName, channelName, alias;
        for (unsigned ix = 0; ix < getNumConnectees(); ++ix) {
            parseConnecteePath(getConnecteePath(ix),
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

