#ifndef OPENSIM_MODEL_H_
#define OPENSIM_MODEL_H_
/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Model.h                              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Peter Loan, Ayman Habib, Ajay Seth           *
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

// INCLUDES
#include <string>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Units.h>
#include <OpenSim/Common/ModelDisplayHints.h>
#include <OpenSim/Simulation/AssemblySolver.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/ComponentSet.h>
#include <OpenSim/Simulation/Model/ContactGeometrySet.h>
#include <OpenSim/Simulation/Model/ControllerSet.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/Model/Ground.h>
#include <OpenSim/Simulation/Model/JointSet.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/ModelVisualPreferences.h>
#include <OpenSim/Simulation/Model/ModelVisualizer.h>
#include <OpenSim/Simulation/Model/ProbeSet.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>

#include "simbody/internal/Force_Gravity.h"
#include "simbody/internal/GeneralContactSubsystem.h"


namespace OpenSim {

class Actuator;
class Analysis;
class Body;
class Constraint;
class ConstraintSet;
class ContactGeometry;
class Controller;
class CoordinateSet;
class Force;
class Frame;
class Muscle;
class Storage;
class ScaleSet;

#ifdef SWIG
    #ifdef OSIMSIMULATION_API
        #undef OSIMSIMULATION_API
        #define OSIMSIMULATION_API
    #endif
#endif

//==============================================================================
/// Model  Exceptions
//==============================================================================
class ModelHasNoSystem : public Exception {
public:
    ModelHasNoSystem(const std::string& file, size_t line,
            const std::string& func,
            const std::string& modelName) :
                OpenSim::Exception(file, line, func) {
        std::string msg = "You must first call initSystem() on your Model";
        if (!modelName.empty()) {
            msg += " '" + modelName + "'";
        }
        msg += ".";
        addMessage(msg);
    }
};

class PhysicalOffsetFramesFormLoop : public Exception {
public:
    PhysicalOffsetFramesFormLoop(const std::string& file,
        size_t line,
        const std::string& func,
        const Object& obj,
        const std::string& frameName) :
        Exception(file, line, func, obj) {
        std::string msg =
            "PhysicalOffsetFrames are not permitted to form loops.\n'" +
            frameName + "' already part of a branch of PhysicalOffsetFrames.";
        addMessage(msg);
    }
};

class JointFramesHaveSameBaseFrame : public Exception {
public:
    JointFramesHaveSameBaseFrame(const std::string& file,
        size_t line,
        const std::string& func,
        const std::string& thisName,
        const std::string& parentName,
        const std::string& childName,
        const std::string& baseName) :
        Exception(file, line, func) {
        std::string msg = "Joint '" + thisName +
            "' cannot connect parent frame '" +
            parentName + "' to child frame '" + childName + "'.\n" +
            "Parent and child frames have the same base frame '" +
            baseName + "'.";
        addMessage(msg);
    }
};



//==============================================================================
//                                  MODEL
//==============================================================================
/** A concrete class that specifies the interface to a musculoskeletal model.
You can read this in from an XML file or create it programmatically, and
modify it via the API.

A Model contains ModelComponents, and is itself a ModelComponent so must
satisfy the ModelComponent interface, as well as the Object interface from
which ModelComponent derives. This allows a Model to allocate "global"
resources using ModelComponent resource-allocation facilities.

Computation using a Model is done by creating a computational representation
of the Model, called a System (SimTK::System), using Simbody. Creation of the
System is initiated by a call to the Model's initSystem() method. The System and
related objects are maintained in a runtime section of the Model object. You
can also ask a Model to provide visualization using the setUseVisualizer()
method, in which case it will allocate and maintain a ModelVisualizer.

@authors Frank Anderson, Peter Loan, Ayman Habib, Ajay Seth, Michael Sherman
@see ModelComponent, ModelVisualizer, SimTK::System
**/

class OSIMSIMULATION_API Model  : public ModelComponent {
// Note, a concrete object typically employs the OpenSim_DECLARE_CONCRETE_OBJECT
// macro, but, for Model we do not want its clone() method to be auto-generated.
// Instead, we want to override and customize it and so we employ the subset of
// macros that OpenSim_DECLARE_CONCRETE_OBJECT calls, excluding one that
// implements clone() and getConcreteClassName(), which are implemented below.
OpenSim_OBJECT_ANY_DEFS(Model, ModelComponent);
OpenSim_OBJECT_NONTEMPLATE_DEFS(Model, ModelComponent);

public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(credits, std::string,
        "Credits (e.g., model author names) associated with the model.");

    OpenSim_DECLARE_PROPERTY(publications, std::string,
        "Publications and references associated with the model.");

    OpenSim_DECLARE_PROPERTY(length_units, std::string,
        "Units for all lengths.");

    OpenSim_DECLARE_PROPERTY(force_units, std::string,
        "Units for all forces.");

    OpenSim_DECLARE_PROPERTY(assembly_accuracy, double,
    "Specify how accurate the resulting configuration of a model assembly "
    "should be. This translates to the number of significant digits in the "
    "resulting coordinate values. Therefore, if you require initial conditions "
    "accurate to four significant digits, use a minimum of 1e-4 as the accuracy."
    "The default setting is 1e-9 as to satisfy the most stringent requirements by "
    "default. NOTE: Failure for a model to satisfy the assembly accuracy often "
    "indicates inconsistency in the constraints. For example, the feet are welded "
    "at locations measured to five significant digits while the model lacks dofs "
    "to change stance width, in which case it cannot achieve 1e-9 accuracy." );

    OpenSim_DECLARE_PROPERTY(gravity, SimTK::Vec3,
        "Acceleration due to gravity, expressed in ground.");

    OpenSim_DECLARE_PROPERTY(ground, Ground,
        "The model's ground reference frame.");

    OpenSim_DECLARE_UNNAMED_PROPERTY(BodySet,
        "List of bodies that make up this model.");

    OpenSim_DECLARE_UNNAMED_PROPERTY(JointSet,
        "List of joints that connect the bodies.");

    OpenSim_DECLARE_UNNAMED_PROPERTY(ConstraintSet,
        "Constraints in the model.");

    OpenSim_DECLARE_UNNAMED_PROPERTY(MarkerSet,
        "Markers in the model.");

    OpenSim_DECLARE_UNNAMED_PROPERTY(ForceSet,
        "Forces in the model (includes Actuators).");

    OpenSim_DECLARE_UNNAMED_PROPERTY(ControllerSet,
        "Controllers that provide the control inputs for Actuators.");

    OpenSim_DECLARE_UNNAMED_PROPERTY(ContactGeometrySet,
        "Geometry to be used in contact forces.");

    OpenSim_DECLARE_UNNAMED_PROPERTY(ProbeSet,
        "Probes in the model.");

    OpenSim_DECLARE_UNNAMED_PROPERTY(ComponentSet,
        "Additional components in the model.");

    OpenSim_DECLARE_UNNAMED_PROPERTY(ModelVisualPreferences,
        "Visual preferences for this model.");

//==============================================================================
// OUTPUTS
//==============================================================================
    OpenSim_DECLARE_OUTPUT(com_position, SimTK::Vec3,
            calcMassCenterPosition, SimTK::Stage::Position);

    OpenSim_DECLARE_OUTPUT(com_velocity, SimTK::Vec3,
            calcMassCenterVelocity, SimTK::Stage::Velocity);

    OpenSim_DECLARE_OUTPUT(com_acceleration, SimTK::Vec3,
            calcMassCenterAcceleration, SimTK::Stage::Acceleration);

    OpenSim_DECLARE_OUTPUT(kinetic_energy, double,
            calcKineticEnergy, SimTK::Stage::Position);

    OpenSim_DECLARE_OUTPUT(potential_energy, double,
            calcPotentialEnergy, SimTK::Stage::Velocity);

    OpenSim_DECLARE_OUTPUT(momentum, SimTK::SpatialVec,
            calcMomentum, SimTK::Stage::Velocity);

    OpenSim_DECLARE_OUTPUT(angular_momentum, SimTK::Vec3,
            calcAngularMomentum, SimTK::Stage::Velocity);

    OpenSim_DECLARE_OUTPUT(linear_momentum, SimTK::Vec3,
            calcLinearMomentum, SimTK::Stage::Velocity);


//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION AND DESTRUCTION
    //--------------------------------------------------------------------------
public:

    /** Default constructor creates a %Model containing only the Ground frame
    and a set of default properties. */
    Model();

    /** Constructor from an OpenSim XML model file.
    NOTE: The Model is read in (deserialized) from the model file, which means
    the properties of the Model and its components are filled in from values in
    the file. In order to evaluate the validity of the properties (e.g. Inertia
    tensors, availability of Mesh files, ...) and to identify properties as
    subcomponents of the Model, one must invoke Model::finalizeFromProperties()
    first. Model::initSystem() invokes finalizeFromProperties() on its way to
    creating the System and initializing the State.

    @param filename     Name of a file containing an OpenSim model in XML
                        format; suffix is typically ".osim".
    **/
    explicit Model(const std::string& filename) SWIG_DECLARE_EXCEPTION;

    /** Satisfy all connections (Sockets and Inputs) in the model, using this
     * model as the root Component. This is a convenience form of
     * Component::finalizeConnections() that uses this model as root.
     */
    void finalizeConnections() { finalizeConnections(*this); }
    // Allow overloading.
   using Component::finalizeConnections;

    /**
     * Perform some set up functions that happen after the
     * object has been deserialized. TODO: this method is
     * not yet designed to be called after a model has been
     * copied.
     */
    void setup() SWIG_DECLARE_EXCEPTION;

    /**
     * Perform some clean up functions that are normally done
     * from the destructor however this gives the GUI a way to
     * proactively do the cleaning without waiting for garbage
     * collection to do the actual cleanup.
     */
    void cleanup();

    /** Model clone() override that invokes finalizeFromProperties()
        on a default copy constructed Model, prior to returning the Model. */
    Model* clone() const override;

    const std::string& getConcreteClassName() const override
    {   return getClassName(); }

    //--------------------------------------------------------------------------
    // VISUALIZATION
    //--------------------------------------------------------------------------
    /** @name                     Visualization
    Methods in this group affect visualization of this Model, which may be
    through the OpenSim GUI or through the use of a ModelVisualizer which can
    provide limited run-time display and user interaction capability for an
    OpenSim API-based program. If you enable visualization at the API level,
    the %Model will create a ModelVisualizer that you can use to control
    display and user interaction. In turn, the ModelVisualizer makes use of a
    Simbody SimTK::Visualizer; for advanced features you can ask the
    ModelVisualizer to give you direct access to the SimTK::Visualizer; consult
    Simbody documentation for more details. **/
    /**@{**/

    /** Get read only access to the ModelDisplayHints object stored with this
    %Model. These should be checked whenever display geometry is being
    generated. **/
    const ModelDisplayHints& getDisplayHints() const {
        return get_ModelVisualPreferences().get_ModelDisplayHints();}
    /** Get writable access to the ModelDisplayHints object stored with this
    %Model. The GUI or ModelVisualizer can update these as a result of user
    requests, or an OpenSim API program can set them programmatically. **/
    ModelDisplayHints& updDisplayHints() {
        return upd_ModelVisualPreferences().upd_ModelDisplayHints(); }

    /** Request or suppress visualization of this %Model. This flag is checked
    during initSystem() and if set causes the %Model to allocate a
    ModelVisualizer that provides visualization and interaction with the %Model
    as it is executing. The default is no visualization.
    @see getModelVisualizer() **/
    void setUseVisualizer(bool visualize) {_useVisualizer=visualize;}
    /** Return the current setting of the "use visualizer" flag, which will
    take effect at the next call to initSystem() on this %Model. **/
    bool getUseVisualizer() const {return _useVisualizer;}

    /** Test whether a ModelVisualizer has been created for this Model. Even
    if visualization has been requested there will be no visualizer present
    until initSystem() has been successfully invoked. Use this method prior
    to calling getVisualizer() or updVisualizer() to avoid an
    unpleasant exception. **/
    bool hasVisualizer() const {return _modelViz != nullptr;}

    /** Obtain read-only access to the ModelVisualizer. This will throw an
    exception if visualization was not requested or initSystem() not yet
    called.
    @return A const reference to the allocated ModelVisualizer. **/
    const ModelVisualizer& getVisualizer() const {
        if (!hasVisualizer())
            throw Exception("Model::getVisualizer(): no visualizer present.");
        return *_modelViz;
    }
    /** Obtain writable access to the ModelVisualizer. This will throw an
    exception if visualization was not requested or initSystem() not yet
    called. Writable access to the ModelVisualizer requires that you have
    writable access to this containing Model.
    @return A non-const reference to the allocated ModelVisualizer. **/
    ModelVisualizer& updVisualizer() {
        if (!hasVisualizer())
            throw Exception("Model::updVisualizer(): no visualizer present.");
        return *_modelViz;
    }
    /**@}**/

    /** After the %Model and its components have been constructed, call this to
    interconnect the components and then create the Simbody
    MultibodySystem needed to represent the %Model computationally. The
    extendConnectToModel() method of each contained ModelComponent will be invoked,
    and then their addToSystem() methods are invoked.
    The resulting MultibodySystem is maintained internally by the %Model. After
    this call, you may obtain a writable reference to the System using
    updMultibodySystem() which you can use to make any additions you want. Then
    when the System is complete, call initializeState() to finalize it and
    obtain an initial State. **/
    void buildSystem();

    /** After buildSystem() has been called, and any additional modifications
    to the Simbody MultibodySystem have been made, call this method to finalize
    the MultibodySystem (by calling its realizeTopology() method), obtain an
    initial state, and assemble it so that position constraints are
    satisfied. The initStateFromProperties() method of each contained
    ModelComponent will be invoked. A reference to the writable internally-
    maintained model State is returned (note that this does not affect the
    system's default state (which is part of the model and hence read only).
    The model's state can be reset to the system's default state at any time
    by re-executing initializeState(). **/
    SimTK::State& initializeState();


    /** Convenience method that invokes buildSystem() and then
    initializeState(). This returns a reference to the writable internally-
    maintained model State. Note that this does not affect the
    system's default state (which is part of the model and hence read-only). **/
    SimTK::State& initSystem() SWIG_DECLARE_EXCEPTION {
        buildSystem();
        return initializeState();
    }


    /** Convenience method that returns a reference to the model's 'working'
    state. This is just returning the reference that was returned by
    initSystem() and initializeState() -- note that either of these methods
    must be called prior to getWorkingState(), otherwise an empty state will
    be returned. **/
    const SimTK::State& getWorkingState() const;
    SimTK::State& updWorkingState();

    /**
     * This is called after the Model is fully created but before starting a simulation.
     * It ONLY initializes the computational system used to simulate the model and
     * addToSystem() has been called already. This method should only be used if
     * if additional SimTK::System components are being added using the SimTK API
     * and the programmer is certain that the model's system has been created.
     */
    void initStateWithoutRecreatingSystem(SimTK::State& state) const { initStateFromProperties(state); };

    /**
     * Mark the computational system as invalid.  This should be called whenever a property
     * of the model is modified.  Once this has been called, no calculations can be done until
     * initSystem() is called again.
     */
    void invalidateSystem();

    /** Check that the underlying computational system representing the model is valid.
        That is, is the system ready for performing calculations. */
    bool isValidSystem() const;

    /**
     * Create a storage (statesStorage) that has same label order as model's states
     * with values populated from originalStorage. Use the default state value if
     * a state is unspecified in the originalStorage. If warnUnspecifiedStates is
     * true then a warning is printed that includes the default value used for
     * the state value unspecified in originalStorage. The input originalStorage
     * must be in meters or radians for Coordinate values and their speeds
     * (m/s, rad/s) otherwise an Exception is thrown.
     */
    void formStateStorage(const Storage& originalStorage,
                          Storage& statesStorage,
                          bool warnUnspecifiedStates = true) const;

    void formQStorage(const Storage& originalStorage, Storage& qStorage);

    /**
     * Update the AssemblySolver to the latest coordinate locking/constraints
     */
    void updateAssemblyConditions(SimTK::State& s);
    /**
     * Find the kinematic state of the model that satisfies constraints and coordinate goals
     * If assemble is being called due to a coordinate set value, provide the option
     * to weight that coordinate value more heavily if specified.
     */
    void assemble(SimTK::State& state, const Coordinate *coord = NULL, double weight = 10);


    /**
     * Update the state of all Muscles so they are in equilibrium.
     */
    void equilibrateMuscles(SimTK::State& state);

    //--------------------------------------------------------------------------
    /**@name       Access to the Simbody System and components

    Methods in this section provide advanced users access to the underlying
    Simbody System and associated subcomponents that are constructed and
    maintained by this %Model. Note that these are not available until after
    initSystem() has been invoked on this %Model. Be very careful if you
    call any of the upd() methods since modifying a System after the %Model
    creates it can require reinitialization.
    @see initStateWithoutRecreatingSystem() **/
    /**@{**/

    /** Get read-only access to the internal Simbody MultibodySystem that was
    created by this %Model at the last initSystem() call. **/
    const SimTK::MultibodySystem& getMultibodySystem() const {return getSystem(); }
    /** (Advanced) Get writable access to the internal Simbody MultibodySystem
    that was created by this %Model at the last initSystem() call. Be careful
    if you make modifications to the System because that will invalidate
    initialization already performed by the Model.
    @see initStateWithoutRecreatingSystem() **/
    SimTK::MultibodySystem& updMultibodySystem() const {return updSystem(); }

    /** Get read-only access to the internal DefaultSystemSubsystem allocated
    by this %Model's Simbody MultibodySystem. **/
    const SimTK::DefaultSystemSubsystem& getDefaultSubsystem() const
    {   return getMultibodySystem().getDefaultSubsystem(); }
    /** (Advanced) Get writable access to the internal DefaultSystemSubsystem
    allocated by this %Model's Simbody MultibodySystem. **/
    SimTK::DefaultSystemSubsystem& updDefaultSubsystem()
    {   return updMultibodySystem().updDefaultSubsystem(); }

    /** Get read-only access to the internal SimbodyMatterSubsystem allocated
    by this %Model. **/
    const SimTK::SimbodyMatterSubsystem& getMatterSubsystem() const
    {   return _system->getMatterSubsystem(); }
    /** (Advanced) Get writable access to the internal SimbodyMatterSubsystem
    allocated by this %Model. **/
    SimTK::SimbodyMatterSubsystem& updMatterSubsystem()
    {   return _system->updMatterSubsystem(); }

    /** Get read-only access to the Simbody Force::Gravity element that was
    allocated by this %Model. **/
    const SimTK::Force::Gravity& getGravityForce() const
    {   return *_gravityForce; }
    /** (Advanced) Get writable access to the Simbody Force::Gravity element
    that was allocated by this %Model. **/
    SimTK::Force::Gravity& updGravityForce()
    {   return *_gravityForce; }

    /** Get read-only access to the internal Simbody GeneralForceSubsystem
    allocated by this %Model. **/
    const SimTK::GeneralForceSubsystem& getForceSubsystem() const
    {   return *_forceSubsystem; }
    /** (Advanced) Get writable access to the internal Simbody
    GeneralForceSubsystem allocated by this %Model. **/
    SimTK::GeneralForceSubsystem& updForceSubsystem()
    {   return *_forceSubsystem; }
    /** (Advanced) Get read only access to internal Simbody RigidBodyForces at Dynamics stage **/
    const SimTK::Vector_<SimTK::SpatialVec>& getRigidBodyForces(const SimTK::State& state)
    {
        return getMultibodySystem().getRigidBodyForces(state, SimTK::Stage::Dynamics);
    }
    /** (Advanced) Get read only access to internal Simbody Mobility Forces at Dynamics stage **/
    const SimTK::Vector& getMobilityForces(const SimTK::State& state)
    {
        return getMultibodySystem().getMobilityForces(state, SimTK::Stage::Dynamics);
    }
    /** (Advanced) Get read only access to internal Simbody Body Forces due to Gravity **/
    const SimTK::Vector_<SimTK::SpatialVec>& getGravityBodyForces(const SimTK::State& state) const
    {
        return getGravityForce().getBodyForces(state);
    }

    /** Get read-only access to the internal Simbody CableSubsystem
    allocated by this %Model. **/
    const SimTK::CableSubsystem& getCableSubsystem() const
    {   return *_cableSubsystem; }
    /** (Advanced) Get writable access to the internal Simbody
    CableSubsystem allocated by this %Model. **/
    SimTK::CableSubsystem& updCableSubsystem()
    {   return *_cableSubsystem; }
    /**@}**/

    /**@name  Realize the Simbody System and State to Computational Stage
    Methods in this section enable advanced and scripting users access to
    realize the Simbody MultibodySystem and the provided state to a desired
    computational (realization) Stage.
    Note that these are not accessible until after initSystem() has been
    invoked on this %Model. **/
    /**@{**/

    /** Perform computations that depend only on time and earlier stages. **/
    void realizeTime(const SimTK::State& state) const;
    /** Perform computations that depend only on position-level state
    variables and computations performed in earlier stages (including time). **/
    void realizePosition(const SimTK::State& state) const;
    /** Perform computations that depend only on velocity-level state
    variables and computations performed in earlier stages (including position,
    and time). **/
    void realizeVelocity(const SimTK::State& state) const;
    /** Perform computations (typically forces) that may depend on
    dynamics-stage state variables, and on computations performed in earlier
    stages (including velocity, position, and time), but not on other forces,
    accelerations, constraint multipliers, or reaction forces. **/
    void realizeDynamics(const SimTK::State& state) const;
    /** Perform computations that may depend on applied forces. **/
    void realizeAcceleration(const SimTK::State& state) const;
    /** Perform computations that may depend on anything but are only used
    for reporting and cannot affect subsequent simulation behavior. **/
    void realizeReport(const SimTK::State& state) const;

    /**@}**/

    /** @name Adding components to the Model
     * Model takes ownership of the ModelComponent and adds it to a specialized
     * (typed) Set within the model. Model will maintain Components added using
     * these methods in separate %Sets of the corresponding type and they will
     * serialize as part of type specific %Sets. These sets can be useful for
     * uniquely identifying components that share the same name, but are of
     * different types since they  live in different %Sets. For example, using
     * addBody(toesBody) and addJoint(toesJoint) will have unique paths:
     * "/bodyset/toes" and "/jointset/toes", respectively, even when they have
     * the same name, "toes".
     * @note these are legacy methods and remain as a convenience alternative to
     * using Component::addComponent().
     * In contrast, components added using addComponent() are not stored in the
     * model's %Sets but live in a flat components list (which is also serialized).
     * Component provides access via getComponentList<%SpecificType> or
     * getComponent<%SpecificType> to get any subcomponent, including those that
     * are contained in Model's %Sets. In this case, either a Body or a Joint has
     * the pathname "/toes" but both cannot share the same name and will throw
     * SubcomponentsWithDuplicateName.
     * Future versions of OpenSim are likely to deprecate the use of Sets and
     * these methods, because they cannot support new Component types without
     * modifying the API (for more add####() methods), whereas
     * getComponentList<%SpecificType>() and getComponent<%SpecificType> will
     * generalize: they do not have these limitations and are applicable for
     * adding to any Component not just Model.
     */
    // @{
    void addModelComponent(ModelComponent* adoptee);
    void addBody(Body *adoptee);
    void addJoint(Joint *adoptee);
    void addConstraint(Constraint *adoptee);
    void addForce(Force *adoptee);
    void addProbe(Probe *adoptee);
    void addContactGeometry(ContactGeometry *adoptee);
    void addMarker(Marker *adoptee);
    // @}

    /** remove passed in Probe from model **/
    void removeProbe(Probe *probe);

    /**
     * Get the XML file name used to construct the model.
     *
     * @return XML file name string.
     */
    const std::string& getInputFileName() const { return _fileName; }

    /**
     * %Set the XML file name used to construct the model.
     *
     * @param fileName The XML file name.
     */
    void setInputFileName(const std::string& fileName) { _fileName = fileName; }

    /**
     * Get the credits (e.g., model author names) associated with the model.
     *
     * @return Credits string.
     */
    const std::string& getCredits() const { return get_credits(); }

    /**
     * %Set the credits (e.g., model author names) associated with the model.
     *
     * @param aCredits The string of credits.
     */
    void setAuthors(const std::string& aCredits) { upd_credits() = aCredits; }

    /**
     * Get the publications associated with the model.
     *
     * @return Publications string.
     */
    const std::string& getPublications() const { return get_publications(); }

    /**
     * %Set the publications associated with the model.
     *
     * @param aPublications The string of publications.
     */
    void setPublications(const std::string& aPublications) { upd_publications() = aPublications; }

    /**
     * Get the length units associated with the model.
     *
     * @return Length units.
     */
    const Units& getLengthUnits() const { return _lengthUnits; }

    /**
     * Get the force units associated with the model.
     *
     * @return Force units
     */
    const Units& getForceUnits() const { return _forceUnits; }

    /**
     * Get the gravity vector in the global frame.
     *
     * @return The XYZ gravity vector in the global frame.
     */
    SimTK::Vec3 getGravity() const;

    /**
     * %Set the gravity vector in the global frame.
     *
     * @param aGrav The XYZ gravity vector
     * @return Whether or not the gravity vector was successfully set.
     */
    bool setGravity(const SimTK::Vec3& aGrav);

    /**
     * Get the number of markers in the model.
     * @return Number of markers.
     */
    int getNumMarkers() const;

    /**
     * Get the number of ContactGeometries in the model.
     * @return Number of ContactGeometries.
     */
    int getNumContactGeometries() const;

    /**
     * Get the total number of bodies in the model.
     * @return Number of bodies.
     */
    int getNumBodies() const;

    /**
    * Get the total number of joints in the model.
    * @return Number of joints.
    */
    int getNumJoints() const;

    /**
     * Get the total number of coordinates in the model.
     * @return Number of coordinates.
     */
    int getNumCoordinates() const;

    /**
    * Get the total number of speeds in the model.
    * @return Number of speeds.
    */
    int getNumSpeeds() const;

    /**
    * Get the total number of constraints in the model.
    * @return Number of constraints.
    */
    int getNumConstraints() const;

    /**
     * Get the total number of probes in the model.
     * @return Number of probes.
     */
    int getNumProbes() const;

    /**
     * Get the subset of Forces in the model which are actuators
     * @return The set of Actuators
     */
    const Set<Actuator>& getActuators() const;
    Set<Actuator>& updActuators();

    /**
     * Get the subset of Forces in the model which are muscles
     * @return The set of Muscles
     */
    const Set<Muscle>& getMuscles() const;
    Set<Muscle>& updMuscles();

    const ForceSet& getForceSet() const { return get_ForceSet(); };
    ForceSet& updForceSet() { return upd_ForceSet(); };

    /**
     * Get the subset of Probes in the model
     * @return The set of Probes
     */
    const ProbeSet& getProbeSet() const { return get_ProbeSet(); };
    ProbeSet& updProbeSet() { return upd_ProbeSet(); };

    /**
     * Get the subst of misc ModelComponents in the model
     * @return The set of misc ModelComponents
     */
    const ComponentSet& getMiscModelComponentSet() const
    {   return get_ComponentSet(); };
    ComponentSet& updMiscModelComponentSet()
    {   return upd_ComponentSet(); };

    /**
     * Get the number of analyses in the model.
     * @return The number of analyses.
     */
    int getNumAnalyses() const;

    /**
     * Get the number of controls for this the model.
     * Only valid once underlying system for the model has been created.
     * Throws an exception if called before Model::initSystem()
     * @return number of controls corresponding to all the actuators in the model
     */
    int getNumControls() const;

    /** Writable access to the default values for controls. These values are used for
        control value during a simulation unless updated, for example, by a Controller */
    SimTK::Vector& updDefaultControls() const { return _defaultControls; }
    void setDefaultControls(const SimTK::Vector& controls) const
    {   _defaultControls = controls; }
    const SimTK::Vector& getDefaultControls() const { return _defaultControls; };

    /**
     * Update the controls for this the model at a given state.
     * Only callable once underlying system for the model has been created.
     * Throws an exception if called before Model::initSystem()
     * This call invalidates the dynamics of the model and invalidates the
     * value of the controls until they are marked as valid when the update
     * is completed (@see markControlsAsValid)
     * @param[in]   s         System state at which to apply the controls
     * @return      writable controls Vector
     */
    SimTK::Vector& updControls(const SimTK::State& s) const;

    /**
     * Mark controls as valid after an update at a given state.
     * Indicates that controls are valid for use at the dynamics stage.
     * If the stage is Velocity or lower the controls will remain invalid.
     * @param[in]   s         System state in which the controls are updated
     */
    void markControlsAsValid(const SimTK::State& s) const;

    /**
     * Mark controls as invalid after an update at a given state.
     * Indicates that controls are not valid for use at the dynamics stage.
     * @param[in]   s         System state in which the controls are updated
     */
    void markControlsAsInvalid(const SimTK::State& s) const;

    /**
     * Alternatively, set the controls on the model at a given state.
     * Note, this method will invalidate the dynamics of the model,
     * but will mark the controls as valid. (E.g. controllers will not be invoked)
     * @param[in]   s         System state at which to apply the controls
     * @param[in]   controls  The complete Vector of controls to be applied
     */
    void setControls(const SimTK::State& s, const SimTK::Vector& controls) const;

    /** Const access to controls does not invalidate dynamics */
    const SimTK::Vector& getControls(const SimTK::State &s) const;

    /** Compute the controls for the model.
    Calls down to the Controllers to make their contributions to the controls.

    @param[in]   state       System state from which Controllers should draw
                             when computing their control outputs.
    @param[out]  controls    The complete vector of controls into which
                             individual controller contributions should be
                             added. **/
    void computeControls(const SimTK::State& state, SimTK::Vector& controls) const;

    /**
     * Get a flag indicating if the model needs controls to operate its actuators
     */
    bool isControlled() const;
    void storeControls( const SimTK::State& s, int step );
    void printControlStorage(const std::string& fileName ) const;
    TimeSeriesTable getControlsTable() const;
    const ControllerSet& getControllerSet() const;
    ControllerSet& updControllerSet();
    bool getAllControllersEnabled() const;
    void setAllControllersEnabled( bool enabled );

    void applyDefaultConfiguration(SimTK::State& s );


    /**
     * Get the model's dynamics engine
     *
     * @return Reference to the Simbody dynamics engine
     */
    const SimbodyEngine& getSimbodyEngine() const { return _simbodyEngine; }
    SimbodyEngine& updSimbodyEngine() { return _simbodyEngine; }

    //--------------------------------------------------------------------------
    // Subsystem computations
    //--------------------------------------------------------------------------
    /**
     * Compute the derivatives of the generalized coordinates and speeds.
     */
    void computeStateVariableDerivatives(const SimTK::State &s) const override;

    /**
     * Get the total mass of the model.
     *
     * @return the mass of the model.
     */
    double getTotalMass(const SimTK::State &s) const;

    /**
     * Get the whole-body inertia of the model about the center of mass,
     * expressed in the Ground frame.
     */
    SimTK::Inertia getInertiaAboutMassCenter(const SimTK::State &s) const;

    /**
     * Return the position vector of the system mass center, measured from the
     * Ground origin, and expressed in Ground.
     */
    SimTK::Vec3 calcMassCenterPosition(const SimTK::State &s) const;

    /**
     * Return the velocity vector of the system mass center, measured from the
     * Ground origin, and expressed in Ground.
     */
    SimTK::Vec3 calcMassCenterVelocity(const SimTK::State &s) const;

    /**
     * Return the acceleration vector of the system mass center, measured from
     * the Ground origin, and expressed in Ground.
     */
    SimTK::Vec3 calcMassCenterAcceleration(const SimTK::State &s) const;

    /**
     * Return the spatial momentum about the system mass center expressed in
     * Ground.
     */
    SimTK::SpatialVec calcMomentum(const SimTK::State &s) const;

    /**
     * Return the angular momentum about the system mass center expressed in
     * Ground.
     */
    SimTK::Vec3 calcAngularMomentum(const SimTK::State &s) const;

    /**
     * Return the linear momentum expressed in Ground.
     */
    SimTK::Vec3 calcLinearMomentum(const SimTK::State &s) const;

    /** Return the total Kinetic Energy for the underlying system.*/
    double calcKineticEnergy(const SimTK::State &s) const {
        return getMultibodySystem().calcKineticEnergy(s);
    }

    /** Return the total Potential Energy for the underlying system.*/
    double calcPotentialEnergy(const SimTK::State &s) const {
        return getMultibodySystem().calcPotentialEnergy(s);
    }

    /** Calulate the sum of body and mobility forces in the system applied by
     * the Force%s at the supplied indexes.
     *
     * @param[in]   state            The system SimTK::State at which to
     *                               calculate forces.
     * @param[in]   forceIndexes     The indexes (SimTK::ForceIndex) of the
     *                               forces in the system for which to calculate
     *                               forces.
     * @param[out]  bodyForces       The sum of the body forces applied by the
     *                               forces at the supplied indexes.
     * @param[out]  mobilityForces   The sum of the mobility forces applied by
     *                               the forces at the supplied indexes.
     *
     * @pre Requires that the system has been realized to Stage::Dynamics.
     * */
    void calcForceContributionsSum(const SimTK::State& state,
            const SimTK::Array_<SimTK::ForceIndex>& forceIndexes,
            SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
            SimTK::Vector& mobilityForces) const {
        getForceSubsystem().calcForceContributionsSum(
            state, forceIndexes, bodyForces, mobilityForces);
    }

    int getNumMuscleStates() const;
    int getNumProbeStates() const;

    //--------------------------------------------------------------------------
    // SETS
    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    // COORDINATES
    //--------------------------------------------------------------------------
    CoordinateSet& updCoordinateSet() {
       return _coordinateSet;
    }
    const CoordinateSet& getCoordinateSet() const {
       return _coordinateSet;
    }

    /** Obtain a list of Model's Coordinates in the order they appear in the
        MultibodySystem after Model::initSystem() has been called.
        Coordinates in the CoordinateSet do not have a predefined order. In
        some instances it is helpful to get the coordinates in order of
        generalized coordinates in the Multibody Tree as defined in the
        underlying MultibodySystem. For example, computing the generalized
        forces from the System, yields a vector of generalized forces
        in order of the Multibody Tree and now that can be attributed to
        corresponding generalized Coordinates of the Model.
        Throws if the MultibodySystem is not valid. */
    std::vector<SimTK::ReferencePtr<const OpenSim::Coordinate>>
        getCoordinatesInMultibodyTreeOrder() const;

    /** A variant of getCoordinatesInMultibodyTreeOrder that returns names for Scripting users */
    SimTK::Array_<std::string> getCoordinateNamesInMultibodyTreeOrder() {
        SimTK::Array_<std::string> namesArray;
        auto coords = getCoordinatesInMultibodyTreeOrder();
        for (int i=0; i < (int)coords.size(); ++i)
            namesArray.push_back(coords[i]->getName());
        return namesArray;
    }
    /** Get a warning message if any Coordinates have a MotionType that is NOT
        consistent with its previous user-specified value that existed in
        Model files prior to OpenSim 4.0 */
    std::string getWarningMesssageForMotionTypeInconsistency() const;

    BodySet& updBodySet() { return upd_BodySet(); }
    const BodySet& getBodySet() const { return get_BodySet(); }

    JointSet& updJointSet();
    const JointSet& getJointSet() const;

    AnalysisSet& updAnalysisSet() {return _analysisSet; }
    const AnalysisSet& getAnalysisSet() const {return _analysisSet; }

    ContactGeometrySet& updContactGeometrySet() { return upd_ContactGeometrySet(); }
    const ContactGeometrySet& getContactGeometrySet() const { return get_ContactGeometrySet(); }

    /** Get a const reference to the Ground reference frame */
    const Ground& getGround() const;
    /** Get a writable reference to the Ground reference frame */
    Ground& updGround();

    //--------------------------------------------------------------------------
    // CONSTRAINTS
    //--------------------------------------------------------------------------
    ConstraintSet& updConstraintSet() { return upd_ConstraintSet(); }
    const ConstraintSet& getConstraintSet() const { return get_ConstraintSet(); }

    //--------------------------------------------------------------------------
    // MARKERS
    //--------------------------------------------------------------------------
    MarkerSet& updMarkerSet() { return upd_MarkerSet(); }
    const MarkerSet& getMarkerSet() const { return get_MarkerSet(); }

    void writeMarkerFile(const std::string& aFileName);

    /**
    * Update the markers in the model by appending the ones in the
    * passed-in marker set. If the marker of the same name exists
    * in the model, then replace it.
    *
    * @param newMarkerSet the set of markers used to update the model's set.
    */
    void updateMarkerSet(MarkerSet& newMarkerSet);
    int deleteUnusedMarkers(const Array<std::string>& aMarkerNames);

    /**
     * Add an Analysis to the %Model.
     *
     * @param adoptee pointer to the Analysis to add
     */
    void addAnalysis(Analysis *adoptee);
    /** Add a Controller to the %Model. **/
    void addController(Controller *adoptee);

    /**
     * Remove an Analysis from the %Model.
     *
     * @param analysis  Pointer to the analysis to remove.
     * @param deleteIt  Whether the removed object should be deleted.
     */
    void removeAnalysis(Analysis* analysis, bool deleteIt=true);
    /** Remove a Controller from the %Model. **/
    void removeController(Controller *aController);

    //--------------------------------------------------------------------------
    // DERIVATIVES
    //--------------------------------------------------------------------------
    //--------------------------------------------------------------------------
    // OPERATIONS
    //--------------------------------------------------------------------------

    /**
     * Scale the model.
     *
     * @param state      State containing parameter values that might be
     *                   modified here.
     * @param scaleSet   The set of XYZ scale factors for the bodies.
     * @param preserveMassDist
     *                   Whether the masses of the bodies should be scaled by
     *                   the scale factors. If `false`, body masses will be
     *                   adjusted only if `finalMass` has been specified; if
     *                   `true`, body masses will be scaled by the product of
     *                   the body's scale factors (and then a second time if
     *                   `finalMass` has been specified). Inertias are always
     *                   updated to reflect changes in body dimensions.
     * @param finalMass  The total mass that the scaled model should have.
     * @returns          Whether or not scaling was successful.
     */
    bool scale(SimTK::State&    state,
               const ScaleSet&  scaleSet,
               bool             preserveMassDist,
               double           finalMass = -1.0);

    //--------------------------------------------------------------------------
    // PRINT
    //--------------------------------------------------------------------------

    /**
     * Print some basic information about the model.
     *
     * @param aOStream Output stream. If this is std::cout, then the info is
     * logged using OpenSim's Logger so that the info is printed to all log
     * sinks.
     */
    void printBasicInfo(std::ostream& aOStream = std::cout) const;

    /**
     * Print detailed information about the model.
     *
     * @param s        the system State.
     * @param aOStream Output stream. If this is std::cout, then the info is
     * logged using OpenSim's Logger so that the info is printed to all log
     * sinks.
     */
    void printDetailedInfo(const SimTK::State& s,
                           std::ostream& aOStream = std::cout) const;

    /**
     * Model relinquishes ownership of all components such as: Bodies, Constraints, Forces,
     * ContactGeometry and so on. That means the freeing of the memory of these objects is up
     * to the caller. This only affects components stored in the Model's Sets,
     * and does not affect those added via Component::addComponent().
     */
    void disownAllComponents();
    /**
     * Convenience function to turn on/off overriding the force for all actuators
     */
    void overrideAllActuators( SimTK::State& s, bool flag);

    /**
     * Get a log of errors/warnings encountered when loading/constructing the model
     */
    const std::string& getValidationLog() const { return _validationLog; };
    /** Append to the Model's validation log without affecting is current contents */
    void appendToValidationLog(const std::string& note);
    void clearValidationLog() { _validationLog = ""; };

    /**
     * Utility to get a reference to an Object based on its name and type
     * throws an exception if the object was not found.
     * names are case sensitive
     * @param typeString the type of object being looked up (Body, Force, Constraint, Coordinate, Joint, Marker, Controller, Frame)
     * @param nameString the name of the object being looked up
     * @return reference to the object if found or throws an exception.
     */
    const Object& getObjectByTypeAndName(const std::string& typeString, const std::string& nameString) SWIG_DECLARE_EXCEPTION;

    //--------------------------------------------------------------------------
    /**@name            Implementation of Object interface

    These methods are %Model's implementation of virtual methods defined in
    the Object class from which %Model derives (indirectly through
    ModelComponent). **/
    /**@{**/

    /** Destructor. */
    ~Model() override {
        // Must ensure the System is deleted after the subsystem handles,
        // otherwise the subsystem handles are "dangling."
        _contactSubsystem.reset();
        _forceSubsystem.reset();
        _gravityForce.reset();
        _matter.reset();
        _cableSubsystem.reset();
    }

    /** Override of the default implementation to account for versioning. */
    void updateFromXMLNode(SimTK::Xml::Element& aNode,
                           int versionNumber = -1) override;
    /**@}**/

    //--------------------------------------------------------------------------
    /**@name         Implementation of Component interface

    These methods are %Model's implementation of virtual methods defined in
    the Component class from which %Model derives. **/
    /**@{**/
    void extendFinalizeFromProperties() override;

    void extendConnectToModel(Model& model)  override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendInitStateFromProperties(SimTK::State& state) const override;
    /**@}**/

    /**
     * Given a State, set all default values for this Model to match those
     * found in the State.
     */
    void extendSetPropertiesFromState(const SimTK::State& state) override;

    //--------------------------------------------------------------------------
    /**@name         Implementation of ModelComponent interface

    These methods are %Model's implementation of virtual methods defined in
    the ModelComponent class from which %Model derives. **/
    /**@{**/
    void generateDecorations
       (bool                                        fixed,
        const ModelDisplayHints&                    hints,
        const SimTK::State&                         state,
        SimTK::Array_<SimTK::DecorativeGeometry>&   appendToThis) const
                                                                override;
    /**@}**/

    //--------------------------------------------------------------------------

private:
    // %Set the values of all data members to an appropriate "null" value.
    void setNull();

    // Construct the properties of a Model.
    void constructProperties();
    void setDefaultProperties();

    // Utility to build a connected graph (tree) of the multibody system
    void createMultibodyTree();

    void createMultibodySystem();

    void createAssemblySolver(const SimTK::State& s);

    // To provide access to private _modelComponents member.
    friend class Component;

//==============================================================================
// DATA MEMBERS
//==============================================================================
private:
    //--------------------------------------------------------------------------
    //                              MODELING
    //--------------------------------------------------------------------------
    // These data members are concerned with the structure and contents of the
    // model and must be given values prior to initiating computation.

    // Name of file from which the model was constructed, if any, otherwise
    // set to "Unassigned".
    std::string _fileName;

    // Private place to save some deserialization/error checking info in case
    // needed later.
    std::string _validationLog;


    //Units for length
    Units _lengthUnits;
    //Units for forces
    Units _forceUnits;

    // Other Model data structures that are derived from the properties
    // or added programmatically.

    // Set containing the analyses in this model.
    AnalysisSet     _analysisSet;

    // Set containing the generalized coordinates in this model.
    CoordinateSet   _coordinateSet;

    SimTK::MultibodyGraphMaker _multibodyTree;

    // This object just provides an alternate interface to the computational
    // SimTK::MultibodySystem. It is constructed just knowing the Model and
    // then forwards requests through the Model at runtime.
    SimbodyEngine _simbodyEngine;

    // This is the internal 'writable' state of the model.
    // _workingState will be set to the system default state when
    // initializeState() or initSystem() is called.
    SimTK::State _workingState;

    // A cached list of `Controller`s that were enabled in the model
    // when `Model::extendConnectToModel(Model&)` was called.
    //
    // This only exists to improve performance. At runtime,
    // `Model::computeControls(...)` may be called many times. Without
    // this cache, the implementation must repeatably call
    // `getComponentList<Controller>`, which is expensive because it
    // uses runtime `dynamic_cast`s to iterate over, and downcast, a
    // sequence of `Component`s. For controller-heavy models,
    // pre-caching controllers into this vector can improve perf by
    // >5%.
    std::vector<std::reference_wrapper<const Controller>> _enabledControllers{};

    //--------------------------------------------------------------------------
    //                              RUN TIME
    //--------------------------------------------------------------------------

    // If this flag is set when initSystem() is called, we'll allocate
    // a ModelVisualizer for display.
    bool _useVisualizer;

    // Global flag used to disable all Controllers.
    bool _allControllersEnabled;


    //                      SIMBODY MULTIBODY SYSTEM
    // We dynamically allocate these because they are not available at
    // construction.
    // The model owns the MultibodySystem, but the
    // subsystems and force elements are owned by the MultibodySystem. However,
    // that memory management happens through Simbody's handles. It's fine for
    // us to manage the heap memory for the handles.

    SimTK::ResetOnCopy<std::unique_ptr<SimTK::SimbodyMatterSubsystem>>
        _matter;
    SimTK::ResetOnCopy<std::unique_ptr<SimTK::Force::Gravity>>
        _gravityForce;
    SimTK::ResetOnCopy<std::unique_ptr<SimTK::GeneralForceSubsystem>>
        _forceSubsystem;
    SimTK::ResetOnCopy<std::unique_ptr<SimTK::GeneralContactSubsystem>>
        _contactSubsystem;
    SimTK::ResetOnCopy<std::unique_ptr<SimTK::CableSubsystem>>
        _cableSubsystem;

    // We place this after the subsystems so that during copy construction and
    // copy assignment, the subsystem handles are copied first. If the system
    // is copied first, the handles end up "dangling."
    SimTK::ResetOnCopy<std::unique_ptr<SimTK::MultibodySystem>> _system;

    // System-dependent objects.

    // Assembly solver used for satisfying constraints and other configuration
    // goals. This object is owned by the Model, and should not be copied
    // when the Model is copied.
    SimTK::ResetOnCopy<std::unique_ptr<AssemblySolver>> _assemblySolver;

    // Model controls as a shared pool (Vector) of individual Actuator controls
    SimTK::MeasureIndex   _modelControlsIndex;
    // Default values pooled from Actuators upon system creation.
    mutable SimTK::Vector _defaultControls;


    //                          VISUALIZATION
    // Anyone generating display geometry from this Model should consult this
    // object to pick up user preferences. Its contents may be modified by the
    // current user interface (GUI or ModelVisualizer) or programmatically.
    ModelDisplayHints   _displayHints;

    // If visualization has been requested at the API level, we'll allocate
    // a ModelVisualizer. The Model owns this object, but it should not be
    // copied.
    SimTK::ResetOnCopy<std::unique_ptr<ModelVisualizer>> _modelViz;

//==============================================================================
};  // END of class Model
//==============================================================================


//==============================================================================
} // end of namespace OpenSim

#endif // OPENSIM_MODEL_H_

