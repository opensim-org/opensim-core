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
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include <OpenSim/Common/Set.h>
#include <OpenSim/Common/ArrayPtrs.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyObjPtr.h>
#include <OpenSim/Common/PropertyDblVec.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/Units.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Model/ModelDisplayHints.h>
#include "Simbody.h"

namespace OpenSim {

class Analysis;
class Body;
class BodySet;
class JointSet;
class Constraint;
class ConstraintSet;
class CoordinateSet;
class Force;
class ForceSet;
class Probe;
class ProbeSet;
class MarkerSet;
class Muscle;
class ContactGeometry;
class Actuator;
class ContactGeometrySet;
class Storage;
class ScaleSet;
class AssemblySolver;
class Controller;
class ControllerSet;
class ModelDisplayHints;
class ModelVisualizer;
class ComponentSet;

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif


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
method, in which case it will allocate an maintain a ModelVisualizer.

@authors Frank Anderson, Peter Loan, Ayman Habib, Ajay Seth, Michael Sherman
@see ModelComponent, ModelVisualizer, SimTK::System
**/

class OSIMSIMULATION_API Model  : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(Model, ModelComponent);

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION AND DESTRUCTION
	//--------------------------------------------------------------------------
public:

	/** Default constructor creates a %Model containing only the ground Body
    and a set of default properties. */
	Model();

	/** Constructor from an OpenSim XML model file. 
	@param filename     Name of a file containing an OpenSim model in XML
                        format; suffix is typically ".osim". 
                        
    @param connectToModel  whether to call connectToModel to create a valid OpenSim Model or not on exit, 
                        defaults to true. If set to false only deserialization is performed.
    **/
	explicit Model(const std::string& filename, bool connectToModel=true) SWIG_DECLARE_EXCEPTION;

	/** Copy constructor copies model components but does not copy any run-time 
    objects.
	@param source   The %Model to be copied. **/
	Model(const Model& source);

    #ifndef SWIG
	/** Copy assignment copies model components but does not copy any run-time 
    objects.
	@param source   The %Model to be copied.
	@returns        Reference to this object. **/
	Model& operator=(const Model& source);
    #endif

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
    const ModelDisplayHints& getDisplayHints() const {return _displayHints;}
    /** Get writable access to the ModelDisplayHints object stored with this
    %Model. The GUI or ModelVisualizer can update these as a result of user
    requests, or an OpenSim API program can set them programmatically. **/
    ModelDisplayHints& updDisplayHints() {return _displayHints;}

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
    bool hasVisualizer() const {return _modelViz != 0;}

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
    connectToModel() method of each contained ModelComponent will be invoked,
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
    satisified. The initStateFromProperties() method of each contained
    ModelComponent will be invoked. A reference to the writable internally-
    maintained model State is returned (note that this does not affect the 
    system's default state (which is part of the model and hence read only).
    The model's state can be reset to the system's default state at any time
    by re-executing initializeState(). **/
    SimTK::State& initializeState();

    
	/** Convenience method that invokes buildSystem() and then 
    initializeState(). A reference to the writable internally-
    maintained model State is returned (note that this does not affect the 
    system's default state (which is part of the model and hence read only). **/
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

    /** Method to copy a model's default state into the space allocated for
    its working state. This is so that the default state does not get corrupted. **/
    void copyDefaultStateIntoWorkingState();

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
	 * create a storage (statesStorage) that has same label order as model's states
	 * with values populated from originalStorage, 0.0 for those states unspecified
	 * in the originalStorage.
	 */
	void formStateStorage(const Storage& originalStorage, Storage& statesStorage);
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
	const SimTK::MultibodySystem& getMultibodySystem() const {return *_system; } 
    /** (Advanced) Get writable access to the internal Simbody MultibodySystem 
    that was created by this %Model at the last initSystem() call. Be careful
    if you make modifications to the System because that will invalidate 
    initialization already performed by the Model. 
    @see initStateWithoutRecreatingSystem() **/    
	SimTK::MultibodySystem& updMultibodySystem() const {return *_system; } 

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

    /**@}**/

	virtual int getNumStateVariables() const;

	//--------------------------------------------------------------------------
	// CREATE THE MULTIBODY SYSTEM
	//--------------------------------------------------------------------------
	/**
	 * Add ModelComponents to the Model. Model takes ownership of the objects.
	 */
	void addComponent(ModelComponent* aComponent);
	void addBody(Body *aBody);
	void addConstraint(Constraint *aConstraint);
	void addForce(Force *aForce);
	void addProbe(Probe *aProbe);
	void addContactGeometry(ContactGeometry *aContactGeometry);
	/** remove passed in Probe from model **/
    void removeProbe(Probe *aProbe);
	//--------------------------------------------------------------------------
	// FILE NAME
	//--------------------------------------------------------------------------
	/** 
	 * Get the XML file name used to construct the model. 
	 *
	 * @return XML file name string.	
	 */
	const std::string& getInputFileName() const { return _fileName; }

	/** 
	 * Set the XML file name used to construct the model.
	 *
	 * @param fileName The XML file name.
	 */
	void setInputFileName(const std::string& fileName) { _fileName = fileName; }

	//--------------------------------------------------------------------------
	// CREDITS
	//--------------------------------------------------------------------------

	/** 
	 * Get the credits (e.g., model author names) associated with the model. 
	 *
 	 * @return Credits string.
	 */
	const std::string& getCredits() const { return _creditsStr; }

	/** 
	 * Set the credits (e.g., model author names) associated with the model.
	 *
	 * @param aCredits The string of credits.
	 */
	void setAuthors(const std::string& aCredits) { _creditsStr = aCredits; }

	/** 
	 * Get the publications associated with the model. 
	 *
	 * @return Publications string.
	 */
	const std::string& getPublications() const { return _publicationsStr; }
	
	/** 
	 * Set the publications associated with the model. 
	 *
	 * @param aPublications The string of publications.
	 */
	void setPublications(const std::string& aPublications) { _publicationsStr = aPublications; }

	//--------------------------------------------------------------------------
	// UNITS
	//--------------------------------------------------------------------------
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

	//--------------------------------------------------------------------------
	// GRAVITY
	//--------------------------------------------------------------------------
	/**
	 * Get the gravity vector in the global frame.
	 *
	 * @return The XYZ gravity vector in the global frame.
	 */
	SimTK::Vec3 getGravity() const;
	
	/**
	 * Set the gravity vector in the gloabl frame.
	 *
	 * @param aGrav The XYZ gravity vector
	 * @return Whether or not the gravity vector was successfully set.
	 */
	bool setGravity(const SimTK::Vec3& aGrav);

	//--------------------------------------------------------------------------
	// NUMBERS
	//--------------------------------------------------------------------------
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

    const ForceSet& getForceSet() const { return _forceSet; };
    ForceSet& updForceSet() { return _forceSet; };

	/**
     * Get the subset of Probes in the model
     * @return The set of Probes
     */
	const ProbeSet& getProbeSet() const { return _probeSet; };
    ProbeSet& updProbeSet() { return _probeSet; };

	/**
     * Get the subset of misc ModelComponents in the model
     * @return The set of misc ModelComponents
     */
	const ComponentSet& getMiscModelComponentSet() const 
    {   return _componentSet; };
	ComponentSet& updMiscModelComponentSet() 
    {   return _componentSet; };

	/**
	 * Get the number of analyses in the model.
	 * @return The number of analyses.
	 */
	int getNumAnalyses() const;

    //--------------------------------------------------------------------------
    // CONTROLS
    //--------------------------------------------------------------------------
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
	{	_defaultControls = controls; }
	const SimTK::Vector& getDefaultControls() const { return _defaultControls; };

	/**
	 * Update the controls for this the model at a given state.
	 * Only callable once underlying system for the model has been created.
	 * Throws an exception if called before Model::initSystem()
	 * This call invalidates the dynamics of the model and invalidates the
	 * value of the controls until they are marked as valid when the update
	 * is completed (@see markControlsAsValid)
	 * @param[in]   s		  System state at which to apply the controls
	 * @return		writable controls Vector
	 */
	SimTK::Vector& updControls(const SimTK::State& s) const;
	
	/**
	 * Mark controls as valid after an update at a given state.
	 * Indicates that controls are valid for use at the dynamics stage.
	 * If the stage is Velocity or lower the controls will remain invalid.
	 * @param[in]   s		  System state in which the controls are updated 
	 */
	void markControlsAsValid(const SimTK::State& s) const;

	/** 
	 * Alternatively, set the controls on the model at a given state.
	 * Note, this method will invalidate the dynamics of the model,
	 * but will mark the controls as valid. (E.g. controllers will not be invoked)
	 * @param[in]   s		  System state at which to apply the controls
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
    const ControllerSet& getControllerSet() const;
    ControllerSet& updControllerSet();
    bool getAllControllersEnabled() const;
    void setAllControllersEnabled( bool enabled );

    //--------------------------------------------------------------------------
    // CONFIGURATION
    //--------------------------------------------------------------------------
    void applyDefaultConfiguration(SimTK::State& s );

	//--------------------------------------------------------------------------
	// DYNAMICS ENGINE
	//--------------------------------------------------------------------------
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
	SimTK::Vector computeStateVariableDerivatives(const SimTK::State &s) const;
	double getTotalMass(const SimTK::State &s) const;
	SimTK::Inertia getInertiaAboutMassCenter(const SimTK::State &s) const;
	SimTK::Vec3 calcMassCenterPosition(const SimTK::State &s) const;
	SimTK::Vec3 calcMassCenterVelocity(const SimTK::State &s) const;
	SimTK::Vec3 calcMassCenterAcceleration(const SimTK::State &s) const;

	//--------------------------------------------------------------------------
	// STATES
	//--------------------------------------------------------------------------

	/**
	 * Get the values of state variables in the same ordering as returned by
     * getStateVariableNames(). Values are dug out from the passed-in \a state
     * based on Y-indices that are associated with the named state variables.
	 *
	 * @param[in]  state        State from which values are to be obtained.
	 * @param[out] stateValues  Array used to collect the reordered state values.
	 */
	void getStateValues(const SimTK::State& state, Array<double>& stateValues) const;
    SimTK::Vector getStateValues(const SimTK::State& s) const;

	/**
	 * Set the values of state variables which are passed in the same ordering
     * as returned by getStateVariableNames(). Values are set in the passed in 
     * \a state object based on Y-indices that are associated with the named 
     * state variables.
	 *
	 * @param[out] state        State to be modified.
	 * @param[in]  stateValues  Pointer to an array of values corresponding to
     *                          each of the named state variables.
	 */	
	void setStateValues(SimTK::State& state, const double* stateValues) const;

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

    BodySet& updBodySet() { return _bodySet; }
    const BodySet& getBodySet() const { return _bodySet; }

    JointSet& updJointSet(); 
    const JointSet& getJointSet() const;

    AnalysisSet& updAnalysisSet() {return _analysisSet; }
    const AnalysisSet& getAnalysisSet() const {return _analysisSet; }

    ContactGeometrySet& updContactGeometrySet() { return _contactGeometrySet; }
    const ContactGeometrySet& getContactGeometrySet() const { return _contactGeometrySet; }

   	Body& getGroundBody() const;


    //--------------------------------------------------------------------------
    // CONSTRAINTS
    //--------------------------------------------------------------------------
    ConstraintSet& updConstraintSet() { return _constraintSet; }
    const ConstraintSet& getConstraintSet() const { return _constraintSet; }

    //--------------------------------------------------------------------------
    // MARKERS
    //--------------------------------------------------------------------------
    MarkerSet& updMarkerSet() { return _markerSet; }
    const MarkerSet& getMarkerSet() const { return _markerSet; }
    int replaceMarkerSet(const SimTK::State& s, MarkerSet& aMarkerSet);
    void writeMarkerFile(const std::string& aFileName) const;
    void updateMarkerSet(MarkerSet& aMarkerSet);
    int deleteUnusedMarkers(const Array<std::string>& aMarkerNames);
 
	/**
	 * Add an Analysis to the %Model.
	 *
	 * @param analysis  pointer to the Analysis to add
	 */
	void addAnalysis(Analysis *analysis);
    /** Add a Controller to the %Model. **/
	void addController(Controller *aController);

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
     * @param state     State containing parameter values that might be 
     *                  modified here.
	 * @param scaleSet  The set of XYZ scale factors for the bodies.
	 * @param finalMass The mass that the scaled model should have.
	 * @param preserveMassDist 
     *                  Whether or not the masses of the individual bodies 
     *                  should be scaled with the body scale factors.
	 * @returns         Whether or not scaling was successful.
	 */
	bool scale(SimTK::State&    state, 
               const ScaleSet&  scaleSet, 
               double           finalMass = -1.0, 
               bool             preserveMassDist = false);

	//--------------------------------------------------------------------------
	// PRINT
	//--------------------------------------------------------------------------

	/**
	 * Print some basic information about the model.
	 *
	 * @param aOStream Output stream.
	 */
	void printBasicInfo(std::ostream &aOStream) const;

	/**
	 * Print detailed information about the model.
	 *
     * @param s   the system State.
	 * @param aOStream Output stream.
	 */
	void printDetailedInfo(const SimTK::State& s, std::ostream &aOStream) const;

	/**
	 * Model relinquishes ownership of all components such as: Bodies, Constraints, Forces, 
	 * ConactGeometry and so on. That means the freeing of the memory of these objects is up
	 * to the caller.
	 */
	void disownAllComponents();
    /**
     * Convenice function to turn on/off overriding the force for all actuators 
     */
    void overrideAllActuators( SimTK::State& s, bool flag);

	/**
	 * Get a log of errors/warnings ecountered when loading/constructing the model
	 */
	const std::string& getValidationLog() { return _validationLog; };
	void clearValidationLog() { _validationLog = ""; };

	/**
	 * Utility to get a reference to an Object based on its name and type
	 * throws an exception if the object was not found.
	 * names are case sensitive
	 * @param typeString the type of object being looked up (Body, Force, Constraint, Coordinate, Joint, Marker, Controller)
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
	~Model();

	/** Override of the default implementation to account for versioning. */
	/*virtual*/ void updateFromXMLNode(SimTK::Xml::Element& aNode, 
                                       int versionNumber=-1);
    /**@}**/

    //--------------------------------------------------------------------------
    /**@name         Implementation of ModelComponent interface

    These methods are %Model's implementation of virtual methods defined in
    the ModelComponent class from which %Model derives. The implementations
    here serve as dispatchers that treat all contained ModelComponents (and
    model elements that are not ModelComponents) as subcomponents whose 
    corresponding methods need to be called. **/
    /**@{**/
	void connectToModel(Model& model)  OVERRIDE_11 {setup();};
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11; 
    void initStateFromProperties(SimTK::State& state) const OVERRIDE_11;

	/**
     * Given a State, set all default values for this Model to match those 
     * found in the State.
     */
    void setPropertiesFromState(const SimTK::State& state) OVERRIDE_11;

    void generateDecorations
       (bool                                        fixed, 
        const ModelDisplayHints&                    hints,
        const SimTK::State&                         state,
        SimTK::Array_<SimTK::DecorativeGeometry>&   appendToThis) const 
                                                                OVERRIDE_11;

    //TODO: Why are the rest of these part of ModelComponent interface?

	/** Get the names of the states. These are the continuous states introduced
    by OpenSim ModelComponents and exposed thru the ModelComponent API. **/
	/*virtual*/ Array<std::string> getStateVariableNames() const;

    const Array<SimTK::SystemYIndex>& getStateVariableSystemIndices() const
    { return _stateVariableSystemIndices;}

    /** Get the value of a state variable of this Model.
    @param s   the State for which to get the value
    @param name    the name (string) of the state variable of interest **/
	/*virtual*/ double getStateVariable(const SimTK::State& s, 
                                        const std::string& name) const;

    /** Set the value of a state variable of this Model.
    @param s   the State for which to get the value
    @param name    the name (string) of the state variable of interest 
    @param value   the value of the state **/
	/*virtual*/ void setStateVariable(SimTK::State& s, 
                                        const std::string& name,
                                        double value) const;
	
    /** Get the Y index of a state variable by name.
    @param stateVariableName    the name (string) of the state variable of interest **/
	/*virtual*/ SimTK::SystemYIndex getStateVariableSystemIndex
       (const std::string &stateVariableName) const;
    /**@}**/
    //--------------------------------------------------------------------------


private:
	// Set the values of all data members to an appropriate "null" value.
	void setNull();

	void createGroundBodyIfNecessary();
    void setDefaultProperties();
	void createMultibodySystem();

	// Copy only the model-defining data members from source.
	void copyData(const Model& source);

	// Connect properties to local pointers.
	void setupProperties();

	// Internal method to check that specified mass properties for the bodies 
    // are physically possible that is, satisfy the triangular inequality 
    // and other requirements specified in the Doxygen description of 
    // SimTK::MassProperties. If not true, then the values are forced to 
    // satisfy the inequality and a warning is issued.
	void validateMassProperties(bool fixMassProperties=true);

	void createAssemblySolver(const SimTK::State& s);

    // To provide access to private _modelComponents member.
	friend class ModelComponent; 

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

    // PROPERTIES
    // These aspects of a Model can be serialized to/from an OpenSim model
    // file (.osim file).

	// Model credits info.
	PropertyStr _creditsStrProp;
	std::string& _creditsStr;

	// Publications and References.
	PropertyStr _publicationsStrProp;
	std::string& _publicationsStr;

	// Units for all length.
	PropertyStr _lengthUnitsStrProp;
	std::string& _lengthUnitsStr;
	Units _lengthUnits;

	// Units for all forces.
	PropertyStr _forceUnitsStrProp;
	std::string& _forceUnitsStr;
	Units _forceUnits;

    // Array containg the acceleration due to gravity.
    PropertyDblVec3 _gravityProp;
    SimTK::Vec3& _gravity;

	// Forces.
	PropertyObj _forceSetProp;
	ForceSet& _forceSet;

	// Probes.
	PropertyObj _probeSetProp;
	ProbeSet& _probeSet;

    // Set containing the bodies in this model.
    PropertyObj _bodySetProp;
    BodySet& _bodySet;

    // Set containing the constraints in this model.
    PropertyObj _constraintSetProp;
    ConstraintSet& _constraintSet;

    // Set of markers for this model.
    PropertyObj _markerSetProp;
    MarkerSet& _markerSet;

    // Set of ContactGeometry objects for this model.
    PropertyObj _contactGeometrySetProp;
    ContactGeometrySet& _contactGeometrySet;

	// Set containing the Model controllers
    PropertyObj _controllerSetProp;
    ControllerSet& _controllerSet;

	// Set containing the user defined components in this model.
    PropertyObj _componentSetProp;
    ComponentSet& _componentSet;

    // Other Model data structures that are derived from the properties
    // or added programmatically.

	// Set containing the analyses in this model.
	AnalysisSet     _analysisSet;

	// Set containing the joints in this model.
	JointSet        _jointSet;

	// Set containing the generalized coordinates in this model.
	CoordinateSet   _coordinateSet;

   	// Body used for ground, the inertial frame. This is just a reference
    // to an existing Body and should not be destructed.
	Body*           _groundBody;

	// A flat list of ModelComponents contained by the model.
	ArrayPtrs<const ModelComponent> _modelComponents;

    // This object just provides an alternate interface to the computational
    // SimTK::MultibodySystem. It is constructed just knowing the Model and
    // then forwards requests through the Model at runtime.
    SimbodyEngine _simbodyEngine;

    // This is the internal 'writable' state of the model.
    // _workingState will be set to the system default state when
    // initializeState() or initSystem() is called.
    SimTK::State _workingState;


    //--------------------------------------------------------------------------
    //                              RUN TIME 
    //--------------------------------------------------------------------------

    // If this flag is set when initSystem() is called, we'll allocate
    // a ModelVisualizer for display.
    bool _useVisualizer;

    // Global flag used to disable all Controllers.
	bool _allControllersEnabled;


    //                      SIMBODY MULTIBODY SYSTEM
	// The model owns the MultibodySystem, but the
    // subsystems and force elements are owned by the MultibodySystem so 
    // should not be deleted in the destructor.
	SimTK::MultibodySystem*         _system; // owned by Model; must destruct

    // These are just references pointing into _system; don't destruct.
	SimTK::SimbodyMatterSubsystem*  _matter;     
	SimTK::Force::Gravity*          _gravityForce;
	SimTK::GeneralForceSubsystem*   _forceSubsystem;
	SimTK::GeneralContactSubsystem* _contactSubsystem;

    // System-dependent objects.

	// Assembly solver used for satisfying constraints and other configuration
    // goals. This object is owned by the Model and must be destructed.
	AssemblySolver*     _assemblySolver;

	// Model controls as a shared pool (Vector) of individual Actuator controls
	SimTK::MeasureIndex _modelControlsIndex;
	// Default values pooled from Actuators upon system creation.
    // CAUTION: although this appears to be a reference it is a heap allocated
    // object owned by the Model and must be destructed!
	SimTK::Vector&      _defaultControls;

	// Members for fast access of state variables in the underlying 
    // SimTK::System.
	OpenSim::Array<std::string>	        _stateVariableNames;
	OpenSim::Array<SimTK::SystemYIndex>	_stateVariableSystemIndices;

    //                          VISUALIZATION

    // Anyone generating display geometry from this Model should consult this
    // object to pick up user preferences. Its contents may be modified by the
    // current user interface (GUI or ModelVisualizer) or programmatically.
    ModelDisplayHints   _displayHints;

    // If visualization has been requested at the API level, we'll allocate 
    // a ModelVisualizer. The Model owns this object.
    ModelVisualizer*    _modelViz; // owned by Model; must destruct

//==============================================================================
};	// END of class Model
//==============================================================================


//==============================================================================
} // end of namespace OpenSim

#endif // OPENSIM_MODEL_H_

