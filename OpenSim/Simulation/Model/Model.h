#ifndef __Model_h__
#define __Model_h__

// Model.h
// Authors: Frank C. Anderson, Peter Loan, Ayman Habib
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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

// INCLUDES
#include <string>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Set.h>
#include <OpenSim/Common/ArrayPtrs.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyObjPtr.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/Units.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Model/ControllerSet.h>
#include "SimTKsimbody.h"



namespace OpenSim {

class Actuator;
class Analysis;
class Body;
class BodySet;
class Constraint;
class ConstraintSet;
class Controller;
class CoordinateSet;
class Force;
class ForceSet;
class MarkerSet;
class ContactGeometry;
class ActuatorPerturbation;
class ContactGeometrySet;
class OpenSimForceSubsystem;
class Storage;
class ScaleSet;
class SimbodyEngine;

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

//=============================================================================
//=============================================================================
/**
 * A concrete class that specifies the interface to a musculoskeletal model.
 * It provides methods create a new model and to add model components and to
 * define their default behavior for simulation.
 *
 * @authors Frank C. Anderson, Peter Loan, Ajay Seth
 * @version 1.0
 */

class OSIMSIMULATION_API Model  : public ModelComponent
{

//=============================================================================
// DATA
//=============================================================================
private:

   /* Simbody  multibody system */    
   SimTK::MultibodySystem* _system;
   SimTK::SimbodyMatterSubsystem* _matter;
   SimTK::Force::Gravity* _gravityForce;
   SimTK::GeneralForceSubsystem* _userForceElements;
   SimTK::GeneralContactSubsystem* _contactSubsystem;
   OpenSimForceSubsystem* _forceSubsystem;

	/** Name of file from which the model was constructed. */
	std::string _fileName;

	/** Model credits Info. */
	PropertyStr _creditsStrProp;
	std::string& _creditsStr;

	/** Publications and References. */
	PropertyStr _publicationsStrProp;
	std::string& _publicationsStr;

	/** Units for all length. */
	PropertyStr _lengthUnitsStrProp;
	std::string& _lengthUnitsStr;
	Units _lengthUnits;

	/** Units for all forces. */
	PropertyStr _forceUnitsStrProp;
	std::string& _forceUnitsStr;
	Units _forceUnits;

    /** Array containg the acceleration due to gravity. */
    PropertyDblVec3 _gravityProp;
    SimTK::Vec3 &_gravity;

	// SETS OF THINGS
	/** Actuators. */
	PropertyObj _forceSetProp;
	ForceSet& _forceSet;

	/** Analyses. */
	AnalysisSet _analysisSet;

    /** Set containing the bodies in this model. */
    PropertyObj _bodySetProp;
    BodySet &_bodySet;

    /** Set containing the constraints in this model. */
    PropertyObj _constraintSetProp;
    ConstraintSet &_constraintSet;

    /** Set of markers for this model. */
    PropertyObj _markerSetProp;
    MarkerSet &_markerSet;

    /** Set of ContactGeometry objects for this model. */
    PropertyObj _contactGeometrySetProp;
    ContactGeometrySet &_contactGeometrySet;

	// WORK VARIABLES

	/** Time normalization. */
	double _tNormConst;

	/** Were all model components properly defined? */
	bool _builtOK;

    /** global flag  used to disable all Controllers */
   bool _allControllersEnabled;

    /** flag indicating the model has actuators that are being perturbed  */
    bool _perturbActuatorForces; 

    /** object which computes the actuator perturbation */
    ActuatorPerturbation* _perturb;

    /** dynamics engine */
    SimbodyEngine _simbodyEngine;

	/** Set containing the joints in this model. */
	// 2008_06_06: No longer a property because the joints are kept
	// local to the bodies in the Simbody Dynamics Engine.
	// For the SIMM and SDFast engines to continue to work, the properties
	// must be moved local to those classes.
	// The strategy now is to get OpenSim working with the Simbody engine
	// as quickly as possible.
	JointSet _jointSet;

	/** Set containing the generalized coordinates in this model. */
	// 2008_06_06: No longer a property because the coordinates are kept
	// local to the joints in the Simbody Dynamics Engine.
	// For the SIMM and SDFast engines to continue to work, the properties
	// must be moved local to those classes.
	// The strategy now is to get OpenSim working with the Simbody engine
	// as quickly as possible.
	CoordinateSet _coordinateSet;

   	/** Body used for ground, the inertial frame. */
	Body *_groundBody;

   /** default controller */
   ControllerSet _controllerSet;


//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION AND DESTRUCTION
	//--------------------------------------------------------------------------
public:

	/** Default constructor. */
	Model();

	/** 
	 * Constructor from an XML file. 
	 *
	 * @param aFileName XML file name.
	 */
	Model(const std::string &aFileName) SWIG_DECLARE_EXCEPTION;

	/**
	 * Copy constructor.
	 *
	 * @param aModel Model to be copied.
	 */
	Model(const Model& aModel);

	/** Destructor. */
	virtual ~Model();

	/** Override of the default implementation to account for versioning. */
	virtual void updateFromXMLNode();

	/**
	 * Copy this Model and return a pointer to the copy.
	 * The copy constructor for this class is used.
	 *
	 * @return Pointer to a copy of this Model.
	 */
	virtual Object* copy() const;

	/**
	 * Copy the member variables of the model.
	 *
	 * @param aModel model to be copied
	 */
	void copyData(const Model &aModel);

	/** Connect properties to local pointers. */
	void setupProperties();


	/**
	 * Dynamic casting across JNI is messy. This method does the upCasting on C++ side
	 */
	Model* clone()
	{
		return static_cast<Model*>(this->copy()); 
	}

	/**
	 * Perform some set up functions that happen after the
	 * object has been deserialized. TODO: this method is
	 * not yet designed to be called after a model has been
	 * copied.
	 */
	virtual void setup() SWIG_DECLARE_EXCEPTION;

	/**
	 * Perform some clean up functions that are normally done 
	 * from the destructor however this gives the GUI a way to 
	 * proactively do the cleaning without waiting for garbage
	 * collection to do the actual cleanup.
	 */
	void cleanup();

    
	/**
	 * This must be called after the Model is fully created but before starting a simulation.
     * It creates and initializes the computational system used to simulate the model.
	 */
    SimTK::State& initSystem() SWIG_DECLARE_EXCEPTION;

	/**
     * Given a State, set all default values for this Model to match those found in the State.
     */
    virtual void setDefaultsFromState(const SimTK::State& state);

	/**
	 * This is called after the Model is fully created but before starting a simulation.
     * It ONLY initializes the computational system used to simulate the model and 
	 * createSystem() has been called already. This method should only be used if 
	 * if additional SimTK::System components are being added using the SimTK API 
	 * and the programmer is certain that the model's system has been created.
	 */
	void initStateWithoutRecreatingSystem(SimTK::State& state) const { initState(state); };

    /**
     * Mark the computational system as invalid.  This should be called whenever a property
     * of the model is modified.  Once this has been called, no calculations can be done until
     * initSystem() is called again.
     */
    void invalidateSystem();

 	/**
	 * create a storage (statesStorage) that has same label order as model's states
	 * with values populated from originalStorage, 0.0 for those states unspecified
	 * in the originalStorage.
	 */
	void formStateStorage(const Storage& originalStorage, Storage& statesStorage);
    void formQStorage(const Storage& originalStorage, Storage& qStorage);


   /**
     * Update the state of all Muscles so they are in equilibrium.
     */
    void equilibrateMuscles(SimTK::State& state);

    /**
	 * Indicates whether or not problems were encountered during
	 * model setup().
	 *
	 * @return True if the number of bodies is > 0.
	 */
	bool builtOK() { return _builtOK; }
    const OpenSimForceSubsystem& getForceSubsystem() const {return *_forceSubsystem; }
    const SimTK::SimbodyMatterSubsystem& getMatterSubsystem() const {return _system->getMatterSubsystem(); }
    SimTK::SimbodyMatterSubsystem& updMatterSubsystem() {return _system->updMatterSubsystem(); }
    const SimTK::Force::Gravity& getGravityForce() const {return *_gravityForce; }
	SimTK::Force::Gravity& updGravityForce() {return *_gravityForce; }
    const SimTK::GeneralForceSubsystem& getUserForceSubsystem() const {return *_userForceElements; }
    SimTK::GeneralForceSubsystem& updUserForceSubsystem() {return *_userForceElements; }


protected:
#ifndef SWIG
	/**
	 * Assignment operator.
	 *
	 * @return Reference to this object.
	 */
	Model& operator=(const Model &Model);
#endif

    void setDefaultProperties();
	virtual void setup(Model& aModel) {setup();};

	virtual void createSystem(SimTK::MultibodySystem& system) const {}; 
	virtual void createSystem();

    virtual void initState(SimTK::State& state) const;
	void createGroundBodyIfNecessary();

private:

	/** Set the values of all data members to an appropriate "null" value. */
	void setNull();
    friend class ForceSet;

public:
	//--------------------------------------------------------------------------
	// CREATE THE MULTIBODY SYSTEM
	//--------------------------------------------------------------------------

	/**
	 * Add ModelComponents to the Model. Model takes ownership of the objects.
	 */
	virtual void addBody(Body *aBody);
	virtual void addConstraint(Constraint *aConstraint);
	virtual void addForce(Force *aForce);
	virtual void addContactGeometry(ContactGeometry *aContactGeometry);

	//--------------------------------------------------------------------------
	// FILE NAME
	//--------------------------------------------------------------------------

	/** 
	 * Get the XML file name used to construct the model. 
	 *
	 * @return XML file name string.	
	 */
	virtual const std::string& getInputFileName() const { return _fileName; }

	/** 
	 * Set the XML file name used to construct the model.
	 *
	 * @param fileName The XML file name.
	 */
	virtual void setInputFileName(const std::string& fileName) { _fileName = fileName; }

	//--------------------------------------------------------------------------
	// CREDITS
	//--------------------------------------------------------------------------

	/** 
	 * Get the credits (e.g., model author names) associated with the model. 
	 *
 	 * @return Credits string.
	 */
	virtual const std::string& getCredits() const { return _creditsStr; }

	/** 
	 * Set the credits (e.g., model author names) associated with the model.
	 *
	 * @param aCredits The string of credits.
	 */
	virtual void setAuthors(const std::string& aCredits) { _creditsStr = aCredits; }

	/** 
	 * Get the publications associated with the model. 
	 *
	 * @return Publications string.
	 */
	virtual const std::string& getPublications() const { return _publicationsStr; }
	
	/** 
	 * Set the publications associated with the model. 
	 *
	 * @param aPublications The string of publications.
	 */
	virtual void setPublications(const std::string& aPublications) { _publicationsStr = aPublications; }

	//--------------------------------------------------------------------------
	// UNITS
	//--------------------------------------------------------------------------
	
	/** 
	 * Get the length units associated with the model. 
	 *
 	 * @return Length units.
	 */
	virtual const Units& getLengthUnits() const { return _lengthUnits; }
	
	/** 
	 * Get the force units associated with the model. 
	 *
	 * @return Force units
	 */
	virtual const Units& getForceUnits() const { return _forceUnits; }

	//--------------------------------------------------------------------------
	// MultibodySystem
	//--------------------------------------------------------------------------
	virtual SimTK::MultibodySystem& getMultibodySystem() {return *_system; } 
	virtual void  setMultibodySystem(SimTK::MultibodySystem& system) { _system = &system; } 
	//--------------------------------------------------------------------------
	//--------------------------------------------------------------------------
	// PERTURBATION
	//--------------------------------------------------------------------------
	virtual ActuatorPerturbation& getPerturbation() {return *_perturb; } 
	virtual void setPerturbation(ActuatorPerturbation* perturbationMethod) {_perturb = perturbationMethod; } 
    virtual bool getPerturbForcesEnabled() const { return ( _perturbActuatorForces ); }
    virtual void setPerturbForcesEnabled( bool enabled ) {  _perturbActuatorForces = enabled; }
	//--------------------------------------------------------------------------
	// GRAVITY
	//--------------------------------------------------------------------------
	
	/**
	 * Get the gravity vector in the gloabl frame.
	 *
	 * @param rGrav The XYZ gravity vector in the global frame is returned here.
	 */
	virtual SimTK::Vec3 getGravity() const;
	
	/**
	 * Set the gravity vector in the gloabl frame.
	 *
	 * @param aGrav The XYZ gravity vector
	 * @return Whether or not the gravity vector was successfully set.
	 */
	virtual bool setGravity(const SimTK::Vec3& aGrav);

	//--------------------------------------------------------------------------
	// ADD JOINT
	//--------------------------------------------------------------------------
	
	/**
	 * Add a joint and associated child body.
	 *
	 * @param aJoint The joint.
	 */
	//virtual void addJoint(OpenSim::Joint& aJoint);

	//--------------------------------------------------------------------------
	// NUMBERS
	//--------------------------------------------------------------------------

	/**
	 * Get the number of states in the model.
	 *
	 * @return Number of states.
	 */
	virtual int getNumStates() const;

	/**
	 * Get the number of controls in the model.
	 *
	 * @return Number of markers.
	 */
    virtual int getNumMarkers() const;

	/**
	 * Get the number of ContactGeometries in the model.
	 *
	 * @return Number of ContactGeometries.
	 */
    virtual int getNumContactGeometries() const;


	/**
	 * Get the total number of bodies in the model.
	 *
	 * @return Number of bodies.
	 */
	virtual int getNumBodies() const;

	/**
	 * Get the total number of joints in the model.
	 *
	 * @return Number of joints.
	 */
	virtual int getNumJoints() const;

	/**
	 * Get the total number of coordinates in the model.
	 *
	 * @return Number of coordinates.
	 */
	virtual int getNumCoordinates() const;

	/**
	 * Get the total number of speeds in the model.
	 *
	 * @return Number of speeds.
	 */
	virtual int getNumSpeeds() const;

    /**
     * Get the subset of Forces in the model which are actuators
     *
     * @return The set of Actuators
     */
    const Set<Actuator>& getActuators() const;
    Set<Actuator>& updActuators();

    const ForceSet& getForceSet() const { return _forceSet; };
    ForceSet& updForceSet() { return _forceSet; };
	/**
	 * Get the number of analyses in the model.
	 *
	 * @return The number of analyses.
	 */
	virtual int getNumAnalyses() const;

	/**
	 * Get the number of configurations in the model.
	 *
	 * @return The number of coordinates plus speeds.
	 */
	int getNumConfigurations() const { return getNumCoordinates() + getNumSpeeds(); }

    //--------------------------------------------------------------------------
    // CONTROLS
    //--------------------------------------------------------------------------
	/**
	 * Get a flag indicating if the model needs controls to operate its actuators
	 */
	bool isControlled() const
	{
		bool isControlled = false;
		for(int i=0; i< getActuators().getSize() && !isControlled; i++){
			isControlled = getActuators().get(i).isControlled();
		}
		return isControlled;
	}
    virtual void storeControls( const SimTK::State& s, int step );
    virtual void printControlStorage(const std::string& fileName ) const;
    virtual const ControllerSet& getControllerSet() const;
    virtual ControllerSet& updControllerSet();
    virtual bool getAllControllersEnabled() const;
    virtual void setAllControllersEnabled( bool enabled );

    //--------------------------------------------------------------------------
    // CONFIGURATION
    //--------------------------------------------------------------------------
    virtual void applyDefaultConfiguration(SimTK::State& s );
	 virtual void enforceCoordinateCouplerConstraints(SimTK::State& s) const;


	//--------------------------------------------------------------------------
	// DYNAMICS ENGINE
	//--------------------------------------------------------------------------
		

	/**
	 * Get the model's dynamics engine
	 *
	 * @return Reference to the abstract dynamics engine
	 */
	const SimbodyEngine& getSimbodyEngine() const { return _simbodyEngine; }
	SimbodyEngine& updSimbodyEngine() { return _simbodyEngine; }

	//-------------------------------------------------------------------------
	// TIME NORMALIZATION
	//--------------------------------------------------------------------------
	
	/**
	 * Set the constant by which time is normalized.
	 * The normalization constant must be greater than or equal to the constant Zero
	 *
	 * @param Time normalization constant.
	 */	
	virtual void setTimeNormConstant(double aNormConst);

	/**
	 * Get the constant by which time is normalized.
	 * By default, the time normalization constant is 1.0.
	 *
	 * @return Current time normalization constant.
	 */
	virtual double getTimeNormConstant() const;

	//--------------------------------------------------------------------------
	// STATES
	//--------------------------------------------------------------------------

	/**
	 * Get the names of the states.
	 *
	 * @param rStateNames Array of state names..
	 */
	virtual void getStateNames(Array<std::string> &rStateNames) const;

    int getNumMuscleStates() const;

	//--------------------------------------------------------------------------
	// INITIAL TIME
	//--------------------------------------------------------------------------
	virtual void setInitialTime(  double ti);
   
	// SYSTEM
	//--------------------------------------------------------------------------
	void setSystem( SimTK::MultibodySystem* mbs ) ;
	SimTK::MultibodySystem& getSystem() const;
	//--------------------------------------------------------------------------
   //--------------------------------------------------------------------------
   // SETS
   //--------------------------------------------------------------------------
   //
   //--------------------------------------------------------------------------
   // COORDINATES
   //--------------------------------------------------------------------------
   virtual CoordinateSet& updCoordinateSet() { 
	   return _coordinateSet; 
   }
   virtual const CoordinateSet& getCoordinateSet() const { 
	   return _coordinateSet; 
   }

   virtual BodySet& updBodySet() { return _bodySet; }
   virtual const BodySet& getBodySet() const { return _bodySet; }

   virtual JointSet& updJointSet(); 
   virtual const JointSet& getJointSet();

   virtual AnalysisSet& updAnalysisSet() {return _analysisSet; }
   virtual const AnalysisSet& getAnalysisSet() const {return _analysisSet; }

   virtual ContactGeometrySet& updContactGeometrySet() { return _contactGeometrySet; }
   virtual const ContactGeometrySet& getContactGeometrySet() const { return _contactGeometrySet; }

   	virtual Body& getGroundBody() const;


    //--------------------------------------------------------------------------
    // CONSTRAINTS
    //--------------------------------------------------------------------------
    virtual ConstraintSet& updConstraintSet() { return _constraintSet; }
    virtual const ConstraintSet& getConstraintSet() const { return _constraintSet; }

    //--------------------------------------------------------------------------
    // MARKERS
    //--------------------------------------------------------------------------
    virtual MarkerSet& updMarkerSet() { return _markerSet; }
    virtual const MarkerSet& getMarkerSet() const { return _markerSet; }
    virtual int replaceMarkerSet(const SimTK::State& s, MarkerSet& aMarkerSet);
    virtual void writeMarkerFile(const std::string& aFileName) const;
    virtual void updateMarkerSet(MarkerSet& aMarkerSet);
    virtual int deleteUnusedMarkers(const Array<std::string>& aMarkerNames);


 
	/**
	 * Add an analysis to the model.
	 *
	 * @param aAnalysis pointer to the analysis to add
	 */
	virtual void addAnalysis(Analysis *aAnalysis);
	virtual void addController(Controller *aController);

	/**
	 * Remove an analysis from the model
	 *
	 * @param aAnalysis Pointer to the analysis to remove.
	 */
	virtual void removeAnalysis(Analysis *aAnalysis, bool deleteIt=true);
	virtual void removeController(Controller *aController);

	//--------------------------------------------------------------------------
	// DERIVATIVES
	//--------------------------------------------------------------------------

	
	/**
	 * Compute values for the auxiliary states (i.e., states other than the
	 * generalized coordinates and speeds) that are in quasi-static equilibrium.
	 * The auxiliary states usually belong to the actuators (e.g., muscle
	 * activation and muscle fiber length).  The equilibrium computations
	 * are passed on to the owner of the the states.
	 *
	 * This methods is useful for computing initial conditions for a simulation
	 * or for computing torque-angle curves, for example.
	 *
	 * @param rY Array of states. The values sent in are used as the initial
	 * guess for equilibrium. The values returned are those that satisfy
	 * equilibrium.
	 */
	virtual void computeEquilibriumForAuxiliaryStates(SimTK::State& s);

	//--------------------------------------------------------------------------
	// OPERATIONS
	//--------------------------------------------------------------------------

	/**
	 * Scale the model
	 *
	 * @param aScaleSet the set of XYZ scale factors for the bodies
	 * @param aFinalMass the mass that the scaled model should have
	 * @param aPreserveMassDist whether or not the masses of the
	 *        individual bodies should be scaled with the body scale factors.
	 * @return Whether or not scaling was successful.
	 */
	virtual bool scale(SimTK::State& s, const ScaleSet& aScaleSet, double aFinalMass = -1.0, bool aPreserveMassDist = false);

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
	 * @param aOStream Output stream.
	 */
	void printDetailedInfo(const SimTK::State& s, std::ostream &aOStream) const;

	/**
	 * Model relinquishes ownership of all components such as: Bodies, Constraints, Forces, 
	 * ConactGeometry and so on. That means the freeing of the memory of these objects is up
	 * to the caller.
	 */
	void disownAllComponents();


//=============================================================================
};	// END of class Model
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Model_h__

