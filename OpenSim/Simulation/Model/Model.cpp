// Model.cpp
// Authors: Frank C. Anderson, Peter Loan, Ayman Habib, Ajay Seth
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <string>
#include <math.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Common/XMLNode.h>
#include "Model.h"
#include "Muscle.h"
#include "CoordinateSet.h"
#include "BodySet.h"
#include "AnalysisSet.h"
#include "ForceSet.h"
#include <OpenSim/Common/ScaleSet.h>
#include "Analysis.h"
#include "ForceAdapter.h"
#include "Actuator.h"
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Simulation/Model/ContactGeometrySet.h>
#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/SimbodyEngine/CoordinateCouplerConstraint.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Control/ControlLinearNode.h>
#include <OpenSim/Simulation/Wrap/WrapCylinder.h>
#include <OpenSim/Simulation/Wrap/WrapEllipsoid.h>
#include <OpenSim/Simulation/Wrap/WrapSphere.h>
#include "SimTKcommon/internal/SystemGuts.h"

#include <OpenSim/Common/Constant.h>
#include <OpenSim/Simulation/AssemblySolver.h>
#include <OpenSim/Simulation/CoordinateReference.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using SimTK::Mat33;

//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Model::Model() :
	_fileName("Unassigned"),
	_creditsStr(_creditsStrProp.getValueStr()),
	_publicationsStr(_publicationsStrProp.getValueStr()),
	_lengthUnitsStr(_lengthUnitsStrProp.getValueStr()),
	_forceUnitsStr(_forceUnitsStrProp.getValueStr()),
	_forceSetProp(PropertyObj("", ForceSet())),
	_forceSet((ForceSet&)_forceSetProp.getValueObj()),
    _gravity(_gravityProp.getValueDblVec()),
    _bodySetProp(PropertyObj("", BodySet())),
    _bodySet((BodySet&)_bodySetProp.getValueObj()),
    _constraintSetProp(PropertyObj("", ConstraintSet())),
    _constraintSet((ConstraintSet&)_constraintSetProp.getValueObj()),
    _markerSetProp(PropertyObj("", MarkerSet())),
    _markerSet((MarkerSet&)_markerSetProp.getValueObj()),
    _contactGeometrySetProp(PropertyObj("", ContactGeometrySet())),
    _contactGeometrySet((ContactGeometrySet&)_contactGeometrySetProp.getValueObj()),
    _jointSet(JointSet()),
    _analysisSet(AnalysisSet()),
    _coordinateSet(CoordinateSet()),
    _controllerSet(ControllerSet()),
    _allControllersEnabled(true),
    _perturbActuatorForces(false),
    _system(NULL),
	_defaultControls(*new Vector(0))
{
	setNull();
	setupProperties();
    _analysisSet.setMemoryOwner(false);
	createGroundBodyIfNecessary();
}
//_____________________________________________________________________________
/**
 * Constructor from an XML file
 */
Model::Model(const string &aFileName) :
	ModelComponent(aFileName, false),
	_fileName("Unassigned"),
	_creditsStr(_creditsStrProp.getValueStr()),
	_publicationsStr(_publicationsStrProp.getValueStr()),
	_lengthUnitsStr(_lengthUnitsStrProp.getValueStr()),
	_forceUnitsStr(_forceUnitsStrProp.getValueStr()),
	_forceSetProp(PropertyObj("", ForceSet())),
	_forceSet((ForceSet&)_forceSetProp.getValueObj()),
    _gravity(_gravityProp.getValueDblVec()),
    _bodySetProp(PropertyObj("", BodySet())),
    _bodySet((BodySet&)_bodySetProp.getValueObj()),
    _constraintSetProp(PropertyObj("", ConstraintSet())),
    _constraintSet((ConstraintSet&)_constraintSetProp.getValueObj()),
    _markerSetProp(PropertyObj("", MarkerSet())),
    _markerSet((MarkerSet&)_markerSetProp.getValueObj()),
    _contactGeometrySetProp(PropertyObj("", ContactGeometrySet())),
    _contactGeometrySet((ContactGeometrySet&)_contactGeometrySetProp.getValueObj()),
    _jointSet(JointSet()),
    _analysisSet(AnalysisSet()),
    _coordinateSet(CoordinateSet()),
    _controllerSet(ControllerSet()),
    _allControllersEnabled(true),
    _perturbActuatorForces(false),
    _system(NULL),
	_defaultControls(*new Vector(0))
{
	setNull();
	setupProperties();
	updateFromXMLNode();
	_fileName = aFileName;
    _analysisSet.setMemoryOwner(false);

	setup();
	cout << "Loaded model " << getName() << " from file " << getInputFileName() << endl;
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aModel Model to be copied.
 */

Model::Model(const Model &aModel) :
   ModelComponent(aModel),
	_creditsStr(_creditsStrProp.getValueStr()),
	_publicationsStr(_publicationsStrProp.getValueStr()),
	_lengthUnitsStr(_lengthUnitsStrProp.getValueStr()),
	_forceUnitsStr(_forceUnitsStrProp.getValueStr()),
	_forceSetProp(PropertyObj("", ForceSet())),
	_forceSet((ForceSet&)_forceSetProp.getValueObj()),
    _gravity(_gravityProp.getValueDblVec()),
    _bodySetProp(PropertyObj("", BodySet())),
    _bodySet((BodySet&)_bodySetProp.getValueObj()),
    _constraintSetProp(PropertyObj("", ConstraintSet())),
    _constraintSet((ConstraintSet&)_constraintSetProp.getValueObj()),
    _markerSetProp(PropertyObj("", MarkerSet())),
    _markerSet((MarkerSet&)_markerSetProp.getValueObj()),
    _contactGeometrySetProp(PropertyObj("", ContactGeometrySet())),
    _contactGeometrySet((ContactGeometrySet&)_contactGeometrySetProp.getValueObj()),
    _allControllersEnabled(true),
    _perturbActuatorForces(false),
    _jointSet(JointSet()),
    _analysisSet(AnalysisSet()),
    _coordinateSet(CoordinateSet()),
    _controllerSet(ControllerSet()),
    _system(NULL),
	_defaultControls(*new Vector(0))
{
	//cout << "Construct copied model " <<  endl;
	// Throw exception if something wrong happened and we don't have a dynamics engine.
	setNull();
	setupProperties();
	copyData(aModel);

	setup();
}
//_____________________________________________________________________________
/**
 * Destructor.
 */
Model::~Model()
{
	delete _matter;
	delete _forceSubsystem;
	delete _contactSubsystem;
	delete _decorationSubsystem;
	delete _gravityForce;
	delete _system;
	delete _assemblySolver;
}
//_____________________________________________________________________________
/**
 * Override default implementation by object to intercept and fix the XML node
 * underneath the model to match current version
 */
/*virtual*/
void Model::updateFromXMLNode()
{
	int documentVersion = getDocument()->getDocumentVersion();
	if ( documentVersion < XMLDocument::getLatestVersion()){
		cout << "Updating Model file to latest format..." << endl;
		// Version has to be 1.6 or later, otherwise assert
		if (_node!=NULL && documentVersion==10600){
			// Get node for DynamicsEngine
			DOMElement* enginesNode = XMLNode::GetFirstChildElementByTagName(_node,"DynamicsEngine");
			//Get node for SimbodyEngine
			if (enginesNode != 0){
				DOMElement* simbodyEngineNode = XMLNode::GetFirstChildElementByTagName(enginesNode,"SimbodyEngine");
				// Move all Children of simbodyEngineNode to be children of _node
				// we'll keep inserting before enginesNode then remove it;
				if (simbodyEngineNode!= 0){
					for(DOMNode *child=simbodyEngineNode->getFirstChild(); child!=NULL;
												child=child->getNextSibling()) {
						DOMElement* childElement = (DOMElement*)child;
						//const XMLCh *   nodeName = childElement->getTagName();
						//char *str1 = XMLString::transcode(nodeName);
						//cout << "Moving child " << std::string(str1) << endl;
						_node->insertBefore(childElement->cloneNode(true), enginesNode);
					}
				}
				XMLNode::RemoveChildren(enginesNode);
				_node->removeChild(enginesNode);
			}
			// Now handling the rename of ActuatorSet to ForceSet
			DOMElement* actuatorsNode = XMLNode::GetFirstChildElementByTagName(_node,"ActuatorSet");
			if (actuatorsNode != 0) {
				DOMElement* forcesNode = XMLNode::CreateDOMElement(getDocument()->getDOMDocument(), "ForceSet");
				DOMNodeList * children = actuatorsNode->getChildNodes();
				for (unsigned int i=0; i<children->getLength(); i++){
					DOMNode* nextChild = children->item(i);
					//actuatorsNode->removeChild(nextChild);
					forcesNode->appendChild(nextChild->cloneNode(true));
				}
				_node->insertBefore(forcesNode, actuatorsNode);
				_node->removeChild(actuatorsNode);
			}
		}
	}
	// Call base class now assuming _node has been corrected for current version
	Object::updateFromXMLNode();

	setDefaultProperties();
}
//_____________________________________________________________________________
/**
 * Copy this Model and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this Model.
 */
Object* Model::copy() const
{
	Model *model = new Model(*this);
	return(model);
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy the member variables of the model.
 *
 * @param aModel model to be copied
 */
void Model::copyData(const Model &aModel)
{
	_fileName = aModel._fileName;
	_creditsStr = aModel._creditsStr;
	_publicationsStr = aModel._publicationsStr;
	_lengthUnits = aModel._lengthUnits;
	_forceUnits = aModel._forceUnits;
	_lengthUnitsStr = aModel._lengthUnitsStr;
	_forceUnitsStr = aModel._forceUnitsStr;
	_forceSet = aModel._forceSet;
	_analysisSet = aModel._analysisSet;
    _gravity = aModel._gravity;
    _bodySet=aModel._bodySet;
    _constraintSet=aModel._constraintSet;
	_controllerSet=aModel._controllerSet;
    _markerSet = aModel._markerSet;
    _contactGeometrySet = aModel._contactGeometrySet;

}
//_____________________________________________________________________________
/**
 * Set the values of all data members to an appropriate "null" value.
 */
void Model::setNull()
{
	setType("Model");
    _allControllersEnabled = true;
	_perturbActuatorForces = false,
    _groundBody = NULL;

    _system = NULL;
    _matter = NULL;

    _forceSubsystem = NULL;
    _contactSubsystem = NULL;
    _decorationSubsystem = NULL;
    _gravityForce = NULL;

	_assemblySolver = NULL;

	_validationLog="";

	_modelComponents.setMemoryOwner(false);
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 *
 */
void Model::setupProperties()
{
	_creditsStrProp.setName("credits");
	_propertySet.append(&_creditsStrProp);

	_publicationsStrProp.setName("publications");
	_propertySet.append(&_publicationsStrProp);

	_forceSetProp.setName("ForceSet");
	_propertySet.append(&_forceSetProp);

	_lengthUnitsStrProp.setName("length_units");
	_lengthUnitsStrProp.setValue("meters");
	_propertySet.append(&_lengthUnitsStrProp);

	_forceUnitsStrProp.setName("force_units");
	_forceUnitsStrProp.setValue("N");
	_propertySet.append(&_forceUnitsStrProp);

   const SimTK::Vec3 defaultGravity(0.0, -9.80665, 0.0);
    _gravityProp.setComment("Acceleration due to gravity.");
    _gravityProp.setName("gravity");
    _gravityProp.setValue(defaultGravity);
    _propertySet.append(&_gravityProp);

    // Note: PropertyObj tag names come from the object's type (e.g. _bodySetProp below will automatically be associated with <BodySet> tag)
    // don't need to call _bodySetProp.setName()...
    _bodySetProp.setComment("Bodies in the model.");
    _propertySet.append(&_bodySetProp);

    _constraintSetProp.setComment("Constraints in the model.");
    _propertySet.append(&_constraintSetProp);

    _markerSetProp.setComment("Markers in the model.");
    _propertySet.append(&_markerSetProp);

    _contactGeometrySetProp.setComment("ContactGeometry objects in the model.");
    _propertySet.append(&_contactGeometrySetProp);
}

SimTK::State& Model::initSystem()
{
	// Some validation
	validateMassProperties();
	if (getValidationLog().size()>0)
		cout << "The following Errors/Warnings were encountered while building the model. " <<
		getValidationLog() << endl;
	
	_modelComponents.setSize(0);	// Make sure we start on a clean slate
	_stateNames.setSize(0);
	_stateYIndices.setSize(0);
	setup();
	createSystem();
    getMultibodySystem().realizeTopology();
    SimTK::State& s = updMultibodySystem().updDefaultState();

	// The folllowing line is commented out as it removes all forces that were
	// added to the system during realizeTopology()
    //_matter->setUseEulerAngles(s, true);
	//getMultibodySystem().realizeModel(s);

    initState(s);
	getStateNames(_stateNames);

    getMultibodySystem().realize(s, Stage::Position );

    updControllerSet().setActuators(updActuators());
    //updControllerSet().constructStorage();

	SimTK::Array_<CoordinateReference> &coordsToTrack = *new SimTK::Array_<CoordinateReference>();
	for(int i=0; i<getNumCoordinates(); i++){
		// iff a coordinate is dependent on other coordinates for its value, do not give it a reference/goal
		if(!_coordinateSet[i].isDependent(s)){
			Constant reference(Constant(_coordinateSet[i].getValue(s)));
			CoordinateReference *coordRef = new CoordinateReference(_coordinateSet[i].getName(), reference);
			coordsToTrack.push_back(*coordRef);
		}
	}

	// Have an AssemblySolver on hand, but delete any old one first
	delete _assemblySolver;

	// Use the assembler to generate the initial pose from Coordinate defaults
	// that also satisfies the constraints
	_assemblySolver = new AssemblySolver(*this, coordsToTrack);
	_assemblySolver->setConstraintWeight(50.0);


	// do the assembly
	assemble(s);

	return(s);
}

void Model::assemble(SimTK::State& s, const Coordinate *coord, double weight)
{
	// Don't bother assembling if the model has no coupled constraints
	// Coordinates have locks as prescribed motion constraints but do not need assemble to be resolved
	if(_constraintSet.getSize()< 1){
		// just realize the current state to position
		getMultibodySystem().realize(s, Stage::Position);
		return;
	}

	const Array_<CoordinateReference>& coordRefs = _assemblySolver->getCoordinateReferences();

	for(unsigned int i=0; i<coordRefs.size(); i++){
		const string &coordName = coordRefs[i].getName();
		_assemblySolver->updateCoordinateReference(coordName, _coordinateSet.get(coordName).getValue(s));
	}

	if(coord)
		_assemblySolver->updateCoordinateReference(coord->getName(), coord->getValue(s), weight);


	try{
		// Try to track first with model satisfying the constraints exactly.
		_assemblySolver->track(s);
	}
	catch (std::exception ex)    {
		try{
			// Otherwise try to do a full-blown assemble
			_assemblySolver->assemble(s);
		}
		catch (std::exception ex){
			// Constraints are probably infeasible so try again relaxing constraints
			cout << "Model unable to assemble: " << ex.what() << endl;
			cout << "Model relaxing constraints and trying again." << endl;

			try{
				// Try to satisfy with constraints as errors weighted heavily.
				_assemblySolver->setConstraintWeight(20.0);
				_assemblySolver->assemble(s);
			}
			catch (std::exception ex){
				cout << "Model unable to assemble with relaxed constraints: " << ex.what() << endl;
			}
		}
	}

	// Have a new working confirguration so should realize to atleast position
	getMultibodySystem().realize(s, Stage::Position);
}

void Model::invalidateSystem()
{
    if (_system != NULL)
        _system->getSystemGuts().invalidateSystemTopologyCache();
}

bool Model::isValidSystem()
{
    if (_system != NULL)
        return _system->systemTopologyHasBeenRealized();
	else
		return false;
}


//_____________________________________________________________________________
/**
 * Create the multibody system.
 *
 */
void Model::createSystem()
{
    if (_system != NULL)
    {
        // Delete the old system.
        delete _matter;
        delete _forceSubsystem;
        delete _contactSubsystem;
        delete _decorationSubsystem;
        delete _gravityForce;
        delete _system;
    }

    // create system
    _system = new SimTK::MultibodySystem;
    _matter = new SimTK::SimbodyMatterSubsystem(*_system);
    _forceSubsystem = new SimTK::GeneralForceSubsystem(*_system);
    _contactSubsystem = new SimTK::GeneralContactSubsystem(*_system);
    _decorationSubsystem = new SimTK::DecorationSubsystem(*_system);
    _decorationSubsystem->addDecorationGenerator(SimTK::Stage::Position, new Model::DefaultGeometry(*this));
    _matter->setShowDefaultGeometry(false);

	// create gravity force, a direction is needed even if magnitude=0 for PotentialEnergy purposes.
	double magnitude = _gravity.norm();
	SimTK::UnitVec3 direction = magnitude==0 ? SimTK::UnitVec3(0,-1,0) : SimTK::UnitVec3(_gravity/magnitude);
	_gravityForce = new SimTK::Force::Gravity(*_forceSubsystem, *_matter, direction, magnitude);

	// Reset the vector of all controls' defaults
	_defaultControls.resize(0);

	// Create the shared cache that will hold all model controls
	// This must be created before Actuator.createSystem() since Actuator will append 
	// its "slots" and retain its index by accessing this cached Vector
	Measure_<Vector>::Result modelControls(_system->updDefaultSubsystem(), Stage::Velocity, Stage::Acceleration);
	_modelControlsIndex = modelControls.getSubsystemMeasureIndex();

    // Let all the ModelComponents add their parts to the System.
    static_cast<const ModelComponentSet<Body>&>(getBodySet()).createSystem(*_system);

	static_cast<const ModelComponentSet<Joint>&>(getJointSet()).createSystem(*_system);
	for(int i=0;i<getBodySet().getSize();i++) {
		OpenSim::Body& body = getBodySet().get(i);
		MobilizedBodyIndex idx(body.getIndex());
        if (!idx.isValid() && body.getName()!= "ground")
			throw Exception("Body: "+body.getName()+" has no Joint... Model initialization aborted.");
	}

    static_cast<const ModelComponentSet<Constraint>&>(getConstraintSet()).createSystem(*_system);
    static_cast<const ModelComponentSet<ContactGeometry>&>(getContactGeometrySet()).createSystem(*_system);


    // Add extra constraints for coordinates.
	static_cast<const ModelComponentSet<Coordinate>&>(getCoordinateSet()).createSystem(*_system);


    static_cast<const ModelComponentSet<Force>&>(getForceSet()).createSystem(*_system);

	// controllers add their parts to the System.
    static_cast<const ModelComponentSet<Controller>&>(getControllerSet()).createSystem(*_system);
}

//_____________________________________________________________________________
/**
 * Add a body to the Model.
 */
void Model::addBody(OpenSim::Body *aBody)
{
	updBodySet().append(aBody);
	updBodySet().setup(*this);
	updJointSet().populate(*this);
	updCoordinateSet().populate(*this);
}

//_____________________________________________________________________________
/**
 * Add a constraint to the Model.
 */
void Model::addConstraint(OpenSim::Constraint *aConstraint)
{
	updConstraintSet().append(aConstraint);
	updConstraintSet().setup(*this);
}

//_____________________________________________________________________________
/**
 * Add a force to the Model.
 */
void Model::addForce(OpenSim::Force *aForce)
{
	updForceSet().append(aForce);
	updForceSet().setup(*this);
}

//_____________________________________________________________________________
/**
 * Add a contact geometry to the Model.
 */
void Model::addContactGeometry(OpenSim::ContactGeometry *aContactGeometry)
{
	updContactGeometrySet().append(aContactGeometry);
	updContactGeometrySet().setup(*this);
}

//_____________________________________________________________________________
/**
 * Add a controller to the Model.
 */
void Model::addController(Controller *aController)
{
	if (aController ) {
	   updControllerSet().append(aController);
	   updControllerSet().setup(*this);
    }
}
//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized. This method is
 * not yet designed to be called after a model has been
 * copied.
 */
void Model::setup()
{
	createGroundBodyIfNecessary();

	// Update model components, not that Joints and Coordinates
	// belong to Bodies, alough model lists are assembled for convenience
	updBodySet().setup(*this);

    // Populate lists of model joints and coordinates according to the Bodies
	// setup here who own the Joints which in turn own the model's Coordinates
	// this list of Coordinates is now available for setting up constraints and forces
	updJointSet().populate(*this);
    updCoordinateSet().populate(*this);

    updConstraintSet().setup(*this);
    updMarkerSet().setup(*this);
    updContactGeometrySet().setup(*this);
	updForceSet().setup(*this);
	updControllerSet().setup(*this);

	// TODO: Get rid of the SimbodyEngine
	updSimbodyEngine().setup(*this);

	updAnalysisSet().setModel(*this);
}

/**
 * Create a ground body if necessary.
 */
void Model::createGroundBodyIfNecessary()
{
    const std::string SimbodyGroundName = "ground";

	// See if the ground body already exists.
	// The ground body is assumed to have the name simbodyGroundName.
	int size = getBodySet().getSize();
	Body *ground=NULL;
	for(int i=0; i<size; i++) {
		Body& body = getBodySet().get(i);
		if(body.getName() == SimbodyGroundName) {
			ground = &body;
			break;
		}
	}

	if(ground==NULL) {
		ground = new Body();
		_bodySet.append(ground);
	}
	// Set member variables
	ground->setName(SimbodyGroundName);
    ground->setMass(0.0);
	ground->setMassCenter(Vec3(0.0));
	_groundBody = ground;
}


//_____________________________________________________________________________
/**
 * Perform some clean up functions that are normally done from the destructor
 * however this gives the GUI a way to proactively do the cleaning without waiting for garbage
 * collection to do the actual cleanup.
 */
void Model::cleanup()
{
	_forceSet.setSize(0);
}

void Model::setDefaultProperties()
{
	if (_creditsStrProp.getUseDefault()){
		_creditsStr = "Model authors names..";
	}
	if (_publicationsStrProp.getUseDefault()){
		_publicationsStr = "List of publications related to model...";
	}

	// Initialize the length and force units from the strings specified in the model file.
	// If they were not specified, use meters and Newtons.

    if (_lengthUnitsStrProp.getUseDefault()){
		_lengthUnits = Units(Units::Meters);
		_lengthUnitsStr = _lengthUnits.getLabel();
	}
	else
		_lengthUnits = Units(_lengthUnitsStr);

	if (_forceUnitsStrProp.getUseDefault()){
		_forceUnits = Units(Units::Newtons);
		_forceUnitsStr = _forceUnits.getLabel();
	}
	else
		_forceUnits = Units(_forceUnitsStr);
}

void Model::initState(SimTK::State& state) const
{
	// Allocate the size and default values for controls
	// Actuators will have a const view into the cache
	Measure_<Vector>::Result controlsCache = Measure_<Vector>::Result::getAs(_system->updDefaultSubsystem().getMeasure(_modelControlsIndex));
	controlsCache.updValue(state).resize(_defaultControls.size());
	controlsCache.updValue(state) = _defaultControls;

    _bodySet.initState(state);
    _constraintSet.initState(state);
    _contactGeometrySet.initState(state);
    _jointSet.initState(state);
    _forceSet.initState(state);
}

void Model::setDefaultsFromState(const SimTK::State& state)
{
    _bodySet.setDefaultsFromState(state);
    _constraintSet.setDefaultsFromState(state);
    _contactGeometrySet.setDefaultsFromState(state);
    _jointSet.setDefaultsFromState(state);
    _forceSet.setDefaultsFromState(state);
}

void Model::equilibrateMuscles(SimTK::State& state)
{
    for (int i = 0; i < _forceSet.getSize(); i++)
    {
        Muscle* muscle = dynamic_cast<Muscle*>(&_forceSet.get(i));
        if (muscle != NULL)
            muscle->equilibrate(state);
    }
}

//_____________________________________________________________________________
/**
 * Register the types used by this class.
void Model::registerTypes()
{
	// now handled by RegisterTypes_osimSimulation()
}
 */


//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
Model& Model::operator=(const Model &aModel)
{
	// BASE CLASS
	Object::operator=(aModel);

	// Class Members


	copyData(aModel);

	setup();

	return(*this);
}


//=============================================================================
// GRAVITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the gravity vector in the gloabl frame.
 *
 * @return the XYZ gravity vector in the global frame is returned here.
 */
SimTK::Vec3 Model::getGravity() const
{
	if(_gravityForce)
		_gravity = _gravityForce->getDefaultGravityVector();

	return _gravity;
}
//_____________________________________________________________________________
/**
 * Set the gravity vector in the gloabl frame.
 *
 * @param aGrav the XYZ gravity vector
 * @return Whether or not the gravity vector was successfully set.
 */
bool Model::setGravity(const SimTK::Vec3& aGrav)
{
	_gravity = aGrav;

	if(_gravityForce)
		_gravityForce->setDefaultGravityVector(aGrav);

	return true;
}


//=============================================================================
// NUMBERS
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the number of markers in the model.
 *
 * @return Number of markers.
 */
int Model::getNumMarkers() const
{
    return _markerSet.getSize();
}
/**
 * Get the number of ContactGeometry objects in the model.
 *
 * @return Number of ContactGeometries.
 */
int Model::getNumContactGeometries() const
{
	return _contactGeometrySet.getSize();
}
int Model::getNumStates(bool includeSimTKStates) const 
{
	return( includeSimTKStates?_system->getDefaultState().getNY():_stateNames.getSize() );
}

/**
 * Get the number of Muscle state variabls in the model.
 *
 * @return Number of MuscleStates.
 */

int Model::getNumMuscleStates() const {

	int n = 0;
	for(int i=0;i<_forceSet.getSize();i++){
        Muscle *mus = dynamic_cast<Muscle*>( &_forceSet.get(i) );
		if(mus!=NULL) {
			n += mus->getNumStateVariables();
		}
	}
	return(n);
}
//_____________________________________________________________________________
/**
 * Get the total number of bodies in the model.
 *
 * @return Number of bodies.
 */
int Model::getNumBodies() const
{
	return  _bodySet.getSize();
}
//_____________________________________________________________________________
/**
 * Get the total number of joints in the model.
 *
 * @return Number of joints.
 */
int Model::getNumJoints() const
{
	return  _jointSet.getSize();
}
//_____________________________________________________________________________
/**
 * Get the total number of coordinates in the model.
 *
 * @return Number of coordinates.
 */
int Model::getNumCoordinates() const
{
	return _coordinateSet.getSize();
}

/**
 * Get the total number of coordinates = number of speeds in the model.
 *
 * @return Number of coordinates.
 */
int Model::getNumSpeeds() const
{
	return _coordinateSet.getSize();
}
//_____________________________________________________________________________
/**
 * Get the subset of Forces in the model which are actuators
 *
 * @return The set of Actuators
 */
const Set<Actuator>& Model::getActuators() const
{
	return _forceSet.getActuators();
}
Set<Actuator>& Model::updActuators()
{
	return _forceSet.updActuators();
}

//_____________________________________________________________________________
/**
 * Get the subset of Forces in the model which are muscles
 *
 * @return The set of Muscles
 */
const Set<Muscle>& Model::getMuscles() const
{
	return _forceSet.getMuscles();
}
Set<Muscle>& Model::updMuscles()
{
	return _forceSet.updMuscles();
}

//_____________________________________________________________________________
/**
 * Get the number of analyses in the model.
 *
 * @return The number of analyses
 */
int Model::getNumAnalyses() const
{
    return _analysisSet.getSize();
}

//_____________________________________________________________________________

//=============================================================================
// TIME NORMALIZATION CONSTANT
//=============================================================================
//_____________________________________________________________________________


//=============================================================================
// STATES
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the names of the states.
 *
 * @param rStateNames Array of state names..
 */
void Model::getStateNames(OpenSim::Array<string> &rStateNames, bool includeInternalStates) const
{

	std::string internalStatesName="_unnamedState_";
	OpenSim::Array<string> allStateNames(internalStatesName, _system->updDefaultState().getY().size());
	//cout << "All states " << _system->updDefaultState().getY().size() << endl;
	for(int i=0; i< _modelComponents.getSize(); i++){
		const ModelComponent* comp=_modelComponents.get(i);
		int numStates = comp->getNumStateVariables();
		for(int j=0; j < numStates; j++){
			//cout << comp->getStateVariableName(j) << ", " <<
			//	    comp->getStateVariableYIndex(j) << endl;
			allStateNames[comp->getStateVariableYIndex(j)] = comp->getStateVariableName(j);
}
	}
	if (includeInternalStates)
		rStateNames = allStateNames;
	else { // pack allStateNames into rStateNames removing internalStates
		rStateNames.setSize(0);
		Model* mutableThis = const_cast<Model *>(this);
		mutableThis->_stateYIndices.setSize(0);
		//int j=0;
		for(int i=0; i< allStateNames.getSize(); i++)
			if (allStateNames[i]!= internalStatesName) {
				rStateNames.append(allStateNames[i]);
				mutableThis->_stateYIndices.append(i);
			}
	}
}

void Model::getStateValues(const SimTK::State& s, Array<double> &rStateValues) const
{
	rStateValues.setSize(getNumStates());
	for(int i=0; i< _stateYIndices.getSize(); i++) 
		rStateValues[i] = s.getY()[_stateYIndices[i]];
}
void Model::setStateValues(SimTK::State& s, double* aStateValues) const
{
	const SimTK::Stage& currentStage=s.getSystemStage();
	for(int i=0; i< _stateYIndices.getSize(); i++) // initialize to NaN
			s.updY()[_stateYIndices[i]]=aStateValues[i]; 
	 _system->realize(s, currentStage );
}
//=============================================================================
// INITIAL STATES
//=============================================================================

void Model::setInitialTime( double ti ) {
	    _system->updDefaultState().updTime() = ti;
}

//_____________________________________________________________________________
/**
 * Add an analysis to the model.
 *
 * @param aAnalysis pointer to the analysis to add
 */
void Model::addAnalysis(Analysis *aAnalysis)
{
	if (aAnalysis )
	{
//		aAnalysis->setModel(this);
		_analysisSet.append(aAnalysis);
	}
}
//_____________________________________________________________________________
/**
 * Remove an analysis from the model
 *
 * @param aAnalysis Pointer to the analysis to remove.
 * If deleteIt is true (default) the Analysis object itself is destroyed
 * else only removed from te list which is the desired behavior when the Analysis
 * is created from the GUI.
 */
void Model::removeAnalysis(Analysis *aAnalysis, bool deleteIt)
{
	// CHECK FOR NULL
	if(aAnalysis==NULL) {
		cout << "Model.removeAnalysis:  ERROR- NULL analysis.\n" << endl;
	}
	if (!deleteIt){
		bool saveStatus = _analysisSet.getMemoryOwner();
		_analysisSet.setMemoryOwner(false);
		_analysisSet.remove(aAnalysis);
		_analysisSet.setMemoryOwner(saveStatus);
	}
	else
		_analysisSet.remove(aAnalysis);
}

//_____________________________________________________________________________
/**
 * Remove a controller from the model
 *
 * @param aController Pointer to the controller to remove.
 */
void Model::removeController(Controller *aController)
{
	// CHECK FOR NULL
	if(aController==NULL) {
		cout << "Model.removeController:  ERROR- NULL controller.\n" << endl;
	}

	_controllerSet.remove(aController);
}

//_____________________________________________________________________________
/**
//_____________________________________________________________________________
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
void Model::
computeEquilibriumForAuxiliaryStates( SimTK::State& s)
{

    _system->realize(s, SimTK::Stage::Velocity );

	//s.getY().dump("y");
	// COMPUTE EQUILIBRIUM STATES
	_forceSet.computeEquilibrium(s);

}


//==========================================================================
// OPERATIONS
//==========================================================================
//--------------------------------------------------------------------------
// SCALE
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Scale the model
 *
 * @param aScaleSet the set of XYZ scale factors for the bodies
 * @param aFinalMass the mass that the scaled model should have
 * @param aPreserveMassDist whether or not the masses of the
 *        individual bodies should be scaled with the body scale factors.
 * @return Whether or not scaling was successful.
 */
bool Model::scale(SimTK::State& s, const ScaleSet& aScaleSet, double aFinalMass, bool aPreserveMassDist)
{
	int i;

	// 1. Save the current pose of the model, then put it in a
	//    default pose, so pre- and post-scale muscle lengths
	//    can be found.
    SimTK::Vector savedConfiguration = s.getY();
	applyDefaultConfiguration(s);
	// 2. For each Actuator, call its preScale method so it
	//    can calculate and store its pre-scale length in the
	//    current position, and then call its scale method to
	//    scale all of the muscle properties except tendon and
	//    fiber length.
	for (i = 0; i < _forceSet.getSize(); i++)
	{
        PathActuator* act = dynamic_cast<PathActuator*>(&_forceSet.get(i));
        if( act ) {
 		    act->preScale(s, aScaleSet);
		    act->scale(s, aScaleSet);
        }
	}
	// 3. Scale the rest of the model
	bool returnVal = updSimbodyEngine().scale(s, aScaleSet, aFinalMass, aPreserveMassDist);

	// 4. If the dynamics engine was scaled successfully,
	//    call each SimmMuscle's postScale method so it
	//    can calculate its post-scale length in the current
	//    position and then scale the tendon and fiber length
	//    properties.


	if (returnVal)
	{
		initSystem();	// This crashes now trying to delete the old matterSubsystem
    	updSimbodyEngine().setup(*this);
	    getMultibodySystem().realizeTopology();
		SimTK::State& newState = updMultibodySystem().updDefaultState();
	    getMultibodySystem().realize( newState, SimTK::Stage::Velocity);

		for (i = 0; i < _forceSet.getSize(); i++) {
            PathActuator* act = dynamic_cast<PathActuator*>(&_forceSet.get(i));
            if( act ) {
	 		    act->postScale(newState, aScaleSet);
            }
        }

		// 5. Put the model back in whatever pose it was in.

        newState.updY() = savedConfiguration;
		getMultibodySystem().realize( newState, SimTK::Stage::Velocity );
	}

    return returnVal;
}


//=============================================================================
// PRINT
//=============================================================================
//_____________________________________________________________________________
/**
 * Print some basic information about the model.
 *
 * @param aOStream Output stream.
 */
void Model::printBasicInfo(std::ostream &aOStream) const
{
	aOStream<<"             MODEL: "<<getName()<<std::endl;
	aOStream<<"            forces: "<<getForceSet().getSize()<<std::endl;
	aOStream<<"          analyses: "<<getNumAnalyses()<<std::endl;
	aOStream<<"            bodies: "<<getBodySet().getSize()<<std::endl;
	aOStream<<"            joints: "<<((OpenSim::Model*)this)->getJointSet().getSize()<<std::endl;
	aOStream<<"           markers: "<<getMarkerSet().getSize()<<std::endl;
}
//_____________________________________________________________________________
/**
 * Print detailed information about the model.
 *
 * @param aOStream Output stream.
 */
void Model::printDetailedInfo(const SimTK::State& s, std::ostream &aOStream) const
{
	//int i;

	aOStream << "MODEL: " << getName() << std::endl;

	aOStream << "\nANALYSES (" << getNumAnalyses() << ")" << std::endl;
	for (int i = 0; i < _analysisSet.getSize(); i++)
		aOStream << "analysis[" << i << "] = " << _analysisSet.get(i).getName() << std::endl;

	aOStream << "\nBODIES (" << getNumBodies() << ")" << std::endl;
	const BodySet& bodySet = getBodySet();
	for(int i=0; i < bodySet.getSize(); i++) {
		const OpenSim::Body& body = bodySet.get(i);
		aOStream << "body[" << i << "] = " << body.getName();
		aOStream << " (mass: "<<body.getMass()<<")";
		Mat33 inertia;
		body.getInertia(inertia);
		aOStream << " (inertia:";
		for(int j=0; j<3; j++) for(int k=0; k<3; k++) aOStream<<" "<<inertia[j][k];
		aOStream << ")"<<endl;
	}

    int j = 0;
	aOStream << "\nACTUATORS (" << getActuators().getSize() << ")" << std::endl;
	for (int i = 0; i < getActuators().getSize(); i++) {
		 aOStream << "actuator[" << j << "] = " << getActuators().get(i).getName() << std::endl;
         j++;
	}

	aOStream << "numStates = " << s.getNY() << std::endl;
	aOStream << "numCoordinates = " << getNumCoordinates() << std::endl;
	aOStream << "numSpeeds = " << getNumSpeeds() << std::endl;
	aOStream << "numActuators = " << getActuators().getSize() << std::endl;
	aOStream << "numBodies = " << getNumBodies() << std::endl;
	aOStream << "numConstraints = " << getConstraintSet().getSize() << std::endl;
	;

	/*
	int n;
	aOStream<<"MODEL: "<<getName()<<std::endl;

	n = getNumBodies();
	aOStream<<"\nBODIES ("<<n<<")" << std::endl;
	for(i=0;i<n;i++) aOStream<<"body["<<i<<"] = "<<getBodyName(i)<<std::endl;

	n = getNQ();
	aOStream<<"\nGENERALIZED COORDINATES ("<<n<<")" << std::endl;
	for(i=0;i<n;i++) aOStream<<"q["<<i<<"] = "<<getCoordinateName(i)<<std::endl;

	n = getNU();
	aOStream<<"\nGENERALIZED SPEEDS ("<<n<<")" << std::endl;
	for(i=0;i<n;i++) aOStream<<"u["<<i<<"] = "<<getSpeedName(i)<<std::endl;

	n = getNA();
	aOStream<<"\nACTUATORS ("<<n<<")" << std::endl;
	for(i=0;i<n;i++) aOStream<<"actuator["<<i<<"] = "<<getActuatorName(i)<<std::endl;

	n = getNP();
	aOStream<<"\nCONTACTS ("<<n<<")" << std::endl;

*/
	Array<string> stateNames("");
	getStateNames(stateNames);
	aOStream<<"\nSTATES ("<<stateNames.getSize()<<")"<<std::endl;
	for(int i=0;i<stateNames.getSize();i++) aOStream<<"y["<<i<<"] = "<<stateNames[i]<<std::endl;
}

//--------------------------------------------------------------------------
// CONFIGURATION
//--------------------------------------------------------------------------

//_____________________________________________________________________________
/**
 * Apply the default configuration to the model.  This means setting the
 * generalized coordinates and spees to their default values.
 */
void Model::applyDefaultConfiguration(SimTK::State& s)
{
	int i;

	// Coordinates
	int ncoords = getCoordinateSet().getSize();

	for(i=0; i<ncoords; i++) {
		Coordinate& coord = getCoordinateSet().get(i);
		coord.setValue(s, coord.getDefaultValue(), false);
		coord.setSpeedValue(s, coord.getDefaultSpeedValue());
	}

	// Satisfy the constraints.
	assemble(s);
}



//--------------------------------------------------------------------------
// MARKERS
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Write an XML file of all the markers in the model.
 *
 * @param aFileName the name of the file to create
 */
void Model::writeMarkerFile(const string& aFileName) const
{
	_markerSet.print(aFileName);
}

//_____________________________________________________________________________
/**
 * Replace all markers in the model with the ones in the passed-in marker set.
 *
 * @param aMarkerSet The new marker set.
 * @return Number of markers that were successfully added to the model.
 */
int Model::replaceMarkerSet(const SimTK::State& s, MarkerSet& aMarkerSet)
{
	int i, numAdded = 0;

	// First remove all existing markers from the model.
	_markerSet.clearAndDestroy();
	_markerSetProp.setUseDefault(false);

	// Now add the markers from aMarkerSet whose body names match bodies in the engine.
	for (i = 0; i < aMarkerSet.getSize(); i++)
	{
		// Eran: we make a *copy* since both _markerSet and aMarkerSet own their elements (so they will delete them)
		Marker* marker = (Marker*)aMarkerSet.get(i).copy();
		const string& bodyName = marker->getBodyName();
		if (getBodySet().contains(bodyName))
		{
    		OpenSim::Body& body = updBodySet().get(bodyName);
			marker->changeBody(body);
			_markerSet.append(marker);
			numAdded++;
		}
	}

	cout << "Replaced marker set in model " << getName() << endl;
	return numAdded;
}

//_____________________________________________________________________________
/**
 * Update all markers in the model with the ones in the
 * passed-in marker set. If the marker does not yet exist
 * in the model, it is added.
 *
 * @param aMarkerSet set of markers to be updated/added
 */
void Model::updateMarkerSet(MarkerSet& aMarkerSet)
{
	_markerSetProp.setUseDefault(false);
	for (int i = 0; i < aMarkerSet.getSize(); i++)
	{
		Marker& updatingMarker = aMarkerSet.get(i);
		const string& updatingBodyName = updatingMarker.getBodyName();

		/* If there is already a marker in the model with that name,
		 * update it with the parameters from the updating marker,
		 * moving it to a new body if necessary.
		 */
		if (updMarkerSet().contains(updatingMarker.getName()))
		{
    		Marker& modelMarker = updMarkerSet().get(updatingMarker.getName());
			/* If the updating marker is on a different body, delete the
			 * marker from the model and add the updating one (as long as
			 * the updating marker's body exists in the model).
			 */
			if (modelMarker.getBody().getName() != updatingBodyName)
			{
				_markerSet.remove(&modelMarker);
				// Eran: we append a *copy* since both _markerSet and aMarkerSet own their elements (so they will delete them)
				_markerSet.append((Marker*)updatingMarker.copy());
			}
			else
			{
				modelMarker.updateFromMarker(updatingMarker);
			}
		}
		else
		{
			/* The model does not contain a marker by that name. If it has
			 * a body by that name, add the updating marker to the markerset.
			 */
			// Eran: we append a *copy* since both _markerSet and aMarkerSet own their elements (so they will delete them)
			if (getBodySet().contains(updatingBodyName))
				_markerSet.append((Marker*)updatingMarker.copy());
		}
	}

	// Todo_AYMAN: We need to call setup again to make sure the _body pointers are up to date; but
	// note that we've already called setup before so we need to make sure the setup() function
	// supports getting called multiple times
	for (int i = 0; i < _markerSet.getSize(); i++)
		_markerSet.get(i).setup(*this);

	cout << "Updated markers in model " << getName() << endl;
}

//_____________________________________________________________________________
/**
 * Remove all markers from the model that are not in the passed-in list.
 *
 * @param aMarkerNames array of marker names not to be deleted
 * @return Number of markers deleted
 *
 * @Todo_AYMAN make sure visuals adjust as well
 */
int Model::deleteUnusedMarkers(const OpenSim::Array<string>& aMarkerNames)
{
	int i, numDeleted = 0;

	for (i = 0; i < _markerSet.getSize(); )
	{
		int index = aMarkerNames.findIndex(_markerSet.get(i).getName());
		if (index < 0)
		{
			// Delete the marker, but don't increment i or else you'll
			// skip over the marker right after the deleted one.
			_markerSet.get(i).removeSelfFromDisplay();
			_markerSet.remove(i);
			numDeleted++;
		}
		else
		{
			i++;
		}
	}

	cout << "Deleted " << numDeleted << " unused markers from model " << getName() << endl;

	return numDeleted;
}

/**
 ** Get a flat list of Joints contained in the model
 **
 **/
JointSet& Model::updJointSet()
{
    return _jointSet;
}

const JointSet& Model::getJointSet()
{

    return _jointSet;
}

/**
 * Get the body that is being used as ground.
 *
 * @return Pointer to the ground body.
 */
OpenSim::Body& Model::getGroundBody() const
{
	assert(_groundBody);
	return *_groundBody;
}


//--------------------------------------------------------------------------
// CONTROLS
//--------------------------------------------------------------------------
/** Get the number of controls for this the model.
 * Throws an exception if called before Model::initSystem()	 */
int Model::getNumControls() const
{
	if(_system == NULL){
		throw Exception("Model::getNumControls() requires an initialized Model./n" 
			"Prior Model::initSystem() required.");
	}

	return _defaultControls.size();
}

/** Update the controls for this the model at a given state.
 * Throws an exception if called before Model::initSystem() */
Vector& Model::updControls(const SimTK::State &s) const
{
	if(_system == NULL || (int(_modelControlsIndex) == SimTK::InvalidIndex)){
		throw Exception("Model::updControls() requires an initialized Model./n" 
			"Prior call to Model::initSystem() is required.");
	}

	// direct the system shared cache 
	Measure_<Vector>::Result controlsCache = Measure_<Vector>::Result::getAs(_system->updDefaultSubsystem().getMeasure(_modelControlsIndex));
	return controlsCache.updValue(s);
}

/** Const access to controls does not invalidate dynamics */
const Vector& Model::getControls(const SimTK::State &s) const
{
	if(_system == NULL || (int(_modelControlsIndex) == SimTK::InvalidIndex)){
		throw Exception("Model::getControls() requires an initialized Model./n" 
			"Prior call to Model::initSystem() is required.");
	}

	// direct the system shared cache 
	Measure_<Vector>::Result controlsCache = Measure_<Vector>::Result::getAs(_system->updDefaultSubsystem().getMeasure(_modelControlsIndex));

	if(!controlsCache.isValid(s)){
		// Always reset controls to their default values before computing controls
		// since default behavior is for controllors to "addInControls" so there should be valid
		// values to begin with.
		controlsCache.updValue(s) = _defaultControls;
		computeControls(s, controlsCache.updValue(s));
		controlsCache.markAsValid(s);
	}

	return controlsCache.getValue(s);
}


/** Compute the controls the model */
void Model::computeControls(const SimTK::State& s, SimTK::Vector &controls) const
{
	getControllerSet().computeControls(s, controls);
}


/** Get a flag indicating if the model needs controls to operate its actuators */
bool Model::isControlled() const
{
	bool isControlled = getActuators().getSize() > 0;

	return isControlled;
}

const ControllerSet& Model::getControllerSet() const{
    return(_controllerSet);
}
ControllerSet& Model::updControllerSet() {
    return(_controllerSet);
}
void Model::storeControls( const SimTK::State& s, int step ) {
    _controllerSet.storeControls(s, step);
    return;
}
void Model::printControlStorage(const string& fileName ) const {
    _controllerSet.printControlStorage(fileName);
}
bool Model::getAllControllersEnabled() const{
  return( _allControllersEnabled );
}
void Model::setAllControllersEnabled( bool enabled ) {
    _allControllersEnabled = enabled;
}
/**
 * Model::formStateStorage is intended to take any storage and populate stateStorage.
 * stateStorage is supposed to be a Storage with labels identical to those obtained by calling
 * Model::getStateNames(). Columns/entries found in the "originalStorage" are copied to the
 * output statesStorage. Entries not found are populated with 0s.
 */
void Model::formStateStorage(const Storage& originalStorage, Storage& statesStorage)
{
	Array<string> rStateNames;
	getStateNames(rStateNames);
    int numStates = getNumStates();
	// make sure same size, otherwise warn
	if (originalStorage.getSmallestNumberOfStates() != rStateNames.getSize()){
		cout << "Number of columns does not match in formStateStorage. Found "
			<< originalStorage.getSmallestNumberOfStates() << " Expected  " << rStateNames.getSize() << "." << endl;
	}
	// Create a list with entry for each desiredName telling which column in originalStorage has the data
	int* mapColumns = new int[rStateNames.getSize()];
	for(int i=0; i< rStateNames.getSize(); i++){
		// the index is -1 if not found, >=1 otherwise since time has index 0 by defn.
		mapColumns[i] = originalStorage.getColumnLabels().findIndex(rStateNames[i]);
		if (mapColumns[i]==-1)
			cout << "Column "<< rStateNames[i] << " not found in formStateStorage, assuming 0." << endl;
	}
	// Now cycle thru and shuffle each

	for (int row =0; row< originalStorage.getSize(); row++){
		StateVector* originalVec = originalStorage.getStateVector(row);
		StateVector* stateVec = new StateVector(originalVec->getTime());
		stateVec->getData().setSize(numStates);  // default value 0f 0.
		for(int column=0; column< numStates; column++){
			double valueInOriginalStorage=0.0;
			if (mapColumns[column]!=-1)
				originalVec->getDataValue(mapColumns[column]-1, valueInOriginalStorage);

			stateVec->setDataValue(column, valueInOriginalStorage);

		}
		statesStorage.append(*stateVec);
	}
	rStateNames.insert(0, "time");
	statesStorage.setColumnLabels(rStateNames);

}

/**
 * Model::formStateStorage is intended to take any storage and populate qStorage.
 * stateStorage is supposed to be a Storage with labels identical to those obtained by calling
 * Model::getStateNames(). Columns/entries found in the "originalStorage" are copied to the
 * output qStorage. Entries not found are populated with 0s.
 */
void Model::formQStorage(const Storage& originalStorage, Storage& qStorage) {

    int nq =  _system->getDefaultState().getNQ();
	Array<string> qNames;
	getCoordinateSet().getNames(qNames);


	int* mapColumns = new int[qNames.getSize()];
	for(int i=0; i< nq; i++){
		// the index is -1 if not found, >=1 otherwise since time has index 0 by defn.
		mapColumns[i] = originalStorage.getColumnLabels().findIndex(qNames[i]);
		if (mapColumns[i]==-1)
			cout << "\n Column "<< qNames[i] << " not found in formQStorage, assuming 0.\n" << endl;
	}


	// Now cycle thru and shuffle each
	for (int row =0; row< originalStorage.getSize(); row++){
		StateVector* originalVec = originalStorage.getStateVector(row);
		StateVector* stateVec = new StateVector(originalVec->getTime());
		stateVec->getData().setSize(nq);  // default value 0f 0.
		for(int column=0; column< nq; column++){
			double valueInOriginalStorage=0.0;
			if (mapColumns[column]!=-1)
				originalVec->getDataValue(mapColumns[column]-1, valueInOriginalStorage);

			stateVec->setDataValue(column, valueInOriginalStorage);
		}
		qStorage.append(*stateVec);
	}
	qNames.insert(0, "time");

	qStorage.setColumnLabels(qNames);

}
void Model::disownAllComponents()
{
	updBodySet().setMemoryOwner(false);
	updConstraintSet().setMemoryOwner(false);
	updForceSet().setMemoryOwner(false);
	updContactGeometrySet().setMemoryOwner(false);
	updControllerSet().setMemoryOwner(false);
	updAnalysisSet().setMemoryOwner(false);
	updMarkerSet().setMemoryOwner(false);
}

void Model::overrideAllActuators( SimTK::State& s, bool flag) {
     Set<Actuator>& as = updActuators();

     for(int i=0;i<as.getSize();i++) {
         as[i].overrideForce(s, flag );
     }

}
//_____________________________________________________________________________
/**
 * validateMassProperties: Internal method to check that specified mass properties for the bodies are physically possible
 * that is, satisfy the triangular inequality condition specified in the Docygen doc. of SimTK::MassPRoperties
 * If not true, then the values are forced to satisfy the inequality and a warning is issued.
 * It is assumed that mass properties are all set already
 *
 *
 */

void Model::validateMassProperties(bool fixMassProperties)
{
	for (int i=0; i < _bodySet.getSize(); i++){
		bool valid = true;
		Body& b = _bodySet.get(i);
		if (b == getGroundBody()) continue;	// Ground's mass properties are unused
		SimTK::Mat33 inertiaMat;
		String msg = "";
		try {
			b.getInertia(inertiaMat);
			SimTK::Inertia_<SimTK::Real>(inertiaMat[0][0], inertiaMat[1][1], inertiaMat[2][2],
				inertiaMat[0][1], inertiaMat[1][2], inertiaMat[0][2]);
		}
		catch(SimTK::Exception::Base& ex){
			valid = false;
			msg = "Body: "+b.getName()+" has non-physical mass properties.";
			msg += ex.getMessage();
		}
		if (!valid){
			if (!fixMassProperties)
				throw Exception(msg);
			else{

				inertiaMat[0][0] = fabs(inertiaMat[0][0]);
				inertiaMat[1][1] = inertiaMat[2][2] = inertiaMat[0][0];
				inertiaMat[0][1] = inertiaMat[0][2] = inertiaMat[1][2] = 0.0;
				inertiaMat[1][0] = inertiaMat[2][0] = inertiaMat[2][1] = 0.0;
				b.setInertia(SimTK::Inertia_<Real>(inertiaMat));
				_validationLog += msg;
				_validationLog += "Inertia vector for body "+b.getName()+" has been reset\n";
			}
		}
	}

}
int Model::getNumStateVariables() const
{
	// Cycle thru all ModelComponents under this model and add up their getNumStateVariables
	int numStateVariables = 0;
	for(int i=0; i< _modelComponents.getSize(); i++)
		numStateVariables += _modelComponents.get(i)->getNumStateVariables();
	
	return numStateVariables;
}

const Object& Model::getObjectByTypeAndName(const std::string& typeString, const std::string& nameString) {
    if (typeString=="Body") 
        return getBodySet().get(nameString);
    else if (typeString=="Force") 
        return getForceSet().get(nameString);
    else if (typeString=="Constraint")
        return getConstraintSet().get(nameString);
    else if (typeString=="Coordinate") 
        return getCoordinateSet().get(nameString);
	else if (typeString=="Marker") 
        return getMarkerSet().get(nameString);
	else if (typeString=="Controller") 
        return getControllerSet().get(nameString);
	else if (typeString=="Joint") 
        return getJointSet().get(nameString);
	throw Exception("Model::getObjectByTypeAndName: no object of type "+typeString+
		" and name "+nameString+" was found in the model.");

}

Model::DefaultGeometry::DefaultGeometry(Model& model) : _model(model) {
}

void Model::DefaultGeometry::generateDecorations(const SimTK::State& state, SimTK::Array_<SimTK::DecorativeGeometry>& geometry) {
    const SimTK::SimbodyMatterSubsystem& matter = _model.getMatterSubsystem();

    // Display a cylinder connecting each pair of joints.

    const JointSet& joints = _model.getJointSet();
    for (int i = 0; i < joints.getSize(); i++) {
        const Joint& joint = joints[i];
        if (joint.getType() != "FreeJoint" && joint.getParentBody().hasJoint()) {
            const Joint& parent = joint.getParentBody().getJoint();
            Vec3 childLocation, parentLocation;
            joint.getLocation(childLocation);
            parent.getLocation(parentLocation);
            childLocation = matter.getMobilizedBody(joint.getBody().getIndex()).getBodyTransform(state)*childLocation;
            parentLocation = matter.getMobilizedBody(parent.getBody().getIndex()).getBodyTransform(state)*parentLocation;
            double length = (childLocation-parentLocation).norm();
            Vec3 center = (childLocation+parentLocation)*0.5;
            Rotation orientation;
            orientation.setRotationFromOneAxis(UnitVec3(childLocation-parentLocation), YAxis);
            geometry.push_back(DecorativeCylinder(0.1*length, 0.5*length).setTransform(Transform(orientation, center)));
        }
    }

    // Display the path of each muscle.

    const Set<Muscle>& muscles = _model.getMuscles();
    for (int i = 0; i < muscles.getSize(); i++) {
        const Muscle& muscle = muscles[i];
        const Array<PathPoint*>& points = muscle.getGeometryPath().getCurrentPath(state);
        Vec3 lastPos = matter.getMobilizedBody(points[0]->getBody().getIndex()).getBodyTransform(state)*points[0]->getLocation();
        for (int j = 1; j < points.getSize(); j++) {
            Vec3 pos = matter.getMobilizedBody(points[j]->getBody().getIndex()).getBodyTransform(state)*points[j]->getLocation();
            geometry.push_back(DecorativeLine(lastPos, pos).setLineThickness(2).setColor(Vec3(1, 0, 0)));
            lastPos = pos;
        }
    }

    // Display wrap objects.

    const bool displayWrapObjects = false;
    if (displayWrapObjects) {
        Transform ztoy;
        ztoy.updR().setRotationFromAngleAboutX(SimTK_PI/2);
        const BodySet& bodies = _model.getBodySet();
        for (int i = 0; i < bodies.getSize(); i++) {
            const Body& body = bodies[i];
            const Transform& bodyTransform = matter.getMobilizedBody(body.getIndex()).getBodyTransform(state);
            const WrapObjectSet& wrapObjects = body.getWrapObjectSet();
            for (int j = 0; j < wrapObjects.getSize(); j++) {
                const string& type = wrapObjects[j].getType();
                if (type == "WrapCylinder") {
                    const WrapCylinder* cylinder = dynamic_cast<const WrapCylinder*>(&wrapObjects[j]);
                    if (cylinder != NULL) {
                        Transform transform = bodyTransform*cylinder->getTransform()*ztoy;
                        geometry.push_back(DecorativeCylinder(cylinder->getRadius(), 0.5*cylinder->getLength()).setTransform(transform).setOpacity(0.5).setColor(Vec3(0, 1, 0)));
                    }
                }
                else if (type == "WrapEllipsoid") {
                    const WrapEllipsoid* ellipsoid = dynamic_cast<const WrapEllipsoid*>(&wrapObjects[j]);
                    if (ellipsoid != NULL) {
                        Transform transform = bodyTransform*ellipsoid->getTransform();
                        geometry.push_back(DecorativeEllipsoid(ellipsoid->getRadii()).setTransform(transform).setOpacity(0.5).setColor(Vec3(0, 1, 0)));
                    }
                }
                else if (type == "WrapSphere") {
                    const WrapSphere* sphere = dynamic_cast<const WrapSphere*>(&wrapObjects[j]);
                    if (sphere != NULL) {
                        Transform transform = bodyTransform*sphere->getTransform();
                        geometry.push_back(DecorativeSphere(sphere->getRadius()).setTransform(transform).setOpacity(0.5).setColor(Vec3(0, 1, 0)));
                    }
                }
            }
        }
    }
}
