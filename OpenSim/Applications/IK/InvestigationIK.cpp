// InvestigationIK.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//=============================================================================
// INCLUDES
//=============================================================================
#include "InvestigationIK.h"
#include <string>
#include <iostream>
#include <OpenSim/Tools/IO.h>	
#include <OpenSim/Simulation/SIMM/CoordinateSet.h>
#include <OpenSim/Simulation/SIMM/MarkerSet.h>
#include <OpenSim/Subject/SimmIKTrialSet.h>
#include <OpenSim/Simulation/SIMM/AbstractModel.h>
#include "SimmIKSolverImpl.h"
#include "SimmInverseKinematicsTarget.h"

using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
InvestigationIK::~InvestigationIK()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
InvestigationIK::InvestigationIK() :
	Investigation(),
	_markerSetProp(PropertyObj("", MarkerSet())),
	_markerSet((MarkerSet&)_markerSetProp.getValueObj()),
	_coordinateSetProp(PropertyObj("", CoordinateSet())),
	_coordinateSet((CoordinateSet&)_coordinateSetProp.getValueObj()),
	_coordinatesFromFile(_coordinatesFromFileProp.getValueStrArray()),
	_IKTrialSetProp(PropertyObj("", SimmIKTrialSet())),
	_IKTrialSet((SimmIKTrialSet&)_IKTrialSetProp.getValueObj())
{
	setType("InvestigationIK");
	setNull();
}
//_____________________________________________________________________________
/**
 * Construct from file.
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
InvestigationIK::InvestigationIK(const string &aFileName, AbstractModel* guiModel) :
	Investigation(aFileName),
	_markerSetProp(PropertyObj("", MarkerSet())),
	_markerSet((MarkerSet&)_markerSetProp.getValueObj()),
	_coordinateSetProp(PropertyObj("", CoordinateSet())),
	_coordinateSet((CoordinateSet&)_coordinateSetProp.getValueObj()),
	_coordinatesFromFile(_coordinatesFromFileProp.getValueStrArray()),
	_IKTrialSetProp(PropertyObj("", SimmIKTrialSet())),
	_IKTrialSet((SimmIKTrialSet&)_IKTrialSetProp.getValueObj())
{
	setType("InvestigationIK");
	setNull();
	string saveWorkingDirectory = IO::getCwd();
	string directoryOfSetupFile = IO::getParentDirectory(aFileName);
	IO::chDir(directoryOfSetupFile);

	updateFromXMLNode();

	if (_model) throw( Exception("InvestigationIK did not expect initialized model") );
	if (guiModel){
		// A valid model is passed in, and is initialized (likely from GUI)
		// In this scenario, _modelFile is ignored (probably with a warning)
		setModel(guiModel);
		addAnalysisSetToModel();
	}
	else {
		if (_modelFile == "") throw( Exception("Model file not specified for inverse kinematics investigation") );
		setModel(new AbstractModel(_modelFile));
		// Cast to SimmModel so we can call functions that don't exist in base Model class
		if (_model) {
			_model->setup();
			addAnalysisSetToModel();
		}
	}
	IO::chDir(saveWorkingDirectory);
}
//_____________________________________________________________________________
/**
 * Construct from a DOMElement.
 */
InvestigationIK::InvestigationIK(DOMElement *aElement) :
	Investigation(aElement),
	_markerSetProp(PropertyObj("", MarkerSet())),
	_markerSet((MarkerSet&)_markerSetProp.getValueObj()),
	_coordinateSetProp(PropertyObj("", CoordinateSet())),
	_coordinateSet((CoordinateSet&)_coordinateSetProp.getValueObj()),
	_coordinatesFromFile(_coordinatesFromFileProp.getValueStrArray()),
	_IKTrialSetProp(PropertyObj("", SimmIKTrialSet())),
	_IKTrialSet((SimmIKTrialSet&)_IKTrialSetProp.getValueObj())
{
	setType("InvestigationIK");
	setNull();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * Copy constructors for all Investigation's only copy the non-XML variable
 * members of the object; that is, the object's DOMnode and XMLDocument
 * are not copied but set to NULL.  The reason for this is that for the
 * object and all its derived classes to establish the correct connection
 * to the XML document nodes, the the object would need to reconstruct based
 * on the XML document not the values of the object's member variables.
 *
 * There are three proper ways to generate an XML document for an Investigation:
 *
 * 1) Construction based on XML file (@see Investigation(const char *aFileName)).
 * In this case, the XML document is created by parsing the XML file.
 *
 * 2) Construction by Investigation(const XMLDocument *aDocument).
 * This constructor explictly requests construction based on an
 * XML document.  In this way the proper connection between an object's node
 * and the corresponding node within the XML document is established.
 * This constructor is a copy constructor of sorts because all essential
 * Investigation member variables should be held within the XML document.
 * The advantage of this style of construction is that nodes
 * within the XML document, such as comments that may not have any
 * associated Investigation member variable, are preserved.
 *
 * 3) A call to generateDocument().
 * This method generates an XML document for the Investigation from scratch.
 * Only the essential document nodes are created (that is, nodes that
 * correspond directly to member variables.).
 *
 * @param aInvestigation Object to be copied.
 * @see Investigation(const XMLDocument *aDocument)
 * @see Investigation(const char *aFileName)
 * @see generateDocument()
 */
InvestigationIK::
InvestigationIK(const InvestigationIK &aInvestigation) :
	Investigation(aInvestigation),
	_markerSetProp(PropertyObj("", MarkerSet())),
	_markerSet((MarkerSet&)_markerSetProp.getValueObj()),
	_coordinateSetProp(PropertyObj("", CoordinateSet())),
	_coordinateSet((CoordinateSet&)_coordinateSetProp.getValueObj()),
	_coordinatesFromFile(_coordinatesFromFileProp.getValueStrArray()),
	_IKTrialSetProp(PropertyObj("", SimmIKTrialSet())),
	_IKTrialSet((SimmIKTrialSet&)_IKTrialSetProp.getValueObj())
{
	setType("InvestigationIK");
	setNull();
	*this = aInvestigation;
}

//_____________________________________________________________________________
/**
 * Virtual copy constructor.
 */
Object* InvestigationIK::
copy() const
{
	InvestigationIK *object = new InvestigationIK(*this);
	return(object);
}
//_____________________________________________________________________________
/**
 * Virtual copy constructor from DOMElement.
 */
Object* InvestigationIK::
copy(DOMElement *aElement) const
{
	InvestigationIK *object = new InvestigationIK(aElement);
	*object = *this;
	object->updateFromXMLNode();
	return(object);
}

//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void InvestigationIK::
setNull()
{
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void InvestigationIK::setupProperties()
{
	string comment;

	_markerSetProp.setComment("Markers to be used by all IK trials");
	_markerSetProp.setName("MarkerSet");
	_propertySet.append(&_markerSetProp);

	_coordinateSetProp.setComment("Specify how to initialize coodinates for IK.");
	_coordinateSetProp.setName("CoordinateSet");
	_propertySet.append(&_coordinateSetProp);

	_coordinatesFromFileProp.setName("coordinates_from_file");
	Array<string> def("");
	_coordinatesFromFileProp.setValue(def);
	_propertySet.append(&_coordinatesFromFileProp);

	_IKTrialSetProp.setComment("Trial parameters, one block per trial");
	_IKTrialSetProp.setName("SimmIKTrialSet");
	_propertySet.append(&_IKTrialSetProp);
}
//_____________________________________________________________________________
/**
 * Register SimmIKTrial type.
 */
void InvestigationIK::registerTypes()
{
	Object::RegisterType(SimmIKTrial());
}
//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
InvestigationIK& InvestigationIK::
operator=(const InvestigationIK &aInvestigation)
{
	// BASE CLASS
	Investigation::operator=(aInvestigation);

	// MEMBER VARIABLES
	_markerSet = aInvestigation._markerSet;
	_coordinateSet = aInvestigation._coordinateSet;
	_coordinatesFromFile = aInvestigation._coordinatesFromFile;
	_IKTrialSet = aInvestigation._IKTrialSet;

	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================


//=============================================================================
// RUN
//=============================================================================
//_____________________________________________________________________________
/**
 * Run the investigation.
 */
void InvestigationIK::run()
{
	cout<<"Running investigation "<<getName()<<".\n";
	
	// Do the maneuver to change then restore working directory 
	// so that the parsing code behaves properly if called from a different directory
	string aFileName = string(getDocument()->getFileName());
	string saveWorkingDirectory = IO::getCwd();
	string directoryOfSetupFile = IO::getParentDirectory(aFileName);
	IO::chDir(directoryOfSetupFile);

	/* Update the markers. */
	_model->getDynamicsEngine().updateMarkerSet(_markerSet);

	/* Now perform the IK trials on the updated model. */
	for (int i = 0; i < _IKTrialSet.getSize(); i++)
	{
		if (_IKTrialSet.get(i)->processTrial(*_model, _coordinateSet, _coordinatesFromFile))
			cout << "Trial " << _IKTrialSet.get(i)->getName() << " processed successfully." << endl;
		else
			cout << "Trial " << _IKTrialSet.get(i)->getName() << " processing failed." << endl;
	}

	IO::chDir(saveWorkingDirectory);
}

