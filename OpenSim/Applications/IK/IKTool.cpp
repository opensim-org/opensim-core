// IKTool.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//=============================================================================
// INCLUDES
//=============================================================================
#include "IKTool.h"
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
IKTool::~IKTool()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
IKTool::IKTool() :
	SimulationTool(),
	_markerSetProp(PropertyObj("", MarkerSet())),
	_markerSet((MarkerSet&)_markerSetProp.getValueObj()),
	_coordinateSetProp(PropertyObj("", CoordinateSet())),
	_coordinateSet((CoordinateSet&)_coordinateSetProp.getValueObj()),
	_coordinatesFromFile(_coordinatesFromFileProp.getValueStrArray()),
	_IKTrialSetProp(PropertyObj("", SimmIKTrialSet())),
	_IKTrialSet((SimmIKTrialSet&)_IKTrialSetProp.getValueObj())
{
	setType("IKTool");
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
IKTool::IKTool(const string &aFileName, AbstractModel* guiModel) :
	SimulationTool(aFileName),
	_markerSetProp(PropertyObj("", MarkerSet())),
	_markerSet((MarkerSet&)_markerSetProp.getValueObj()),
	_coordinateSetProp(PropertyObj("", CoordinateSet())),
	_coordinateSet((CoordinateSet&)_coordinateSetProp.getValueObj()),
	_coordinatesFromFile(_coordinatesFromFileProp.getValueStrArray()),
	_IKTrialSetProp(PropertyObj("", SimmIKTrialSet())),
	_IKTrialSet((SimmIKTrialSet&)_IKTrialSetProp.getValueObj())
{
	setType("IKTool");
	setNull();
	string saveWorkingDirectory = IO::getCwd();
	string directoryOfSetupFile = IO::getParentDirectory(aFileName);
	IO::chDir(directoryOfSetupFile);

	updateFromXMLNode();

	if (_model) throw( Exception("IKTool did not expect initialized model") );
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
 * Copy constructor.
 *
 * Copy constructors for all Tools only copy the non-XML variable
 * members of the object; that is, the object's DOMnode and XMLDocument
 * are not copied but set to NULL.  The reason for this is that for the
 * object and all its derived classes to establish the correct connection
 * to the XML document nodes, the the object would need to reconstruct based
 * on the XML document not the values of the object's member variables.
 *
 * There are three proper ways to generate an XML document for a Tool:
 *
 * 1) Construction based on XML file (@see Tool(const char *aFileName)).
 * In this case, the XML document is created by parsing the XML file.
 *
 * 2) Construction by Tool(const XMLDocument *aDocument).
 * This constructor explictly requests construction based on an
 * XML document.  In this way the proper connection between an object's node
 * and the corresponding node within the XML document is established.
 * This constructor is a copy constructor of sorts because all essential
 * Tool member variables should be held within the XML document.
 * The advantage of this style of construction is that nodes
 * within the XML document, such as comments that may not have any
 * associated Tool member variable, are preserved.
 *
 * 3) A call to generateXMLDocument().
 * This method generates an XML document for the Tool from scratch.
 * Only the essential document nodes are created (that is, nodes that
 * correspond directly to member variables.).
 *
 * @param aTool Object to be copied.
 * @see Tool(const XMLDocument *aDocument)
 * @see Tool(const char *aFileName)
 * @see generateXMLDocument()
 */
IKTool::
IKTool(const IKTool &aTool) :
	SimulationTool(aTool),
	_markerSetProp(PropertyObj("", MarkerSet())),
	_markerSet((MarkerSet&)_markerSetProp.getValueObj()),
	_coordinateSetProp(PropertyObj("", CoordinateSet())),
	_coordinateSet((CoordinateSet&)_coordinateSetProp.getValueObj()),
	_coordinatesFromFile(_coordinatesFromFileProp.getValueStrArray()),
	_IKTrialSetProp(PropertyObj("", SimmIKTrialSet())),
	_IKTrialSet((SimmIKTrialSet&)_IKTrialSetProp.getValueObj())
{
	setType("IKTool");
	setNull();
	*this = aTool;
}

//_____________________________________________________________________________
/**
 * Virtual copy constructor.
 */
Object* IKTool::
copy() const
{
	IKTool *object = new IKTool(*this);
	return(object);
}

//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void IKTool::
setNull()
{
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void IKTool::setupProperties()
{
	string comment;

	_markerSetProp.setComment("Marker set to be used for solving the IK trials.");
	_markerSetProp.setName("MarkerSet");
	_propertySet.append(&_markerSetProp);

	_coordinateSetProp.setComment("Specifies weights for matching coordinates specified "
		"in a file (the coordinate_file)");
	_coordinateSetProp.setName("CoordinateSet");
	_propertySet.append(&_coordinateSetProp);

	_coordinatesFromFileProp.setComment("List specifying which coordinate values "
		"should be taken from the coordinate_file.");
	_coordinatesFromFileProp.setName("coordinates_from_file");
	Array<string> def("");
	_coordinatesFromFileProp.setValue(def);
	_propertySet.append(&_coordinatesFromFileProp);

	_IKTrialSetProp.setComment("Parameters for solving the IK problem for each trial. "
		"Each trial should get a seperate SimmIKTril block.");
	_IKTrialSetProp.setName("SimmIKTrialSet");
	_propertySet.append(&_IKTrialSetProp);
}
//_____________________________________________________________________________
/**
 * Register SimmIKTrial type.
 */
void IKTool::registerTypes()
{
	Object::RegisterType(IKTool());
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
IKTool& IKTool::
operator=(const IKTool &aTool)
{
	// BASE CLASS
	SimulationTool::operator=(aTool);

	// MEMBER VARIABLES
	_markerSet = aTool._markerSet;
	_coordinateSet = aTool._coordinateSet;
	_coordinatesFromFile = aTool._coordinatesFromFile;
	_IKTrialSet = aTool._IKTrialSet;

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
void IKTool::run()
{
	cout<<"Running investigation "<<getName()<<".\n";
	
	// Do the maneuver to change then restore working directory 
	// so that the parsing code behaves properly if called from a different directory
	string saveWorkingDirectory = IO::getCwd();
	string directoryOfSetupFile = IO::getParentDirectory(getDocumentFileName());
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

