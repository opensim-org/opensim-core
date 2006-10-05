// InvestigationIK.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//=============================================================================
// INCLUDES
//=============================================================================
#include "InvestigationIK.h"
#include <string>
#include <iostream>
#include <OpenSim/Simulation/SIMM/SimmCoordinateSet.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerSet.h>
#include <OpenSim/Simulation/SIMM/SimmIKTrialParamsSet.h>
#include <OpenSim/Simulation/SIMM/SimmModel.h>
#include <OpenSim/Simulation/SIMM/SimmIKParams.h>
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
	_markerSetProp(PropertyObj("", SimmMarkerSet())),
	_markerSet((SimmMarkerSet&)_markerSetProp.getValueObj()),
	_coordinateSetProp(PropertyObj("", SimmCoordinateSet())),
	_coordinateSet((SimmCoordinateSet&)_coordinateSetProp.getValueObj()),
	_IKTrialParamsSetProp(PropertyObj("", SimmIKTrialParamsSet())),
	_IKTrialParamsSet((SimmIKTrialParamsSet&)_IKTrialParamsSetProp.getValueObj())
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
InvestigationIK::InvestigationIK(const string &aFileName) :
	Investigation(aFileName),
	_markerSetProp(PropertyObj("", SimmMarkerSet())),
	_markerSet((SimmMarkerSet&)_markerSetProp.getValueObj()),
	_coordinateSetProp(PropertyObj("", SimmCoordinateSet())),
	_coordinateSet((SimmCoordinateSet&)_coordinateSetProp.getValueObj()),
	_IKTrialParamsSetProp(PropertyObj("", SimmIKTrialParamsSet())),
	_IKTrialParamsSet((SimmIKTrialParamsSet&)_IKTrialParamsSetProp.getValueObj())
{
	setType("InvestigationIK");
	setNull();
	updateFromXMLNode();

	if (_model) throw( Exception("InvestigationIK did not expect initialized model") );
	if (_modelFile == "") throw( Exception("Model file not specified for inverse kinematics investigation") );
	setModel(new SimmModel(_modelFile));
	if (_model) addAnalysisSetToModel();
}
//_____________________________________________________________________________
/**
 * Construct from a DOMElement.
 */
InvestigationIK::InvestigationIK(DOMElement *aElement) :
	Investigation(aElement),
	_markerSetProp(PropertyObj("", SimmMarkerSet())),
	_markerSet((SimmMarkerSet&)_markerSetProp.getValueObj()),
	_coordinateSetProp(PropertyObj("", SimmCoordinateSet())),
	_coordinateSet((SimmCoordinateSet&)_coordinateSetProp.getValueObj()),
	_IKTrialParamsSetProp(PropertyObj("", SimmIKTrialParamsSet())),
	_IKTrialParamsSet((SimmIKTrialParamsSet&)_IKTrialParamsSetProp.getValueObj())
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
	_markerSetProp(PropertyObj("", SimmMarkerSet())),
	_markerSet((SimmMarkerSet&)_markerSetProp.getValueObj()),
	_coordinateSetProp(PropertyObj("", SimmCoordinateSet())),
	_coordinateSet((SimmCoordinateSet&)_coordinateSetProp.getValueObj()),
	_IKTrialParamsSetProp(PropertyObj("", SimmIKTrialParamsSet())),
	_IKTrialParamsSet((SimmIKTrialParamsSet&)_IKTrialParamsSetProp.getValueObj())
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

	_markerSetProp.setName("SimmMarkerSet");
	_markerSetProp.setComment("Markers to be used by all IK trials");
	_propertySet.append(&_markerSetProp);

	_coordinateSetProp.setName("SimmCoordinateSet");
	_coordinateSetProp.setComment("Specify how to initialize coodinates for IK. Use value 'fromFile' to force IK to use a file to set the initial values. Filename is specified in the appropriate trial block.");
	_propertySet.append(&_coordinateSetProp);

	_IKTrialParamsSetProp.setName("SimmIKTrialParamsSet");
	_IKTrialParamsSetProp.setComment("Trial paramaters, one block per trial");
	_propertySet.append(&_IKTrialParamsSetProp);
}
//_____________________________________________________________________________
/**
 * Register SimmIKTrialParams type.
 */
void InvestigationIK::registerTypes()
{
	Object::RegisterType(SimmIKTrialParams());
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
	_IKTrialParamsSet = aInvestigation._IKTrialParamsSet;

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
	
	// Cast to SimmModel so we can call functions that don't exist in base Model class
	SimmModel *simmModel = dynamic_cast<SimmModel*>(getModel());
	simmModel->setup();

	// Update markers to correspond to those specified in IKParams block
	simmModel->updateMarkers(_markerSet);
	// Initialize coordinates based on user input
	simmModel->updateCoordinates(_coordinateSet);
	/* Now perform the IK trials on the updated model. */
	for (int i = 0; i < _IKTrialParamsSet.getSize(); i++)
	{
		// Get trial params
		SimmIKTrialParams& trialParams = *_IKTrialParamsSet.get(i);
		// Handle coordinates file
		SimmMotionData* coordinateValues = trialParams.getCoordinateValues(*simmModel);

		// Setup IK problem for trial
		// We need SimmInverseKinematicsTarget, iksolver (SimmIKSolverImpl)
		// Create SimmMarkerData Object from trc file of experimental motion data
		SimmMarkerData motionTrialData(trialParams.getMarkerDataFilename());
		motionTrialData.convertToUnits(simmModel->getLengthUnits());

		Storage inputStorage;
		// Convert read trc fil into "common" rdStroage format
		motionTrialData.makeRdStorage(inputStorage);
		if (coordinateValues != 0) {
			/* Adjust the user-defined start and end times to make sure they are in the
			* range of the marker data. This must be done so that you only look in the
			* coordinate data for rows that will actually be solved.
			*/
			double firstStateTime = inputStorage.getFirstTime();
			double lastStateTime = inputStorage.getLastTime();
			double startTime = (firstStateTime>trialParams.getStartTime()) ? firstStateTime : trialParams.getStartTime();
			double endTime =  (lastStateTime<trialParams.getEndTime()) ? lastStateTime : trialParams.getEndTime();

			/* Add the coordinate data to the marker data. There must be a row of
			* corresponding coordinate data for every row of marker data that will
			* be solved, or it is a fatal error.
			*/
			coordinateValues->addToRdStorage(inputStorage, startTime, endTime);

			//inputStorage.setWriteSIMMHeader(true);
			//inputStorage.print("preIK.mot");
		}
		// Create target
		SimmInverseKinematicsTarget *target = new SimmInverseKinematicsTarget(*simmModel, inputStorage);
		// Create solver
		SimmIKSolverImpl *ikSolver = new SimmIKSolverImpl(*target);
		// Solve
		Storage	outputStorage;
		ikSolver->solveFrames(trialParams, inputStorage, outputStorage);
		outputStorage.setWriteSIMMHeader(true);
		outputStorage.print(trialParams.getOutputMotionFilename().c_str());

		delete coordinateValues;
		delete ikSolver;
		delete target;
	}
}

