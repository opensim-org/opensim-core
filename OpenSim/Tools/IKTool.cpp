// IKTool.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//=============================================================================
// INCLUDES
//=============================================================================
#include "IKTool.h"
#include <string>
#include <iostream>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include "IKTaskSet.h"
#include "IKTrialSet.h"
#include <OpenSim/Simulation/Model/Model.h>
#include "IKSolverImpl.h"
#include "IKTarget.h"

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
	AbstractTool(),
	_ikTaskSetProp(PropertyObj("", IKTaskSet())),
	_ikTaskSet((IKTaskSet&)_ikTaskSetProp.getValueObj()),
	_IKTrialSetProp(PropertyObj("", IKTrialSet())),
	_IKTrialSet((IKTrialSet&)_IKTrialSetProp.getValueObj())
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
IKTool::IKTool(const string &aFileName, Model* guiModel) :
	AbstractTool(aFileName, guiModel),
	_ikTaskSetProp(PropertyObj("", IKTaskSet())),
	_ikTaskSet((IKTaskSet&)_ikTaskSetProp.getValueObj()),
	_IKTrialSetProp(PropertyObj("", IKTrialSet())),
	_IKTrialSet((IKTrialSet&)_IKTrialSetProp.getValueObj())
{
	setType("IKTool");
	setNull();
	string saveWorkingDirectory = IO::getCwd();
	string directoryOfSetupFile = IO::getParentDirectory(aFileName);
	IO::chDir(directoryOfSetupFile);

	updateFromXMLNode();

#if 0
	if (_model) 
		throw( Exception("IKTool did not expect initialized model ("+_modelLibraryProp.getName()+
							  " property should not be set)",__FILE__,__LINE__) );
#endif
	if (guiModel){
		// A valid model is passed in, and is initialized (likely from GUI)
		// In this scenario, _modelFile is ignored (probably with a warning)
		setModel(guiModel);
		addAnalysisSetToModel();
	}
	else {
#if 0
		if (_modelFile == "") 
			throw( Exception("Model file not specified for inverse kinematics investigation",__FILE__,__LINE__) );
		setModel(new Model(_modelFile));
#endif
		if (_model) {
			if(!_model->hasDynamicsEngine()) 
				throw( Exception("No dynamics engine found for model.  Make sure the OpenSim model specified in the "+
									  _modelFileProp.getName()+" property (currently '"+_modelFile+"') is the model containing the "+
									  "SimmKinematicsEngine (and not the SD/Fast-based model generated using makeSDFastModel)",__FILE__,__LINE__) );
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
	AbstractTool(aTool),
	_ikTaskSetProp(PropertyObj("", IKTaskSet())),
	_ikTaskSet((IKTaskSet&)_ikTaskSetProp.getValueObj()),
	_IKTrialSetProp(PropertyObj("", IKTrialSet())),
	_IKTrialSet((IKTrialSet&)_IKTrialSetProp.getValueObj())
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
	_ikTaskSetProp.setComment("Task set used to specify IK weights.");
	_ikTaskSetProp.setName("IKTaskSet");
	_propertySet.append(&_ikTaskSetProp);

	_IKTrialSetProp.setComment("Parameters for solving the IK problem for each trial. "
		"Each trial should get a seperate SimmIKTril block.");
	_IKTrialSetProp.setName("IKTrialSet");
	_propertySet.append(&_IKTrialSetProp);
}
//_____________________________________________________________________________
/**
 * Register IKTrial type.
 */
void IKTool::registerTypes()
{
	Object::RegisterType(IKTool());
	Object::RegisterType(IKTrial());
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
	AbstractTool::operator=(aTool);

	// MEMBER VARIABLES
	_ikTaskSet = aTool._ikTaskSet;
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

	/* Now perform the IK trials on the updated model. */
	for (int i = 0; i < _IKTrialSet.getSize(); i++)
	{
		if (_IKTrialSet.get(i)->processTrial(*_model, _ikTaskSet))
			cout << "Trial " << _IKTrialSet.get(i)->getName() << " processed successfully." << endl;
		else
			cout << "Trial " << _IKTrialSet.get(i)->getName() << " processing failed." << endl;
	}

	IO::chDir(saveWorkingDirectory);
}

