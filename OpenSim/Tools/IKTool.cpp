// IKTool.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Copyright (c)  2006 Stanford University
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
	_IKTrialSet((IKTrialSet&)_IKTrialSetProp.getValueObj()),
	_optimizerAlgorithm(_optimizerAlgorithmProp.getValueStr())
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
IKTool::IKTool(const string &aFileName, bool aLoadModel) :
	AbstractTool(aFileName, false),
	_ikTaskSetProp(PropertyObj("", IKTaskSet())),
	_ikTaskSet((IKTaskSet&)_ikTaskSetProp.getValueObj()),
	_IKTrialSetProp(PropertyObj("", IKTrialSet())),
	_IKTrialSet((IKTrialSet&)_IKTrialSetProp.getValueObj()),
	_optimizerAlgorithm(_optimizerAlgorithmProp.getValueStr())
{
	setType("IKTool");
	setNull();
	updateFromXMLNode();

	if(aLoadModel) {
		loadModel(aFileName);
		if (_model && !_model->hasDynamicsEngine()) 
			throw( Exception("No dynamics engine found for model.  Make sure the OpenSim model specified in the "+
								  _modelFileProp.getName()+" property (currently '"+_modelFile+"') is the model containing the "+
								  "SimmKinematicsEngine (and not the SD/Fast-based model generated using makeSDFastModel)",__FILE__,__LINE__) );
	}
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
	_IKTrialSet((IKTrialSet&)_IKTrialSetProp.getValueObj()),
	_optimizerAlgorithm(_optimizerAlgorithmProp.getValueStr())
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

	_optimizerAlgorithm = "ipopt";
	_printResultFiles = true;
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

	_optimizerAlgorithmProp.setComment("Preferred optimizer algorithm (currently support \"ipopt\" or \"cfsqp\", "
		"the latter requiring the osimFSQP library.");
	_optimizerAlgorithmProp.setName("optimizer_algorithm");
	_propertySet.append( &_optimizerAlgorithmProp );
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
	_optimizerAlgorithm = aTool._optimizerAlgorithm;
	_printResultFiles = aTool._printResultFiles;

	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================


//=============================================================================
// RUN
//=============================================================================
bool IKTool::initializeTrial(int i) 
{
	bool success = true;

	// Do the maneuver to change then restore working directory 
	// so that the parsing code behaves properly if called from a different directory
	string saveWorkingDirectory = IO::getCwd();
	string directoryOfSetupFile = IO::getParentDirectory(getDocumentFileName());
	IO::chDir(directoryOfSetupFile);

	_IKTrialSet.get(i)->setOptimizerAlgorithm(_optimizerAlgorithm);
	_IKTrialSet.get(i)->setPrintResultFiles(_printResultFiles);
	if(!_IKTrialSet.get(i)->initializeTrial(*_model, _ikTaskSet)) {
		success = false;
		cout << "Trial " << _IKTrialSet.get(i)->getName() << " initialization failed." << endl;
	}

	IO::chDir(saveWorkingDirectory);

	return success;
}
bool IKTool::solveTrial(int i) 
{
	// Do the maneuver to change then restore working directory 
	// so that the parsing code behaves properly if called from a different directory
	string saveWorkingDirectory = IO::getCwd();
	string directoryOfSetupFile = IO::getParentDirectory(getDocumentFileName());
	IO::chDir(directoryOfSetupFile);

	bool result = _IKTrialSet.get(i)->solveTrial(*_model, _ikTaskSet);
	if(result)
		cout << "Trial " << _IKTrialSet.get(i)->getName() << " processed successfully." << endl;
	else
		cout << "Trial " << _IKTrialSet.get(i)->getName() << " processing failed." << endl;

	IO::chDir(saveWorkingDirectory);

	return result;
}
//_____________________________________________________________________________
/**
 * Run the investigation.
 */
bool IKTool::run()
{
	cout<<"Running tool "<<getName()<<".\n";

	bool success = true;

	/* Now perform the IK trials on the updated model. */
	for (int i = 0; i < _IKTrialSet.getSize(); i++)
	{
		if(!initializeTrial(i) || !solveTrial(i)) success = false;
	}

	return success;
}
