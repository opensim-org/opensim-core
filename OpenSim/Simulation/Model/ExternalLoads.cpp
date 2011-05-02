// ExternalLoads.cpp
// Authors:Ajay Seth
/*
 * Copyright (c)  2011, Stanford University. All rights reserved. 
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
#include "ExternalLoads.h"
#include "Model.h"
#include "BodySet.h"
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Common/XMLNode.h>

using namespace std;
using namespace OpenSim;
using SimTK::Vec3;
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
ExternalLoads::~ExternalLoads()
{
	_storages.clear();
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ExternalLoads::ExternalLoads():
_dataFileName(_dataFileNameProp.getValueStr()),
_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl())
{
	setNull();
}

ExternalLoads::ExternalLoads(Model& model) : 
	ModelComponentSet<ExternalForce>(model),
	_dataFileName(_dataFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl())
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Construct an actuator set from file.
 *
 * @param aFileName Name of the file.
 */
ExternalLoads::ExternalLoads(Model& model, const std::string &aFileName, bool aUpdateFromXMLNode) :
	ModelComponentSet<ExternalForce>(model, aFileName, false),
	_dataFileName(_dataFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl())
{
	setNull();

	if(aUpdateFromXMLNode)
		updateFromXMLNode();
}


//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param otherExternalLoads ExternalLoads to be copied.
 */
ExternalLoads::ExternalLoads(const ExternalLoads &otherExternalLoads) :
	ModelComponentSet<ExternalForce>(otherExternalLoads),
	_dataFileName(_dataFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl())
{
	setNull();

	// Class Members
	copyData(otherExternalLoads);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this ExternalLoads to their null values.
 */
void ExternalLoads::setNull()
{
	// TYPE
	setType("ExternalLoads");
	// NAME
	//setName("ExternalLoads");

	// PROPERTIES
	setupSerializedMembers();
	_storages.clear();
}

//_____________________________________________________________________________
/**
 * Copy this ExternalLoads and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this ExternalLoads.
 */
Object* ExternalLoads::copy() const
{
	ExternalLoads *actSet = new ExternalLoads(*this);
	return(actSet);
}

//_____________________________________________________________________________
/**
 * Copy the member variables of the ExternalLoads.
 *
 * @param otherExternalLoads actuator set to be copied
 */
void ExternalLoads::copyData(const ExternalLoads &aAbsExternalLoads)
{
    // ACTUATORS
	_dataFileName = aAbsExternalLoads._dataFileName;
	_externalLoadsModelKinematicsFileName = aAbsExternalLoads._externalLoadsModelKinematicsFileName;
	_lowpassCutoffFrequencyForLoadKinematics = aAbsExternalLoads._lowpassCutoffFrequencyForLoadKinematics;
}

//_____________________________________________________________________________
/**
 * Set up the serialized member variables.
 */
void ExternalLoads::setupSerializedMembers()
{
	string comment;
	_dataFileNameProp.setName("datafile");
	_dataFileName="";
	comment =	"Storage file (.sto) containing (3) components of force and/or torque and point of application."
				"Note: this file overrides the data source specified by the individual external forces if specified.";
	_dataFileNameProp.setComment(comment);
	_propertySet.append(&_dataFileNameProp);

	_externalLoadsModelKinematicsFileName="";
	comment =	"Optional motion file (.mot) or storage file (.sto) containing the model kinematics "
				"used to transform a point expressed in ground to the body of force application."
				"If the point is not expressed in ground, the point is not transformed";
	_externalLoadsModelKinematicsFileNameProp.setComment(comment);
	_externalLoadsModelKinematicsFileNameProp.setName("external_loads_model_kinematics_file");
	_propertySet.append( &_externalLoadsModelKinematicsFileNameProp );

	comment = "Optional low-pass cut-off frequency for filtering the model kinematics corresponding "
			  "used to transform the point of application. A negative value results in no filtering. "
			  "The default value is -1.0, so no filtering.";
	_lowpassCutoffFrequencyForLoadKinematicsProp.setComment(comment);
	_lowpassCutoffFrequencyForLoadKinematicsProp.setName("lowpass_cutoff_frequency_for_load_kinematics");
	_propertySet.append( &_lowpassCutoffFrequencyForLoadKinematicsProp );
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
ExternalLoads& ExternalLoads::operator=(const ExternalLoads &otherExternalLoads)
{
	// BASE CLASS
	ModelComponentSet<ExternalForce>::operator=(otherExternalLoads);

	// Class Members
	copyData(otherExternalLoads);

	return(*this);
}

void ExternalLoads::setup(Model& aModel)
{

	Storage *forceData = new Storage(_dataFileName);

	for(int i=0; i<getSize(); ++i)
		get(i).setDataSource(forceData);

	// BASE CLASS
	ModelComponentSet<ExternalForce>::setup(aModel);

	// add loaded storage into list of storages for later garbage collection
	_storages.push_back(forceData);
}

//-----------------------------------------------------------------------------
// RE-EXPRESS POINT DATA 
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Re-express the position of the point of application for all ExternalForces
 * in this collection of ExternalLoads, such that all points expressed in
 * ground are transformed (re-expressed) in the appliedToBody of the External-
 * Force. The pointExpressedInBody is correspondingly updated to be the
 * appliedToBody as well.
 * Note: If the point is not expressed in ground to begin with, it is not
 * re-expressed. If the ExternalForce does not specify a point of application
 * (body force or torque) it remains untouched.
 *
 * @param kinematics Storage containing the time history of generalized
 * coordinates for the model. Note that all generalized coordinates must
 * be specified and in radians and Euler parameters.
 */
void ExternalLoads::transformPointsExpressedInGroundToAppliedBodies(const Storage &kinematics, double startTime, double endTime)
{
	for(int i=0; i<getSize(); i++){
		ExternalForce *transformedExf = transformPointExpressedInGroundToAppliedBody(get(i), kinematics, startTime, endTime);
		if(transformedExf){
			// replace the force
			set(i, transformedExf);
		}
	}
}

ExternalForce* ExternalLoads::transformPointExpressedInGroundToAppliedBody(const ExternalForce &exForce, const Storage &kinematics, double startTime, double endTime)
{
	if(!_model || !_model->isValidSystem()) // no model and no system underneath, cannot proceed
		throw Exception("ExternalLoads::transformPointExpressedInGroundToAppliedBody() requires a model with a valid system."); 

	if(!exForce._specifiesPoint){ // The external force does not apply a force to a point
		cout << "ExternalLoads: WARNING ExternalForce '"<< exForce.getName() <<"' does not specify a point of application." << endl;
		return NULL;
	}

	if(exForce.getPointExpressedInBodyName() != _model->getGroundBody().getName()){
		cout << "ExternalLoads: WARNING ExternalForce '"<< exForce.getName() <<"' is not expressed in ground and will not be transformed." << endl;
		return NULL;
	}

	if(exForce.getAppliedToBodyName() == _model->getGroundBody().getName()){
		cout << "ExternalLoads: WARNING ExternalForce '"<< exForce.getName() <<"' is applied to a point on ground and will not be transformed." << endl;
		return NULL;
	}


	int nq = _model->getNumCoordinates();
	int nt = kinematics.getSize();

	int startIndex=0;
	int lastIndex=nt;

	if (nt > 0){
		if (startTime!= -SimTK::Infinity){	// Start time was actually specified.
			startIndex = kinematics.findIndex(startTime); 
		}
		if (endTime!= SimTK::Infinity){	// Start time was actually specified.
			lastIndex = kinematics.findIndex(endTime);
		}
	}
	else{
		cout << "ExternalLoads: WARNING specified load kinematics contains no coordinate values. " 
			<< "Point of force application cannot be transformed." << endl;
		return NULL;
	}

	nt = lastIndex-startIndex+1;

	// Construct a new storage to conatan the re-expressed point data for the new external force
	Storage *newDataSource = new Storage(nt);
	Array<string> labels;
	labels.append("time");

	labels.append(exForce._forceIdentifier + ".x");
	labels.append(exForce._forceIdentifier + ".y");
	labels.append(exForce._forceIdentifier + ".z");
	labels.append(exForce._pointIdentifier + ".x");
	labels.append(exForce._pointIdentifier + ".y");
	labels.append(exForce._pointIdentifier + ".z");
	if(exForce._appliesTorque){
		labels.append(exForce._torqueIdentifier + ".x");
		labels.append(exForce._torqueIdentifier + ".y");
		labels.append(exForce._torqueIdentifier + ".z");
	}

	newDataSource->setColumnLabels(labels);
	int ncols = labels.getSize()-1; // time treated separately when appended to storage

	SimTK::Vector datarow(ncols, SimTK::NaN);

	double time = 0;
	Array<double> Q(0.0,nq);

	Vec3 pGround(SimTK::NaN);
	Vec3 pAppliedBody(SimTK::NaN);
	Vec3 force(SimTK::NaN); 
	Vec3 torque(SimTK::NaN);
	
	// Checked that we had a model with a valid system, so get a writable state from it defaults
	SimTK::State &s = _model->updMultibodySystem().updDefaultState();

	// get from (ground) and to (applied) bodies 
	const Body &ground = _model->getGroundBody();
	const Body &appliedToBody = _model->getBodySet().get(exForce.getAppliedToBodyName());

	for(int i=startIndex; i<=lastIndex; ++i) {
		// transform data on an instant-by-instant basis
		kinematics.getTime(i, time);
		kinematics.getData(i, nq, &Q[0]);

		// Set the coordinates values in the state in order to position the model according to specified kinematics
		for (int j = 0; j < nq; j++) {
			Coordinate& coord = _model->getCoordinateSet().get(j);
			coord.setValue(s, Q[j], j==nq-1);
		}

		// get forcce data
		force = exForce.getForceAtTime(time);
		if(exForce._appliesTorque)
			torque = exForce.getTorqueAtTime(time);
		
		// get the untransformed point expressed in ground in the ExternalForce specified in  ground (check made above)
		pGround = exForce.getPointAtTime(time);
		_model->getSimbodyEngine().transformPosition(s, ground, pGround, appliedToBody, pAppliedBody);

		// populate the force data for this instant in time
		for(int j =0; j<3; ++j){
			datarow[j] = force[j];
			datarow[j+3] = pAppliedBody[j];
			if(exForce._appliesTorque)
				datarow[j+6] = torque[j];
		}

		newDataSource->append(time, datarow); 
	}

	// assign a name to the new data source
	newDataSource->setName(exForce._dataSourceName + "_transformedP");

	ExternalForce *exF_transformedPoint = (ExternalForce *)exForce.copy();
	exF_transformedPoint->setName(exForce.getName()+"_transformedP");
	exF_transformedPoint->setPointExpressedInBodyName(exForce.getAppliedToBodyName());
	exF_transformedPoint->setDataSource(newDataSource);

	_storages.push_back(newDataSource);

	newDataSource->print("NewDataSource_TransformedP.sto");

	return exF_transformedPoint;
}

//-----------------------------------------------------------------------------
// UPDATE FROM OLDER VERSION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void ExternalLoads::updateFromXMLNode()
{
	int documentVersion = getDocument()->getDocumentVersion();
	if ( documentVersion < XMLDocument::getLatestVersion()){
		DOMDocument* document = _node->getOwnerDocument();
		if (Object::getDebugLevel()>=1)
			cout << "Updating ExternalLoad object to latest format..." << endl;
		if (_node!=NULL){
			string objectName = XMLNode::TranscodeAndTrim(_node->getTagName());
			DOMElement* listOfForces = XMLNode::GetFirstChildElementByTagName(_node,"objects");
			string listName = XMLNode::TranscodeAndTrim(listOfForces->getTagName());
			DOMNodeList *list = listOfForces->getChildNodes();
			unsigned int listLength = list->getLength();
			DOMElement *objElmt = (DOMElement*) list->item(4);
			string elmtName = XMLNode::TranscodeAndTrim(objElmt->getTagName());
			DOMElement* pf = XMLNode::GetFirstChildElementByTagName(objElmt, "PrescribedForce");
			if (pf) {
				DOMElement* appliedToBody = XMLNode::GetFirstChildElementByTagName(pf, "body");
				// update the properties of a PrescribeForce to match an ExternalForce
				renameChildNode("body", "applied_to_body", pf);
				// ...
			}
			renameChildNode("PrescribedForce", "ExternalForce", pf);
			
		}
	}
	// Call base class now assuming _node has been corrected for current version
	ModelComponentSet<ExternalForce>::updateFromXMLNode();
}