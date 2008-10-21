// AnalysisTemplate.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
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
#include <iostream>
#include <string>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Simulation/Model/DerivCallbackSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include "AnalysisTemplate.h"



using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTANTS
//=============================================================================



//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 * Delete any variables allocated using the "new" operator.  You will not
 * necessarily have any of these.
 */
AnalysisTemplate::~AnalysisTemplate()
{

}
//_____________________________________________________________________________
/**
 * Construct an AnalysisTemplate instance.
 *
 * @param aModel Model for which the analysis is to be run.
 */
AnalysisTemplate::AnalysisTemplate(Model *aModel) :
	Analysis(aModel),
	_bool(_boolProp.getValueBool()),
	_boolArray(_boolArrayProp.getValueBoolArray()),
	_int(_intProp.getValueInt()),
	_intArray(_intArrayProp.getValueIntArray()),
	_dbl(_dblProp.getValueDbl()),
	_dblArray(_dblArrayProp.getValueDblArray()),
	_vec3(_vec3Prop.getValueDblVec3()),
	_str(_strProp.getValueStr()),
	_strArray(_strArrayProp.getValueStrArray())
{
	setNull();
	if(_model==NULL) return;

	// DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();

}
//_____________________________________________________________________________
/**
 * Construct an object from file.
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
AnalysisTemplate::AnalysisTemplate(const std::string &aFileName):
	Analysis(aFileName, false),
	_bool(_boolProp.getValueBool()),
	_boolArray(_boolArrayProp.getValueBoolArray()),
	_int(_intProp.getValueInt()),
	_intArray(_intArrayProp.getValueIntArray()),
	_dbl(_dblProp.getValueDbl()),
	_dblArray(_dblArrayProp.getValueDblArray()),
	_vec3(_vec3Prop.getValueDblVec3()),
	_str(_strProp.getValueStr()),
	_strArray(_strArrayProp.getValueStrArray())
{
	setNull();

	// Serialize from XML
	updateFromXMLNode();

	/* The rest will be done by setModel().
	// CONSTRUCT DESCRIPTION AND LABELS
	constructDescription();
	updateBodiesToRecord();
	constructColumnLabels();

	// STORAGE
	allocateStorage();
	*/
}

// Copy constrctor and virtual copy 
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 */
AnalysisTemplate::AnalysisTemplate(const AnalysisTemplate &aAnalysisTemplate):
	Analysis(aAnalysisTemplate),
	_bool(_boolProp.getValueBool()),
	_boolArray(_boolArrayProp.getValueBoolArray()),
	_int(_intProp.getValueInt()),
	_intArray(_intArrayProp.getValueIntArray()),
	_dbl(_dblProp.getValueDbl()),
	_dblArray(_dblArrayProp.getValueDblArray()),
	_vec3(_vec3Prop.getValueDblVec3()),
	_str(_strProp.getValueStr()),
	_strArray(_strArrayProp.getValueStrArray())
{
	setNull();
	// COPY TYPE AND NAME
	*this = aAnalysisTemplate;
}
//_____________________________________________________________________________
/**
 * Clone
 *
 */
Object* AnalysisTemplate::copy() const
{
	AnalysisTemplate *object = new AnalysisTemplate(*this);
	return(object);

}
//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assign this object to the values of another.
 *
 * @return Reference to this object.
 */
AnalysisTemplate& AnalysisTemplate::
operator=(const AnalysisTemplate &aAnalysisTemplate)
{
	// Base Class
	Analysis::operator=(aAnalysisTemplate);

	// Member Variables
	_bool = aAnalysisTemplate._bool;

	return(*this);
}

//_____________________________________________________________________________
/**
 * SetNull().
 */
void AnalysisTemplate::
setNull()
{
	setType("AnalysisTemplate");
	setupProperties();

	// Property Default Values
	_bool = true;
	_boolArray.setSize(0);
	_int = 0;
	_intArray.setSize(0);
	_dbl = 0.0;
	_dblArray.setSize(0);
	_vec3[0] = _vec3[1] = _vec3[2] = 0.0;
	_str = "";
	_strArray.setSize(0);


}
//_____________________________________________________________________________
/**
 * Set up the properties for your analysis.
 *
 * You should give each property a meaningful name and an informative comment.
 * The name you give each property is the tag that will be used in the XML
 * file.  The comment will appear before the property in the XML file.
 * In addition, the comments are used for tool tips in the OpenSim GUI.
 *
 * All properties are added to the property set.  Once added, they can be
 * read in and written to file.
 */
void AnalysisTemplate::
setupProperties()
{
	_boolProp.setName("express_in_local_frame");
	_boolProp.setComment("Flag indicating whether or not to ...");
	_propertySet.append(&_boolProp);

	_boolArrayProp.setName("bool_array_param");
	_boolArrayProp.setComment("Array of flags indicating whether or not to ...");
	_propertySet.append(&_boolArrayProp);


	_intProp.setName("maximum_number_of_iterations");
	_intProp.setComment("Maximum number of interations ...");
	_propertySet.append(&_intProp);

	_intArrayProp.setName("maximum_number_of_iterations_for_each_body");
	_intArrayProp.setComment("Array specifying ...");
	_propertySet.append(&_intArrayProp);


	_dblProp.setName("convergence_criterion");
	_dblProp.setComment("Convergence criterion used for...");
	_propertySet.append(&_dblProp);

	_dblArrayProp.setName("dbl_array_param");
	_dblArrayProp.setComment("Array specifying ...");
	_propertySet.append(&_dblArrayProp);

	_vec3Prop.setName("body_point");
	_vec3Prop.setComment("Point for which to record the kinematics.");
	_propertySet.append(&_vec3Prop);


	_strProp.setName("body_name");
	_strProp.setComment("Name of the body for which to perform the analysis.");
	_propertySet.append(&_strProp);

	_strArrayProp.setName("body_names");
	_strArrayProp.setComment("Names of the bodies on which to perform the analysis."
		"The key word 'All' indicates that the analysis should be performed for all bodies.");
	_propertySet.append(&_strArrayProp);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct a description for the body kinematics files.
 */
void AnalysisTemplate::
constructDescription()
{
	string descrip;

	descrip = "\nThis file contains the kinematics positions and orientations,\n";
	descrip += "velocities and angular velocities, or accelerations and ";
	descrip += "angular accelerations)\n";
	descrip += "\nUnits are S.I. units (seconds, meters, Newtons, ...)";
	if(getInDegrees()) {
		descrip += "\nAngles are in degrees.";
	} else {
		descrip += "\nAngles are in radians.";
	}
	descrip += "\n\n";

	setDescription(descrip);
}

//_____________________________________________________________________________
/**
 * Construct column labels for the output results.
 *
 * For analyses that run during a simulation, the first column is almost
 * always time.  For the purpose of example, the code below adds labels
 * appropriate for recording the translation and orientation of each
 * body in the model.
 *
 * This method needs to be called as necessary to update the column labels.
 */
void AnalysisTemplate::
constructColumnLabels()
{
	if(_model==NULL) return;

	Array<string> labels;
	labels.append("time");

	BodySet *bodySet = _model->getDynamicsEngine().getBodySet();
	int numBodies = bodySet->getSize();
	for(int i=0; i<numBodies; i++) {
		AbstractBody *body = bodySet->get(i);
		labels.append(body->getName() + "_X");
		labels.append(body->getName() + "_Y");
		labels.append(body->getName() + "_Z");
		labels.append(body->getName() + "_Ox");
		labels.append(body->getName() + "_Oy");
		labels.append(body->getName() + "_Oz");
	}

	setColumnLabels(labels);
}

//_____________________________________________________________________________
/**
 * Set up storage objects.
 *
 * In general, the storage objects in your analysis are used to record
 * the results of your analysis and write them to file.  You will often
 * have a number of storage objects, each for recording a different
 * kind of result.
 */
void AnalysisTemplate::
setupStorage()
{
	// Positions
	_storePos.reset(0);
	_storePos.setName("Positions");
	_storePos.setDescription(getDescription());
	_storePos.setColumnLabels(getColumnLabels());

	// Velocities
	_storeVel.reset(0);
	_storeVel.setName("Velocities");
	_storeVel.setDescription(getDescription());
	_storeVel.setColumnLabels(getColumnLabels());

	// Accelerations
	_storeAcc.reset(0);
	_storeAcc.setName("Accelerations");
	_storeAcc.setDescription(getDescription());
	_storeAcc.setColumnLabels(getColumnLabels());
}


//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the model for which this analysis is to be run.
 *
 * Sometimes the model on which an analysis should be run is not available
 * at the time an analysis is created.  Or, you might want to change the
 * model.  This method is used to set the model on which the analysis is
 * to be run.
 *
 * @param aModel Model pointer
 */
void AnalysisTemplate::
setModel(Model *aModel)
{
	// SET THE MODEL IN THE BASE CLASS
	Analysis::setModel(aModel);

	// UPDATE VARIABLES IN THIS CLASS
	constructDescription();
	constructColumnLabels();
	setupStorage();
	_dydt.setSize(_model->getNumStates());
	int numBodies = _model->getNumBodies();
	_kin.setSize(6*numBodies);
}


//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute and record the results.
 *
 * This method, for the purpose of example, records the position and
 * orientation of each body in the model.  You will need to customize it
 * to perform your analysis.
 *
 * @param aT Current time in the simulation.
 * @param aX Current values of the controls.
 * @param aY Current values of the states.
 */
int AnalysisTemplate::
record(double aT,double *aX,double *aY)
{
	// GET THE MODEL READY ----------------------------------
	// Set the configuration of the model.
	_model->set(aT,aX,aY);
	_model->getDerivCallbackSet()->set(aT,aX,aY);

	// Comput and apply all actuator forces.
	_model->getActuatorSet()->computeActuation();
	_model->getDerivCallbackSet()->computeActuation(aT,aX,aY);
	_model->getActuatorSet()->apply();
	_model->getDerivCallbackSet()->applyActuation(aT,aX,aY);

	// compute and apply all contact forces.
	_model->getContactSet()->computeContact();
	_model->getDerivCallbackSet()->computeContact(aT,aX,aY);
	_model->getContactSet()->apply();
	_model->getDerivCallbackSet()->applyContact(aT,aX,aY);

	// Compute the acclerations.
	int nq = _model->getNumCoordinates();
	_model->getDynamicsEngine().computeDerivatives(&_dydt[0],&_dydt[nq]);
	// -----------------------------------------------------


	// VARIABLES
	double dirCos[3][3];
	SimTK::Vec3 vec,angVec;
	double Mass = 0.0;

	// GROUND BODY
	AbstractBody &ground = _model->getDynamicsEngine().getGroundBody();

	// POSITION
	BodySet *bodySet = _model->getDynamicsEngine().getBodySet();
	int numBodies = bodySet->getSize();
	for(int i=0;i<numBodies;i++) {

		AbstractBody *body = bodySet->get(i);
		SimTK::Vec3 com;
		body->getMassCenter(com);

		// GET POSITIONS AND EULER ANGLES
		_model->getDynamicsEngine().getPosition(*body,com,vec);
		_model->getDynamicsEngine().getDirectionCosines(*body,dirCos);
		_model->getDynamicsEngine().convertDirectionCosinesToAngles(dirCos,
			&angVec[0],&angVec[1],&angVec[2]);

		// CONVERT TO DEGREES?
		if(getInDegrees()) {
			angVec[0] *= SimTK_RADIAN_TO_DEGREE;
			angVec[1] *= SimTK_RADIAN_TO_DEGREE;
			angVec[2] *= SimTK_RADIAN_TO_DEGREE;
		}			

		// FILL KINEMATICS ARRAY
		int I=6*i;
		memcpy(&_kin[I],&vec[0],3*sizeof(double));
		memcpy(&_kin[I+3],&angVec[0],3*sizeof(double));
	}
	_storePos.append(aT,_kin.getSize(),&_kin[0]);

	// VELOCITY
	for(int i=0;i<numBodies;i++) {
		AbstractBody *body = bodySet->get(i);
		SimTK::Vec3 com;
		body->getMassCenter(com);
		// GET VELOCITIES AND ANGULAR VELOCITIES
		_model->getDynamicsEngine().getVelocity(*body,com,vec);
		_model->getDynamicsEngine().getAngularVelocity(*body,angVec);

		// CONVERT TO DEGREES?
		if(getInDegrees()) {
			angVec[0] *= SimTK_RADIAN_TO_DEGREE;
			angVec[1] *= SimTK_RADIAN_TO_DEGREE;
			angVec[2] *= SimTK_RADIAN_TO_DEGREE;
		}			

		// FILL KINEMATICS ARRAY
		int I = 6*i;
		memcpy(&_kin[I],&vec[0],3*sizeof(double));
		memcpy(&_kin[I+3],&angVec[0],3*sizeof(double));
	}
	_storeVel.append(aT,_kin.getSize(),&_kin[0]);

	// ACCELERATIONS
	for(int i=0;i<numBodies;i++) {
		AbstractBody *body = bodySet->get(i);
		SimTK::Vec3 com;
		body->getMassCenter(com);

		// GET ACCELERATIONS AND ANGULAR ACCELERATIONS
		_model->getDynamicsEngine().getAcceleration(*body,com,vec);
		_model->getDynamicsEngine().getAngularAcceleration(*body,angVec);

		// CONVERT TO DEGREES?
		if(getInDegrees()) {
			angVec[0] *= SimTK_RADIAN_TO_DEGREE;
			angVec[1] *= SimTK_RADIAN_TO_DEGREE;
			angVec[2] *= SimTK_RADIAN_TO_DEGREE;
		}			

		// FILL KINEMATICS ARRAY
		int I = 6*i;
		memcpy(&_kin[I],&vec[0],3*sizeof(double));
		memcpy(&_kin[I+3],&angVec[0],3*sizeof(double));
	}
	_storeAcc.append(aT,_kin.getSize(),&_kin[0]);

	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 * This method is meant to be called at the begining of an integration in
 * Model::integBeginCallback() and has the same argument list.
 *
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that will be attempted.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int AnalysisTemplate::
begin(int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,
		void *aClientData)
{
	if(!proceed()) return(0);

	// RESET STORAGE
	_storePos.reset(aT);
	_storeVel.reset(aT);
	_storeAcc.reset(aT);

	// RECORD
	int status = 0;
	if(_storePos.getSize()<=0) {
		status = record(aT,aX,aY);
	}

	return(status);
}
//_____________________________________________________________________________
/**
 * This method is called to perform the analysis.  It can be called during
 * the execution of a forward integrations or after the integration by
 * feeding it the necessary data.
 *
 * When called during an integration, this method is meant to be called in
 * Model::integStepCallback(), which has the same argument list.
 *
 * @param aXPrev Controls at the beginining of the current time step.
 * @param aYPrev States at the beginning of the current time step.
 * @param aYPPrev Pseudo states at the beginning of the current time step.
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that was just taken.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int AnalysisTemplate::
step(double *aXPrev,double *aYPrev,double *aYPPrev,
	int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,
	void *aClientData)
{
	if(!proceed(aStep)) return(0);

	record(aT,aX,aY);

	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the end of an analysis so that any
 * necessary finalizations may be performed.
 *
 * This method is meant to be called at the end of an integration in
 * Model::integEndCallback() and has the same argument list.
 *
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that was just completed.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int AnalysisTemplate::
end(int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,
		void *aClientData)
{
	if(!proceed()) return(0);

	record(aT,aX,aY);

	return(0);
}




//=============================================================================
// IO
//=============================================================================
//_____________________________________________________________________________
/**
 * Print results.
 * 
 * The file names are constructed as
 * aDir + "/" + aBaseName + "_" + ComponentName + aExtension
 *
 * @param aDir Directory in which the results reside.
 * @param aBaseName Base file name.
 * @param aDT Desired time interval between adjacent storage vectors.  Linear
 * interpolation is used to print the data out at the desired interval.
 * @param aExtension File extension.
 *
 * @return 0 on success, -1 on error.
 */
int AnalysisTemplate::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	// ACCELERATIONS
	_storeAcc.scaleTime(_model->getTimeNormConstant());
	Storage::printResult(&_storeAcc,aBaseName+"_"+getName()+"_acc",aDir,aDT,aExtension);

	// VELOCITIES
	_storeVel.scaleTime(_model->getTimeNormConstant());
	Storage::printResult(&_storeVel,aBaseName+"_"+getName()+"_vel",aDir,aDT,aExtension);

	// POSITIONS
	_storePos.scaleTime(_model->getTimeNormConstant());
	Storage::printResult(&_storePos,aBaseName+"_"+getName()+"_pos",aDir,aDT,aExtension);

	return(0);
}


