// BodyKinematics.cpp
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
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include "BodyKinematics.h"



using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTANTS
//=============================================================================
#define MAXLEN 10000
#define CENTER_OF_MASS_NAME string("center_of_mass")


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
BodyKinematics::~BodyKinematics()
{
	deleteStorage();
}
//_____________________________________________________________________________
/**
 * Construct an BodyKinematics instance for recording the kinematics of
 * the bodies of a model during a simulation.
 *
 * @param aModel Model for which the analyses are to be recorded.
 */
BodyKinematics::BodyKinematics(Model *aModel, bool aInDegrees) :
	Analysis(aModel),
	_bodies(_bodiesProp.getValueStrArray()),
	_expressInLocalFrame(_expressInLocalFrameProp.getValueBool())
{
	 setNull();

	// STORAGE
	allocateStorage();

	if (_model ==0)
		return;
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
BodyKinematics::BodyKinematics(const std::string &aFileName):
	Analysis(aFileName, false),
	_bodies(_bodiesProp.getValueStrArray()),
	_expressInLocalFrame(_expressInLocalFrameProp.getValueBool())
{
	setNull();

	// Serialize from XML
	updateFromXMLDocument();

}

// Copy constrctor and virtual copy 
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 */
BodyKinematics::BodyKinematics(const BodyKinematics &aBodyKinematics):
	Analysis(aBodyKinematics),
	_bodies(_bodiesProp.getValueStrArray()),
	_expressInLocalFrame(_expressInLocalFrameProp.getValueBool())
{
	setNull();
	// COPY TYPE AND NAME
	*this = aBodyKinematics;
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
BodyKinematics& BodyKinematics::
operator=(const BodyKinematics &aBodyKinematics)
{
	// BASE CLASS
	Analysis::operator=(aBodyKinematics);
	_bodies = aBodyKinematics._bodies;
	_expressInLocalFrame = aBodyKinematics._expressInLocalFrame;
	return(*this);
}

//_____________________________________________________________________________
/**
 * SetNull().
 */
void BodyKinematics::
setNull()
{
	setupProperties();

	// POINTERS
	_pStore = NULL;
	_vStore = NULL;
	_aStore = NULL;
	_bodies.setSize(1);
	_bodies[0] = "all";
	_recordCenterOfMass = true;

	// OTHER VARIABLES

	//?_body
	setName("BodyKinematics");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void BodyKinematics::
setupProperties()
{
	_bodiesProp.setName("bodies");
	_bodiesProp.setComment("Names of bodies to record kinematics for.  Use 'all' to record all bodies."
								  "  The special name '"+CENTER_OF_MASS_NAME+"' refers to the combined center of mass.");
	_propertySet.append( &_bodiesProp );

	_expressInLocalFrameProp.setName("express_results_in_body_local_frame");
	_expressInLocalFrameProp.setComment("Flag (true or false) indicating whether"
		" to express results in the global frame or local-frames of the bodies. "
		"Body positions and center of mass results are always given in the global frame. "
		"This flag is set to false by default.");
	_expressInLocalFrameProp.setValue(false);
	_propertySet.append(&_expressInLocalFrameProp);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct a description for the body kinematics files.
 */
void BodyKinematics::
constructDescription()
{
	char descrip[1024];
	char tmp[MAXLEN];

	strcpy(descrip,"\nThis file contains the kinematics ");
	strcat(descrip,"(positions and orientations,\n");
	strcat(descrip,"velocities and angular velocities, or");
	strcat(descrip," accelerations and angular accelerations)\n");
	strcat(descrip,"of the centers of mass");
	sprintf(tmp," of the body segments in model %s.\n",
		_model->getName().c_str());
	strcat(descrip,tmp);
	strcat(descrip,"\nBody segment orientations are described using");
	strcat(descrip," body-fixed X-Y-Z Euler angles.\n");
	strcat(descrip,"\nAngular velocities and accelerations are given about");
	strcat(descrip," the body-local axes.\n");
	strcat(descrip,"\nUnits are S.I. units (seconds, meters, Newtons, ...)");
	if(getInDegrees()) {
		strcat(descrip,"\nAngles are in degrees.");
	} else {
		strcat(descrip,"\nAngles are in radians.");
	}
	strcat(descrip,"\n\n");

	setDescription(descrip);
}

//_____________________________________________________________________________
/**
 * Construct column labels for the body kinematics files.
 */
void BodyKinematics::
constructColumnLabels()
{
	Array<string> labels;
	labels.append("time");

	BodySet& bs = _model->updBodySet();
	for(int i=0; i<_bodyIndices.getSize(); i++) {
		Body& body = bs.get(_bodyIndices[i]);
		labels.append(body.getName() + "_X");
		labels.append(body.getName() + "_Y");
		labels.append(body.getName() + "_Z");
		labels.append(body.getName() + "_Ox");
		labels.append(body.getName() + "_Oy");
		labels.append(body.getName() + "_Oz");
	}

	if(_recordCenterOfMass) {
		// ADD NAMES FOR POSITION, VELOCITY, AND ACCELERATION OF WHOLE BODY
		labels.append(CENTER_OF_MASS_NAME + "_X");
		labels.append(CENTER_OF_MASS_NAME + "_Y");
		labels.append(CENTER_OF_MASS_NAME + "_Z");
	}

	setColumnLabels(labels);
}

//_____________________________________________________________________________
/**
 * Allocate storage for the kinematics.
 */
void BodyKinematics::
allocateStorage()
{
	// ACCELERATIONS
	_aStore = new Storage(1000,"Accelerations");
	_aStore->setDescription(getDescription());
	_aStore->setColumnLabels(getColumnLabels());

	// VELOCITIES
	_vStore = new Storage(1000,"Velocities");
	_vStore->setDescription(getDescription());
	_vStore->setColumnLabels(getColumnLabels());

	// POSITIONS
	_pStore = new Storage(1000,"Positions");
	_pStore->setDescription(getDescription());
	_pStore->setColumnLabels(getColumnLabels());
}


//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Delete storage objects.
 */
void BodyKinematics::
deleteStorage()
{
	if(_aStore!=NULL) { delete _aStore;  _aStore=NULL; }
	if(_vStore!=NULL) { delete _vStore;  _vStore=NULL; }
	if(_pStore!=NULL) { delete _pStore;  _pStore=NULL; }
}

//_____________________________________________________________________________
/**
 * Update bodies to record
 */
void BodyKinematics::
updateBodiesToRecord()
{
	if(!_model) {
		_bodyIndices.setSize(0);
		_kin.setSize(0);
		return;
	}

	BodySet& bs = _model->updBodySet();
	_recordCenterOfMass = false;
	_bodyIndices.setSize(0);
	for(int i=0; i<_bodies.getSize(); i++) {
		if(_bodies[i] == "all") {
			_bodyIndices.setSize(bs.getSize());
			for(int j=0;j<bs.getSize();j++) _bodyIndices[j]=j;
			_recordCenterOfMass = true;
			break;
		}
		if(_bodies[i] == CENTER_OF_MASS_NAME) {
			_recordCenterOfMass = true;
			continue;
		}
		int index = bs.getIndex(_bodies[i]);
		if(index<0) 
			throw Exception("BodyKinematics: ERR- Cound not find body named '"+_bodies[i]+"'",__FILE__,__LINE__);
		_bodyIndices.append(index);
	}
	_kin.setSize(6*_bodyIndices.getSize()+(_recordCenterOfMass?3:0));

	if(_kin.getSize()==0) cout << "WARNING: BodyKinematics analysis has no bodies to record kinematics for" << endl;
}



//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the model for which the body kinematics are to be computed.
 *
 * @param aModel Model pointer
 */
void BodyKinematics::
setModel(Model& aModel)
{
	Analysis::setModel(aModel);


	// DESCRIPTION AND LABELS
	constructDescription();
	updateBodiesToRecord();
	constructColumnLabels();

	deleteStorage();
	allocateStorage();
}

//-----------------------------------------------------------------------------
// STORAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the acceleration storage.
 *
 * @return Acceleration storage.
 */
Storage* BodyKinematics::
getAccelerationStorage()
{
	return(_aStore);
}
//_____________________________________________________________________________
/**
 * Get the velocity storage.
 *
 * @return Velocity storage.
 */
Storage* BodyKinematics::
getVelocityStorage()
{
	return(_vStore);
}
//_____________________________________________________________________________
/**
 * Get the position storage.
 *
 * @return Position storage.
 */
Storage* BodyKinematics::
getPositionStorage()
{
	return(_pStore);
}

//-----------------------------------------------------------------------------
// STORAGE CAPACITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the capacity increments of all storage instances.
 *
 * @param aIncrement Increment by which storage capacities will be increased
 * when storage capcities run out.
 */
void BodyKinematics::
setStorageCapacityIncrements(int aIncrement)
{
	_aStore->setCapacityIncrement(aIncrement);
	_vStore->setCapacityIncrement(aIncrement);
	_pStore->setCapacityIncrement(aIncrement);
}

//-----------------------------------------------------------------------------
// ANGULAR VELOCITY IN LOCAL FRAME FLAG
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set a flag indicating that the angular velocities should be output in the 
 * the body local reference frame.
 *
 * @param aTrueFalse  False will result in the angular velocities being output
 *		in the global reference frame
 */
void BodyKinematics::
setExpressResultsInLocalFrame(bool aTrueFalse)
{
	_expressInLocalFrame = aTrueFalse;
}

/**
 * Get a flag indicating whether the angular velocities should be output in the 
 * the body local reference frame.
 *
 * @param rTrueFalse  False indicates the angular velocities being output
 *		in the global reference frame
 */
bool BodyKinematics::
getExpressResultsInLocalFrame()
{
	return(_expressInLocalFrame);
}


//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Record the kinematics.
 */
int BodyKinematics::
record(const SimTK::State& s)
{

	// Realize to Acceleration first since we'll ask for Accelerations 
	_model->getMultibodySystem().realize(s, SimTK::Stage::Acceleration);
	// VARIABLES
	double dirCos[3][3];
	SimTK::Vec3 vec,angVec;
	double Mass = 0.0;

	// GROUND BODY
	Body &ground = _model->getSimbodyEngine().getGroundBody();

	// POSITION
	BodySet& bs = _model->updBodySet();

	for(int i=0;i<_bodyIndices.getSize();i++) {
		Body& body = bs.get(_bodyIndices[i]);
		SimTK::Vec3 com;
		body.getMassCenter(com);
		// GET POSITIONS AND EULER ANGLES
		_model->getSimbodyEngine().getPosition(s, body,com,vec);
		_model->getSimbodyEngine().getDirectionCosines(s, body,dirCos);
		_model->getSimbodyEngine().convertDirectionCosinesToAngles(dirCos,
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

	if(_recordCenterOfMass) {
		double rP[3] = { 0.0, 0.0, 0.0 };
		for(int i=0;i<bs.getSize();i++) {
			Body& body = bs.get(i);
			SimTK::Vec3 com;
			body.getMassCenter(com);
			_model->getSimbodyEngine().getPosition(s, body,com,vec);
			// ADD TO WHOLE BODY MASS
			Mass += body.getMass();
			rP[0] += body.getMass() * vec[0];
			rP[1] += body.getMass() * vec[1];
			rP[2] += body.getMass() * vec[2];
		}

		//COMPUTE COM OF WHOLE BODY AND ADD TO ARRAY
		rP[0] /= Mass;
		rP[1] /= Mass;
		rP[2] /= Mass;
		int I = 6*_bodyIndices.getSize();
		memcpy(&_kin[I],rP,3*sizeof(double));
	}
	
	_pStore->append(s.getTime(),_kin.getSize(),&_kin[0]);

	// VELOCITY
	for(int i=0;i<_bodyIndices.getSize();i++) {
		Body& body = bs.get(_bodyIndices[i]);
		SimTK::Vec3 com;
		body.getMassCenter(com);
		// GET VELOCITIES AND ANGULAR VELOCITIES
		_model->getSimbodyEngine().getVelocity(s, body,com,vec);
		if(_expressInLocalFrame) {
			_model->getSimbodyEngine().transform(s, ground,vec,body,vec);
			_model->getSimbodyEngine().getAngularVelocityBodyLocal(s, body,angVec);
		} else {
			_model->getSimbodyEngine().getAngularVelocity(s, body,angVec);
		}

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

	if(_recordCenterOfMass) {
		double rV[3] = { 0.0, 0.0, 0.0 };
		for(int i=0;i<bs.getSize();i++) {
			Body& body = bs.get(i);
			SimTK::Vec3 com;
			body.getMassCenter(com);
			_model->getSimbodyEngine().getVelocity(s, body,com,vec);
			rV[0] += body.getMass() * vec[0];
			rV[1] += body.getMass() * vec[1];
			rV[2] += body.getMass() * vec[2];
		}

		//COMPUTE VELOCITY OF COM OF WHOLE BODY AND ADD TO ARRAY
		rV[0] /= Mass;
		rV[1] /= Mass;
		rV[2] /= Mass;
		int I = 6*_bodyIndices.getSize();
		memcpy(&_kin[I],rV,3*sizeof(double));
	}

	_vStore->append(s.getTime(),_kin.getSize(),&_kin[0]);

	// ACCELERATIONS
	for(int i=0;i<_bodyIndices.getSize();i++) {
		Body& body = bs.get(_bodyIndices[i]);
		SimTK::Vec3 com;
		body.getMassCenter(com);

		// GET ACCELERATIONS AND ANGULAR ACCELERATIONS
		_model->getSimbodyEngine().getAcceleration(s, body,com,vec);
		if(_expressInLocalFrame) {
			_model->getSimbodyEngine().transform(s, ground,vec,body,vec);
			_model->getSimbodyEngine().getAngularAccelerationBodyLocal(s, body,angVec);
		} else {
			_model->getSimbodyEngine().getAngularAcceleration(s, body,angVec);
		}

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

	if(_recordCenterOfMass) {
		double rA[3] = { 0.0, 0.0, 0.0 };
		for(int i=0;i<bs.getSize();i++) {
			Body& body = bs.get(i);
			SimTK::Vec3 com;
			body.getMassCenter(com);
			_model->getSimbodyEngine().getAcceleration(s, body,com,vec);
			rA[0] += body.getMass() * vec[0];
			rA[1] += body.getMass() * vec[1];
			rA[2] += body.getMass() * vec[2];
		}

		//COMPUTE ACCELERATION OF COM OF WHOLE BODY AND ADD TO ARRAY
		rA[0] /= Mass;
		rA[1] /= Mass;
		rA[2] /= Mass;
		int I = 6*_bodyIndices.getSize();
		memcpy(&_kin[I],rA,3*sizeof(double));
	}

	_aStore->append(s.getTime(),_kin.getSize(),&_kin[0]);

	//printf("BodyKinematics:\taT:\t%.16f\trA[1]:\t%.16f\n",s.getTime(),rA[1]);
	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 * This method is meant to be called at the begining of an integration 
 *
 * This method should be overriden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @param s System state
 *
 * @return -1 on error, 0 otherwise.
 */
int BodyKinematics::
begin(SimTK::State& s )
{
	if(!proceed()) return(0);

	// RESET STORAGE
	_pStore->reset(s.getTime());
	_vStore->reset(s.getTime());
	_aStore->reset(s.getTime());

	// RECORD
	int status = 0;
	if(_pStore->getSize()<=0) {
		status = record(s);
	}

	return(status);
}
//_____________________________________________________________________________
/**
 * This method is called to perform the analysis.  It can be called during
 * the execution of a forward integrations or after the integration by
 * feeding it the necessary data.
 *
 * When called during an integration, this method is meant to be called 
 *
 * This method should be overriden in derived classes.  It is
 * included here so that the derived class will not have to implement it if
 * it is not necessary.
 *
 * @param s System state
 *
 * @return -1 on error, 0 otherwise.
 */
int BodyKinematics::
step(const SimTK::State& s, int stepNumber)
{
	if(!proceed(stepNumber )) return(0);

	record(s);

	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the end of an analysis so that any
 * necessary finalizations may be performed.
 *
 * This method is meant to be called at the end of an integration 
 *
 * This method should be overriden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @param s System state
 *
 * @return -1 on error, 0 otherwise.
 */
int BodyKinematics::
end(SimTK::State& s )
{
	if(!proceed()) return(0);

	record(s);

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
int BodyKinematics::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	string suffix;
	if(_expressInLocalFrame) suffix = "_bodyLocal";
	else suffix = "_global";

	// ACCELERATIONS
	Storage::printResult(_aStore,aBaseName+"_"+getName()+"_acc"+suffix,aDir,aDT,aExtension);

	// VELOCITIES
	Storage::printResult(_vStore,aBaseName+"_"+getName()+"_vel"+suffix,aDir,aDT,aExtension);

	// POSITIONS
	Storage::printResult(_pStore,aBaseName+"_"+getName()+"_pos_global",aDir,aDT,aExtension);

	return(0);
}


