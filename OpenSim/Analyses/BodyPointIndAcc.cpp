// BodyPointIndAcc.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson and Saryn Goldberg
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/Model/AbstractBody.h>
#include <OpenSim/Simulation/Model/AbstractActuator.h>
#include "BodyPointIndAcc.h"


//=============================================================================
// DEFINES
//=============================================================================


//=============================================================================
// CONSTANTS
//=============================================================================


using namespace OpenSim;
const int BodyPointIndAcc::NAME_LENGTH = BodyPointIndAcc_NAME_LENGTH;
const int BodyPointIndAcc::BUFFER_LENGTH = BodyPointIndAcc_BUFFER_LENGTH;


//=============================================================================
// GLOBALS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
BodyPointIndAcc::~BodyPointIndAcc()
{
	// STORAGE
	deletePointStorage();
}
//_____________________________________________________________________________
/**
 * Construct an instance for computing the induced accelerations of a point
 * on a segment of a model during a simulation.
 *
 * @param aModel Model on which the analyses are to be performed.
 * @param aBody Body ID.
 * @point aPoint Point value.
 */
BodyPointIndAcc::
BodyPointIndAcc(Model *aModel,AbstractBody *aBody,double aPoint[3])
	: IndAcc(aModel)
{
	// STATES
	setNull();
	setBody(aBody);
	setPoint(aPoint);

	// DESCRIPTION
	constructDescription();

	// STORAGE
	allocatePointStorage();
}
//_____________________________________________________________________________
/**
 * Construct an instance for computing the induced accelerations of a point
 * on a segment of a model from a set of force decomposition files.
 *
 * Note that the induced accelerations are not read in from file.  The
 * induced accelerations are recomputed based on the force decomposition.
 *
 * @param aModel Model on which the analyses were performed.
 * @param aBody Body ID.
 * @point aPoint Point value.
 * @param aStates Set of model states.
 * @param aBaseName Base name for the force decompositon files.
 * @param aDir Directory in which the results reside.
 * @param aExtension File extension.
 * @todo	verify induced accelerations are correct
 * @todo	check that code is correct for generalized force case
 * @todo add initial velocity and ind pos due to init vel and pos to all
 */
BodyPointIndAcc::
BodyPointIndAcc(Model *aModel,AbstractBody *aBody,double aPoint[3],
	Storage *aStates,Storage *aControls,char *aBaseName,char *aDir,char *aExtension) :
	IndAcc(aModel,aStates,aControls,aBaseName,aDir,aExtension)
{
	printf("BodyPointIndAcc: constructing analysis from file.\n");
	printf("baseName = %s  aDir = %s  aExtension= %s\n",
		aBaseName,aDir,aExtension);

	// STATES
	setNull();
	setBody(aBody);
	setPoint(aPoint);

	// DESCRIPTION
	constructDescription();

	// STORAGE
	allocatePointStorage();
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the default or null values for the member variables.
 */
void BodyPointIndAcc::
setNull()
{
	_axPointStore = NULL;
	_ayPointStore = NULL;
	_azPointStore = NULL;
	_vxPointStore = NULL;
	_vyPointStore = NULL;
	_vzPointStore = NULL;
	_pxPointStore = NULL;
	_pyPointStore = NULL;
	_pzPointStore = NULL;
	setName("BodyPointIndAcc");
	setPointName("Unknown");
	_body = NULL;
	_point[0] = _point[1] = _point[2] = 0.0;
}
//_____________________________________________________________________________
/**
 * Construct a description for the body kinematics files.
 */
void BodyPointIndAcc::
constructDescription()
{
	char descrip[BUFFER_LENGTH];
	char tmp[BUFFER_LENGTH];

	strcpy(descrip,"\n");
	sprintf(tmp,"This file contains the accelerations of point (%lf, %lf, %lf)",
		_point[0],_point[1],_point[2]);
	strcat(descrip,tmp);
	sprintf(tmp,"\non body %s induced by the action forces of model %s.\n",
		_body->getName().c_str(),_model->getName().c_str());
	strcat(descrip,tmp);
	strcat(descrip,"\nThe accelerations are in the inertial frame.\n");
	strcat(descrip,"\nUnits are S.I. units (seconds, meters, Newtons, ...)");
	strcat(descrip,"\n\n");

	setDescription(descrip);
}

//_____________________________________________________________________________
/**
 * Construct column labels for the inducded induced accelerations of the
 * body points.
 */
char* BodyPointIndAcc::
getColumnLabels(const char *aTag)
{
	int i;
	strcpy(_buffer,"time");
	for(i=0;i<getNumComponents();i++) {
		sprintf(_tmp,"\t%s_%s",getComponentName(i),aTag);
		strcat(_buffer,_tmp);
	}
	strcat(_buffer,"\n");

	return(_buffer);
}
//_____________________________________________________________________________
/**
 * Allocate storage.
 */
void BodyPointIndAcc::
allocatePointStorage()
{
	// ALLOCATE STORAGE
	// X
	_axPointStore = new Storage(500,"BodyPointInducedAccelerations_X");
	_axPointStore->setCapacityIncrement(1000);
	_axPointStore->setDescription(getDescription());
	_axPointStore->setColumnLabels(getColumnLabels("X"));
	// Y
	_ayPointStore = new Storage(500,"BodyPointInducedAccelerations_Y");
	_ayPointStore->setCapacityIncrement(1000);
	_ayPointStore->setDescription(getDescription());
	_ayPointStore->setColumnLabels(getColumnLabels("Y"));
	// Z
	_azPointStore = new Storage(500,"BodyPointInducedAccelerations_Z");
	_azPointStore->setCapacityIncrement(1000);
	_azPointStore->setDescription(getDescription());
	_azPointStore->setColumnLabels(getColumnLabels("Z"));

}


//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Delete storage objects.
 */
void BodyPointIndAcc::
deletePointStorage()
{
	if(_axPointStore!=NULL) { delete _axPointStore;  _axPointStore=NULL; }
	if(_ayPointStore!=NULL) { delete _ayPointStore;  _ayPointStore=NULL; }
	if(_azPointStore!=NULL) { delete _azPointStore;  _azPointStore=NULL; }

	if(_vxPointStore!=NULL) { delete _vxPointStore;  _vxPointStore=NULL; }
	if(_vyPointStore!=NULL) { delete _vyPointStore;  _vyPointStore=NULL; }
	if(_vzPointStore!=NULL) { delete _vzPointStore;  _vzPointStore=NULL; }

	if(_pxPointStore!=NULL) { delete _pxPointStore;  _pxPointStore=NULL; }
	if(_pyPointStore!=NULL) { delete _pyPointStore;  _pyPointStore=NULL; }
	if(_pzPointStore!=NULL) { delete _pzPointStore;  _pzPointStore=NULL; }
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// BODY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the body for which the induced accelerations are to be computed.
 *
 * @param aBody Body ID
 */
void BodyPointIndAcc::
setBody(AbstractBody *aBody)
{
	if (!aBody)
	{
		printf("BodyPointIndAcc.setBody:  WARN- invalid body ID %p.\n", aBody);
		_body = NULL;
		return;
	}

	// RESET STORAGE
	if(_axPointStore!=NULL) {
		constructDescription();
		_axPointStore->reset();
		_ayPointStore->reset();
		_azPointStore->reset();
		_axPointStore->setDescription(getDescription());
		_ayPointStore->setDescription(getDescription());
		_azPointStore->setDescription(getDescription());
	}
}
//_____________________________________________________________________________
/**
 * Get the body for which the induced accelerations are to be computed.
 *
 * @return Body ID
 */
AbstractBody* BodyPointIndAcc::
getBody()
{
	return(_body);
}

//-----------------------------------------------------------------------------
// POINT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the point for which the induced accelerations are to be computed.
 *
 * @param aPoint X-Y-Z Point
 */
void BodyPointIndAcc::
setPoint(double aPoint[3])
{
	_point[0] = aPoint[0];
	_point[1] = aPoint[1];
	_point[2] = aPoint[2];

	// RESET STORAGE
	if(_axPointStore!=NULL) {
		constructDescription();
		_axPointStore->reset();
		_ayPointStore->reset();
		_azPointStore->reset();
		_axPointStore->setDescription(getDescription());
		_ayPointStore->setDescription(getDescription());
		_azPointStore->setDescription(getDescription());
	}
}
//_____________________________________________________________________________
/**
 * Get the point for which the induced accelerations are to be computed.
 *
 * @param rPoint X-Y-Z Point
 */
void BodyPointIndAcc::
getPoint(double rPoint[3])
{
	rPoint[0] = _point[0];
	rPoint[1] = _point[1];
	rPoint[2] = _point[2];
}

//-----------------------------------------------------------------------------
// POINT NAME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set a name for the point.
 *
 * @param aName Name for the point.
 */
void BodyPointIndAcc::
setPointName(const char *aName)
{
	strncpy(_pointName,aName,NAME_LENGTH);
	_pointName[NAME_LENGTH-1] = 0;
}
//_____________________________________________________________________________
/**
 * Get the point name.
 *
 * @param aName Name for the point.
 */
const char* BodyPointIndAcc::
getPointName()
{
	return(_pointName);
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
void BodyPointIndAcc::
setStorageCapacityIncrements(int aIncrement)
{
	// BASE CLASS
	IndAcc::setStorageCapacityIncrements(aIncrement);

	// THIS CLASS
	_axPointStore->setCapacityIncrement(aIncrement);
	_ayPointStore->setCapacityIncrement(aIncrement);
	_azPointStore->setCapacityIncrement(aIncrement);
}


//=============================================================================
// OPERATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the acceleration of a point on a body induced by the active
 * force components of the model given a force decomposition and a
 * set of states.
 *
 * @return 0 on success, -1 on error.
 */
int BodyPointIndAcc::
computePointAccelerations()
{
	if(_yStore==NULL) return(-1);

	// CHECK FOR TIME CORRESPONDENCE
	if(!getUseNullDecomposition()) {
		double ti = _feStore[0]->getFirstTime();
		double tf = _feStore[0]->getLastTime();
		if((ti!=_yStore->getFirstTime())||(tf!=_yStore->getLastTime())) {
			printf("BodyPointIndAcc.computeAccelerations: WARN-\n");
			printf("\tTime range for states is %lf to %lf\n",
				_yStore->getFirstTime(),_yStore->getLastTime());
			printf("\tTime range for force decomposition is %lf to %lf\n",
				ti,tf);
		}
	}

	// NUMBERS
	int nq = _model->getNumCoordinates();
	int nu = _model->getNumSpeeds();
	int ny = _model->getNumStates();
	int nx = _model->getNumControls();
	int np = _model->getNumContacts();

	// GRAVITY
	double g[3];
	double g0[] = { 0.0, 0.0, 0.0 };
	_model->getGravity(g);
	printf("gravity = %lf %lf %lf\n",g[0],g[1],g[2]);

	// LOOP OVER TIME
	int i,j,c,I,J;
	AbstractBody *bodyB;
	double pointB[3];
	double t;
	double *fe = new double[3*np];
	double *y = new double[ny];
	double *x = new double[nx];
	double *dqdt = new double[nq];
	double *dudt = new double[nu];
	double *ax = new double[getNumComponents()];
	double *ay = new double[getNumComponents()];
	double *az = new double[getNumComponents()];
	double acc[3], vel[3], pos[3];
	StateVector *yVec;

	AbstractDynamicsEngine &engine = _model->getDynamicsEngine();

	for(i=0;i<_yStore->getSize();i++) {

		// LOOP OVER INDEPENDENT COMPONENTS
		int ai=0;	// index for looping over actuators
		ActuatorSet* as = _model->getActuatorSet();
		AbstractActuator* act = as->get(ai);
		for(c=0;c<getNumIndependentComponents();c++) {

			// SET GRAVITY
			if(c!=getGravityIndex()) {
				_model->setGravity(g0);
			} else {
				_model->setGravity(g);
			}

			// GET STATES
			yVec = _yStore->getStateVector(i);
			t = yVec->getTime();
			if(i==0) _ti = t;
			if(i==(_yStore->getSize() - 1)) _tf = t;
			_yStore->getDataAtTime(t,ny,y);

			// GET CONTROLS
			if(_xStore!=NULL)
				_xStore->getDataAtTime(t,nx,x);

			// GET CONTACT POINT FORCES
			if(!getUseNullDecomposition()) {
				_feStore[c]->getDataAtTime(t,3*np,fe);
			}

			// SET
			if(c!=getVelocityIndex()) {
				for(j=0;j<nu;j++) y[nq+j] = 0.0;
			}
			_model->set(t,x,y);

			// COMPUTE ACTUATION
			_model->getActuatorSet()->computeActuation();

			// NEED TO RECORD THE INITIAL BODY VELOCITIES AND POSITIONS
				if(c==getVelocityIndex() && t==_ti){
					_model->getActuatorSet()->apply();
					engine.computeDerivatives(dqdt,dudt);					
					engine.getVelocity(*_body,_point,vel);
					engine.getPosition(*_body,_point,pos);

					// RESET STATES
					_model->set(t,x,y);

					// RECOMPUTE ACTUATION
					_model->getActuatorSet()->computeActuation();	

					// APPLY ELEMENT FORCES
					if(!getUseNullDecomposition()) {
						for(j=0;j<_model->getNumContacts();j++) {
							J = Mtx::ComputeIndex(j,3,0);
							bodyB = _model->getContactSet()->getContactBodyB(j);
							_model->getContactSet()->getContactPointB(j,pointB);
							engine.applyForce(*bodyB,pointB,&fe[J]);
						}	
					}
				}

			// APPLY ACTUATOR FORCE
			if(c<=getLastActuatorIndex()) {
				act->apply();
			}

			// APPLY ELEMENT FORCES
			if(!getUseNullDecomposition()) {
				for(j=0;j<_model->getNumContacts();j++) {
					J = Mtx::ComputeIndex(j,3,0);
					bodyB = _model->getContactSet()->getContactBodyB(j);
					_model->getContactSet()->getContactPointB(j,pointB);
					engine.applyForce(*bodyB,pointB,&fe[J]);
				}
			}

			// COMPUTE THE ACCELERATIONS
			engine.computeDerivatives(dqdt,dudt);

			// COMPUTE THE BODY POINT ACCELERATION
			engine.getAcceleration(*_body,_point,acc);

			// FILL ARRAYS
			ax[c] = acc[0];
			ay[c] = acc[1];
			az[c] = acc[2];
		}

		// SUM ACTUATORS
		acc[0] = acc[1] = acc[2] = 0.0;
		for(c=0;c<=getLastActuatorIndex();c++) {
			acc[0] += ax[c];
			acc[1] += ay[c];
			acc[2] += az[c];
		}
		ax[getAllActuatorsIndex()] = acc[0];
		ay[getAllActuatorsIndex()] = acc[1];
		az[getAllActuatorsIndex()] = acc[2];

		// SUM ALL
		acc[0] = acc[1] = acc[2] = 0.0;
		for(c=getGravityIndex();c<=getAllActuatorsIndex();c++) {
			acc[0] += ax[c];
			acc[1] += ay[c];
			acc[2] += az[c];
		}
		I = Mtx::ComputeIndex(getAllIndex(),3,0);
		ax[getAllIndex()] = acc[0];
		ay[getAllIndex()] = acc[1];
		az[getAllIndex()] = acc[2];

		// APPEND TO STORAGE
		_axPointStore->append(t,getNumComponents(),ax);
		_ayPointStore->append(t,getNumComponents(),ay);
		_azPointStore->append(t,getNumComponents(),az);

		ai++;
		act = as->get(ai);
	}

	// COMPUTE THE INDUCED VELOCITIES AND POSITIONS
		
		// INTEGRATE TO GET VELOCITIES
		_vxPointStore = _axPointStore->integrate(_ti,_tf);
		_vyPointStore = _ayPointStore->integrate(_ti,_tf);
		_vzPointStore = _azPointStore->integrate(_ti,_tf);

		// INTEGRATE TO GET POSITIONS
		_pxPointStore = _vxPointStore->integrate(_ti,_tf);
		_pyPointStore = _vyPointStore->integrate(_ti,_tf);
		_pzPointStore = _vzPointStore->integrate(_ti,_tf);
	

	// INDUCED POSITION DUE TO INITIAL POSITION AND INITIAL VELOCITY
	
		//STORE INITIAL VELOCITIES
		_iVelStore = new Storage(500,"InitialPointVelocityX");		
		_iVelStore->append(_ti,3,&vel[0]);

		// ADD INITIAL VELOCITY OF POINT TO ALL COLLUMN
		//NOTE: this funtionality is waiting for a collumn add feature

		// "INTEGRATE" INITIAL VELOCITES TO YIELD INITIAL POSITIONS
		_iPosStore = new Storage(500,"InducedPointPositionDueToInitVel&Pos");	
		_iPosStore->append(_ti,3,&pos[0]);
		_iPosStore->append(_tf,3,&vel[0]);
		_iPosStore->getStateVector(1)->multiply(_tf -_ti);
		_iPosStore->getStateVector(1)->add(3,
			_iPosStore->getStateVector(0)->getData().get());

		// ADD INDUCED POSITION DUE TO INITIAL VELOCITY AND POSITION
		//NOTE: this funtionality is waiting for a collumn add feature

	// RESET GRAVITY
	_model->setGravity(g);

	// CLEANUP
	if(y!=NULL) { delete[] y;  y=NULL; }
	if(dqdt!=NULL) { delete[] dqdt;  dqdt=NULL; }
	if(dudt!=NULL) { delete[] dudt;  dudt=NULL; }
	if(ax!=NULL) { delete[] ax;  ax=NULL; }
	if(ay!=NULL) { delete[] ay;  ay=NULL; }
	if(az!=NULL) { delete[] az;  az=NULL; }

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
 * @param aExtension File extension.
 *
 * @return 0 on success, -1 on error.
 */
int BodyPointIndAcc::
printResults(const std::string &aBaseName,const std::string &aDir,double aDT,
				 const std::string &aExtension)
{
	// X
	Storage::printResult(_axPointStore,"ap_"+aBaseName+"_"+getPointName()+"_X",aDir,aDT,aExtension);
	Storage::printResult(_vxPointStore,"vp_"+aBaseName+"_"+getPointName()+"_X",aDir,aDT,aExtension);
	Storage::printResult(_pxPointStore,"pp_"+aBaseName+"_"+getPointName()+"_X",aDir,aDT,aExtension);

	// Y
	Storage::printResult(_ayPointStore,"ap_"+aBaseName+"_"+getPointName()+"_Y",aDir,aDT,aExtension);
	Storage::printResult(_vyPointStore,"vp_"+aBaseName+"_"+getPointName()+"_Y",aDir,aDT,aExtension);
	Storage::printResult(_pyPointStore,"pp_"+aBaseName+"_"+getPointName()+"_Y",aDir,aDT,aExtension);

	// Z
	Storage::printResult(_azPointStore,"ap_"+aBaseName+"_"+getPointName()+"_Z",aDir,aDT,aExtension);
	Storage::printResult(_vzPointStore,"vp_"+aBaseName+"_"+getPointName()+"_Z",aDir,aDT,aExtension);
	Storage::printResult(_pzPointStore,"pp_"+aBaseName+"_"+getPointName()+"_Z",aDir,aDT,aExtension);

	//INITIAL VELOCITY
	Storage::printResult(_iVelStore,aBaseName+"_initVel",aDir,aDT,aExtension);

	//INDUCED POSTION DUE TO INITIAL VELOCITY AND POSITION
	Storage::printResult(_iPosStore,"p_"+aBaseName+"_initVelPos",aDir,(aDT<=0)?0.005:aDT,aExtension);

	return(0);
}
