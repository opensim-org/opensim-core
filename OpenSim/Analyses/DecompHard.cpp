// DecompHard.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson, Saryn R. Goldberg, Jennifer L. Hicks
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Actuators/Springs.h>
#include <OpenSim/SQP/rdSQP.h>
#include <OpenSim/Simulation/Model/PointConstraint.h>
#include <OpenSim/Simulation/Model/DerivCallbackSet.h>
#include "DecompTarget.h"
#include "DecompHard.h"




using namespace OpenSim;
using namespace std;
typedef double v3d[3];

//=============================================================================
// CONSTANTS
//=============================================================================


//=============================================================================
// CONSTANTS
//=============================================================================



//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
DecompHard::~DecompHard()
{
	if(_x!=NULL) { delete[] _x;  _x=NULL; }
	if(_yCopy!=NULL) { delete[] _yCopy;  _yCopy=NULL; }
	if(_yTmp!=NULL) { delete[] _yTmp;  _yTmp=NULL; }
	if(_xsSprMap!=NULL) { delete[] _xsSprMap;  _xsSprMap=NULL; }
	if(_xsXYZMap!=NULL) { delete[] _xsXYZMap;  _xsXYZMap=NULL; }
}
//_____________________________________________________________________________
/**
 * Construct a hard-constraint decomposition analysis 
 */
DecompHard::DecompHard(Model *aModel) :
	Decomp(aModel)
{
	setNull();

	// LOCAL WORK ARRAYS
	_yCopy = new double[_model->getNumStates()];
	_yTmp = new double[_model->getNumStates()];
	_x = new double[_model->getNumControls()];

	// MAPPING ARRAYS
	int ns = _model->getNumContacts();
	_xsSprMap = new int[3*ns];
	_xsXYZMap = new int[3*ns];

	// PERFORMANCE CRITERION WEIGHTS
	_wXS = 1.0e-2;
	_wAcc = 1.0;

	// BODY CONSTRAINTS
	clearBodyConstraints();
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void DecompHard::
setNull()
{
	_c = 0;
	_nxs = 0;
	_xsSprMap = NULL,
	_xsXYZMap = NULL;
	_wXS = 1.0;
	_wAcc = 1.0;
	_point[0] = _point[1] = _point[2] = 0.0;
	_yCopy = NULL;
	_yTmp = NULL;
	_x = NULL;
}


//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the contact point for a specified contact element.
 *
 * @param aIndex Index to the model contact element.
 * @return Pointer to the contact point.  The contact point is a local
 * variable member, so there is no danger of the pointer becoming
 * invalid.
 * 
 * It is assumed that the contact points for the model are stored in the 
 * following way:
 * Index 0 to Index 3:  Right Calcaneus
 * Index 4: Right Toes
 * Index 5 to Index 8: Left Calcaneus
 * Index 9: Left Toes
 */
double* DecompHard::
getContactPoint(int aIndex)
{
	_model->getContactSet()->getContactPointB(aIndex,_point);
	return(_point);
}


//=============================================================================
// DECOMPOSITION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Decompose the ground reaction force by assuming hard constraints.
 */
void DecompHard::
compute(double *aXPrev,double *aYPrev,int step,double dt,double t,
		  double *xt,double *y)
{
	printf("\n\ntime = %lf\n",t);

	// INDEX TO ACTIVATIONS
	int nq = _model->getNumCoordinates();
	int nu = _model->getNumSpeeds();
	//int nx = _model->getNumControls();
	//int iatv = nq + nu + nx;

	// SET CONFIGURATION (Why twice??)
	_model->set(t,xt,y);

	// VARIABLE DECLARATIONS
	int i,j;
	int np = _model->getNumContacts();
	Array<double> xs(0.0,3*np);
	
	v3d *fs = new v3d[np];

	double g[3];
	_model->getGravity(g);

	// COMPUTE THE NOMINAL SPRING ACCELERATIONS
	Array<double> dqdt(0.0,nq),dudt(0.0,nu);
	v3d *sacc = new v3d[np];
	v3d *saccOpt = new v3d[np];
	
	// ----------------------------------
	// SET (Why twice??)
	_model->set(t,xt,y);
	_model->getDerivCallbackSet()->set(t,xt,y);

	// ACTUATION
	// Until DerivCallbackSet is implemented, actuator forces will be set
	// to one in the main code
	//_model->getActuatorSet()->computeActuation();	// muscle forces are computed
	//_model->getDerivCallbackSet()->computeActuation(t,xt,y);
	_model->getActuatorSet()->apply();
	_model->getDerivCallbackSet()->applyActuation(t,xt,y);

	// CONTACT - not necessary for doing the decomposition
	_model->getContactSet()->computeContact();
	_model->getDerivCallbackSet()->computeContact(t,xt,y);
	_model->getContactSet()->apply();
	_model->getDerivCallbackSet()->applyContact(t,xt,y);

	// ACCELERATIONS
	_model->getDynamicsEngine().computeDerivatives(&dqdt[0],&dudt[0]);
	// ----------------------------------
	for(i=0;i<np;i++) {
		_model->getDynamicsEngine().getAcceleration(*_model->getContactSet()->getContactBodyB(i),getContactPoint(i),
			sacc[i]);
	}

	// DETERMINE THE CONTROLS
	determineControls();
	if(_nxs<=0) {
		printf("\n\nNo springs in contact!\n\n");
		return;
	}

	// DETERMINE THE CONSTRAINTS
	determineConstraints();

	// CREATE THE OPTIMIZATION OBJECTS
	DecompTarget *decompTarget =
		new DecompTarget(_nxs,getNumberConstraints(),this);
	rdFSQP *sqp = new rdFSQP(decompTarget);
	sqp->setMaxIterations(200);
	sqp->setNonlinearEqualityConstraintTolerance(1.0e-4);
	sqp->setConvergenceCriterion(1.0e-2);
	sqp->setPrintLevel(0);

	// SOLVE OPTIMIZATION PROBLEM FOR EACH COMPONENT
	int id[] = {0,1,2,3,4,5,6,7,8,9};
	for(_c=0;_c<_nic;_c++) {

		cout<<"component = "<<_cNames[_c]<<endl;

		// SET CONSTRAINT VALUES
		if(_c==getInertialIndex()) {
			setBodyConstraintValues(np,id,sacc);
		}

		// SET INITIAL GUESS
		for(i=0;i<_nxs;i++) xs[i] = 1.0;

		sqp->computeOptimalControls(&xs[0],&xs[0]);

		// RECORD CONTACT POINT ACCELERATIONS
		if(_recordContactPointAccelerations){
			suComputeContactPointAccelerations(&xs[0], _c, saccOpt);
			_cpaStore[_c]->append(t*_model->getTimeNormConstant(),3*np,&saccOpt[0][0]);
		}

		// RESET CONSTRAINT VALUES
		clearBodyConstraintValues();

		// RESET GRAVITY
		_model->setGravity(g);

		// SET SPRING FORCES
		setSpringForces(&xs[0],fs);
		for(i=0;i<np;i++) for(j=0;j<3;j++) _f[_c][i][j] = fs[i][j];

		// DRAW
		//if(_c<_model->getNumControls()) {
		//	y[iatv+_c] = 1.0;
		//	model->setMuson(_c,1);
		//}
		//while(status!=-1) {
		//	status = model->draw(t,xt,y,&fs[0][0],&spos[0][0]);
		//}
		//if(_c<_model->getNumControls()) {
		//	y[iatv+_c] = 0.0;
		//	model->setMuson(_c,0);
		//}
		//status = 0;

		// FILL STORAGE ARRAY
		_fStore[_c]->append(t*_model->getTimeNormConstant(),3*np,&_f[_c][0][0]);
	}

	// CLEANUP
	if(decompTarget!=NULL) { delete decompTarget; decompTarget=NULL; }
	if(sqp!=NULL) { delete sqp; sqp=NULL; }
	delete[] fs;
	delete[] sacc;
	delete[] saccOpt;
}
//_____________________________________________________________________________
/**
 * Determine the controls for the decomposition.
 */
void DecompHard::
determineControls()
{
	int i;

	// RESET MAPPINGS
	int np = _model->getNumContacts();
	_nxs = 0;
	for(i=0;i<3*np;i++) {
		_xsSprMap[i] = -1;
		_xsXYZMap[i] = -1;
	}

	// SET CONTACT ESTABLISHED
	if(!getUsePresetContactEstablishedSettings()) {
		printf("\n\nNo Preset Contact Settings!\n\n");
		return;
	}

	// SET CONTROLS MAP
	for(i=0;i<np;i++) {
		if(getContactEstablished(i)) {
			_xsSprMap[_nxs] = i;
			_xsSprMap[_nxs+1] = i;
			_xsSprMap[_nxs+2] = i;
			_xsXYZMap[_nxs] = 0;
			_xsXYZMap[_nxs+1] = 1;
			_xsXYZMap[_nxs+2] = 2;
			_nxs += 3;
		}
	}

	// PRINT
//	printf("\n\nControl Maps:\n");
	for(i=0;i<3*np;i++) {
//		printf("%d: %d %d\n",i,_xsSprMap[i],_xsXYZMap[i]);
	}
//	printf("\n");
}
//_____________________________________________________________________________
/**
 * Determine the constraints for the decomposition.
 */
void DecompHard::
determineConstraints()
{
	int i,ibc,ipc;
	double x[] = {1.0,0.0,0.0};
	double y[] = {0.0,1.0,0.0};
	double z[] = {0.0,0.0,1.0};
	double p[3];
	PointConstraint *pc[3];
	int rHindInContact=0;
	int lHindInContact=0;

	// CLEAR ANY PREVIOUS CONSTRAINTS
	clearBodyConstraints();

	// RIGHT HINDFOOT
	ibc = 0;
	for(ipc=0,i=0;i<4;i++) {

		if(!_contactEstablished[i]) continue;

		// pc0
		if(ipc==0) {
			_bc[ibc].setBody(_model->getContactSet()->getContactBodyB(i));
			pc[ipc] = _bc[ibc].getPC(ipc);
			pc[ipc]->setID(i);
			_model->getDynamicsEngine().getPosition(*_model->getContactSet()->getContactBodyB(i),getContactPoint(i),p);
			pc[ipc]->setPoint(p);
			pc[ipc]->setC0(x);
			pc[ipc]->setC1(y);
			pc[ipc]->setC2(z);
			ipc++;

		// pc1
		} else if(ipc==1) {
			pc[ipc] = _bc[ibc].getPC(ipc);
			pc[ipc]->setID(i);
			_model->getDynamicsEngine().getPosition(*_model->getContactSet()->getContactBodyB(i),getContactPoint(i),p);
			pc[ipc]->setPoint(p);
			_bc[ibc].constructConstraintsForPoint1();
			ipc++;

		// pc2
		} else if(ipc==2) {
			pc[ipc] = _bc[ibc].getPC(ipc);
			pc[ipc]->setID(i);
			_model->getDynamicsEngine().getPosition(*_model->getContactSet()->getContactBodyB(i),getContactPoint(i),p);
			pc[ipc]->setPoint(p);
			_bc[ibc].constructConstraintsForPoint2();
			ipc++;

		// NO MORE THAN 3 ON A BODY
		} else {
			printf("Enough constraints on body.\n");
		}

	}
	rHindInContact = ipc;
	if(ipc>0) ibc++;

	// RIGHT TOES
	for(ipc=0,i=4;i<5;i++) {

		if(!_contactEstablished[i]) continue;

		_bc[ibc].setBody(_model->getContactSet()->getContactBodyB(i));
		pc[0] = _bc[ibc].getPC(ipc);
		pc[0]->setID(i);
		_model->getDynamicsEngine().getPosition(*_model->getContactSet()->getContactBodyB(i),getContactPoint(i),p);
		pc[ipc]->setPoint(p);

		// THREE CONSTRAINT DIRECTIONS BECAUSE HINDFOOT IS NOT IN CONTACT
		// Actually, when only one contact point on the hindfoot is in contact, the toe
		// actually has three degrees of freedom because of the metatarsal joint.
		// Therefore, it must be constrained in three directions.
		if(rHindInContact<=1) {
			pc[0]->setC0(x);
			pc[0]->setC1(y);
			pc[0]->setC2(z);

		// ONE CONSTRAINT DIRECTION ORTHOGONAL TO JOINT BECAUSE
		// HINDFOOT IS IN CONTACT
		} else {
			double p2[3],p3[3],p4[3],c0[3];
			_model->getDynamicsEngine().getPosition(*_model->getContactSet()->getContactBodyB(2),getContactPoint(2),p2);
			_model->getDynamicsEngine().getPosition(*_model->getContactSet()->getContactBodyB(3),getContactPoint(3),p3);
			_model->getDynamicsEngine().getPosition(*_model->getContactSet()->getContactBodyB(4),getContactPoint(4),p4);
			double r23[3],r34[3];
			Mtx::Subtract(1,3,p3,p2,r23);
			Mtx::Subtract(1,3,p4,p3,r34);
			Mtx::CrossProduct(r23,r34,c0);
			pc[0]->zeroConstraints();
			pc[0]->setC0(c0);
			pc[0]->normalizeConstraints();
		}
		ipc++;
	}
	if(ipc>0) ibc++;

	// LEFT HINDFOOT
	for(ipc=0,i=5;i<9;i++) {

		if(!_contactEstablished[i]) continue;

		// pc0
		if(ipc==0) {
			_bc[ibc].setBody(_model->getContactSet()->getContactBodyB(i));
			pc[ipc] = _bc[ibc].getPC(ipc);
			pc[ipc]->setID(i);
			_model->getDynamicsEngine().getPosition(*_model->getContactSet()->getContactBodyB(i),getContactPoint(i),p);
			pc[ipc]->setPoint(p);
			pc[ipc]->setC0(x);
			pc[ipc]->setC1(y);
			pc[ipc]->setC2(z);
			ipc++;

		// pc1
		} else if(ipc==1) {
			pc[ipc] = _bc[ibc].getPC(ipc);
			pc[ipc]->setID(i);
			_model->getDynamicsEngine().getPosition(*_model->getContactSet()->getContactBodyB(i),getContactPoint(i),p);
			pc[ipc]->setPoint(p);
			_bc[ibc].constructConstraintsForPoint1();
			ipc++;

		// pc2
		} else if(ipc==2) {
			pc[ipc] = _bc[ibc].getPC(ipc);
			pc[ipc]->setID(i);
			_model->getDynamicsEngine().getPosition(*_model->getContactSet()->getContactBodyB(i),getContactPoint(i),p);
			pc[ipc]->setPoint(p);
			_bc[ibc].constructConstraintsForPoint2();
			ipc++;

		// NO MORE THAN 3 ON A BODY
		} else {
			printf("Enough constraints on body.\n");
		}
	}
	lHindInContact = ipc;
	if(ipc>0) ibc++;

	// LEFT TOES
	for(ipc=0,i=9;i<10;i++) {

		if(!_contactEstablished[i]) continue;

		_bc[ibc].setBody(_model->getContactSet()->getContactBodyB(i));
		pc[0] = _bc[ibc].getPC(ipc);
		pc[0]->setID(i);
		_model->getDynamicsEngine().getPosition(*_model->getContactSet()->getContactBodyB(i),getContactPoint(i),p);
		pc[ipc]->setPoint(p);

		// THREE CONSTRAINT DIRECTIONS BECAUSE HINDFOOT IS NOT IN CONTACT
		// Actually, when only one contact point on the hindfoot is in contact, the toe
		// actually has three degrees of freedom because of the metatarsal joint.
		// Therefore, it must be constrained in three directions.
		if(lHindInContact<=1 ) {
			pc[0]->setC0(x);
			pc[0]->setC1(y);
			pc[0]->setC2(z);

		// ONE CONSTRAINT DIRECTION ORTHOGONAL TO TO JOINT BECAUSE
		// HINDFOOT IS IN CONTACT
		} else {
			double p2[3],p3[3],p4[3],c0[3];
			_model->getDynamicsEngine().getPosition(*_model->getContactSet()->getContactBodyB(7),getContactPoint(7),p2);
			_model->getDynamicsEngine().getPosition(*_model->getContactSet()->getContactBodyB(8),getContactPoint(8),p3);
			_model->getDynamicsEngine().getPosition(*_model->getContactSet()->getContactBodyB(9),getContactPoint(9),p4);
			double r23[3],r34[3];
			Mtx::Subtract(1,3,p3,p2,r23);
			Mtx::Subtract(1,3,p4,p3,r34);
			Mtx::CrossProduct(r23,r34,c0);
			pc[0]->zeroConstraints();
			pc[0]->setC0(c0);
			pc[0]->normalizeConstraints();
		}
		ipc++;
	}
	if(ipc>0) ibc++;

//	printf("DecompHardUTWalk.determineConstraints: NC = %d\n",
//		getNumberConstraints());
}
//_____________________________________________________________________________
/**
 * Set spring force array based on decomposition controls.
 */
int DecompHard::
getNumberConstraints()
{
	int i,nc;
	for(nc=i=0;i<4;i++) {
		nc += _bc[i].getNC();
	}
	return(nc);
}
//_____________________________________________________________________________
/**
 * Clear all body constraints.
 */
void DecompHard::
clearBodyConstraints()
{
	int i;
	for(i=0;i<4;i++) {
		_bc[i].clear();
	}
}
//_____________________________________________________________________________
/**
 * Clear all constraint values--- set them to 0.0.
 */
void DecompHard::
clearBodyConstraintValues()
{
	int i;
	for(i=0;i<4;i++) {
		_bc[i].clearValues();
	}
}
//_____________________________________________________________________________
/**
 * Set the relavent constraint values.
 */
void DecompHard::
setBodyConstraintValues(int aN,int aID[],double aV[][3])
{
	int i;
	for(i=0;i<4;i++) {
		_bc[i].setValues(aN,aID,aV);
	}
}
//_____________________________________________________________________________
/**
 * Set spring force array based on decomposition controls.
 */
void DecompHard::
setSpringForces(double *aXS,double aFS[][3])
{
	int i,j;

	// ZERO SPRING FORCES
	for(i=0;i<_model->getNumContacts();i++) {
		for(j=0;j<3;j++) aFS[i][j] = 0.0;
	}

	// STEP THROUGH CONTROLS
	for(i=0;i<_nxs;i++) {
		aFS[_xsSprMap[i]][_xsXYZMap[i]] = aXS[i];
	}
}
//_____________________________________________________________________________
/**
 * Apply appropriate component force.
 *
 * @param aC Index of the component.
 */
void DecompHard::
applyComponentForce(int aC)
{
	int i;

	// GET A COPY OF THE STATES AND CONTROLS
	_model->getStates(_yTmp);
	_model->getControls(_x);

	// GRAVITY
	double g0[3] = { 0.0, 0.0, 0.0 };
	double g[3];
	_model->getGravity(g);

	// ZERO GRAVITY
	_model->setGravity(g0);

	// ZERO VELOCITIES
	int nq = _model->getNumCoordinates();
	int nu = _model->getNumSpeeds();
	for(i=0;i<nu;i++) _yTmp[nq+i] = 0.0;

	// SET STATES
	_model->setStates(_yTmp);

	// APPLY ACTION FORCES SELECTIVELY
	// ACTUATOR
	if(aC<=getLastActuatorIndex()) {
		DerivCallbackSet *callbackSet;
		callbackSet = _model->getDerivCallbackSet();
		// Until DerivCallbackSet is implemented, actuator forces will be set
		// to one in the main code
		//_model->getActuatorSet()->computeActuation();
		//callbackSet->computeActuation(_model->getTime(),_x,_yTmp);
		_model->getActuatorSet()->get(aC)->apply();
		callbackSet->applyActuation(_model->getTime(),_x,_yTmp);

	// GRAVITY
	} else if(aC==getGravityIndex()) {
		_model->setGravity(g);
		_model->setStates(_yTmp);

	// CENTRIPETAL
	} else if(aC==getVelocityIndex()) {
		for(i=0;i<nu;i++) _yTmp[nq+i] = _yCopy[nq+i];
		_model->setStates(_yTmp);
	}
}



//==============================================================================
// OPTIMIZATION
//==============================================================================
//------------------------------------------------------------------------------
// PERFORMANCE
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Compute performance given x.
 */
int DecompHard::
suComputePerformance(double *x,double *p)
{
	int i;

	// COMPUTE PERFORMANCE
	*p = 0.0;
	for(i=0;i<_nxs;i++) {
		*p += _wXS*_wXS * x[i]*x[i];
	}

	return(0);
}
//------------------------------------------------------------------------------
// PERFORMANCE
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Compute performance given x.
 */
int DecompHard::
suComputePerformanceGradient(double *x,double *p)
{
	int i;

	// COMPUTE PERFORMANCE
	*p = 0.0;
	for(i=0;i<_nxs;i++) {
		*p += 2.0 * _wXS*_wXS * x[i];
	}

	return(0);
}
//______________________________________________________________________________
/**
 * Compute a constraint given x.
 */
int DecompHard::
suComputeConstraint(double *x,int ic,double *c)
{
	int i,j;

	// COPY THE STATES
	_model->getStates(_yCopy);

	// APPLY COMPONENT FORCES
	applyComponentForce(_c);

	// APPLY SPRING FORCES
	int np = _model->getNumContacts();
	v3d *fs = new v3d[np];
	setSpringForces(x,fs);
	for(i=0;i<np;i++) {
		_model->getDynamicsEngine().applyForce(*_model->getContactSet()->getContactBodyB(i),getContactPoint(i),fs[i]);
	}

	// COMPUTE ACCELERATIONS
	int nq = _model->getNumCoordinates();
	int nu = _model->getNumSpeeds();
	Array<double> dqdt(0.0,nq),dudt(0.0,nu);
	_model->getDynamicsEngine().computeDerivatives(&dqdt[0],&dudt[0]);

	// EVALUATE THE CONSTRAINT
	bool evaluated = false;
	int nc,id,whichC;
	double fsAcc[3];
	for(nc=i=0;i<4;i++) {

		// LOOP ON POINT CONSTRAINTS
		for(j=0;j<3;j++) {
			nc += _bc[i].getPC(j)->getNC();
			if(nc<ic) continue;

			// GET THE ACCELERATION
			id = _bc[i].getPC(j)->getID();
			_model->getDynamicsEngine().getAcceleration(*_bc[i].getBody(),
				getContactPoint(id),fsAcc);

			// WHICH CONSTRAINT DIRECTION
			whichC = _bc[i].getPC(j)->getNC() - (nc-ic) - 1;

			// EVALUATE
			if(whichC==0) {
				*c = _bc[i].getPC(j)->evaluateC0(fsAcc);
				evaluated = true;
			} else if(whichC==1) {
				*c = _bc[i].getPC(j)->evaluateC1(fsAcc);
				evaluated = true;
			} else if(whichC==2) {
				*c = _bc[i].getPC(j)->evaluateC2(fsAcc);
				evaluated = true;
			} else {
				printf("DecompHard.suComputeConstraint: ERROR- invalid C\n");
			}

			// BREAK?
			if(evaluated) break;
		}

		// BREAK?
		if(evaluated) break;
	}

	// WEIGHT FACTOR
	*c *= _wAcc;
	//printf("%d = %lf\n",ic-1,*c);

	// RESTORE THE STATES
	_model->setStates(_yCopy);
	
	// CLEANUP
	delete[] fs;

	return(0);
}

//______________________________________________________________________________
/**
 * Compute contact point accelerations.
 */
int DecompHard::
suComputeContactPointAccelerations(double *x,int c,double cpa[][3])
{
	int i;

	// COPY THE STATES
	_model->getStates(_yCopy);

	// APPLY COMPONENT FORCES
	applyComponentForce(c);

	// APPLY SPRING FORCES
	int np = _model->getNumContacts();

	v3d *fs = new v3d[np];

	setSpringForces(x,fs);
	for(i=0;i<np;i++) {
		_model->getDynamicsEngine().applyForce(*_model->getContactSet()->getContactBodyB(i),getContactPoint(i),fs[i]);
	}

	// COMPUTE ACCELERATIONS
	int nq = _model->getNumCoordinates();
	int nu = _model->getNumSpeeds();
	Array<double> dqdt(0.0,nq),dudt(0.0,nu);
	_model->getDynamicsEngine().computeDerivatives(&dqdt[0],&dudt[0]);

	// RECORD CONTACT POINT ACCELERATIONS
	for(i=0;i<np;i++) {
		_model->getDynamicsEngine().getAcceleration(*_model->getContactSet()->getContactBodyB(i),getContactPoint(i),
			cpa[i]);
	}

	// RESTORE THE STATES
	_model->setStates(_yCopy);

	// CLEANUP
	delete[] fs;

	return(0);
}
