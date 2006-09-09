// DecompTaylor.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/Mtx.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/DerivCallbackSet.h>
#include "DecompTaylor.h"

//=============================================================================
// CONSTANTS
//=============================================================================



//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________


using namespace OpenSim;
/**
 * Destructor.
 */
DecompTaylor::~DecompTaylor()
{
}
//_____________________________________________________________________________
/**
 * Construct an object for performing a reaction force decomposition analysis.
 * The decomposition is performed using the perturbed integration method.
 *
 * It is critical that a twin or copy of the model used to conduct the nominal
 * simulation be used to construct this decomposition analysis.  If the
 * nominal simulation model is used, this analysis may change the trajectory
 * of the simulation because model states and contact setpoints are
 * altered during the analysis.
 *
 * @param aManager Simulation manager of the nominal simulation.
 * @param aContact Contact analysis of the nominal simulation.
 * @param aModelTwin Twin of the model used to conduct the nominal
 * simulation.
 * @param aDT Perturbed integration time window in real time.
 * @param aDF Force perturbation.
 * @see setIntegrationWindow()
 * @see setPerturbation()
 */
DecompTaylor::
DecompTaylor(Model *aModel) :
	Decomp(aModel)
{
	setNull();

	// MEMBER VARIABLES
	setName("DecompTaylor");

	// DESCRIPTIONS
	constructDescription();
	updateStorageDescriptions();

	// ALLOCATE MEMORY
	allocate();
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set member variables to approprate NULL values.
 */
void DecompTaylor::
setNull()
{
	_contactJustEstablished = NULL;

	_v = NULL;
	_a = NULL;
	_fp = NULL;
	_fv = NULL;
	_at = NULL;
	_ah = NULL;

	_yTmp = NULL;
	_xTmp = NULL;
	_qPrev = NULL;
	_uPrev = NULL;
	_q = NULL;
	_u = NULL;
	_dqdt = NULL;
	_dudt = NULL;
	_ddudt = NULL;
	_dddudt = NULL;
	_du = NULL;
	_pa = NULL;
	_paPrev = NULL;
	_svel = NULL;
	_sfrc = NULL;
	_pfrc = NULL;
	_dfpFric = NULL;
	_dfvFric = NULL;
	_pAlpha = NULL;
	_vAlpha = NULL;
	_J = NULL;
	_JPrev = NULL;
	_JDiff = NULL;
}
//_____________________________________________________________________________
/**
 * Allocate memory.
 */
void DecompTaylor::
allocate()
{
	if(_model==NULL) return;

	// NUMBERS
	int ny = _model->getNY();
	int nq = _model->getNQ();
	int nu = _model->getNU();
	int nx = _model->getNX();
	int np = _model->getNP();
	int nc = getNumComponents();

	_contactJustEstablished = new bool[np];

	_v = new double[nc*np*3];
	_a = new double[nc*np*3];
	_fp = new double[nc*np*3];
	_fv = new double[nc*np*3];

	_at = new double[nc*3];
	_ah = new double[nc*nu*3];

	_yTmp = new double[ny];
	_xTmp = new double[nx];
	_qPrev = new double[nq];
	_uPrev = new double[nu];
	_q = new double[nq];
	_u = new double[nu];
	_dqdt = new double[nq];
	_dudt = new double[nu];
	_ddudt = new double[nu];
	_dddudt = new double[nu];
	_du = new double[nu];
	_pa = new double[np*3];
	_paPrev = new double[np*3];
	_svel = new double[np*3];
	_sfrc = new double[np*3];
	_pfrc = new double[np*3];
	_dfpFric = new double[np*3];
	_dfvFric = new double[np*3];
	_pAlpha = new double[np*3];
	_vAlpha = new double[np*3];
	_J = new double[3*nu];
	_JPrev = new double[3*nu];
	_JDiff = new double[3*nu];
}

//_____________________________________________________________________________
/**
 * Construct a description.
 */
void DecompTaylor::
constructDescription()
{
	printf("DecompTaylor.constructDescription:\n");
	char tmp[1024],descrip[1024];

	strcpy(descrip,"\nThis file contains the reaction forces induced by\n");
	sprintf(tmp,"a particular action force of model %s.\n\n",
		_model->getName().c_str());
	strcat(descrip,tmp);

	strcat(descrip,
		"The decomposition was performed by Taylor expansion.\n");

	strcat(descrip,"\n\nUnits are S.I. units (seconds, meters, Newtons, ...)");
	strcat(descrip,"\n\n");

	setDescription(descrip);
}
//_____________________________________________________________________________
/**
 * Update storage descriptions.
 */
void DecompTaylor::
updateStorageDescriptions()
{
	if(_fStore==NULL) return;

	int c;
	for(c=0;c<getNumComponents();c++) {
		if(_fStore[c]==NULL) continue;
		_fStore[c]->setDescription(getDescription());
	}
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// INTEGRATION WINDOW
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the size of the integration window used to approximate the changes
 * in the ground reaction force.
 *
 * The integration window must be greater than or equal to zero.  If a
 * negative value is sent in, the window is set to 0.0.
 *
 * @param aDT Integration window- must be zero or positive.
void DecompTaylor::
setIntegrationWindow(double aDT)
{
	_dt = aDT;
	if(_dt<0.0) _dt=0.0;

	// RECONSTRUCT THE DESCRIPTION
	constructDescription();
	updateStorageDescriptions();
}
 */

//_____________________________________________________________________________
/**
 * Get the size of the integration window used to approximate the changes
 * in the ground reaction force.
 *
 * @return Integration window.
 * @see setIntegrationWindow()
double DecompTaylor::
getIntegrationWindow() const
{
	return(_dt);
}
 */


//=============================================================================
// DECOMPOSITION
//=============================================================================
//-----------------------------------------------------------------------------
// BEGIN
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * At the beginning of an integration, initialize the decomposition.
 */
int DecompTaylor::
begin(int aStep,double aDT,double aT,double *aX,double *aY,
	void *aClientData)
{
	if(_model==NULL) return(0);

	// NUMBERS
	int nu = _model->getNU();
	int np = _model->getNP();
	int nc = getNumComponents();

	// SET
	_model->set(aT,aX,aY);
	_model->computeContact();

	// ACCELERATION HISTORY
	int c,i,I;
	for(c=0;c<nc;c++) {
		I = Mtx::ComputeIndex(c,3,0);
		_at[I+0] = rdMath::NAN;
		_at[I+1] = rdMath::NAN;
		_at[I+2] = rdMath::NAN;
	}
	for(c=0;c<nc;c++) {
		for(i=0;i<nu;i++) {
			I = Mtx::ComputeIndex(c,nu,i,3,0);
			_ah[I+0] = 0.0;
			_ah[I+1] = 0.0;
			_ah[I+2] = 0.0;
		}
	}	

	// CONTACT POINTS
	for(i=0;i<np;i++) {
		I = Mtx::ComputeIndex(i,3,0);
		_model->getContactPointA(i,&_pa[I]);
		_model->getContactPointA(i,&_paPrev[I]);
	}

	// FORCES AND VELOCITIES
	initializeForceElements();
	initializeVelocityElements();

	return(0);
}

//-----------------------------------------------------------------------------
// COMPUTE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute the reaction force decomposition using Taylor expansion.
 */
void DecompTaylor::
compute(double *aXPrev,double *aYPrev,
		int aStep,double aDT,double aT,double *aX,double *aY)
{
	if(_model==NULL) return;
	//printf("DecompTaylor.compute: \n");

	//--------------------------------------------------------------------------
	// PREPARATION
	//--------------------------------------------------------------------------
	int i,j,I,J,c;
	int nu = _model->getNU();
	int np = _model->getNP();

	// REAL TIME
	double tReal = aT * _model->getTimeNormConstant();

	// FRICTIONAL SLIDING
	computeFrictionFactors(aT,aX,aY);

	// EXTRACT THE PREVIOUS AND CURRENT COORDINATES AND SPEEDS
	_model->extractConfiguration(aYPrev,_qPrev,_uPrev);
	_model->extractConfiguration(aY,_q,_u);

	// CURRENT CONTACT FORCES
	computeContactForces(_sfrc);

	// CURRENT CONTACT VELOCITIES
	computeContactVelocities(_svel);

	// LOOP OVER CONTACT POINTS
	for(i=0;i<np;i++) {

		// NO CONTACT
		I = Mtx::ComputeIndex(i,3,1);
		if(fabs(_sfrc[I])<=_contactThreshold) {
	
			// SET CONTACT FLAG
			_contactEstablished[i] = false;
			_contactJustEstablished[i] = false;

			// FORCES
			for(c=0;c<_nc;c++) {
				for(j=0;j<3;j++) {
					_f[c][i][j] = 0.0;
				}
			}

			// VELOCITIES
			for(c=0;c<_nc;c++) {
				for(j=0;j<3;j++) {
					I = Mtx::ComputeIndex(i,3,j);
					J = Mtx::ComputeIndex(c,np,i,3,j);
					if(c==getVelocityIndex()) {
						_v[J] = _svel[I];
					} else {
						_v[J] = 0.0;
					}
				}
			}

		// CONTACT JUST ESTABLISHED
		} else if(_contactEstablished[i]==false) {
			printf("t=%lf:  handling damping force discontinuity in spring %d\n",
				tReal,i);

			// SET CONTACT FLAG
			_contactEstablished[i] = true;
			_contactJustEstablished[i] = true;

			// INITIALIZE VISCOUS CONTACT FORCES
			// The contact force needs to be broken up into its elastic and
			// viscous components.
			double fnp[3],fnv[3],fn[3];
			double ftp[3],ftv[3],ft[3];
			_model->getContactNormalForce(i,fnp,fnv,fn);
			_model->getContactTangentForce(i,ftp,ftv,ft);
			J = Mtx::ComputeIndex(_cVel,np,i,3,0);
			Mtx::Add(1,3,fnv,ftv,&_fv[J]);
			printf("damp force = %lf %lf %lf\n",_fv[J+0],_fv[J+1],_fv[J+2]);
		}
	}

	//--------------------------------------------------------------------------
	// INTEGRATE
	//--------------------------------------------------------------------------
	int bod;
	double pnt[3];
	double dfp[3],dfv[3];
	double ddt = aDT*aDT/2.0;
	double dddt = aDT*aDT*aDT/6.0;
	double dud,dudd,duddd;
	double dv[3],dvTmp[3],dp[3];
	double g[3];  _model->getGravity(g);

	// RESTORE MODEL TO PREVIOUS STATES
	_model->set(aT-aDT,aXPrev,aYPrev);
	for(i=0;i<np;i++) {
		I = Mtx::ComputeIndex(i,3,0);
		_model->setContactPointA(i,&_paPrev[I]);
	}
	_model->computeContact();

	// LOOP OVER ACTION FORCES
	for(c=0;c<=getVelocityIndex();c++) {

		// APPLY ACTION FORCE
		applyActionForce(c,aT,aXPrev,aYPrev);

		// APPLY REACTION FORCE
		for(i=0;i<np;i++) {
			// BODY A
			// Need to negate! _f is applied to BodyB
			//bod = _model->getContactBodyA(i);
			//_model->getContactPointA(i,pnt);
			//_model->applyForceBody(bod,pnt,_f[c][i]);

			// BODY B
			// ?? In what frame is the force expressed?  I believe BodyA
			bod = _model->getContactBodyB(i);
			_model->getContactPointB(i,pnt);
			_model->applyForce(bod,pnt,_f[c][i]);
		}

		// COMPUTE INDUCED ACCELERATIONS
		_model->computeAccelerations(_dqdt,_dudt);
		for(i=0;i<np;i++) {
			bod = _model->getContactBodyB(i);
			_model->getContactPointB(i,pnt);
			I = Mtx::ComputeIndex(c,np,i,3,0);
			_model->getAcceleration(bod,pnt,&_a[I]);
		}

		// COMPUTE HIGHER ORDER DERIVATIVES
		computeHigherOrderDerivatives(c,aT-aDT,_dudt,_ddudt,_dddudt);

		// COMPUTE CHANGE IN GENERALIZED SPEEDS
		for(i=0;i<nu;i++) {
			dud = _dudt[i]*aDT;
			dudd = _ddudt[i]*ddt;
			duddd = _dddudt[i]*dddt;
			_du[i] = dud + dudd + duddd;
			if(c==-1) {
				printf("t=%lf  %s  du's = %le %le %le\n",
				 aT-aDT,_cNames[c].c_str(),dud,dudd,duddd);
			}
		}

		// PRINT OUT DIAGNOSTICS
		if(c==-1) {
			printf("t=%lf Component=%s\nForce:        ",tReal,_cNames[c].c_str());
			for(i=0;i<np;i++) {
				I = Mtx::ComputeIndex(c,np,i,3,1);
				printf("%lf ",_f[c][i][1]);
			}
			printf("\nVelocity:     ");
			for(i=0;i<np;i++) {
				I = Mtx::ComputeIndex(c,np,i,3,1);
				printf("%lf ",_v[I]);
			}
			printf("\nAcceleration: ");
			for(i=0;i<np;i++) {
				I = Mtx::ComputeIndex(c,np,i,3,1);
				printf("%lf ",_a[I]);
			}
			printf("\n\n");
		}

		// JACOBIAN METHOD
		// ADVANCE DECOMPOSITION
		for(i=0;i<np;i++) {

			if(!_contactEstablished[i]) continue;

			// GET THE PREVIOUS AND CURRENT JACOBIAN
			bod = _model->getContactBodyB(i);
			_model->getContactPointB(i,pnt);
			_model->setConfiguration(_qPrev,_uPrev);
			_model->formJacobianTranslation(bod,pnt,_JPrev);
			_model->setConfiguration(_q,_u);
			_model->formJacobianTranslation(bod,pnt,_J);

			// ACCELERATION-BASED CHANGES IN SPRING POINT VELOCITIES
			Mtx::Multiply(3,nu,1,_J,_du,dv);
 
			// VELOCITY BASED CHANGES IN VELOCITY
			if(c==getVelocityIndex()) {
				Mtx::Subtract(3,nu,_J,_JPrev,_JDiff);
				Mtx::Multiply(3,nu,1,_JDiff,_uPrev,dvTmp);
				Mtx::Add(1,3,dv,dvTmp,dv);
			}

			// CHANGES IN POSITION
			for(j=0;j<3;j++) {
				I = Mtx::ComputeIndex(c,np,i,3,j);
				dp[j] = _v[I]*aDT + _a[I]*ddt;
			}

			// GET STIFFNESS AND VISCOSITY FORCE CHANGES
			//double u[3] = { 1.0, 1.0, 1.0 };
			//_model->getContactStiffnessA(i,u,dfp);
			//_model->getContactViscosityA(i,u,dfv);
			_model->getContactStiffness(i,dp,dfp);
			_model->getContactViscosity(i,dv,dfv);

			// UPDATE INDUCED REACTION FORCES AND VELOCITIES
			//printf("_pAlpha = %le %le %le\n",_pAlpha[0],_pAlpha[1],_pAlpha[2]);
			//printf("_vAlpha = %le %le %le\n",_vAlpha[0],_vAlpha[1],_vAlpha[2]);
			for(j=0;j<3;j++) {
				I = Mtx::ComputeIndex(c,np,i,3,j);
				J = Mtx::ComputeIndex(i,3,j);

				// INDUCED VELOCITY
				_v[I] += dv[j];

				// ELASTIC INDUCED REACTION FORCE
				_fp[I] += dfp[j];  _fp[I] *= _pAlpha[J];

				// VISCOUS INDUCED REACTION FORCE
				if(!_contactJustEstablished[i]) _fv[I] += dfv[j];

				// TOTAL INDUCED REACTION FORCE
				_f[c][i][j] = _fp[I] + _vAlpha[J]*_fv[I];
			}

			// CONTACT IS NO LONGER JUST ESTABLISHED
			if(c==_cVel) _contactJustEstablished[i] = false;
		}

		// STORE DECOMPOSITION RESULTS
		_fStore[c]->append(tReal,3*np,_f[c][0]);

		// RESET GRAVITY
		_model->setGravity(g);
	}

	// RESET STATES AND CONTACT POINTS
	_model->set(aT,aX,aY);
	for(i=0;i<np;i++) {
		I = Mtx::ComputeIndex(i,3,0);
		_model->setContactPointA(i,&_pa[I]);
	}
	_model->computeContact();
	for(i=0;i<3*np;i++) _paPrev[i] = _pa[i];
}


//=============================================================================
// SUPPORTING METHODS
//=============================================================================
//-----------------------------------------------------------------------------
// APPLY ACTION FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Apply appropriate action force.
 */
void DecompTaylor::
applyActionForce(int aC,double aT,double *aX,double *aY)
{
	if(_model==NULL) return;
	int i;

	// NUMBERS
	int ny = _model->getNY();
	int nx = _model->getNX();
	int nq = _model->getNQ();
	int nu = _model->getNU();

	// COPY STATES AND CONTROLS
	memcpy(_yTmp,aY,ny*sizeof(double));
	memcpy(_xTmp,aX,nx*sizeof(double));

	// GRAVITY
	double g0[3] = { 0.0, 0.0, 0.0 };
	double g[3];  _model->getGravity(g);

	// ZERO GRAVITY
	_model->setGravity(g0);

	// ZERO VELOCITIES
	for(i=0;i<nu;i++) _yTmp[nq+i] = 0.0;

	// SET STATES
	_model->set(aT,_xTmp,_yTmp);
	_model->getDerivCallbackSet()->set(aT,_xTmp,_yTmp);

	// APPLY ACTION FORCES SELECTIVELY
	// ACTUATOR
	if(aC<=getLastActuatorIndex()) {
		_model->computeActuation();
		_model->getDerivCallbackSet()->computeActuation(_model->getTime(),_xTmp,_yTmp);
		_model->applyActuatorForce(aC);
		_model->getDerivCallbackSet()->applyActuation(_model->getTime(),_xTmp,_yTmp);

	// GRAVITY
	} else if(aC==getGravityIndex()) {
		_model->setGravity(g);
		_model->setStates(_yTmp);

	// CENTRIPETAL
	} else if(aC==getVelocityIndex()) {
		for(i=0;i<nu;i++) _yTmp[nq+i] = aY[nq+i];
		_model->setStates(_yTmp);
	}
}

//-----------------------------------------------------------------------------
// INITIALIZATION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Initialize the force values for the component elements.
 */
void DecompTaylor::
initializeForceElements()
{
	if(_model==NULL) return;

	// NUMBERS
	int np = _model->getNP();

	// ZERO
	int c,i,j,I;
	for(c=0;c<getNumComponents();c++) {
		for(i=0;i<np;i++) {
			for(j=0;j<3;j++) {
				I = Mtx::ComputeIndex(c,np,i,3,j);
				_f[c][i][j] = 0.0;
				_fp[I] = 0.0;
				_fv[I] = 0.0;
			}
		}
	}

	// COMPUTE CONTACT FORCES
	computeContactForces(_sfrc);

	// CONTACT FLAG
	for(i=0;i<np;i++) {
		I = Mtx::ComputeIndex(i,3,1);
		if(fabs(_sfrc[I])>_contactThreshold) {
			_contactEstablished[i] = true;
			_contactJustEstablished[i] = true;
		} else {
			_contactEstablished[i] = false;
			_contactJustEstablished[i] = false;
		}
	}

	// INITIALIZE
	char fileName[2048];
	double *feInit;
	StateVector *initVec=NULL;
	Storage *store=NULL;
	for(c=0;c<=getVelocityIndex();c++) {

		// LOAD DATA
		sprintf(fileName,"./DecompInit/fs_%s.xls",_cNames[c].c_str());
		store = new Storage(fileName);
		if(store->getSize()<1) {
			printf("DecompTaylor.integBeginCallback: WARN- failed to open ");
			printf("file %s\n",fileName);
			printf("\tInitial force values for component %s were set to zero.\n",
			 _cNames[c].c_str());
			if(store!=NULL) { delete store;  store=NULL; }
			continue;
		}

		// GET DATA
		initVec = store->getStateVector(store->getSize()-1);
		feInit = initVec->getData().get();

		// SET DATA
		for(i=0;i<np;i++) {
			for(j=0;j<3;j++) {
				I = Mtx::ComputeIndex(i,3,j);
				_f[c][i][j] = feInit[I];
			}
		}

		// CLEANUP
		if(store!=NULL) { delete store;  store=NULL; }
	}

	// ATTRIBUTE ALL UNACCOUNTED FOR FORCE TO THE VELOCITY COMPONENT
	//sumSpringForces();
	//for(i=0;i<NS;i++) {
	//	for(j=0;j<3;j++) {
	//		_fe[CVEL][i][j] = aSFrc[i][j] - _fe[CALL][i][j];
	//	}
	//}
}

//_____________________________________________________________________________
/**
 * Initialize the velocities values for the component elements.
 */
void DecompTaylor::
initializeVelocityElements()
{
	if(_model==NULL) return;

	// NUMBERS
	int np = _model->getNP();

	// BRING CONTACT COMPUTATIONS UP-TO-DATE
	_model->computeContact();

	// COMPUTE CONTACT VELOCITIES
	computeContactVelocities(_svel);

	// INITIALIZE
	int c,i,j,I,J;
	for(c=0;c<getNumComponents();c++) {
		for(i=0;i<np;i++) {
			for(j=0;j<3;j++) {
				J = Mtx::ComputeIndex(c,np,i,3,j);
				if(c==getVelocityIndex()) {
					I = Mtx::ComputeIndex(i,3,j);
					_v[J] = _svel[I];
				} else {
					_v[J] = 0.0;
				}
			}
		}
	}
}

//-----------------------------------------------------------------------------
// FRICTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute the factors for accounting for reductions in the
 * contact force made to enforce frictional constraints.
 *
 * @param aT Time.
 * @param aX Controls.
 * @param aY States.
 * @todo Account for changes in the contact point on BodyB.
 */
void DecompTaylor::
computeFrictionFactors(double aT,double *aX,double *aY)
{
	if(_model==NULL) return;

	// NUMBERS
	int i,j,I,J;
	int nq = _model->getNQ();
	int nu = _model->getNU();
	int ny = _model->getNY();
	int np = _model->getNP();

	// SET
	_model->set(aT,aX,aY);
	_model->computeContact();

	// GET CURRENT CONTACT POINTS
	// Only for BodyA for now--- This should also be done for BodyB.
	// For most models now, the contact points on BodyB are not changing,
	// so this is unncessesary.
	for(i=0;i<np;i++) {
		I = Mtx::ComputeIndex(i,3,0);
		_model->getContactPointA(i,&_pa[I]);
	}

	// COMPUTE CONTACT FORCES
	computeContactForces(_sfrc);

	//---------------------------------------------
	// CORRECTION IN VISCOUS PART OF CONTACT FORCES
	// All reductions in contact forces should be due to reductions in damping
	// forces since the model has presumable been brought up-to-date.
	double fv[3],fp[3],f[3];
	double fnp[3],fnv[3],fn[3];
	double ftp[3],ftv[3],ft[3];
	for(i=0;i<np;i++) {

		// COMPUTE DAMPING CORRECTION
		I = Mtx::ComputeIndex(i,3,0);
		_model->getContactFrictionCorrection(i,&_dfvFric[I]);

		// COMPUTE FORCE BEFORE CORRECTION
		//Mtx::Add(1,3,&_sfrc[I],&_dfvFric[I],f);
		_model->getContactNormalForce(i,fnp,fnv,fn);
		_model->getContactTangentForce(i,ftp,ftv,ft);
		Mtx::Add(1,3,fnv,ftv,fv);
		Mtx::Add(1,3,fnp,ftp,fp);
		Mtx::Add(1,3,fn,ft,f);
		Mtx::Subtract(1,3,f,fp,f);

		//Mtx::Add(1,3,fnp,fnv,fn);
		//Mtx::Add(1,3,ftp,ftv,ft);

		// COMPUTE ALPHA
		for(j=0;j<3;j++) {
			J = Mtx::ComputeIndex(i,3,j);

			// CONTACT FORCE IS ~ZERO
			if(fabs(fv[j])<rdMath::ZERO) {
				_vAlpha[J] = 1.0;

			// CONTACT FORCE IS LARGE ENOUGH FOR GOOD COMPUTATION
			} else {
				//_vAlpha[J] = _sfrc[J]/f[j];
				_vAlpha[J] = f[j]/fv[j];
				if(fabs(_vAlpha[J]) <= rdMath::ZERO)  _vAlpha[J] = 0.0;
			}
		}

		// CHECK
		I = Mtx::ComputeIndex(i,3,1);
		if(_vAlpha[I]!=1.0) {
			//printf("ERROR-  friction in vertical direction!!!!\n");
			//printf("\tt=%lf  vAlpha[%d]=%le\n",aT,i,_vAlpha[I]);
		} 
	}

	//---------------------------------------------
	// CORRECTION IN ELASTIC PART OF CONTACT FORCES
	// To accomplish this the contact points on BodyA from the previous
	// time step are restored.

	// SET VELOCITIES TO ZERO
	//memcpy(_yTmp,aY,ny*sizeof(double));
	//for(i=0;i<nu;i++) _yTmp[nq+i] = 0.0;
	//_model->set(aT,aX,_yTmp);

	// COMPUTE ELASTIC PART OF CONTACT FORCES
	//_model->computeContact();
	//computeContactForces(_pfrc);
	double fPrev[3];
	double fnpPrev[3],fnvPrev[3],fnPrev[3];
	double ftpPrev[3],ftvPrev[3],ftPrev[3];

	// RESTORE PREVIOUS CONTACT POINTS
	for(i=0;i<np;i++) {
		I = Mtx::ComputeIndex(i,3,0);
		_model->setContactPointA(i,&_paPrev[I]);
	}
	_model->computeContact();

	// LOOP THROUGH SPRINGS
	for(i=0;i<np;i++) {

		// COMPUTE ELASTIC CORRECTION
		//I = Mtx::ComputeIndex(i,3,0);
		//_model->getContactFrictionCorrection(i,&_dfpFric[I]);
		//Mtx::Subtract(1,3,&_dfpFric[I],&_dfvFric[I],&_dfpFric[I]);

		// COMPUTE ELASTIC FORCE BEFORE CORRECTION
		//Mtx::Add(1,3,&_sfrc[I],&_dfpFric[I],f);
		_model->getContactNormalForce(i,fnpPrev,fnvPrev,fnPrev);
		_model->getContactTangentForce(i,ftpPrev,ftvPrev,ftPrev);
		Mtx::Add(1,3,fnp,ftp,f);
		Mtx::Add(1,3,fnpPrev,ftpPrev,fPrev);

		// COMPUTE ALPHA
		for(j=0;j<3;j++) {
			J = Mtx::ComputeIndex(i,3,j);

			// CONTACT FORCE IS ~ZERO
			if(fabs(fPrev[j])<rdMath::ZERO) {
				_pAlpha[J] = 1.0;

			// CONTACT FORCE IS LARGE ENOUGH FOR GOOD COMPUTATION
			} else {
				_pAlpha[J] = f[J]/fPrev[j];
			}
		}

		// CHECK
		I = Mtx::ComputeIndex(i,3,1);
		if(_pAlpha[I]!=1.0) {
			//printf("ERROR-  friction in vertical direction!!!!\n");
			//printf("\tt=%lf  pAlpha[%d]=%le\n",aT,i,_pAlpha[I]);
		} 
	}

	//---------------------------------------
	// RESET CONTACT POINTS TO CURRENT VALUES
	for(i=0;i<np;i++) {
		I = Mtx::ComputeIndex(i,3,0);
		_model->setContactPointA(i,&_pa[I]);
	}
	_model->computeContact();
}

//-----------------------------------------------------------------------------
// UTILITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute the force applied to BodyB expressed in the frame of BodyA
 * for all contact points.
 *
 * Prior to calling this method, the model states must be set and
 * Model::computeContact() should be called.
 *
 * @param rForces Forces of contact points arranged as follows:
 * rForces[point index][3], so the dimension of rForces should be np*3.
 */
void DecompTaylor::
computeContactForces(double *rForces)
{
	if(rForces==NULL) return;
	if(_model==NULL) return;

	// NUMBERS
	int np = _model->getNP();

	// COMPUTE
	int i,j,I;
	double f[3];
	for(i=0;i<np;i++) {

		_model->getContactForce(i,f);

		I = Mtx::ComputeIndex(i,3,0);
		for(j=0;j<3;j++) rForces[I+j] = f[j];
	}
}

//_____________________________________________________________________________
/**
 * Compute the velocities of all contact points expressed in the reference
 * frame of BodyA.
 *
 * Prior to calling this method, the model states must be set and
 * Model::computeContact() should be called.
 *
 * @param rVels Velocities of contact points arranged as follows:
 * rVels[point index][3], so the dimension of rVels should be np*3.
 */
void DecompTaylor::
computeContactVelocities(double *rVels)
{
	if(rVels==NULL) return;
	if(_model==NULL) return;

	// NUMBERS
	int np = _model->getNP();

	// COMPUTE CONTACT VELOCITIES
	int i,j,I;
	int a,b;
	double pa[3],pb[3];
	double va[3],vb[3];
	double v[3];
	for(i=0;i<np;i++) {
		a = _model->getContactBodyA(i);
		b = _model->getContactBodyB(i);

		_model->getContactPointA(i,pa);
		_model->getContactPointB(i,pb);

		_model->getVelocity(a,pa,va);
		_model->getVelocity(b,pb,vb);

		for(j=0;j<3;j++) v[j] = vb[j] - va[j];
		_model->transform(_model->getGroundID(),v,a,v);

		I = Mtx::ComputeIndex(i,3,0);
		for(j=0;j<3;j++) rVels[I+j] = v[j];
	}
}

//_____________________________________________________________________________
/**
 * Compute higher order derivatives of the accelerations of the generalized
 * coordinates.
 */
void DecompTaylor::
computeHigherOrderDerivatives(int c,double t,double *dudt,
	double *ddudt,double *dddudt)
{
	if(_model==NULL) return;

	// NUMBERS
	int i,I,J;
	int nu = _model->getNU();
	int np = _model->getNP();

	// SET NEW ACCELERATIONS HISTORY
	J = Mtx::ComputeIndex(c,3,0);
	_at[J+0] = _at[J+1];
	_at[J+1] = _at[J+2];
	_at[J+2] = t;
	for(i=0;i<nu;i++) {
		I = Mtx::ComputeIndex(c,nu,i,3,0);
		_ah[I+0] = _ah[I+1];
		_ah[I+1] = _ah[I+2];
		_ah[I+2] = dudt[i];
	}

	// CHECK FOR VALID HISTORY
	if((_at[J+0]==rdMath::NAN)||(_at[J+1]==rdMath::NAN)||
															(_at[J+2]==rdMath::NAN)) {
		for(i=0;i<nu;i++) {
			ddudt[i] = 0.0;
			dddudt[i] = 0.0;
		}
		return;
	}

	// FIT A PARABOLA TO THE POINTS
	double c0,c1,c2;
	for(i=0;i<nu;i++) {
		I = Mtx::ComputeIndex(c,nu,i,3,0);
		rdMath::FitParabola(_at[J+0],_ah[I+0],_at[J+1],_ah[I+1],
			_at[J+2],_ah[I+2],&c0,&c1,&c2);

		ddudt[i] = c1 + 2.0*c2*t;
		dddudt[i] = 2.0*c2;

		// PRINT
		if(c==-1) {
			printf("%lf %s dudt[%d] = %lf %lf %lf\n",
			t,_cNames[c].c_str(),i,dudt[i],ddudt[i],dddudt[i]);
		}
	}
}


