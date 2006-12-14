// DecompInteg.cpp
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
#include <OpenSim/Simulation/Simm/AbstractModel.h>
#include <OpenSim/Simulation/Simm/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/Simm/AbstractBody.h>
#include "DecompInteg.h"

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
DecompInteg::~DecompInteg()
{
	// DELETE STATE AND CONTROLS STORAGE OBJECTS
	Storage *store;
	store = _manager->getIntegrand()->getStateStorage();
	if(store!=NULL) { delete store;  store=NULL; }
	store = _manager->getIntegrand()->getControlStorage();
	if(store!=NULL) { delete store;  store=NULL; }

	// DELETE MEMBER VARIABLES
	if(_manager!=NULL) { delete _manager;  _manager=NULL; }
	if(_contact!=NULL) { delete _contact;  _contact=NULL; }
	if(_y!=NULL) { delete[] _y;  _y=NULL; }
	if(_fAct!=NULL) { delete[] _fAct;  _fAct=NULL; }
	if(_pctx!=NULL) { delete[] _pctx;  _pctx=NULL; }
	if(_fctx!=NULL) { delete[] _fctx;  _fctx=NULL; }
	if(_perturbCallback!=NULL) {delete _perturbCallback; _perturbCallback=NULL;}
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
DecompInteg::
DecompInteg(const Manager *aManager,const Contact *aContact,
	const Actuation *aActuation,AbstractModel *aModelTwin,double aDT,double aDF) :
	Decomp(aModelTwin)
{
	setNull();

	// PERTURBATION CALLBACK
	_perturbCallback = new ActuatorPerturbation(aModelTwin);
//	_perturbCallback->setPerturbation(aDF);
	_perturbCallback->setPerturbation(ActuatorPerturbation::SCALE,aDF);
	_perturbCallback->setName("DecompInteg_ActuatorPerturbation");

	// MEMBER VARIABLES
	setName("DecompInteg");
	setIntegrationWindow(aDT);

	// MANAGER
	_managerNom = aManager;
	constructManager();

	// CONTACT ANALYSIS
	_contactNom = aContact;
	constructContactAnalysis();

	// ACTUATION
	_actuationNom = aActuation;
	int na = _model->getNumActuators();
	if(na>0) _fAct = new double[na];

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
void DecompInteg::
setNull()
{
	_managerNom = NULL;
	_actuationNom = NULL;
	_contactNom = NULL;
	_fAct = NULL;
	_dt = 0.0;
	_manager= NULL;
	_contact = NULL;
	_iLast = -1;
	_y = NULL;
	_pctx = NULL;
	_fctx = NULL;
	_perturbCallback = NULL;
	_printWindow = false;
}
//_____________________________________________________________________________
/**
 * Construct a perturbed simulation manager.
 */
void DecompInteg::
constructManager()
{
	// NEW MANAGER WITH COPY OF NOMINAL CONTROLS
	ModelIntegrand *integrand = new ModelIntegrand(_model);
	_manager = new Manager(integrand);
	ControlSet *x = _managerNom->getIntegrand()->getControlSet();
	integrand->setControlSet(*x);

	// REPLICATE INTEGRATOR SETTINGS
	IntegRKF *integNom = _managerNom->getIntegrator();
	IntegRKF *integ = _manager->getIntegrator();
	integ->setMaximumNumberOfSteps(integNom->getMaximumNumberOfSteps());
	integ->setMaxDT(integNom->getMaxDT());
	integ->setTolerance(integNom->getTolerance());
	integ->setFineTolerance(integNom->getFineTolerance());
}
//_____________________________________________________________________________
/**
 * Construct a perturbed contact analysis.
 */
void DecompInteg::
constructContactAnalysis()
{
	// CONTACT
	_contact = new Contact(_model);
	_contact->setStepInterval(_contactNom->getStepInterval());
	_contact->setOn(true);
	_model->addAnalysis(_contact);
}
//_____________________________________________________________________________
/**
 * Construct a description.
 */
void DecompInteg::
constructDescription()
{
	printf("DecompInteg.constructDescription:\n");
	char tmp[1024],descrip[1024];

	strcpy(descrip,"\nThis file contains the reaction forces induced by\n");
	sprintf(tmp,"a particular action force of model %s.\n\n",
		_model->getName().c_str());
	strcat(descrip,tmp);

	strcat(descrip,
		"The decomposition was performed by perturbed integration.\n");

	sprintf(tmp,"\nIntegration Window:\n%20.15lf\n",
		getIntegrationWindow());
	strcat(descrip,tmp);

//	if(getUseAbsolutePerturbation()) {
//		sprintf(tmp,"\nAbsolute Perturbation:\n%20.15lf\n",
//			getPerturbation());
//	} else {
//		sprintf(tmp,"\nPerturbation Factor:\n%20.15lf\n",
//			getPerturbationFactor());
//	}
	sprintf(tmp,"\nPerturbation Type:\n%d\nPerturbation Factor:\n%20.15lf\n",
		getPerturbationType(),getPerturbation());
	strcat(descrip,tmp);

	strcat(descrip,"\n\nUnits are S.I. units (seconds, meters, Newtons, ...)");
	strcat(descrip,"\n\n");

	setDescription(descrip);
}
//_____________________________________________________________________________
/**
 * Update storage descriptions.
 */
void DecompInteg::
updateStorageDescriptions()
{
	if(_fStore==NULL) return;

	int c;
	for(c=0;c<getNumComponents();c++) {
		if(_fStore[c]==NULL) continue;
		_fStore[c]->setDescription(getDescription());
	}
}

//_____________________________________________________________________________
/**
 * Allocate memory.
 */
void DecompInteg::
allocate()
{
	_y = new double[_model->getNumStates()];
	_pctx = new double[6*_model->getNumContacts()];
	_fctx = new double[6*_model->getNumContacts()];
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
 */
void DecompInteg::
setIntegrationWindow(double aDT)
{
	_dt = aDT;
	if(_dt<0.0) _dt=0.0;

	// RECONSTRUCT THE DESCRIPTION
	constructDescription();
	updateStorageDescriptions();
}

//_____________________________________________________________________________
/**
 * Get the size of the integration window used to approximate the changes
 * in the ground reaction force.
 *
 * @return Integration window.
 * @see setIntegrationWindow()
 */
double DecompInteg::
getIntegrationWindow() const
{
	return(_dt);
}

//-----------------------------------------------------------------------------
// PERTURBATION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the force perturbation.
 *
 * @param aDF Force perturbation.
 */
void DecompInteg::
setPerturbation(ActuatorPerturbation::PertType aPerturbationType,double aDF)
//setPerturbation(double aDF)
{
	//_perturbCallback->setPerturbation(aDF);
	_perturbCallback->setPerturbation(ActuatorPerturbation::SCALE,aDF);

	// RECONSTRUCT THE DESCRIPTION
	constructDescription();
	updateStorageDescriptions();
}
//_____________________________________________________________________________
/**
 * Get the force perturbation.
 *
 * @return Force perturbation.
 */
double DecompInteg::
getPerturbation() const
{
	return(_perturbCallback->getPerturbation());
}
//_____________________________________________________________________________
/**
 * Get the force perturbation type.
 *
 * @return Force perturbation type.
 */
ActuatorPerturbation::PertType DecompInteg::
getPerturbationType() const
{
	return(_perturbCallback->getPerturbationType());
}

//-----------------------------------------------------------------------------
// PERTURBATION FACTOR
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the factor for the size of the force perturbation that is made to the
 * action forces.
 *
 * The actual force perturbation (df) that is made is computed by multiplying
 * the action force (f) by the perturbation factor (factor):
 *		df = factor * f.
 *
 * The perturbation factor may be any value; however, a value between 0.0 and
 * 1.0 what is typically used.
 *
 * @param aFactor Perturbation factor.
 */
//void DecompInteg::
//setPerturbationFactor(double aFactor)
//{
//	_perturbCallback->setPerturbationFactor(aFactor);

	// RECONSTRUCT THE DESCRIPTION
//	constructDescription();
//	updateStorageDescriptions();
//}
//_____________________________________________________________________________
/**
 * Get the factor for the size of the force perturbation that is made to the
 * action forces.
 *
 * @param aFactor Perturbation factor.
 * @see setPerturbationFactor()
 */
//double DecompInteg::
//getPerturbationFactor() const
//{
//	return(_perturbCallback->getPerturbationFactor());
//}

//-----------------------------------------------------------------------------
// PERTURBATION TYPE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set wheter to use an absolute force perturbation.  The alternative is a
 * relative perturbation that is the perturbation factor times the
 * actuator force.
 *
 * @param aTrueFalse True indicates to us an absolute perturbation.  False
 * indicates to use a relative perturbation.
 */
//void DecompInteg::
//setUseAbsolutePerturbation(bool aTrueFalse)
//{
//	_perturbCallback->setUseAbsolutePerturbation(aTrueFalse);
//}
//_____________________________________________________________________________
/**
 * Get wheter to use an absolute force perturbation.  The alternative is a
 * relative perturbation that is the perturbation factor times the
 * actuator force.
 *
 * @param aTrueFalse True indicates to us an absolute perturbation.  False
 * indicates to use a relative perturbation.
 */
//bool DecompInteg::
//getUseAbsolutePerturbation() const
//{
//	return(_perturbCallback->getUseAbsolutePerturbation());
//}

//-----------------------------------------------------------------------------
// PRINT WINDOW
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set wheter to print detailed information concerning the integration
 * window.
 *
 * @param aTrueFalse True indicates to print information; False not.
 */
void DecompInteg::
setPrintWindow(bool aTrueFalse)
{
	_printWindow = aTrueFalse;
}
//_____________________________________________________________________________
/**
 * Get wheter to print detailed information concerning the integration
 * window.
 *
 * @return True indicates that printing is on; False not.
 */
bool DecompInteg::
getPrintWindow() const
{
	return(_printWindow);
}


//=============================================================================
// OPERATIONS
//=============================================================================
//-----------------------------------------------------------------------------
// COMPUTE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute the reaction force decomposition based on perturbed integrations.
 */
void DecompInteg::
compute(double *aXPrev,double *aYPrev,
		int aStep,double aDT,double aT,double *aX,double *aY)
{
/*	if(getUseAbsolutePerturbation()) {
		printf("DecompInteg.compute: dt=%lf df=%lf\n",
		getIntegrationWindow(),getPerturbation());
	} else {
		printf("DecompInteg.compute: dt=%lf dfFactor=%lf\n",
		getIntegrationWindow(),getPerturbationFactor());
	}
*/	//_perturbCallback->setOn(false);

		printf("DecompInteg.compute: dt=%lf df=%lf %d\n",
		getIntegrationWindow(),getPerturbation(), getPerturbationType());

	// REAL TIME
	double tReal = aT * _model->getTimeNormConstant();

	// GET CONTACT POINTS
	int np = _managerNom->getIntegrand()->getModel()->getNumContacts();
	Storage *pointsNom = _contactNom->getPointsStorage();

	// DESIRED INITIAL TIME
	double t,ti,tf;
	t = pointsNom->getLastTime();
	ti = t - _dt;

	// RETURN IF NOT ENOUGH HISTORY TO PERFORM PERTURBED INTEGRATION
	if(ti < pointsNom->getFirstTime()) return;

	// INITIAL TIME
	int i = pointsNom->findIndex(_iLast,ti);
	if(i == _iLast) return;
	_iLast = i;
	ti = pointsNom->getStateVector(i)->getTime();

	// FINAL TIME
	tf = ti + _dt;
	i = pointsNom->findIndex(_iLast,tf);
	tf = pointsNom->getStateVector(i)->getTime();

	// INITIAL AND FINAL TIME
	double tNormConst = _model->getTimeNormConstant();
	_manager->setInitialTime(ti/tNormConst);
	_manager->setFinalTime(tf/tNormConst);

	// INITIAL STATES
	Storage *yStore = _managerNom->getIntegrand()->getStateStorage();
	yStore->getDataAtTime(ti,_model->getNumStates(),_y);
	_model->setInitialStates(_y);

	// ACTUATOR FORCES
	Storage *fActStore = _actuationNom->getForceStorage();
	fActStore->getDataAtTime(tf,_model->getNumActuators(),_fAct);

	// INITIAL CONTACT POINTS
	pointsNom->getDataAtTime(ti,6*np,_pctx);

	// RECIPROCAL OF PERTURBATION FACTOR
	double df = _perturbCallback->getPerturbation();
	double dfRecip = 1.0;
	if(df!=0.0) {
		dfRecip = 1.0/df;
	}

	// LOOP OVER ACTUATORS
	int c,j,I;
	AbstractBody *a;
	static double com[] = { 0.0, 0.0, 0.0 };
	double pcom[3],pctx[3];
	char outName[Object::NAME_LENGTH];
	StateVector *vec;
	Storage *forces,*forcesNom;
	AbstractActuator *actuator;
	int lastIndex = getLastActuatorIndex();
	for(c=0;c<=lastIndex;c++) {

		// SET WHICH ACTUATOR FORCE TO PERTURB
		actuator = _model->getActuatorSet()->get(c);
		_perturbCallback->setActuator(actuator);

		// SET THE MODEL STATES
		_model->set(aT,aX,_y);

		// SET CONTACT POINTS
		for(i=0;i<np;i++) {
			I = Mtx::ComputeIndex(i,6,0);

			// CONTACT POINT A
			a = _model->getContactSet()->getContactBodyA(i);
			_model->getDynamicsEngine().getPosition(*a,com,pcom);
			Mtx::Subtract(1,3,&_pctx[I],pcom,pctx);
			_model->getDynamicsEngine().transform(_model->getDynamicsEngine().getGroundBody(),pctx,*a,pctx);
			_model->getContactSet()->setContactPointA(i,pctx);

			// CONTACT POINT B
			//b = _model->getContactBodyB(i);
			//_model->getPosition(b,com,pcom);
			//Mtx::Subtract(1,3,&_pctx[I+3],pcom,pctx);
			//_model->transform(_model->getGroundID(),pctx,b,pctx);
			//_model->setContactPointB(i,pctx);
		}

		// EXECUTE THE PERTURBED INTEGRATION
		_manager->integrate();

		// GET PERTURBED CONTACT FORCES
		forces = _contact->getForceStorage();

		// PRINT TIME INFO
		printf("ti= %lf tf=%lf  fti=%lf  ftf=%lf  delta=%lf\n",ti,tf,
			forces->getFirstTime(),forces->getLastTime(),tf-ti);

		// PRINT PERTURBED CONTACT FORCES AND STATES
		if(_printWindow) {

			// PERTURBED CONTACT RESULTS
			sprintf(outName,"fs_%s_%.5lf_dt%.3lf_df%.3lf.sto",
				actuator->getName().c_str(),tf,getIntegrationWindow(),
				getPerturbation());
			forces->print(outName);

			// PERTURBED STATES RESULTS
			//Storage *states = _manager->getIntegrator()->getStateStorage();
			//sprintf(outName,"states_%.10lf",tf);
			//states->print(outName);
		}

		// COMPUTE DIFFERENCE IN CONTACT FORCES
		forcesNom = _contactNom->getForceStorage();
		forces->subtract(forcesNom);

		// COMPUTE DECOMPOSITION
		for(i=0;i<forces->getSize();i++) {
			vec = forces->getStateVector(i);  if(vec==NULL) continue;
			t = vec->getTime();
			fActStore->getDataAtTime(t,_model->getNumActuators(),_fAct);
			vec->multiply(dfRecip*_fAct[c]);
		}

		// PRINT WINDOW
		if(_printWindow) {
			sprintf(outName,"fsm_%s_%.5lf_dt%.3lf_df%.3lf.sto",
				actuator->getName().c_str(),tf,getIntegrationWindow(),
				getPerturbation());
			forces->print(outName);
		}

		// EXTRACT GROUND REACTION
		int I1,I2;
		forces->getDataAtTime(tf,6*np,_fctx);
		for(i=0;i<np;i++) {
			for(j=0;j<3;j++) {
				I1 = Mtx::ComputeIndex(i,3,j);
				I2 = Mtx::ComputeIndex(i,6,j+3);
				_fctx[I1] = _fctx[I2];
			}
		} 

		// APPEND RESULTS
		_fStore[c]->append(tf,3*np,_fctx);
	}
}



