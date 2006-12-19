// IndContactPowers.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <string>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/Mtx.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Simulation/SIMM/AbstractModel.h>
#include <OpenSim/Simulation/SIMM/AbstractBody.h>
#include "IndContactPowers.h"



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
 */
IndContactPowers::~IndContactPowers()
{
	deleteStorage();
}
//_____________________________________________________________________________
/**
 * Construct an induced contact powers object based on a set of contact point
 * velocities, model states, and a contact force decomposition.
 *
 * @param aContactVelocities Velocities of the contact point expressed in the
 * global frame.  See class Contact.
 * @param aModel AbstractModel on which the simulation was run.
 * @param aStates Set of model states.
 * @param aBaseName Base name for the force decompositon files.  If NULL,
 * accelerations are computed based on a NULL decompostion.
 * @param aDir Directory in which the results reside.
 * @param aExtension File extension of the force decomposition files.
 * @see Contact, IndAcc
 */
IndContactPowers::IndContactPowers(Storage *aContactVelocities,
	AbstractModel *aModel,Storage *aStates,Storage *aControls,char *aBaseName,
	char *aDir,char *aExtension) :
	IndAcc(aModel,aStates,aControls,aBaseName,aDir,aExtension)
{
	printf("IndContactPowers: constructing from file.\n");
	printf("baseName = %s  aDir = %s  aExtension= %s\n",
		aBaseName,aDir,aExtension);

	setNull();

	// NAME
	setName("InducedContactPowers");

	// CONTACT POINT VELOCITIES
	_velStore = aContactVelocities;

	// DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();

	// STORAGE
	allocateStorage();
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set member variables to approprate NULL values.
 */
void IndContactPowers::
setNull()
{
	_velStore = NULL;
	_pwrStore = NULL;
}

//_____________________________________________________________________________
/**
 * Construct a description.
 */
void IndContactPowers::
constructDescription()
{
	string descrip;

	descrip = "\nThis file contains the powers delivered to contact";
	descrip += " elements\nby the individual actuators of a model";
	descrip += " during a simulation.\n";
	descrip += "\nUnits are S.I. units (second, meters, Newtons, ...)";
	descrip += "\n\n";

	setDescription(descrip);
}

//_____________________________________________________________________________
/**
 * Construct the column labels.
 */
void IndContactPowers::
constructColumnLabels()
{
	string labels;
	char tmp[Object::NAME_LENGTH];

	// GET GENERALIZED SPEED NAMES
	int i;
	AbstractBody *a, *b;
	labels = "Time";
	for(i=0;i<_model->getNumContacts();i++) {
		a = _model->getContactSet()->getContactBodyA(i);
		b = _model->getContactSet()->getContactBodyB(i);
		sprintf(tmp,"\t%d_%s_%s",i,
			a->getName().c_str(),b->getName().c_str());
		labels += tmp;
	}
	labels += "\tTotal\n";

	setColumnLabels(labels.c_str());
}

//_____________________________________________________________________________
/**
 * Allocate storage pointers for the contact powers.
 */
void IndContactPowers::
allocateStoragePointers()
{
	_pwrStore = new Storage*[_nc];

	int c;
	for(c=0;c<_nc;c++) {
		_pwrStore[c] = NULL;
	}
}
//_____________________________________________________________________________
/**
 * Allocate storage for the decomposition.
 */
void IndContactPowers::
allocateStorage()
{
	// POINTERS
	allocateStoragePointers();

	// CONSTRUCT
	int c;
	for(c=0;c<_nc;c++) {

		// FORCE
		_pwrStore[c] = new Storage(1000,"InducedContactPowers");
		_pwrStore[c]->setCapacityIncrement(1000);
		_pwrStore[c]->setDescription(getDescription());
		_pwrStore[c]->setColumnLabels(getColumnLabels());
	}
}

//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Delete storage objects.
 */
void IndContactPowers::
deleteStorage()
{
	// FORCE DECOMPOSITION
	int c;
	if(_pwrStore!=NULL) {
		for(c=0;c<_nc;c++) {
			if(_pwrStore[c]!=NULL) { delete _pwrStore[c];  _pwrStore[c]=NULL; }
		}
		delete []_pwrStore;
	}
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// CONTACT VELOCITIES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the contact velocities.
 *
 * @return Contact velocities.
 */
Storage* IndContactPowers::
getContactVelocities()
{
	return(_velStore);
}


//=============================================================================
// OPERATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute induced contact powers.
 */
void IndContactPowers::
computeContactPowers()
{
	// CHECK
	if(_velStore==NULL) {
		printf("IndContactPowers.computeContactPowers: ERROR- no contact ");
		printf("velocities.");
		return;
	}
	if(getUseNullDecomposition()) {
		printf("IndContactPowers.computeContactPowers: ERROR- NULL ");
		printf("decomposition.  Induced contact powers are zero.\n");
		return;
	}

	// NUMBERS
	int np = _model->getNumContacts();

	// LOOP OVER TIME
	int i,c,p;
	int V,F;
	int nfrc = 3*np;
	double t;
	double v[3],*vel;
	double *frc = new double[nfrc];
	double *pwr = new double[np+1];
	StateVector *velVec;
	for(i=0;i<_velStore->getSize();i++) {

		// GET TIME AND VELOCITIES
		velVec = _velStore->getStateVector(i);
		t = velVec->getTime();
		vel = velVec->getData().get();

		// LOOP OVER INDEPENDENT COMPONENTS
		for(c=0;c<_nc;c++) {

			// GET CONTACT FORCES
			if(!getUseNullDecomposition()) {
				_feStore[c]->getDataAtTime(t,nfrc,frc);
			}

			// CONTACT POWERS
			for(pwr[np]=0.0,p=0;p<np;p++) {
				V = Mtx::ComputeIndex(p,6,0);
				F = Mtx::ComputeIndex(p,3,0);
				Mtx::Subtract(1,3,&vel[V+3],&vel[V],v);
				pwr[p] = Mtx::DotProduct(3,v,&frc[F]);
				pwr[np] += pwr[p];
			}

			// STORE
			_pwrStore[c]->append(t,np+1,pwr);
		}
	}

	// CLEANUP
	if(pwr!=NULL) { delete[] pwr;  pwr=NULL; }
	if(frc!=NULL) { delete[] frc;  frc=NULL; }

	return;
}


//=============================================================================
// UTILITY
//=============================================================================


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
int IndContactPowers::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	// COMPONENTS
	for(int c=0;c<_nc;c++) {
		// INDUCED CONTACT POWERS
		if(!getUseNullDecomposition()) {
			Storage::printResult(_pwrStore[c],aBaseName+"_"+getName()+"_"+_cNames[c],aDir,aDT,aExtension);
		}
	}

	return(0);
}


