// DecompNoComp.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <string>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/Mtx.h>
#include <OpenSim/Simulation/Simm/AbstractModel.h>
#include "DecompNoComp.h"




using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTANTS
//=============================================================================

// NAMES
const char *DecompNoComp::ADDON_COMPONENT_NAMES[] = {
	"Gravity","Velocity","Inertial","Actuators","All"
};



//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
DecompNoComp::~DecompNoComp()
{
	// CONTACT FLAG
	if(_contactEstablished!=NULL)
		{ delete []_contactEstablished;  _contactEstablished=NULL; }
	
	// FORCE ELEMENT ARRAYS
	int c;
	if(_fContig!=NULL) { delete []_fContig;  _fContig=NULL; }
	if(_f!=NULL) {
		for(c=0;c<_nc;c++) {
			if(_f[c]!=NULL) { delete []_f[c];  _f[c]=NULL; }
		}
		delete []_f;  _f=NULL;
	}

	// STORAGE
	deleteStorage();
}
//_____________________________________________________________________________
/**
 * Construct an decomposition instance for a model.
 * This constructor is used if the decomposition analysis is going to
 * be performed during the course of a simulation.
 *
 * @param aModel AbstractModel on which the analyses are to be performed.
 */
DecompNoComp::DecompNoComp(AbstractModel *aModel) :
	Analysis(aModel)
{
	int i;

	setNull();

	// NAME
	setName("Decomp");

	// INITIALIZE NUMBERS
	initializeNumbers();

	// CONTACT
	_contactEstablished = new bool[_np];
	for(i=0;i<_np;i++) {
		_contactEstablished[i] = false;
	}

	// ALLOCATE ELEMENT VECTORS
	allocateElementVectors();

	// DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();

	// STORAGE
	allocateStorage();
}
//_____________________________________________________________________________
/**
 * Construct an induced acceleration instance for a model.  This constructor
 * is used when the analysis is to be performed following a simulation.
 * The states recorded during the simulation as well as an
 * appropriate force decomposition must be used to construct the induced
 * acceleration instance.  The induced accelerations are not read in from
 * file.  They are recomputed based on the force decomposition.
 *
 * If NULL is sent in as the base name for the force decomposition
 * files, it is assumed that induced accelerations are to be computed
 * based on a NULL decomposition.  A NULL decomposition is one in which
 * all induced contact forces are assmed to be zero:  each actuator
 * accelerates the model in a pure sense, without its associated reaction
 * forces.
 *
 * @param aModel AbstractModel on which the simulation was run.
 * @param aStates Set of model states.
 * @param aBaseName Base name for the force decompositon files.  If NULL,
 * accelerations are computed based on a NULL decompostion.
 * @param aDir Directory in which the results reside.
 * @param aExtension File extension of the force decomposition files.
 */
DecompNoComp::DecompNoComp(AbstractModel *aModel,char *aBaseName,char *aDir,char *aExtension) :
	Analysis(aModel)
{
	printf("DecompNoComp: constructing induced acceleration analysis from file.\n");
	printf("baseName = %s  aDir = %s  aExtension= %s\n",
		aBaseName,aDir,aExtension);

	setNull();

	// NAME
	setName("indAcc");

	// INITIALIZE NUMBERS
	initializeNumbers();

	// CONTACT
	_contactEstablished = new bool[_np];
	int i;
	for(i=0;i<_np;i++) {
		_contactEstablished[i] = false;
	}

	// ALLOCATE ELEMENT VECTORS
	allocateElementVectors();

	// DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();

	// STORAGE
	allocateStoragePointers();

	// READ DECOMPOSITION
	if(aBaseName==NULL) {
		_useNullDecomposition = true;
		createNullDecomposition();
	} else {
		readDecomposition(aBaseName,aDir,aExtension);
	}
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set member variables to approprate NULL values.
 */
void DecompNoComp::
setNull()
{
	_nc = 0;
	_nic = 0;
	_np = 0;

	_contactThreshold = 0.0;
	_usePresetContactEstablishedSettings = false;
	_contactEstablished = NULL;
	_fContig = NULL;
	_f = NULL;
	_fStore = NULL;
	_useNullDecomposition = false;
	_cpaStore = NULL;
	_recordContactPointAccelerations = false;
}
//_____________________________________________________________________________
/**
 * Initialize numbers of things.
 */
void DecompNoComp::
initializeNumbers()
{
	// NUMBERS OF THINGS
	int na = _model->getNumActuators();
	_nc = 1;
	_nic = 1;
	_np = _model->getNumContacts();

}
//-----------------------------------------------------------------------------
// DESCRIPTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct a description.
 */
void DecompNoComp::
constructDescription()
{
	printf("DecompNoComp.constructDescription:\n");
	char tmp[1024],descrip[1024];

	strcpy(descrip,"\nThis file contains the reaction forces induced by\n");
	sprintf(tmp,"a particular action force of model %s.\n\n",
		_model->getName().c_str());
	strcat(descrip,tmp);
	strcat(descrip,"\nUnits are S.I. units (seconds, meters, Newtons, ...)");
	strcat(descrip,"\n\n");

	setDescription(descrip);
}
//_____________________________________________________________________________
/**
 * Construct the column labels.
 */
void DecompNoComp::
constructColumnLabels()
{
	// CREATE FORCE HEADERS
	int i;
	char label[256],labels[2048];
	strcpy(labels,"time");
	for(i=0;i<_np;i++) {
		sprintf(label,"\tp%dx\tp%dy\tp%dz",i,i,i);
		strcat(labels,label);
	}

	setColumnLabels(labels);
}

//_____________________________________________________________________________
/**
 * Allocate element vectors.
 */
void DecompNoComp::
allocateElementVectors()
{
	int c,i,j,I;

	// FORCE
	// Note- memory must be contiguous!
	_fContig = new double[_nc*_np*3];
	_f = new double**[_nc];
	for(c=0;c<_nc;c++) {
		_f[c] = new double*[_np];
		for(i=0;i<_np;i++) {
			I = Mtx::ComputeIndex(c,_np,i,3,0);
			_f[c][i] = &_fContig[I];
			for(j=0;j<3;j++) _f[c][i][j] = 0.0;
		}
	}
}
//_____________________________________________________________________________
/**
 * Allocate storage pointers for the force decomposition and induced
 * accelerations.
 */
void DecompNoComp::
allocateStoragePointers()
{
	_fStore = new Storage*[_nc];

	int c;
	for(c=0;c<_nc;c++) {
		_fStore[c] = NULL;
	}
}
//_____________________________________________________________________________
/**
 * Allocate storage for the decomposition.
 */
void DecompNoComp::
allocateStorage()
{
	// ALLOCATE STORAGE POINTERS
	allocateStoragePointers();

	// ALLOCATE STORAGE
	int c;
	for(c=0;c<_nc;c++) {
		_fStore[c] = new Storage(1000,"ForceDecomposition");
		_fStore[c]->setDescription(getDescription());
		_fStore[c]->setColumnLabels(getColumnLabels());
		_fStore[c]->setCapacityIncrement(1000);
	}
}
//_____________________________________________________________________________
/**
 * Create a NULL decomposition.
 */
void DecompNoComp::
createNullDecomposition()
{
	// ZERO VECTOR
	int i;
	int n = 3*_model->getNumContacts();
	if(n<=0) return;
	double *zero = new double[n];
	for(i=0;i<n;i++) zero[i] = 0.0;

	// ALLOCATE STORAGE
	int c;
	for(c=0;c<_nc;c++) {
		_fStore[c] = new Storage(1,"NullDecomposition");
		_fStore[c]->append(0.0,n,zero);
	}

	// CLEANUP
	delete[] zero;
}


//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Delete storage objects.
 */
void DecompNoComp::
deleteStorage()
{
	// FORCE DECOMPOSITION
	int c;
	if(_fStore!=NULL) {
		for(c=0;c<_nc;c++) {
			if(_fStore[c]!=NULL) { delete _fStore[c];  _fStore[c]=NULL; }
		}
		delete []_fStore;
	}
	if(_cpaStore!=NULL) {
		for(c=0;c<_nc;c++) {
			if(_cpaStore[c]!=NULL) { delete _cpaStore[c];  _cpaStore[c]=NULL; }
		}
		delete []_cpaStore;
	}
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// COMPONENTS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the number of components
 */
int DecompNoComp::
getNumComponents()
{
	return(_nc);
}
//_____________________________________________________________________________
/**
 * Get the number of independent components.
 */
int DecompNoComp::
getNumIndependentComponents()
{
	return(_nic);
}
//_____________________________________________________________________________
/**
 * Get the number of contact elements.
 */
int DecompNoComp::
getNumElements()
{
	return(_np);
}

//-----------------------------------------------------------------------------
// CONTACT THREASHOLD
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the force threashold above which contact is assumed to be established.
 *
 * @param aThreashold Force threashold above which contact is established.
 */
void DecompNoComp::
setContactThreshold(double aThreshold)
{
	_contactThreshold = aThreshold;
}
//_____________________________________________________________________________
/**
 * Get the force threashold above which contact is assumed to be established.
 *
 * @return Force threashold above which contact is established.
 */
double DecompNoComp::
getContactThreshold()
{
	return(_contactThreshold);
}

//-----------------------------------------------------------------------------
// USE PRESET CONTACT ESTABLISHED SETTINGS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether or not to use preset contact established settings.
 * If this flag is set to true, computation of contact forces
 * is not performed in order to determine if contact has been established.
 * Rather, it is assumed that the caller has preset the desired
 * contact-established settings.
 *
 * @param aTrueFalse If true, preset contact established settings are used
 * in the decomposition.  If false, contact forces are computed to determine
 * if contact has been established.
 * @see setContactEstablished()
 */
void DecompNoComp::
setUsePresetContactEstablishedSettings(bool aTrueFalse)
{
	_usePresetContactEstablishedSettings = aTrueFalse;
}
//_____________________________________________________________________________
/**
 * Get whether or not to use preset contact established settings.
 * If this flag is set to true, computation of contact forces
 * is not performed in order to determine if contact has been established.
 * Rather, it is assumed that the caller has preset the desired
 * contact-established settings.
 *
 * @return True if preset contact established settings are used
 * in the decomposition.  False if contact forces are computed to determine
 * if contact has been established.
 * @see setContactEstablished()
 */
bool DecompNoComp::
getUsePresetContactEstablishedSettings() const
{
	return(_usePresetContactEstablishedSettings);
}

//-----------------------------------------------------------------------------
// CONTACT ESTABLISHED
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether or not contact has been established at a specified
 * contact point.
 *
 * @param aIndex Index of the contact point: 0 <= aIndex < AbstractModel::getNumContacts().
 * @param aTrueFalse Wheter or not contact has been established.
 * @see setUsePresetContactEstablished()
 */
void DecompNoComp::
setContactEstablished(int aIndex,bool aTrueFalse)
{
	if((aIndex<0)||(aIndex>=_model->getNumContacts())) {
		printf("DecompNoComp.setContactEstablished: WARN- index out of range.\n");
		return;
	}

	_contactEstablished[aIndex] = aTrueFalse;
}
//_____________________________________________________________________________
/**
 * Get whether or not contact has been established at a specified
 * contact point.
 *
 * @param aIndex Index of the contact point: 0 <= aIndex < AbstractModel::getNumContacts().
 * @return True if contact has been established; false if not or on an error.
 * @see setUsePresetContactEstablished()
 */
bool DecompNoComp::
getContactEstablished(int aIndex) const
{
	if((aIndex<0)||(aIndex>=_model->getNumContacts())) {
		printf("DecompNoComp.getContactEstablished: WARN- index out of range.\n");
		return(false);
	}

	return(_contactEstablished[aIndex]);
}

//-----------------------------------------------------------------------------
// RECORD CONTACT POINT ACCELERATIONS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Determine whether or not contact point accelerations will be recorded.  If
 * set to true, space is allocated to store them.
 *
 * @param aTrueFalse 
 */
void DecompNoComp::
setRecordContactPointAccelerations(bool aTrueFalse)
{
	_recordContactPointAccelerations = aTrueFalse;

	if(_recordContactPointAccelerations)
	{
		_cpaStore = new Storage*[_nc];

		int c;
		for(c=0;c<_nc;c++) {
		_cpaStore[c] = new Storage(1000,"ContactPointAccelerations");
		_cpaStore[c]->setColumnLabels(getColumnLabels());
		_cpaStore[c]->setCapacityIncrement(1000);
		}
	}
}

//_____________________________________________________________________________
/**
 * Get whether or not the contact points are being recorded.
 *
 * @return bool
 */
bool DecompNoComp::
getRecordContactPointAccelerations()
{
	return(_recordContactPointAccelerations);
}

//-----------------------------------------------------------------------------
// FORCE STORAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the force decomposition for a particular component.
 *
 * @return Pointer to the force decomposition for component aC.
 */
const Storage* DecompNoComp::
getDecomposition(int aC) const
{
	if((aC<0)||(aC>=_nc)) return(NULL);
	return(_fStore[aC]);
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
void DecompNoComp::
setStorageCapacityIncrements(int aIncrement)
{
	int c;
	for(c=0;c<_nc;c++) {
		if(_fStore[c]!=NULL) _fStore[c]->setCapacityIncrement(aIncrement);
	}
}

//-----------------------------------------------------------------------------
// NULL DECOMPOSITION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get whether or not accelerations are being computed using a
 * NULL decomposition.  A NULL decomposition is one in which all
 * induced contact forces are assumed to be zero.
 *
 * The value returned by this method can be used to decide whether or not
 * it is necessary to apply contact forces.
 *
 * @return True when a NULL decomposition is used, false otherwise.
 */
bool DecompNoComp::
getUseNullDecomposition()
{
	return(_useNullDecomposition);
}


//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________


//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 * This method is meant to be called at the begining of an integration in
 * AbstractModel::integBeginCallback() and has the same argument list.
 *
 * This method should be overriden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that will be attempted.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int DecompNoComp::
begin(int aStep,double aDT,double aT,double *aX,double *aY,
		void *aClientData)
{
	if(!proceed()) return(0);

	compute(aX,aY,aStep,aDT,aT,aX,aY);

	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called to perform the analysis.  It can be called during
 * the execution of a forward integrations or after the integration by
 * feeding it the necessary data.
 *
 * When called during an integration, this method is meant to be called in
 * AbstractModel::integStepCallback(), which has the same argument list.
 *
 * This method should be overriden in derived classes.  It is
 * included here so that the derived class will not have to implement it if
 * it is not necessary.
 *
 * @param aXPrev Controls at the beginining of the current time step.
 * @param aYPrev States at the beginning of the current time step.
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that was just taken.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int DecompNoComp::
step(double *aXPrev,double *aYPrev,
	int aStep,double aDT,double aT,double *aX,double *aY,
	void *aClientData)
{
	if(!proceed(aStep)) return(0);

	compute(aX,aY,aStep,aDT,aT,aX,aY);

	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the end of an analysis so that any
 * necessary finalizations may be performed.
 *
 * This method is meant to be called at the end of an integration in
 * AbstractModel::integEndCallback() and has the same argument list.
 *
 * This method should be overriden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that was just completed.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int DecompNoComp::
end(int aStep,double aDT,double aT,double *aX,double *aY,
		void *aClientData)
{
	if(!proceed()) return(0);
	printf("DecompNoComp.end: Finalizing analysis %s.\n",getName().c_str());
	return(0);
}


//=============================================================================
// IO
//=============================================================================
//_____________________________________________________________________________
/**
 * Read the results of a force decomposition from file.
 *
 * Note that the induced accelerations are not read in from file.  The
 * induced accelerations are recomputed based on the force decomposition.
 *
 * The file names are constructed as
 * aDir + "/" + aBaseName + "_" + ComponentName + aExtension
 *
 * @param aDir Directory in which the results reside.
 * @param aBaseName Base file name of the files.
 * @param aExtension File extension.
 *
 * @return 0 on success, -1 on error.
 */
int DecompNoComp::
readDecomposition(char *aBaseName,char *aDir,char *aExtension)
{
	if(aBaseName==NULL) return(-1);

	// CONSTRUCT PATH
	char path[2048];
	if(aDir==NULL) {
		strcpy(path,".");
	} else {
		strcpy(path,aDir);
	}

	// COMPONENTS
	int c=0;
	char name[2048];

	// FORCE DECOMPOSITION
	if(aExtension==NULL) {
		sprintf(name,"%s/%s_noComp",path,aBaseName);
	} else {
		sprintf(name,"%s/%s_nComp%s",path,aBaseName,aExtension);
	}
	_fStore[c] = new Storage(name);


	return(0);
}
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
int DecompNoComp::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	if(!getOn()) {
		printf("DecompNoComp.printResults: Off- not printing.\n");
		return(-1);
	}
	if(getUseNullDecomposition()) return(-1);

	// COMPONENTS
	int c=0;
	if(_fStore) Storage::printResult(_fStore[c],aBaseName+"_"+getName()+"_noComp",aDir,aDT,aExtension);
	if(_cpaStore) Storage::printResult(_cpaStore[c],aBaseName+"_"+getName()+"_cpAccel_noComp",aDir,aDT,aExtension);
	
	return(0);
}


