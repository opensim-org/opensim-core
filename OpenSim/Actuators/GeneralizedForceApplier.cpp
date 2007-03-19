// GeneralizedForceApplier.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson and Saryn Goldberg
// 
//
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Simulation/Model/AbstractModel.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/Model/AbstractCoordinate.h>
#include "GeneralizedForceApplier.h"

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________


using namespace OpenSim;
/**
 * Destructor.
 */
GeneralizedForceApplier::~GeneralizedForceApplier()
{
	deleteStorage();
}
//_____________________________________________________________________________
/**
 * Construct a derivative callback instance for applying a generalized force
 * during an integration.
 *
 * @param aModel AbstractModel for which generalized forces are to be applied.
 */
GeneralizedForceApplier::
GeneralizedForceApplier(AbstractModel *aModel) :
	DerivCallback(aModel)
{
	setNull();

	// BASE-CLASS MEMBER VARIABLES
	setType("GeneralizedForceApplier");

	// STORAGE
	allocateStorage();

	// DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();
}

//_____________________________________________________________________________
/**
 * Construct a derivative callback instance for applying a generalized forces
 * during an integration.
 *
 * @param aModel AbstractModel for which generalized forces are to be applied.
 * @param aGenCoord Generalized coordinate to which generalized forces are to be applied.
 */
GeneralizedForceApplier::
GeneralizedForceApplier(AbstractModel *aModel,AbstractCoordinate *aGenCoord) :
	DerivCallback(aModel)
{
	setNull();

	// MEMBER VARIABLES
	setGeneralizedCoordinate(aGenCoord);

	// STORAGE
	allocateStorage();

	// DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();
}

//_____________________________________________________________________________
/**
 * Construct a derivative callback instance for applying generalized forces
 * during an integration.
 *
 * @param aModel AbstractModel for which generalized forces are to be applied.
 * @param aGenCoord Generalized coordinate to which generalized forces are to be applied.
 * @param aGeneralizedForce Generalized force to be applied
 */
GeneralizedForceApplier::
GeneralizedForceApplier(AbstractModel *aModel,AbstractCoordinate *aGenCoord,double aGeneralizedForce) :
	DerivCallback(aModel)
{
	setNull();

	// MEMBER VARIABLES
	setGeneralizedCoordinate(aGenCoord);
	setGeneralizedForce(aGeneralizedForce);

	// STORAGE
	allocateStorage();

	// DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();

}

//_____________________________________________________________________________
/**
 * Construct a derivative callback instance for applying generalized forces
 * during an integration. 
 *
 * @param aModel AbstractModel for which generalized forces are to be applied.
 * @param aGenCoord Generalized coordinate to which generalized forces are to be applied.
 * @param aGeneralizedForceStorage Storage containing (t,x,y,z) of generalized force to be applied 
 * @param aIndex Collumn of storage corresponding to generalized force
 */
GeneralizedForceApplier::
GeneralizedForceApplier(AbstractModel *aModel,AbstractCoordinate *aGenCoord,Storage* aGeneralizedForceStorage,int aIndex) :
	DerivCallback(aModel)
{
	setNull();

	// MEMBER VARIABLES
	setGeneralizedCoordinate(aGenCoord);
	setGeneralizedForceStorage(aGeneralizedForceStorage);
	setGeneralizedForceIndex(aIndex);

	// STORAGE
	allocateStorage();

	// DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();

}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set member variables to approprate NULL values.
 */
void GeneralizedForceApplier::
setNull()
{
	setType("GeneralizedForceApplier");
	_genCoord = 0;
	_scaleFactor = 1.0;
	_generalizedForce = 0.0;
	setStartTime(0.0);
	setEndTime(1.0);
	_genForceIndex = 0;
	_generalizedForceStorage = NULL;
	_recordAppliedLoads = false;
	_appliedGeneralizedForceStore = NULL;
	_aTSet = -1.0;
}
//_____________________________________________________________________________
/**
 * Construct a description for the generalized force files.
 */
void GeneralizedForceApplier::
constructDescription()
{
	char descrip[1024];

	strcpy(descrip,"\nThis file contains the generalized forces ");
	strcat(descrip,"that were applied to the body segment,\n");
	strcat(descrip,"as a function of time.\n");
	strcat(descrip,"\nUnits are S.I. units (seconds, meters, Newtons, ...)");
	strcat(descrip,"\n\n");

	setDescription(descrip);
}

//_____________________________________________________________________________
/**
 * Construct column labels for the generalized force files.
 */
void GeneralizedForceApplier::
constructColumnLabels()
{
	char labels[2048];
	strcpy(labels,"time\tGenForce\n");
	_appliedGeneralizedForceStore->setColumnLabels(labels);
}

//_____________________________________________________________________________
/**
 * Allocate storage for the generalized forces.
 */
void GeneralizedForceApplier::
allocateStorage()
{
	_appliedGeneralizedForceStore = new Storage(1000,"Generalized forces applied to " + _genCoord->getName());
	_appliedGeneralizedForceStore->setDescription(getDescription());
	_appliedGeneralizedForceStore->setColumnLabels(_appliedGeneralizedForceStore->getColumnLabels());
}


//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Delete storage objects.
 */
void GeneralizedForceApplier::
deleteStorage()
{
	if(_appliedGeneralizedForceStore!=NULL) { delete _appliedGeneralizedForceStore;  _appliedGeneralizedForceStore=NULL; }
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// GENERALIZED COORDINATE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set to which generalized coordinate a generalized force should be applied.
 *
 * @param aIndex Index of the generalized coordinate to which a generalized force should be applied.
 */
void GeneralizedForceApplier::
setGeneralizedCoordinate(AbstractCoordinate *aGenCoord)
{
	_genCoord = aGenCoord;
}
//_____________________________________________________________________________
/**
 * Get to which generalized coordinate a generalized force should be applied.
 *
 * @return aIndex Index of the generalized coordinate to which a generalized force should be applied.
 */
AbstractCoordinate* GeneralizedForceApplier::
getGeneralizedCoordinate() const
{
	return(_genCoord);
}


//-----------------------------------------------------------------------------
// SCALE FACTOR
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the scale factor that the generalized force will be multiplied by
 * 
 * @param aFactor Scale factor that the generalized force will be multiplied by
 */
void GeneralizedForceApplier::
setScaleFactor(double aFactor)
{
	_scaleFactor = aFactor;
}
//_____________________________________________________________________________
/**
 * Get the scale factor that the generalized force will be multiplied by
 * @return rFactor
 */
double GeneralizedForceApplier::
getScaleFactor() const
{
	return(_generalizedForce);

}
//-----------------------------------------------------------------------------
// GENERALIZED FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the generalized force to be applied
 * 
 * @param aGeneralizedForce Generalized force to be applied.
 */
void GeneralizedForceApplier::
setGeneralizedForce(double aGeneralizedForce)
{
	_generalizedForce = aGeneralizedForce;
}
//_____________________________________________________________________________
/**
 * Get the generalized force to be applied
 * @return aGeneralizedForce
 */
double GeneralizedForceApplier::
getGeneralizedForce() const
{
		return(_generalizedForce);

}

//-----------------------------------------------------------------------------
// GENERALIZED FORCE STORAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the Storage containing the generalized force
 * to be applied.
 *
 * @param aGeneralizedForceStorage Storage containing the generalized force
 * to be applied
 */
void GeneralizedForceApplier::
setGeneralizedForceStorage(Storage* aGeneralizedForceStorage)
{
	_generalizedForceStorage = aGeneralizedForceStorage;
}
//_____________________________________________________________________________
/**
 * Get the Storage containing the generalized force
 * to applied.
 *
 * @return aGeneralizedForceStorage.
 */
Storage* GeneralizedForceApplier::
getGeneralizedForceStorage() const
{
	return(_generalizedForceStorage);
}
//_____________________________________________________________________________
/**
 * Set the index corresponding to the collumn of the generalized force storage
 * that contains the generalized force to be applied
 * 
 * @param aIndex Collumn in storage that contains the generalized force to apply.
 */
void GeneralizedForceApplier::
setGeneralizedForceIndex(int aIndex)
{
	_genForceIndex = aIndex;
}
//_____________________________________________________________________________
/**
 * Set the index corresponding to the collumn of the generalized force storage
 * that contains the generalized force to be applied
 *
 * @return rIndex
 */
int GeneralizedForceApplier::
getGeneralizedForceIndex() const
{
	return(_genForceIndex);

}
//-----------------------------------------------------------------------------
// APPLIED GENERALIZED FORCE STORAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether or not to record the loads that are applied during an
 * integration.  Recording these loads takes a lot of memory as they
 * are stored every time the derivatives are evaluated (e.g., 6 times per
 * integration step).
 *
 * @param aTrueFalse Flag to turn on and off recording of the applied loads.
 */
void GeneralizedForceApplier::
setRecordAppliedLoads(bool aTrueFalse)
{
	_recordAppliedLoads = aTrueFalse;
}
//_____________________________________________________________________________
/**
 * Get whether or not to record the loads that are applied during an
 * integration.  Recording these loads takes a lot of memory as they
 * are stored every time the derivatives are evaluated (e.g., 6 times per
 * integration step).
 *
 * @return True if the applied loads are being stored, false otherwise.
 */
bool GeneralizedForceApplier::
getRecordAppliedLoads() const
{
	return(_recordAppliedLoads);
}//_____________________________________________________________________________
/**
 * Get the generalized force storage.
 *
 * @return Applied generalized force storage.
 */
Storage* GeneralizedForceApplier::
getAppliedGeneralizedForceStorage()
{
	return(_appliedGeneralizedForceStore);
}
//-----------------------------------------------------------------------------
// STORAGE CAPACITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the capacity increments of applied generalized force storage.
 *
 * @param aIncrement Increment by which storage capacities will be increased
 * when storage capcities run out.
 */
void GeneralizedForceApplier::
setStorageCapacityIncrements(int aIncrement)
{
	_appliedGeneralizedForceStore->setCapacityIncrement(aIncrement);
}


//=============================================================================
// CALLBACKS
//=============================================================================
//_____________________________________________________________________________
/**
 * Callback called right after actuation has been applied by the model.
 *
 * *
 * @param aT Real time.
 * @param aX Controls.
 * @param aY States.
 */
void GeneralizedForceApplier::
applyActuation(double aT,double *aX,double *aY)
{
	double genForceToStore[1];
	double *genForceArray = new double[_model->getNumCoordinates()];
	double time;

	if(_model==NULL) {
		printf("GeneralizedForceApplier.applyActuation: WARN- no model.\n");
		return;
	}
	if(!getOn()) return;

	if((aT>=getStartTime()) && (aT<getEndTime())){

		if(aT>_aTSet){
			time = aT*_model->getTimeNormConstant();
			_aTSet = aT;
		}

		if(_generalizedForceStorage!=NULL) {
			_generalizedForceStorage->getDataAtTime(aT*_model->getTimeNormConstant(),_genForceIndex+1,genForceArray);
			setGeneralizedForce(genForceArray[_genForceIndex]);
		}
		
		_model->getDynamicsEngine().applyGeneralizedForce(*_genCoord,_generalizedForce*_scaleFactor);

		genForceToStore[0] = _generalizedForce;

		if(_recordAppliedLoads) _appliedGeneralizedForceStore->append(aT,1,genForceToStore);

	}	
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
int GeneralizedForceApplier::
printResults(const std::string &aBaseName,const std::string &aDir,double aDT,const std::string &aExtension)
{
	// ACCELERATIONS
	_appliedGeneralizedForceStore->scaleTime(_model->getTimeNormConstant());
	Storage::printResult(_appliedGeneralizedForceStore,aBaseName+"_body_"+_genCoord->getName()+"_appTorque",aDir,aDT,aExtension);

	return(0);
}




