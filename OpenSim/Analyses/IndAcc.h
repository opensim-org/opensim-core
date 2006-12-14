#ifndef _IndAcc_h_
#define _IndAcc_h_
// IndAcc.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include "suAnalysesDLL.h"


//=============================================================================
//=============================================================================
/**
 * A class that performs a basic induced acceleration analysis.
 *
 * An induced acceleration analysis can be peformed in two ways using this
 * class:  1) during the course of a simulation and 2) after a simulation
 * has completed.  For the second way, the states recorded during
 * the simulation as well an appropriate contact force decomposition must be
 * used to construct the IndAcc instance.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class AbstractModel;

class SUANALYSES_API IndAcc : public Analysis 
{
//=============================================================================
// DATA
//=============================================================================
protected:
	int _nc;
	int _nic;
	int _ne;
	int _cAct;
	int _cGrav;
	int _cVel;
	int _cIner;
	int _cAllAct;
	int _cAll;
	double _ti;
	double _tf;

	const char **_cNames;
	double _contactThreshold;
	bool *_contactEstablished;
	double *_feContig,***_fe;
	Storage *_yStore;
	Storage *_xStore;
	Storage **_feStore;
	Storage **_aeStore;
	Storage **_velStore;
	Storage **_posStore;
	Storage *_iPosStore;
	Storage *_iVelStore;
	char *_aeDescrip;
	char *_aeLabels;
	/** Flag that determines whether accelerations are normalized by force. */
	bool _computeNormalizedAccelerations;

private:
	/** Flag which indicates whether or not accelerations are being computed
	using a NULL decomposition. */
	bool _useNullDecomposition;

//=============================================================================
// METHODS
//=============================================================================
public:
	IndAcc(AbstractModel *aModel);
	IndAcc(AbstractModel *aModel,Storage *aStates,Storage *aControls,
		char *aBaseName,char *aDir=NULL,char *aExtension=NULL);
	virtual ~IndAcc();
private:
	void setNull();
	void initializeNumbers();
	void constructComponentNames();
	void constructDescription();
	void constructColumnLabels();
	void allocateElementVectors();
	void allocateStoragePointers();
	void allocateStorage();
	void deleteStorage();
	void createNullDecomposition();

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// NUMBERS
	int getNumComponents();
	int getNumIndependentComponents();
	int getNumElements();
	// INDICES
	int getLastActuatorIndex();
	int getGravityIndex();
	int getVelocityIndex();
	int getInertialIndex();
	int getAllActuatorsIndex();
	int getAllIndex();
	// CONTACT TOLERANCE
	void setContactThreshold(double aThreshold);
	double getContactThreshold();
	// NAMES
	const char* getComponentName(int aC);
	// STORAGE
	virtual void setStorageCapacityIncrements(int aIncrement);
	Storage** getForceStorage();
	// NULL DECOMPOSITION
	bool getUseNullDecomposition();
	// NORMALIZED ACCELERATIONS
	void setComputeNormalizedAccelerations(bool aBool);
	bool getComputeNormalizedAccelerations();

	//--------------------------------------------------------------------------
	// OPERATIONS
	//--------------------------------------------------------------------------
	int computeAccelerations();

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	void sumForceResults();
	void sumAccelerationResults();
	void sumDecomposition();

	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
	virtual void store();
	virtual int
		printResults(const char *aBaseName,const char *aDir=NULL,
		double aDT=-1.0,const char *aExtension=".sto");
private:
	virtual int
		readDecomposition(char *aBaseName,char *aDir=NULL,
		char *aExtension=NULL);


//=============================================================================
};	// END of class IndAcc

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __IndAcc_h__
