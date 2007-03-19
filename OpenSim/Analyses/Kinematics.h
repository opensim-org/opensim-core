#ifndef _Kinematics_h_
#define _Kinematics_h_
// Kinematics.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include "osimAnalysesDLL.h"


//=============================================================================
//=============================================================================
/**
 * A class for recording the kinematics of the generalized coordinates
 * of a model during a simulation.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class OSIMANALYSES_API Kinematics : public Analysis 
{
//=============================================================================
// DATA
//=============================================================================
private:

protected:
	/** Names of generalized coordinates whose kinematics are to be recorded. */
	PropertyStrArray _coordinatesProp;
	Array<std::string> &_coordinates;

	Array<int> _coordinateIndices;
	Array<double> _values;

	double *_y;
	double *_dy;
	Storage *_pStore;
	Storage *_vStore;
	Storage *_aStore;

//=============================================================================
// METHODS
//=============================================================================
public:
	Kinematics(AbstractModel *aModel=0);
	Kinematics(const std::string &aFileName);
	// Copy constrctor and virtual copy 
	Kinematics(const Kinematics &aObject);
	virtual Object* copy() const;
	virtual ~Kinematics();
private:
	void setNull();
	void setupProperties();
	void constructDescription();
	void constructColumnLabels();
	void allocateStorage();
	void deleteStorage();
	void updateCoordinatesToRecord();

public:
	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	Kinematics& operator=(const Kinematics &aKinematics);
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// STORAGE
	void setStorageCapacityIncrements(int aIncrement);
	Storage* getAccelerationStorage();
	Storage* getVelocityStorage();
	Storage* getPositionStorage();

	// MODEL
	virtual void setModel(AbstractModel *aModel);

	//--------------------------------------------------------------------------
	// ANALYSIS
	//--------------------------------------------------------------------------
	virtual int
		begin(int aStep,double aDT,double aT,double *aX,double *aY,
		void *aClientData=NULL);
	virtual int
		step(double *aXPrev,double *aYPrev,
		int aStep,double aDT,double aT,double *aX,double *aY,
		void *aClientData=NULL);
	virtual int
		end(int aStep,double aDT,double aT,double *aX,double *aY,
		void *aClientData=NULL);
protected:
	virtual int
		record(double aT,double *aX,double *aY);

	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
public:
	virtual int
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");

	virtual ArrayPtrs<Storage>& getStorageList();

//=============================================================================
};	// END of class Kinematics

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __Kinematics_h__
