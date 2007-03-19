#ifndef _BodyKinematics_h_
#define _BodyKinematics_h_
// BodyKinematics.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include "osimAnalysesDLL.h"


//=============================================================================
//=============================================================================
/**
 * A class for recording the kinematics of the bodies
 * of a model during a simulation.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class AbstractModel;

class OSIMANALYSES_API BodyKinematics : public Analysis 
{
//=============================================================================
// DATA
//=============================================================================
private:

protected:
	/** Names of bodies whose kinematics are to be recorded. */
	PropertyStrArray _bodiesProp;
	Array<std::string> &_bodies;

	Array<int> _bodyIndices;
	bool _recordCenterOfMass;
	Array<double> _kin;

	double *_dy;
	Storage *_pStore;
	Storage *_vStore;
	Storage *_aStore;
	/** Whether or not to write output of angles in degrees. */
	PropertyBool _angVelInLocalFrameProp;
	bool &_angVelInLocalFrame;

//=============================================================================
// METHODS
//=============================================================================
public:
	BodyKinematics(AbstractModel *aModel=0, bool aInDegrees=true);
	BodyKinematics(const std::string &aFileName);
	// Copy constrctor and virtual copy 
	BodyKinematics(const BodyKinematics &aObject);
	virtual Object* copy() const;
	virtual ~BodyKinematics();
	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	BodyKinematics& operator=(const BodyKinematics &aBodyKinematics);
#endif
private:
	void setNull();
	void setupProperties();
	void constructDescription();
	void constructColumnLabels();
	void allocateStorage();
	void deleteStorage();
	void updateBodiesToRecord();

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// STORAGE
	void setStorageCapacityIncrements(int aIncrement);
	Storage* getAccelerationStorage();
	Storage* getVelocityStorage();
	Storage* getPositionStorage();
	void setAngVelInLocalFrame(bool aTrueFalse);
	bool getAngVelInLocalFrame();

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

//=============================================================================
};	// END of class BodyKinematics

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __BodyKinematics_h__
