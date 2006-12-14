#ifndef _PointKinematics_h_
#define _PointKinematics_h_
// PointKinematics.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include "suAnalysesDLL.h"
#ifdef SWIG
	#ifdef SUANALYSES_API
		#undef SUANALYSES_API
		#define SUANALYSES_API
	#endif
#endif
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/PropertyStr.h>
#include <OpenSim/Tools/PropertyDblArray.h>
#include <OpenSim/Simulation/Model/Analysis.h>

const int PointKinematicsNAME_LENGTH = 256;
const int PointKinematicsBUFFER_LENGTH = 2048;

//=============================================================================
//=============================================================================
/**
 * A class for recording the kinematics of a point on a body
 * of a model during a simulation.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class AbstractModel;
class AbstractBody;

class SUANALYSES_API PointKinematics : public Analysis 
{
//=============================================================================
// DATA
//=============================================================================
public:
	static const int NAME_LENGTH;
	static const int BUFFER_LENGTH;
private:
	//char _buffer[PointKinematicsBUFFER_LENGTH];
	//char _tmp[PointKinematicsBUFFER_LENGTH];
	AbstractBody *_body;
protected:
	// Properties
	PropertyStr _bodyNameProp;
	PropertyDblArray _pointProp;
	PropertyStr _pointNameProp;

	// References
	std::string &_bodyName;
	Array<double> &_point;
	std::string &_pointName;

	double *_dy;
	double *_kin;
	Storage *_pStore;
	Storage *_vStore;
	Storage *_aStore;

//=============================================================================
// METHODS
//=============================================================================
public:
	PointKinematics(AbstractModel *aModel=0);
	PointKinematics(const std::string &aFileName);
	PointKinematics(DOMElement *aElement);
	// Copy constrctor and virtual copy 
	PointKinematics(const PointKinematics &aObject);
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;
	virtual ~PointKinematics();
private:
	void setNull();
	void setupProperties();
	void constructDescription();
	void constructColumnLabels();
	void allocateStorage();
	void deleteStorage();
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	PointKinematics& operator=(const PointKinematics &aPointKinematics);
#endif
public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// BODY
	void setBodyPoint(std::string& aBody, double aPoint[3]);
	void setBody(AbstractBody* aBody);
	AbstractBody* getBody();
	// POINT
	void setPoint(double aPoint[3]);
	void getPoint(double rPoint[3]);
	// POINT NAME
	void setPointName(const char *aName);
	const char* getPointName();
	// MODEL
	virtual void setModel(AbstractModel *aModel);
	
	// STORAGE
	void setStorageCapacityIncrements(int aIncrement);
	Storage* getAccelerationStorage();
	Storage* getVelocityStorage();
	Storage* getPositionStorage();

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
		printResults(const char *aBaseName,const char *aDir=NULL,
		double aDT=-1.0,const char *aExtension=".sto");

//=============================================================================
};	// END of class PointKinematics

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __PointKinematics_h__
