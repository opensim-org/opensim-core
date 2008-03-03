#ifndef _BodyPointIndAcc_h_
#define _BodyPointIndAcc_h_
// BodyPointIndAcc.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/rdMath.h>
#include "osimAnalysesDLL.h"
#include "IndAcc.h"
#include "SimTKcommon.h"

const int BodyPointIndAcc_BUFFER_LENGTH = 2048;

//=============================================================================
//=============================================================================
/**
 * A class for computing the induced accelerations of a point on a body
 * segment of a model.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class Model;
class AbstractBody;

class OSIMANALYSES_API BodyPointIndAcc : public IndAcc 
{
//=============================================================================
// DATA
//=============================================================================
public:
	static const int NAME_LENGTH;
	static const int BUFFER_LENGTH;
protected:
	AbstractBody *_body;
	SimTK::Vec3 _point;
	std::string _pointName;
	Storage *_axPointStore;
	Storage *_ayPointStore;
	Storage *_azPointStore;
	Storage *_vxPointStore;
	Storage *_vyPointStore;
	Storage *_vzPointStore;
	Storage *_pxPointStore;
	Storage *_pyPointStore;
	Storage *_pzPointStore;

//=============================================================================
// METHODS
//=============================================================================
public:
	BodyPointIndAcc(Model *aModel,AbstractBody *aBody,SimTK::Vec3& aPoint);
	BodyPointIndAcc(Model *aModel,AbstractBody *aBody,SimTK::Vec3& aPoint,
		Storage *aStates,Storage *aControls,char *aBaseName,
		char *aDir=NULL,char *aExtension=NULL);
	virtual ~BodyPointIndAcc();
private:
	void setNull();
	void constructDescription();
	void allocatePointStorage();
	void deletePointStorage();

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	void setBody(AbstractBody* aBody);
	AbstractBody* getBody();
	void setPoint(const SimTK::Vec3& aPoint);
	void getPoint(SimTK::Vec3& rPoint);
	void setPointName(const std::string &aName);
	const std::string &getPointName();
	virtual void setStorageCapacityIncrements(int aIncrement);
	void getColumnLabels(const char *aTag, Array<std::string> &rLabels);

	//--------------------------------------------------------------------------
	// OPERATIONS
	//--------------------------------------------------------------------------
	int computePointAccelerations();

	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
	virtual int
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");

//=============================================================================
};	// END of class BodyPointIndAcc

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __BodyPointIndAcc_h__
