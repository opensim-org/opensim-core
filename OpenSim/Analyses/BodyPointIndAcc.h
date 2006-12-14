#ifndef _BodyPointIndAcc_h_
#define _BodyPointIndAcc_h_
// BodyPointIndAcc.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/rdTools.h>
#include "suAnalysesDLL.h"
#include "IndAcc.h"

const int BodyPointIndAcc_NAME_LENGTH = 256;
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

class AbstractModel;
class AbstractBody;

class SUANALYSES_API BodyPointIndAcc : public IndAcc 
{
//=============================================================================
// DATA
//=============================================================================
public:
	static const int NAME_LENGTH;
	static const int BUFFER_LENGTH;
private:
	char _buffer[BodyPointIndAcc_BUFFER_LENGTH];
	char _tmp[BodyPointIndAcc_BUFFER_LENGTH];
protected:
	AbstractBody *_body;
	double _point[3];
	char _pointName[BodyPointIndAcc_NAME_LENGTH];
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
	BodyPointIndAcc(AbstractModel *aModel,AbstractBody *aBody,double aPoint[3]);
	BodyPointIndAcc(AbstractModel *aModel,AbstractBody *aBody,double aPoint[3],
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
	void setPoint(double aPoint[3]);
	void getPoint(double rPoint[3]);
	void setPointName(const char *aName);
	const char* getPointName();
	virtual void setStorageCapacityIncrements(int aIncrement);
	char* getColumnLabels(const char *aTag);

	//--------------------------------------------------------------------------
	// OPERATIONS
	//--------------------------------------------------------------------------
	int computePointAccelerations();

	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
	virtual int
		printResults(const char *aBaseName,const char *aDir=NULL,
		double aDT=-1.0,const char *aExtension=".sto");

//=============================================================================
};	// END of class BodyPointIndAcc

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __BodyPointIndAcc_h__
