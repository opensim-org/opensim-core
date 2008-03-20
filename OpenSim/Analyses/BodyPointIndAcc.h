#ifndef _BodyPointIndAcc_h_
#define _BodyPointIndAcc_h_
// BodyPointIndAcc.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


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
