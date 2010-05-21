#ifndef _PointKinematics_h_
#define _PointKinematics_h_
// PointKinematics.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson, Ajay Seth
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
#include "osimAnalysesDLL.h"
#ifdef SWIG
	#ifdef OSIMANALYSES_API
		#undef OSIMANALYSES_API
		#define OSIMANALYSES_API
	#endif
#endif

#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyDblVec3.h>
#include <OpenSim/Simulation/Model/Analysis.h>

const int PointKinematicsNAME_LENGTH = 256;
const int PointKinematicsBUFFER_LENGTH = 2048;

//=============================================================================
//=============================================================================
namespace OpenSim { 

class Model;
class Body;

/**
 * A class for recording the kinematics of a point on a body
 * of a model during a simulation.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMANALYSES_API PointKinematics : public Analysis 
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
	Body *_body;
	Body *_relativeToBody;
protected:
	// Properties
	PropertyStr _bodyNameProp;
	PropertyDblVec3 _pointProp;
	PropertyStr _pointNameProp;
	PropertyStr _relativeToBodyNameProp;

	// References
	std::string &_bodyName;
	SimTK::Vec3 &_point;
	std::string &_pointName;
	std::string &_relativeToBodyName;

	double *_dy;
	double *_kin;
	Storage *_pStore;
	Storage *_vStore;
	Storage *_aStore;

//=============================================================================
// METHODS
//=============================================================================
public:
	PointKinematics(Model *aModel=0);
	PointKinematics(const std::string &aFileName);
	// Copy constrctor and virtual copy 
	PointKinematics(const PointKinematics &aObject);
	virtual Object* copy() const;
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
	void setBodyPoint(const std::string& aBody, const SimTK::Vec3& aPoint);
	void setBody(Body* aBody);
	void setRelativeToBody(Body* aBody);
	Body* getBody();
	Body* getRelativeToBody();
	// POINT
	void setPoint(const SimTK::Vec3& aPoint);
	void getPoint(SimTK::Vec3& rPoint);
	// POINT NAME
	void setPointName(const char *aName);
	const std::string &getPointName();
	// MODEL
	virtual void setModel(Model& aModel);
	
	// STORAGE
	void setStorageCapacityIncrements(int aIncrement);
	Storage* getAccelerationStorage();
	Storage* getVelocityStorage();
	Storage* getPositionStorage();

	//--------------------------------------------------------------------------
	// ANALYSIS
	//--------------------------------------------------------------------------

    virtual int
        begin( SimTK::State& s);
    virtual int
        step(const SimTK::State& s, int setNumber);
    virtual int
        end( SimTK::State& s);
protected:
    virtual int
        record(const SimTK::State& s );

	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
public:
	virtual int
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");

//=============================================================================
};	// END of class PointKinematics

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __PointKinematics_h__
