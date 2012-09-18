#ifndef _PointKinematics_h_
#define _PointKinematics_h_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  PointKinematics.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ajay Seth                                    *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


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
#include <OpenSim/Common/PropertyDblVec.h>
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
class OSIMANALYSES_API PointKinematics : public Analysis {
OpenSim_DECLARE_CONCRETE_OBJECT(PointKinematics, Analysis);

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
	PointKinematics(const PointKinematics &aObject);
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
	void setPointName(const std::string &aName);
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
