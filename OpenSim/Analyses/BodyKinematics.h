#ifndef _BodyKinematics_h_
#define _BodyKinematics_h_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  BodyKinematics.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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
#include <OpenSim/Simulation/Model/Analysis.h>
#include "osimAnalysesDLL.h"


//=============================================================================
//=============================================================================
namespace OpenSim { 

class Model;
/**
 * A class for recording the kinematics of the bodies
 * of a model during a simulation.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */

class OSIMANALYSES_API BodyKinematics : public Analysis {
OpenSim_DECLARE_CONCRETE_OBJECT(BodyKinematics, Analysis);

//=============================================================================
// DATA
//=============================================================================
private:

protected:
	/** Names of bodies whose kinematics are to be recorded. */
	PropertyStrArray _bodiesProp;
	Array<std::string> &_bodies;

	/** Flag indicating whether or not to express the results
	in the global or body-local frame. */
	PropertyBool _expressInLocalFrameProp;
	bool &_expressInLocalFrame;

	Array<int> _bodyIndices;
	bool _recordCenterOfMass;
	Array<double> _kin;

	Storage *_pStore;
	Storage *_vStore;
	Storage *_aStore;

//=============================================================================
// METHODS
//=============================================================================
public:
	BodyKinematics(Model *aModel=0, bool aInDegrees=true);
	BodyKinematics(const std::string &aFileName);
	// Copy constrctor and virtual copy 
	BodyKinematics(const BodyKinematics &aObject);
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
	void setExpressResultsInLocalFrame(bool aTrueFalse);
	bool getExpressResultsInLocalFrame();

	void setRecordCenterOfMass(bool aTrueFalse) {_recordCenterOfMass = aTrueFalse;}
	void setBodiesToRecord(Array<std::string> &listOfBodies) {_bodies = listOfBodies;}


	virtual void setModel(Model& aModel);
	//--------------------------------------------------------------------------
	// ANALYSIS
	//--------------------------------------------------------------------------
    virtual int
        begin(SimTK::State& s );
    virtual int
        step(const SimTK::State& s, int setNumber );
    virtual int
        end(SimTK::State& s );
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
};	// END of class BodyKinematics

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __BodyKinematics_h__
