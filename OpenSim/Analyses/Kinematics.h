#ifndef _Kinematics_h_
#define _Kinematics_h_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Kinematics.h                           *
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
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include "osimAnalysesDLL.h"


//=============================================================================
//=============================================================================
namespace OpenSim { 

/**
 * A class for recording the kinematics of the generalized coordinates
 * of a model during a simulation.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMANALYSES_API Kinematics : public Analysis {
OpenSim_DECLARE_CONCRETE_OBJECT(Kinematics, Analysis);

//=============================================================================
// DATA
//=============================================================================
private:

protected:

	OpenSim_DECLARE_LIST_PROPERTY(coordinates, std::string, "Names of generalized coordinates whose kinematics are to be recorded.");

	Array<int> _coordinateIndices;
	Array<double> _values;

	Storage *_pStore;
	Storage *_vStore;
	Storage *_aStore;

	// Make recording accelerations optional since it is more computationally intensive (invokes dynamics engine calls)
	bool _recordAccelerations;

//=============================================================================
// METHODS
//=============================================================================
public:
	Kinematics(Model *aModel=0);
	Kinematics(const std::string &aFileName);
	virtual ~Kinematics();

private:
	void setNull();
	void constructProperties();
	void constructDescription();
	void constructColumnLabels();
	void allocateStorage();
	void deleteStorage();
	void updateCoordinatesToRecord();

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// STORAGE
	void setStorageCapacityIncrements(int aIncrement);
	Storage* getAccelerationStorage();
	Storage* getVelocityStorage();
	Storage* getPositionStorage();

	// MODEL
	virtual void setModel(Model& aModel);

	void setRecordAccelerations(bool aRecordAccelerations) { _recordAccelerations = aRecordAccelerations; } // TODO: re-allocate storage or delete storage

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
};	// END of class Kinematics

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __Kinematics_h__
