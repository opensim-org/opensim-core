#ifndef StateTrackingTask_h__
#define StateTrackingTask_h__
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  StateTrackingTask.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ayman Habib, Ajay Seth                                          *
 * Contributor(s): Ayman Habib, Ajay Seth                                     *
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

// INCLUDES
#include "osimToolsDLL.h"
#include <OpenSim/Common/Object.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include "TrackingTask.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A target for a tracking problem that corresponds to a state variable.
 *
 * @author Ayman Habib & Ajay Seth
 * @version 1.0
 */
class OSIMTOOLS_API StateTrackingTask : public TrackingTask
{

//=============================================================================
// DATA
//=============================================================================
protected:

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	StateTrackingTask() { setNull(); };
	StateTrackingTask(const StateTrackingTask &aTaskObject) :
	TrackingTask(aTaskObject)
	{
		setNull();
		copyData(aTaskObject);
	};
	virtual ~StateTrackingTask() {};
private:
	void setNull() {_stateIndex=-1;};
	void copyData(const StateTrackingTask &aTaskObject) {};

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:

#ifndef SWIG
	StateTrackingTask& operator=(const StateTrackingTask &aTaskObject) {
	// BASE CLASS
	TrackingTask::operator =(aTaskObject);
	// DATA
	copyData(aTaskObject);

	return(*this);
	}
#endif
	virtual double getTaskError(const SimTK::State& s) {
		return (_pTrk[0]->calcValue(SimTK::Vector(1,s.getTime()))- _model->getStateVariable(s, getName()));
	}
	/**
	 * Return the gradient of the tracking error as a vector, whose length 
	 * is the number of _controller->getModel().getActuators()
	 */
	virtual SimTK::Vector getTaskErrorGradient(const SimTK::State& s) {
		const Set<Actuator>& fSet = _model->getActuators();
		SimTK::Vector g = SimTK::Vector(fSet.getSize());
		g = 0.;
		double taskGradient = -2 * getTaskError(s)* getWeight(0);
		// set the entry idx in g where fSet[idx] is a substring of getName()
		for(int i=0; i< fSet.getSize(); i++){
			if (getName().find(fSet[i].getName())!=std::string::npos){
				g[i]=taskGradient;
				break;
			}
		}
		return g;
	}
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
private:
	int _stateIndex;	// YIndex of state being tracked in _model, -1 if not found
//=============================================================================
};	// END of class StateTrackingTask
//=============================================================================
//=============================================================================

}; // end namespace

#endif // __StateTrackingTask_h__


