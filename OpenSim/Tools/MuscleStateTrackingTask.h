#ifndef MuscleStateTrackingTask_h__
#define MuscleStateTrackingTask_h__
/* -------------------------------------------------------------------------- *
 *                    OpenSim:  MuscleStateTrackingTask.h                     *
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
#include "osimToolsDLL.h"
#include "StateTrackingTask.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A TrackingTask for that corresponds to a muscle state variable.
 *
 * @author Ayman Habib & Ajay Seth
 * @version 1.0
 */
class OSIMTOOLS_API MuscleStateTrackingTask : public StateTrackingTask {
OpenSim_DECLARE_CONCRETE_OBJECT(MuscleStateTrackingTask, StateTrackingTask);

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
	MuscleStateTrackingTask();
	MuscleStateTrackingTask(const MuscleStateTrackingTask &aTaskObject);
	virtual ~MuscleStateTrackingTask();

private:
	void setNull();
	void copyData(const MuscleStateTrackingTask &aTaskObject);
	void setupProperties();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:

#ifndef SWIG
	MuscleStateTrackingTask& operator=(const MuscleStateTrackingTask &aTaskObject) ;
#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------

//=============================================================================
};	// END of class MuscleStateTrackingTask
//=============================================================================
//=============================================================================

}; // end namespace

#endif // __MuscleStateTrackingTask_h__


