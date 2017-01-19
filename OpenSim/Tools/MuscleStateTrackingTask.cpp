/* -------------------------------------------------------------------------- *
 *                   OpenSim:  MuscleStateTrackingTask.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include "MuscleStateTrackingTask.h"

using namespace std;
using namespace OpenSim;
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
MuscleStateTrackingTask::~MuscleStateTrackingTask()
{
}
//_____________________________________________________________________________
/**
 * Construct a default track object for a specified model.
 */
MuscleStateTrackingTask::MuscleStateTrackingTask()
{
    setNull();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aTask Task object to be copied.
 */
MuscleStateTrackingTask::MuscleStateTrackingTask(const MuscleStateTrackingTask& aTask) :
    StateTrackingTask(aTask)
{
    setNull();
    copyData(aTask);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void MuscleStateTrackingTask::
setNull()
{
    setName(DEFAULT_NAME);
    setupProperties();
    _nTrk = 1;
}
//_____________________________________________________________________________
/**
 * Set up the properties.
 */
void MuscleStateTrackingTask::
setupProperties()
{
}

//_____________________________________________________________________________
/**
 * Copy the member data for this class only.
 *
 * @param aTask Object whose data is to be copied.
 */
void MuscleStateTrackingTask::
copyData(const MuscleStateTrackingTask &aTask)
{
    
}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return  Reference to the altered object.
 */
MuscleStateTrackingTask& MuscleStateTrackingTask::
operator=(const MuscleStateTrackingTask &aTask)
{
    // BASE CLASS
    StateTrackingTask::operator =(aTask);

    // DATA
    copyData(aTask);

    return(*this);
}
