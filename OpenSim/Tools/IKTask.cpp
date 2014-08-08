/* -------------------------------------------------------------------------- *
 *                            OpenSim:  IKTask.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Eran Guendelman                                                 *
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
#include "IKTask.h"

//=============================================================================
// NAMESPACES
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
IKTask::IKTask() :
    _apply(_applyProp.getValueBool()),
   _weight(_weightProp.getValueDbl())
{
    _apply = true;
    _weight = 0;
    setupProperties();
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 */
IKTask::IKTask(const IKTask &aIKTask) :
   Object(aIKTask),
    _apply(_applyProp.getValueBool()),
   _weight(_weightProp.getValueDbl())
{
    _apply = aIKTask._apply;
    _weight = aIKTask._weight;
    setupProperties();
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void IKTask::setupProperties()
{
    _applyProp.setComment("Whether or not this task will be used during inverse kinematics solve."); 
    _applyProp.setName("apply");
    _propertySet.append(&_applyProp);

    _weightProp.setComment("Weight given to a marker or coordinate for solving inverse kinematics problems."); 
    _weightProp.setName("weight");
    _propertySet.append(&_weightProp);
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
IKTask& IKTask::operator=(const IKTask &aIKTask)
{
    Object::operator=(aIKTask);
    _apply = aIKTask._apply;
    _weight = aIKTask._weight;
    return *this;
}
