// IKTask.cpp
// Author: Eran Guendelman
/*
 * Copyright (c) 2007, Stanford University. All rights reserved. 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

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
	setType("IKTask");
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
