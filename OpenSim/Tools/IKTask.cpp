// IKTask.cpp
// Author: Eran Guendelman
/*
 * Copyright (c)  2007, Stanford University. All rights reserved. 
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
