// IKCoordinateTask.cpp
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
#include "IKCoordinateTask.h"

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
IKCoordinateTask::IKCoordinateTask() :
   _fromFile(_fromFileProp.getValueBool()),
   _value(_valueProp.getValueDbl())
{
	setType("IKCoordinateTask");
	_fromFile = false;
	_value = 0;
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 */
IKCoordinateTask::IKCoordinateTask(const IKCoordinateTask &aIKCoordinateTask) :
   IKTask(aIKCoordinateTask),
   _fromFile(_fromFileProp.getValueBool()),
   _value(_valueProp.getValueDbl())
{
	_fromFile = aIKCoordinateTask._fromFile;
	_value = aIKCoordinateTask._value;
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Copy method
 */
Object* IKCoordinateTask::copy() const
{
	return new IKCoordinateTask(*this);
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void IKCoordinateTask::setupProperties()
{
	_fromFileProp.setComment("Indicates whether the coordinate values should come from the coordinates_file."
									 "  If false, then the desired value (or prescribed value in case of locked coordinates) will"
									 " come from (a) the <value> attribute in this task, or if that's not defined then (b) the"
									 " <value> attribute in the coordinate, or if that's not defined then (c) the <default_value>"
									 " attribute in the coordinate.");
	_fromFileProp.setName("from_file");
	_propertySet.append(&_fromFileProp);

	_valueProp.setComment("This value will be used as the desired (or prescribed) coordinate value if from_file is set to false.");
	_valueProp.setName("value");
	_propertySet.append(&_valueProp);
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
IKCoordinateTask& IKCoordinateTask::operator=(const IKCoordinateTask &aIKCoordinateTask)
{
	IKTask::operator=(aIKCoordinateTask);
	_fromFile = aIKCoordinateTask._fromFile;
	_value = aIKCoordinateTask._value;
	return *this;
}
