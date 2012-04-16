/*
 * Copyright (c) 2007, Stanford University. All rights reserved. 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions
 * are met: 
 *  - Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer. 
 *  - Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 *  - Neither the name of the Stanford University nor the names of its 
 *    contributors may be used to endorse or promote products derived 
 *    from this software without specific prior written permission. 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE. 
 */
#include "Scale.h"



using namespace OpenSim;
using namespace std;
using SimTK::Vec3;

//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
Scale::~Scale(void)
{
}
//_____________________________________________________________________________
/**
 * Default constructor of an Scale
 */
Scale::Scale():
_scaleFactors(_propScaleFactors.getValueDblVec()),
_segmentName(_propSegmentName.getValueStr()),
_apply(_propApply.getValueBool())
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param an Scale to copy
 */
Scale::Scale(const Scale &aScale) :
Object(aScale),
_scaleFactors(_propScaleFactors.getValueDblVec()),
_segmentName(_propSegmentName.getValueStr()),
_apply(_propApply.getValueBool())
{
	setNull();

	// ASSIGN
	*this = aScale;
}
//_____________________________________________________________________________
/**
 * Constructor of a scaleSet from a file.
 */
Scale::Scale(const string& scaleFileName):
Object(scaleFileName, false),
_scaleFactors(_propScaleFactors.getValueDblVec()),
_segmentName(_propSegmentName.getValueStr()),
_apply(_propApply.getValueBool())
{
	setNull();
	updateFromXMLDocument();
}

//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assign this object to the values of another.
 *
 * @return Reference to this object.
 */
Scale& Scale::
operator=(const Scale &aScale)
{
	// BASE CLASS
	_segmentName = aScale.getSegmentName();
	aScale.getScaleFactors(_scaleFactors);
	_apply = aScale.getApply();

	return(*this);
}


void Scale::setNull()
{
	setName("");
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Set up the serialized member variables.  
 */
void Scale::
setupProperties()
{
	Vec3 one3(1.0);	

	// scale factors
	_propScaleFactors.setName("scales");
	_propScaleFactors.setValue(one3);
	//_propScaleFactors.setAllowableListSize(3);
	_propertySet.append( &_propScaleFactors );

	// segment name
	_propSegmentName.setName("segment");
	_propSegmentName.setValue("unnamed_segment");
	_propertySet.append( &_propSegmentName );

	// whether or not to apply the scale
	_propApply.setName("apply");
	_propApply.setValue(true);
	_propertySet.append(&_propApply);
}
//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get segment name
 */
const std::string& Scale::
getSegmentName() const
{
	return _segmentName;
}
//_____________________________________________________________________________
/**
 * Set the value of scale factors
 */
void Scale::
getScaleFactors(SimTK::Vec3& aScaleFactors) const
{
	aScaleFactors = _scaleFactors;
}

//_____________________________________________________________________________
/**
 * Set segment name
 */
void Scale::
setSegmentName(const string& aSegmentName)
{
	_segmentName = aSegmentName;
}
//_____________________________________________________________________________
/**
 * Set scale factors
 */
void Scale::
setScaleFactors(const SimTK::Vec3& aScaleFactors)
{
	_scaleFactors = aScaleFactors;
}
