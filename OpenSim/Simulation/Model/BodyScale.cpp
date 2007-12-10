// BodyScale.cpp
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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
#include "BodyScale.h"

//=============================================================================
// STATICS
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
BodyScale::BodyScale() :
	_axisNames(_axisNamesProp.getValueStrArray())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
BodyScale::~BodyScale()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aBodyScale BodyScale to be copied.
 */
BodyScale::BodyScale(const BodyScale &aBodyScale) :
   Object(aBodyScale),
	_axisNames(_axisNamesProp.getValueStrArray())
{
	setNull();
	setupProperties();
	copyData(aBodyScale);
}

//_____________________________________________________________________________
/**
 * Copy this BodyScale and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this BodyScale.
 */
Object* BodyScale::copy() const
{
	BodyScale *bodyScale = new BodyScale(*this);
	return(bodyScale);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one BodyScale to another.
 *
 * @param aBodyScale BodyScale to be copied.
 */
void BodyScale::copyData(const BodyScale &aBodyScale)
{
	_axisNames = aBodyScale._axisNames;
}

//_____________________________________________________________________________
/**
 * Set the data members of this BodyScale to their null values.
 */
void BodyScale::setNull()
{
	setType("BodyScale");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void BodyScale::setupProperties()
{
	_axisNamesProp.setComment("Axes (X Y Z) along which to scale a body. "
		"For example, 'X Y Z' scales along all three axes, and 'Y' scales "
		"just along the Y axis.");
	_axisNamesProp.setName("axes");
	_propertySet.append(&_axisNamesProp);
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
BodyScale& BodyScale::operator=(const BodyScale &aBodyScale)
{
	// BASE CLASS
	Object::operator=(aBodyScale);

	copyData(aBodyScale);

	return(*this);
}
