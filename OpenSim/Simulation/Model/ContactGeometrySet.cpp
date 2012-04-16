// ContactGeometrySet.cpp
// Author: Peter Eastman
/*
 * Copyright (c) 2006-2009 Stanford University. All rights reserved. 
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

#include "ContactGeometrySet.h"
#include <OpenSim/Common/ScaleSet.h>

using namespace std;
using namespace OpenSim;

//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
ContactGeometrySet::~ContactGeometrySet(void)
{
}

//_____________________________________________________________________________
/**
 * Default constructor of a ContactGeometrySet.
 */
ContactGeometrySet::ContactGeometrySet()
{
	setNull();
}

ContactGeometrySet::ContactGeometrySet(Model& model) : Super(model)
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Construct an ContactGeometrySet set from file.
 *
 * @param aModel reference to the Model to add contacts to.
 * @param aFileName Name of the file.
 * @param aUpdateFromXMLNode a flag indicating if UpdateFromXMLNode needs to be called.
 */
ContactGeometrySet::ContactGeometrySet
   (Model& model, const std::string &aFileName, bool aUpdateFromXMLNode)
:   Super(model, aFileName, false)
{
	setNull();

	if(aUpdateFromXMLNode) updateFromXMLDocument();
}

//_____________________________________________________________________________
/**
 * Copy constructor of a ContactGeometrySet.
 */
ContactGeometrySet::ContactGeometrySet
   (const ContactGeometrySet& aContactGeometrySet)
:   Super(aContactGeometrySet)
{
	setNull();
	*this = aContactGeometrySet;
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
/**
 * Set the data members of this ContactGeometrySet to their null values.
 */
void ContactGeometrySet::setNull()
{
}

/**
 * Post construction initialization.
 */
void ContactGeometrySet::setup(Model& aModel)
{
	// Base class
	ModelComponentSet<ContactGeometry>::setup(aModel);
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
#ifndef SWIG
ContactGeometrySet& ContactGeometrySet::operator=(const ContactGeometrySet &aContactGeometrySet)
{
	Set<ContactGeometry>::operator=(aContactGeometrySet);
	return (*this);
}
#endif

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Scale ContactGeometrySet by a set of scale factors
 */
void ContactGeometrySet::scale(const ScaleSet& aScaleSet)
{
	for(int i=0; i<getSize(); i++) get(i).scale(aScaleSet);
}
