/* -------------------------------------------------------------------------- *
 *                      OpenSim:  ContactGeometrySet.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Eastman                                                   *
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
    setAuthors("Peter Eastman");
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
