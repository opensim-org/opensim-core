/* -------------------------------------------------------------------------- *
 *                          OpenSim:  MarkerSet.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ayman Habib, Peter Loan                                         *
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

#include "MarkerSet.h"
#include "Marker.h"
#include "Model.h"
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>

using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
MarkerSet::~MarkerSet(void)
{
}

//_____________________________________________________________________________
/**
 * Constructor of a markerSet from a file.
 */
MarkerSet::MarkerSet(Model& aModel, const string& aMarkersFileName) :
ModelComponentSet<Marker>(aModel, aMarkersFileName, false)
{
    setNull();
    SimTK::Xml::Element e = updDocument()->getRootDataElement(); 
    updateFromXMLNode(e, getDocument()->getDocumentVersion());
}

//_____________________________________________________________________________
/**
 * Default constructor of a markerSet.
 */
MarkerSet::MarkerSet() :
ModelComponentSet<Marker>()
{
    setNull();
}

//_____________________________________________________________________________
/**
 * Copy constructor of a markerSet.
 */
MarkerSet::MarkerSet(const MarkerSet& aMarkerSet):
ModelComponentSet<Marker>(aMarkerSet)
{
    setNull();
    *this = aMarkerSet;
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
/**
 * Set the data members of this MarkerSet to their null values.
 */
void MarkerSet::setNull()
{
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
MarkerSet& MarkerSet::operator=(const MarkerSet &aMarkerSet)
{
    Set<Marker>::operator=(aMarkerSet);
    return (*this);
}
#endif

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Get names of markers in the marker set
 */
void MarkerSet::getMarkerNames(Array<string>& aMarkerNamesArray)
{
    for (int i = 0; i < getSize(); i++)
    {
        Marker& nextMarker = get(i);
        aMarkerNamesArray.append(nextMarker.getName());
    }
}

//_____________________________________________________________________________
/**
 * Scale marker set by a set of scale factors
 */
void MarkerSet::scale(const ScaleSet& scaleSet)
{
    Vec3    scaleFactors(1.0);

    for (int i = 0; i < getSize(); i++)
    {
        Marker& nextMarker = get(i);
        const string& refFrameName = nextMarker.getFrameName();
        //assert(refBodyName);
        bool found = false;
        for (int j = 0; j < scaleSet.getSize() && !found; j++)
        {
            Scale& nextScale = scaleSet.get(j);
            if (nextScale.getSegmentName() == refFrameName)
            {
                found = true;
                nextScale.getScaleFactors(scaleFactors);
                nextMarker.scale(scaleFactors);
            }
        }
    }
}

//_____________________________________________________________________________
/**
 * Add name prefix.
 */
void MarkerSet::addNamePrefix(const string& prefix)
{
    int i;

    // Cycle through set and add prefix
    for (i = 0; i < getSize(); i++)
        get(i).setName(prefix + get(i).getName());
}

//_____________________________________________________________________________
/**
 * Create a new marker and add it to the set.
 */
Marker* MarkerSet::addMarker(const string& aName, const SimTK::Vec3& aOffset, OpenSim::PhysicalFrame& aPhysicalFrame)
{
    // If a marker by this name already exists, do nothing.
    if (contains(aName))
        return NULL;

    // Create a marker and add it to the set.
    Marker* m = new Marker();
    m->setName(aName);
    m->set_location(aOffset);
    // Frame will be based on this name when marker is connected to Model.

    m->setFrameName(aPhysicalFrame.getName()); 
    aPhysicalFrame.updModel().addMarker(m);

    return m;
}
