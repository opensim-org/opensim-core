/* -------------------------------------------------------------------------- *
 *                         OpenSim:  Measurement.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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
#include "Measurement.h"
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Simulation/Model/BodyScaleSet.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Measurement::Measurement() :
    _markerPairSetProp(PropertyObj("", MarkerPairSet())),
    _markerPairSet((MarkerPairSet&)_markerPairSetProp.getValueObj()),
    _bodyScaleSetProp(PropertyObj("", BodyScaleSet())),
    _bodyScaleSet((BodyScaleSet&)_bodyScaleSetProp.getValueObj()),
    _apply(_applyProp.getValueBool())
{
    setNull();
    setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
Measurement::~Measurement()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMeasurement Measurement to be copied.
 */
Measurement::Measurement(const Measurement &aMeasurement) :
   Object(aMeasurement),
    _markerPairSetProp(PropertyObj("", MarkerPairSet())),
    _markerPairSet((MarkerPairSet&)_markerPairSetProp.getValueObj()),
    _bodyScaleSetProp(PropertyObj("", BodyScaleSet())),
    _bodyScaleSet((BodyScaleSet&)_bodyScaleSetProp.getValueObj()),
    _apply(_applyProp.getValueBool())
{
    setNull();
    setupProperties();
    copyData(aMeasurement);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one Measurement to another.
 *
 * @param aMeasurement Measurement to be copied.
 */
void Measurement::copyData(const Measurement &aMeasurement)
{
    _markerPairSet = aMeasurement._markerPairSet;
    _bodyScaleSet = aMeasurement._bodyScaleSet;
    _apply = aMeasurement._apply;
}

//_____________________________________________________________________________
/**
 * Set the data members of this Measurement to their null values.
 */
void Measurement::setNull()
{
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Measurement::setupProperties()
{
    _applyProp.setComment("Flag to turn on and off scaling for this measurement.");
    _applyProp.setName("apply");
    _applyProp.setValue(true);
    _propertySet.append(&_applyProp);

    _markerPairSetProp.setComment("Set of marker pairs used to determine the scale factors.");
    _markerPairSetProp.setName("MarkerPairSet");
    _propertySet.append(&_markerPairSetProp);

    _bodyScaleSetProp.setComment("Set of bodies to be scaled by this measurement.");
    _bodyScaleSetProp.setName("BodyScaleSet");
    _propertySet.append(&_bodyScaleSetProp);
}

//_____________________________________________________________________________
/**
 * Register the types used by Measurement.
 */
void Measurement::registerTypes()
{
    Object::registerType(MarkerPair());
    Object::registerType(BodyScale());
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
Measurement& Measurement::operator=(const Measurement &aMeasurement)
{
    // BASE CLASS
    Object::operator=(aMeasurement);

    copyData(aMeasurement);

    return(*this);
}

/* Apply a scale factor to a scale set, according to the elements of
 * the Measurement's BodyScaleSet.
 */
//_____________________________________________________________________________
/**
 * Apply a scale factor to a scale set, according to the elements of
 * the Measurement's _bodyScaleSet.
 *
 * @param aFactor the scale factor to apply
 * @param aScaleSet the set of scale factors to modify
 */
void Measurement::applyScaleFactor(double aFactor, ScaleSet& aScaleSet)
{
    for (int i = 0; i < _bodyScaleSet.getSize(); i++)
    {
        const string& bodyName = _bodyScaleSet[i].getName();
        for (int j = 0; j < aScaleSet.getSize(); j++)
        {
            if (aScaleSet[j].getSegmentName() == bodyName)
            {
                const Array<std::string>& axisNames = _bodyScaleSet[i].getAxisNames();
                Vec3 factors(1.0);
                aScaleSet[j].getScaleFactors(factors);

                for (int k = 0; k < axisNames.getSize(); k++)
                {
                    if (axisNames[k] == "x" || axisNames[k] == "X")
                        factors[0] = aFactor;
                    else if (axisNames[k] == "y" || axisNames[k] == "Y")
                        factors[1] = aFactor;
                    else if (axisNames[k] == "z" || axisNames[k] == "Z")
                        factors[2] = aFactor;
                }
                aScaleSet[j].setScaleFactors(factors);
            }
        }
    }
}
