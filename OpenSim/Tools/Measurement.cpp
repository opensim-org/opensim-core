// Measurement.cpp
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
#include "Measurement.h"

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
//_____________________________________________________________________________
/**
 * Copy this measurement and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this Measurement.
 */
Object* Measurement::copy() const
{
	Measurement *measurement = new Measurement(*this);
	return(measurement);
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
	setType("Measurement");
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
	Object::RegisterType(MarkerPair());
	Object::RegisterType(BodyScale());
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
		const string& bodyName = _bodyScaleSet[i]->getName();
		for (int j = 0; j < aScaleSet.getSize(); j++)
		{
			if (aScaleSet[j]->getSegmentName() == bodyName)
			{
				const Array<std::string>& axisNames = _bodyScaleSet[i]->getAxisNames();
				Array<double> factors(1.0, 3);
				aScaleSet[j]->getScaleFactors(factors);

				for (int k = 0; k < axisNames.getSize(); k++)
				{
					if (axisNames[k] == "x" || axisNames[k] == "X")
						factors[0] = aFactor;
					else if (axisNames[k] == "y" || axisNames[k] == "Y")
						factors[1] = aFactor;
					else if (axisNames[k] == "z" || axisNames[k] == "Z")
						factors[2] = aFactor;
				}
				aScaleSet[j]->setScaleFactors(factors);
			}
		}
	}
}
