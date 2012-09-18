#ifndef __MarkersReference_h__
#define __MarkersReference_h__
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  MarkersReference.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

#include "Reference.h"
#include <OpenSim/Common/Set.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/MarkerData.h>

namespace OpenSim {

class Units;

class OSIMSIMULATION_API MarkerWeight : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MarkerWeight, Object);

private:
	PropertyDbl _weightProp;
	double &_weight;

public:
	MarkerWeight() : Object(), _weight(_weightProp.getValueDbl()) {}

	MarkerWeight(std::string name, double weight) 
    :   Object(), _weight(_weightProp.getValueDbl())
	{   setName(name); _weight = weight; }

	//Copy
	MarkerWeight(const MarkerWeight& source) 
    :   Object(source), _weight(_weightProp.getValueDbl()) 
    {   _weight = source._weight; }

    #ifndef SWIG
	MarkerWeight& operator=(const MarkerWeight& source) {
        if (&source != this) {
            Super::operator=(source);
            _weight = source._weight;
        }
        return *this;
    }
    #endif

	void setWeight(double weight) {_weight = weight; }
	double getWeight() const {return _weight; };

}; // end of MarkerWeight class


//=============================================================================
//=============================================================================
/**
 * Reference values to be achieved for specified Markers that will be used
 * via optimization and/or tracking. Also contains a weighting that identifies
 * the relative importance of achieving one marker's reference relative to
 * another.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class IKTaskSet;

class OSIMSIMULATION_API MarkersReference : public Reference_<SimTK::Vec3> {
OpenSim_DECLARE_CONCRETE_OBJECT(MarkersReference, Reference_<SimTK::Vec3>);

//=============================================================================
// MEMBER VARIABLES
//=============================================================================

protected:

	/** Specify the reference markers value from a file of columns in time. */
	PropertyStr _markersFileProp;
	std::string &_markersFile;

	/** Specify the individual weightings of markers to be matched.  */
	PropertyObj _markerWeightSetProp;
	Set<MarkerWeight> &_markerWeightSet;

	/** Specify the default weight for markers not specified individually.  */
	PropertyDbl _defaultWeightProp;
	double &_defaultWeight;


private:
	// Implementation details

	// Use a specialized data structure for holding the marker data
	MarkerData *_markerData;
	// marker names inside the marker data
	SimTK::Array_<std::string> _markerNames;
	// corresponding list of weights guaranteed to be in the same order as names above
	SimTK::Array_<double> _weights;

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	MarkersReference();

	// Convenience load markers from a file
	MarkersReference(const std::string filename, Units modelUnits=Units(Units::Meters));

	MarkersReference(MarkerData& aMarkerData, const Set<MarkerWeight>* aMarkerWeightSet=NULL);

	MarkersReference& operator=(const MarkersReference &aRef) {Reference_<SimTK::Vec3>::operator=(aRef); copyData(aRef); return(*this); };
    
    virtual ~MarkersReference() {}

	void copyData(const MarkersReference &aRef) {
		_markersFile = aRef._markersFile;
		setMarkerWeightSet(aRef._markerWeightSet);
		_defaultWeight = aRef._defaultWeight;  
    }

	/** load the marker data for this MarkersReference from markerFile  */
	void loadMarkersFile(const std::string markerFile, Units modelUnits=Units(Units::Meters));


	//--------------------------------------------------------------------------
	// Reference Interface
	//--------------------------------------------------------------------------
	virtual int getNumRefs() const {return _markerData->getNumMarkers(); }
	/** get the time range for which the MarkersReference values are valid,
	    based on the loaded marker data.*/
	virtual SimTK::Vec2 getValidTimeRange() const;
	/** get the names of the markers serving as references */
	virtual const SimTK::Array_<std::string>& getNames() const;
	/** get the value of the MarkersReference */
	virtual void getValues(const SimTK::State &s, SimTK::Array_<SimTK::Vec3> &values) const;
	/** get the speed value of the MarkersReference */
	virtual void getSpeedValues(const SimTK::State &s, SimTK::Array_<SimTK::Vec3> &speedValues) const;
	/** get the acceleration value of the MarkersReference */
	virtual void getAccelerationValues(const SimTK::State &s, SimTK::Array_<SimTK::Vec3> &accValues) const;
	/** get the weighting (importance) of meeting this MarkersReference in the same order as names*/
	virtual void getWeights(const SimTK::State &s, SimTK::Array_<double> &weights) const;

	//--------------------------------------------------------------------------
	// Convenience Access
	//--------------------------------------------------------------------------
	double getSamplingFrequency() {return _markerData->getDataRate(); }
	Set<MarkerWeight> &updMarkerWeightSet() {return _markerWeightSet; }
	void setMarkerWeightSet(Set<MarkerWeight> &markerWeights);
	void setDefaultWeight(double weight) {_defaultWeight = weight; }

private:
	// utility to define object properties including their tags, comments and
	// default values.
	void setupProperties();

	void populateFromMarkerData(MarkerData& aMarkerData);

//=============================================================================
};	// END of class MarkersReference
//=============================================================================
} // namespace

#endif // __MarkersReference_h__
