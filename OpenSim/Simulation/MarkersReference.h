#ifndef __MarkersReference_h__
#define __MarkersReference_h__
// MarkersReference.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2010, Stanford University. All rights reserved.
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

#include "Reference.h"
#include <OpenSim/Common/Set.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/MarkerData.h>

namespace OpenSim {

class Units;

class OSIMSIMULATION_API MarkerWeight : public Object
{
private:
	PropertyDbl _weightProp;
	double &_weight;

public:
	MarkerWeight() : Object(), _weight(_weightProp.getValueDbl()) {};
	MarkerWeight(std::string name, double weight) : Object(), _weight(_weightProp.getValueDbl())
		{_name = name; _weight = weight;}

	//Copy
	MarkerWeight(const MarkerWeight &aWeight) : Object(aWeight), _weight(_weightProp.getValueDbl()) {_weight = aWeight._weight;}


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
class OSIMSIMULATION_API MarkersReference : public Reference_<SimTK::Vec3>
{
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
	virtual ~MarkersReference() {};

	MarkersReference();

	// Convenience load markers from a file
	MarkersReference(const std::string filename, Units modelUnits=Units(Units::Meters));

	MarkersReference& operator=(const MarkersReference &aRef) {Reference_<SimTK::Vec3>::operator=(aRef); copyData(aRef); return(*this); };

	void copyData(const MarkersReference &aRef){_type = "MarkersReference";
												_markersFile = aRef._markersFile;
												setMarkerWeightSet(aRef._markerWeightSet);
												_defaultWeight = aRef._defaultWeight;  }

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

//=============================================================================
};	// END of class MarkersReference
//=============================================================================
} // namespace

#endif // __MarkersReference_h__
