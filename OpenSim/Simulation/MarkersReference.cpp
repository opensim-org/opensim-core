/* MarkersReference.cpp 
* Author: Ajay Seth 
* Copyright (c)  2010 Stanford University
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

#include "MarkersReference.h"
#include <OpenSim/Common/Units.h>

using namespace std;
using namespace SimTK;

namespace OpenSim {

//______________________________________________________________________________
/**
 * An implementation of the MarkersReference 
 *
 * @param model to assemble
 */
MarkersReference::MarkersReference() : Reference_<SimTK::Vec3>(), 
		_markersFile(_markersFileProp.getValueStr()),
		_markerWeightSetProp(PropertyObj("", Set<MarkerWeight>())),
		_markerWeightSet((Set<MarkerWeight>&)_markerWeightSetProp.getValueObj()),
		_defaultWeight(_defaultWeightProp.getValueDbl()),
		_markerData(NULL)
{
}

MarkersReference::MarkersReference(const std::string markerFile, Units modelUnits) : Reference_<SimTK::Vec3>(), 
		_markersFile(_markersFileProp.getValueStr()),
		_markerWeightSetProp(PropertyObj("", Set<MarkerWeight>())),
		_markerWeightSet((Set<MarkerWeight>&)_markerWeightSetProp.getValueObj()),
		_defaultWeight(_defaultWeightProp.getValueDbl()),
		_markerData(NULL)
{
	loadMarkersFile(markerFile, modelUnits);
}

/**
 * Convenience constructor to be used for Marker placement. 
 *
 * @param aMarkerData: MarkerData, assumed to be in the correct units already
 */
MarkersReference::MarkersReference(MarkerData& aMarkerData, const Set<MarkerWeight>* aMarkerWeightSet) : Reference_<SimTK::Vec3>(), 
		_markersFile(_markersFileProp.getValueStr()),
		_markerWeightSetProp(PropertyObj("", Set<MarkerWeight>())),
		_markerWeightSet((Set<MarkerWeight>&)_markerWeightSetProp.getValueObj()),
		_defaultWeight(_defaultWeightProp.getValueDbl()),
		_markerData(NULL)
{
	if (aMarkerWeightSet!=NULL) _markerWeightSet= *aMarkerWeightSet;
	populateFromMarkerData(aMarkerData);
}

/** load the marker data for this MarkersReference from markerFile  */
void MarkersReference::loadMarkersFile(const std::string markerFile, Units modelUnits)
{
	_markersFile = markerFile;
	_markerData = new MarkerData(_markersFile);

	// Convert the marker data into the model's units
	_markerData->convertToUnits(modelUnits);

	populateFromMarkerData(*_markerData);
}


/** A convenience method yo populate MarkersReference from MarkerData **/
void MarkersReference::populateFromMarkerData(MarkerData& aMarkerData)
{
	_markerData = &aMarkerData;
	const Array<std::string> &tempNames = aMarkerData.getMarkerNames();
	int nm = tempNames.getSize();

	// empty any lingering names and weights
	_markerNames.clear();
	_weights.clear();
	// pre-allocate arrays to the number of markers in the file with default weightings
	_markerNames.assign(nm, "");
	_weights.assign(nm, _defaultWeight);

	int index = 0;
	// Build flat lists (arrays) of marker names and weights in the same order as the marker data
	for(int i=0; i<tempNames.getSize(); i++){
		const std::string &name = tempNames[i];
		_markerNames[i] = name;
		index = _markerWeightSet.getIndex(name, index);
		//Assign user weighting for markers that are user listed in the input set
		if(index >= 0)
			_weights[i] = _markerWeightSet[index].getWeight();
	}

	if(_markerNames.size() != _weights.size())
		throw Exception("MarkersReference: Mismatch between the number of marker names and weights. Verify that marker names are unique.");
}

SimTK::Vec2 MarkersReference::getValidTimeRange() const
{
	return Vec2(_markerData->getStartFrameTime(), _markerData->getLastFrameTime());
}

// utility to define object properties including their tags, comments and 
// default values.
void MarkersReference::setupProperties()
{
	_markersFileProp.setComment("TRC file (.trc) containing the time history of observations of marker."
								"positions.");
	_markersFileProp.setName("marker_file");
	_propertySet.append( &_markersFileProp );

	_markerWeightSetProp.setComment("Set of marker weights identified by marker name with weight being a positive scalar.");
	_markerWeightSetProp.setName("marker_weights");
	_propertySet.append( &_markerWeightSetProp );
}

/** get the names of the markers serving as references */
const SimTK::Array_<std::string>& MarkersReference::getNames() const
{
	return _markerNames;
}

/** get the values of the MarkersReference */
void  MarkersReference::getValues(const SimTK::State &s, SimTK::Array_<Vec3> &values) const
{
	double time =  s.getTime();

	int before=0, after=0;
	// get index for time 
	_markerData->findFrameRange(time, time, before, after);
	if(before > after || after < 0)
		throw Exception("MarkersReference: No index corresponding to time of frame.");
	else if(after-before > 0){
		before = abs(_markerData->getFrame(before).getFrameTime()-time) < abs(_markerData->getFrame(after).getFrameTime()-time) ? before : after;
	}

	values = _markerData->getFrame(before).getMarkers();
}

/** get the speed value of the MarkersReference */
void MarkersReference::getSpeedValues(const SimTK::State &s, SimTK::Array_<Vec3> &speedValues) const
{
	throw Exception("MarkersReference: getSpeedValues not implemented.");
}

/** get the acceleration value of the MarkersReference */
void MarkersReference::getAccelerationValues(const SimTK::State &s, SimTK::Array_<Vec3> &accValues) const
{
	throw Exception("MarkersReference: getAccelerationValues not implemented.");
}

/** get the weights of the Markers */
void  MarkersReference::getWeights(const SimTK::State &s, SimTK::Array_<double> &weights) const
{
	weights = _weights;
}

void MarkersReference::setMarkerWeightSet(Set<MarkerWeight> &markerWeights)
{
	_markerWeightSet.setSize(0);
    for(int i=0; i<markerWeights.getSize(); i++)
		_markerWeightSet.adoptAndAppend(&markerWeights[i]);

	//Make sure the input set no longer owns the weightings
	_markerWeightSet.setMemoryOwner(false);
}


} // end of namespace OpenSim