#include "Marker.h"
#include <OpenSim/Simulation/Model/Body.h>


using namespace OpenSim;
using namespace std;
//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
Marker::~Marker(void)
{
}
//_____________________________________________________________________________
/**
 * Default constructor of an Marker
 */
Marker::Marker():
VisibleObject(),
_markerLocation(_propMarkerLocation.getValueDblArray()),
_referenceSegmentName(_propReferenceSegmentName.getValueStr()),
_markerWeight(_propMarkerWeight.getValueDbl())
{
	setNull();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aBody Body to copy.
 */
Marker::Marker(const Marker &aMarker) :
VisibleObject(aMarker),
_markerLocation(_propMarkerLocation.getValueDblArray()),
_referenceSegmentName(_propReferenceSegmentName.getValueStr()),
_markerWeight(_propMarkerWeight.getValueDbl())
{
	setNull();

	// ASSIGN
	*this = aMarker;
}
//_____________________________________________________________________________
/**
 * Construct and return a copy of this object.
 *
 * The object is allocated using the new operator, so the caller is
 * responsible for deleting the returned object.
 *
 * @return Copy of this object.
 */
Object* Marker::
copy() const
{
	Object *object = new Marker(*this);
	return(object);
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
Marker& Marker::
operator=(const Marker &aMarker)
{
	// BASE CLASS
	VisibleObject::operator=(aMarker);
	Array<double>	aLocation(0.0);
	aMarker.getLocation(aLocation);
	setLocation(aLocation);
	_referenceSegmentName = aMarker.getReferenceSegmentName();
	_markerWeight = aMarker.getWeight();

	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the value of marker weight
 */
const double Marker::
getWeight() const
{
	return _markerWeight;
}
//_____________________________________________________________________________
/**
 * Set the value of marker weight
 */
void Marker::
setWeight(const double aWeight)
{
	_markerWeight = aWeight;
}
//_____________________________________________________________________________
/**
 * Get the name of the segment that the marker lives on
 */
const std::string& Marker::
getReferenceSegmentName() const
{
	return _referenceSegmentName;
}
//_____________________________________________________________________________
/**
 * Set the name of the segment that the marker lives on
 */
void Marker::
setRefSegment(const int aBodyIndex)
{
	_refSegmentForMarker=aBodyIndex;
}
//_____________________________________________________________________________
/**
 * Get the index of the segment that the marker lives on
 */
const int Marker::
getRefSegment() const
{
	return _refSegmentForMarker;
}
//_____________________________________________________________________________
/**
 * Get the location of the marker in reference segment frame
 */
void Marker::
getLocation(Array<double>& aLocation) const
{
	for(int i=0; i < aLocation.getSize(); i++)
	aLocation[i] = _markerLocation[i];
}
//_____________________________________________________________________________
/**
 * Set the location of the marker in reference segment frame
 */
void Marker::
setLocation(Array<double>& aLocation)
{
	for(int i=0; i < aLocation.getSize(); i++)
	_markerLocation[i] = aLocation[i];
}
//_____________________________________________________________________________
/**
 * initialize object to null.  
 */
void Marker::
setNull()
{
	setType("Marker");
	//setName("unnamed_marker");
	setupProperties();
	_refSegmentForMarker=-2;	// SDFast uses -1 for ground
}
//_____________________________________________________________________________
/**
 * Set up the serialized member variables.  
 */
void Marker::
setupProperties()
{
	Array<double> zero3(0.0, 3);	

	// _markerLocation
	_propMarkerLocation.setName("marker_location");
	_propMarkerLocation.setValue(zero3);
	_propertySet.append( &_propMarkerLocation );

	// reference_segment
	_propReferenceSegmentName.setName("ref_segment");
	_propReferenceSegmentName.setValue("unnamed_segment");
	_propertySet.append( &_propReferenceSegmentName );

	// Weight
	_propMarkerWeight.setName("weight");
	_propMarkerWeight.setValue(1.0);
	_propertySet.append( &_propMarkerWeight );

}
//_____________________________________________________________________________
/**
 * Scale marker location by indicate dscale factors.  
 */
void Marker::
scaleBy(Array<double>& aScales)
{
	for(int i=0; i < aScales.getSize(); i++)
	_markerLocation[i] *= aScales[i];

}

void Marker::
update(const Object& aObject, Event& aEvent)
{
//	const Body& refBody = dynamic_cast<const Body &>(aObject);
	
}
