#include "Scale.h"



using namespace OpenSim;
using namespace std;
using SimTK::Vec3;

//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
Scale::~Scale(void)
{
}
//_____________________________________________________________________________
/**
 * Default constructor of an Scale
 */
Scale::Scale():
_scaleFactors(_propScaleFactors.getValueDblVec3()),
_segmentName(_propSegmentName.getValueStr()),
_apply(_propApply.getValueBool())
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param an Scale to copy
 */
Scale::Scale(const Scale &aScale) :
Object(aScale),
_scaleFactors(_propScaleFactors.getValueDblVec3()),
_segmentName(_propSegmentName.getValueStr()),
_apply(_propApply.getValueBool())
{
	setNull();

	// ASSIGN
	*this = aScale;
}
//_____________________________________________________________________________
/**
 * Constructor of a scaleSet from a file.
 */
Scale::Scale(const string& scaleFileName):
Object(scaleFileName, false),
_scaleFactors(_propScaleFactors.getValueDblVec3()),
_segmentName(_propSegmentName.getValueStr()),
_apply(_propApply.getValueBool())
{
	setNull();
	updateFromXMLNode();
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
Object* Scale::
copy() const
{
	Object *object = new Scale(*this);
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
Scale& Scale::
operator=(const Scale &aScale)
{
	// BASE CLASS
	_segmentName = aScale.getSegmentName();
	aScale.getScaleFactors(_scaleFactors);
	_apply = aScale.getApply();

	return(*this);
}


void Scale::setNull()
{
	setType("Scale");
	setName("");
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Set up the serialized member variables.  
 */
void Scale::
setupProperties()
{
	Vec3 one3(1.0);	

	// scale factors
	_propScaleFactors.setName("scales");
	_propScaleFactors.setValue(one3);
	//_propScaleFactors.setAllowableArraySize(3);
	_propertySet.append( &_propScaleFactors );

	// segment name
	_propSegmentName.setName("segment");
	_propSegmentName.setValue("unnamed_segment");
	_propertySet.append( &_propSegmentName );

	// whether or not to apply the scale
	_propApply.setName("apply");
	_propApply.setValue(true);
	_propertySet.append(&_propApply);
}
//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get segment name
 */
const std::string& Scale::
getSegmentName() const
{
	return _segmentName;
}
//_____________________________________________________________________________
/**
 * Set the value of scale factors
 */
void Scale::
getScaleFactors(SimTK::Vec3& aScaleFactors) const
{
	aScaleFactors = _scaleFactors;
}

//_____________________________________________________________________________
/**
 * Set segment name
 */
void Scale::
setSegmentName(const string& aSegmentName)
{
	_segmentName = aSegmentName;
}
//_____________________________________________________________________________
/**
 * Set scale factors
 */
void Scale::
setScaleFactors(const SimTK::Vec3& aScaleFactors)
{
	_scaleFactors = aScaleFactors;
}
