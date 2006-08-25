#include "Scale.h"



using namespace OpenSim;
using namespace std;
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
_segmentName(_propSegmentName.getValueStr()),
_scaleFactors(_propScaleFactors.getValueDblArray()),
_apply(_propApply.getValueBool())
{
	setNull();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param an Scale to copy
 */
Scale::Scale(const Scale &aScale) :
Object(aScale),
_segmentName(_propSegmentName.getValueStr()),
_scaleFactors(_propScaleFactors.getValueDblArray()),
_apply(_propApply.getValueBool())
{
	setNull();

	// ASSIGN
	*this = aScale;
}
//_____________________________________________________________________________
/**
 * Construct an Scale from DOMElement.
 *
 * @param aElement to use for construction
 */
Scale::Scale(DOMElement *aElement) :
Object(aElement),
_segmentName(_propSegmentName.getValueStr()),
_scaleFactors(_propScaleFactors.getValueDblArray()),
_apply(_propApply.getValueBool())
{
	setNull();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Constructor of a scaleSet from a file.
 */
Scale::Scale(const string& scaleFileName):
Object(scaleFileName),
_segmentName(_propSegmentName.getValueStr()),
_scaleFactors(_propScaleFactors.getValueDblArray()),
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
//_____________________________________________________________________________
/**
 * Create object from DOMElement.
 *
 * @param aElement XMLnode to construct Scale from.
 */

Object* Scale::
copy(DOMElement *aElement) const
{
	Scale *m = new Scale(aElement);
	*m = *this;
	m->updateFromXMLNode();
	return(m);
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
	Array<double> one3(1.0, 3);	

	// scale factors
	_propScaleFactors.setName("scales");
	_propScaleFactors.setValue(one3);
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
getScaleFactors(Array<double>& aScaleFactors) const
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
setScaleFactors(Array<double>& aScaleFactors)
{
	_scaleFactors = aScaleFactors;
}
