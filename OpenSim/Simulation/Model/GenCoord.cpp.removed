#include "GenCoord.h"
#include <OpenSim/Tools/Range.h>



using namespace OpenSim;
using namespace std;
//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
GenCoord::~GenCoord(void)
{
}
//_____________________________________________________________________________
/**
 * Default constructor of an GenCoord
 */
GenCoord::GenCoord():
_propGencoordRange(PropertyObj("Range", Range())),
_range((Range &)_propGencoordRange.getValueObj()),
_inDegrees(_propInDegrees.getValueBool())
{
	setNull();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aGencoord Gencoord to copy.
 */
GenCoord::GenCoord(const GenCoord &aGencoord) :
Object(aGencoord),
_propGencoordRange(PropertyObj("Range", Range())),
_range((Range &)_propGencoordRange.getValueObj()),
_inDegrees(_propInDegrees.getValueBool())
{
	setNull();

	// ASSIGN
	*this = aGencoord;
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
Object* GenCoord::
copy() const
{
	Object *object = new GenCoord(*this);
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
GenCoord& GenCoord::
operator=(const GenCoord &aGencoord)
{
	// BASE CLASS
	Object::operator=(aGencoord);
	_range = aGencoord.getRange();
	_inDegrees = aGencoord.getInDegrees();
	return(*this);
}



void GenCoord::setNull()
{
	setType("GenCoord");
	setName("");
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Set up the serialized member variables.  
 */
void GenCoord::
setupProperties()
{
	// Range
	_propertySet.append( &_propGencoordRange );
	// InDegrees
	_propInDegrees.setName("in_degrees");
	_propInDegrees.setValue(true);
	_propertySet.append( &_propInDegrees );


}
//=============================================================================
// GET AND SET
//=============================================================================
//------------- Get range -------------------------------------------
/**
 * gets the range for the GenCoord
 */
const Range &GenCoord::
	getRange() const
{
	return _range;
}
//_____________________________________________________________________________
/**
 * gets the range for the GenCoord, const object
 */
Range &GenCoord::
getRange()
{
	return _range;
}
//_____________________________________________________________________________
/**
 * Sets the range for the GenCoord
 */
void GenCoord::
setRange(const Range &aRange)
{
	_range = aRange;
}
//_____________________________________________________________________________
/**
 * Get inDegrees attribute, const object
 */
const bool GenCoord::
getInDegrees() const
{
	return _inDegrees;
}
//_____________________________________________________________________________
/**
 * Get inDegrees attribute
 */
bool GenCoord::
getInDegrees()
{
	return _inDegrees;
}
//_____________________________________________________________________________
/**
 * Set inDegrees attribute
 */
void GenCoord::
setInDegrees(const bool &aInDegrees)
{
	_inDegrees = aInDegrees;
}
