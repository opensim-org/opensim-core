#include "Range.h"



using namespace OpenSim;
using namespace std;
//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
Range::~Range(void)
{
}
//_____________________________________________________________________________
/**
 * Default constructor of an Range
 */
Range::Range():
_min(_propMin.getValueDbl()),
_max(_propMax.getValueDbl())
{
	setNull();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aRange range to copy.
 */
Range::Range(const Range &aRange) :
Object(aRange),
_min(_propMin.getValueDbl()),
_max(_propMax.getValueDbl())
{
	setNull();

	// ASSIGN
	*this = aRange;
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
Object* Range::
copy() const
{
	Object *object = new Range(*this);
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
Range& Range::
operator=(const Range &aRange)
{
	// BASE CLASS
	Object::operator=(aRange);
	_min = aRange._min;
	_max = aRange._max;

	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Range min. get
 */
const double Range::getMin() const
{
	return _min;
}
//_____________________________________________________________________________
/**
 * Range max. get
 */
const double Range::getMax() const
{
	return _max;
}

//_____________________________________________________________________________
/**
 * Range min. set
 */
void Range::setMin(const double aMin)
{
	_min = aMin;
}
//_____________________________________________________________________________
/**
 * Range max. set
 */
void Range::setMax(const double aMax)
{
	_max = aMax;
}


void Range::setNull()
{
	setType("Range");
	setName("");
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Set up the serialized member variables.  
 */
void Range::
setupProperties()
{
	// _min
	_propMin.setName("min");
	_propMin.setValue(0.0);
	_propertySet.append( &_propMin );
	// _max
	_propMax.setName("max");
	_propMax.setValue(1.0);
	_propertySet.append( &_propMax );


}
