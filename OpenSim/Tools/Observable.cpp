// Observable.cpp

//============================================================================
// INCLUDES
//============================================================================
#include "Object.h"
#include "Observable.h"

//============================================================================
// CONSTANTS
//============================================================================

//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________


using namespace OpenSim;
/**
 * Destructor.
 */
Observable::~Observable()
{
	// ArrayPtrs deletes memory on its own	
}

//_____________________________________________________________________________
/**
 * Default constructor.
 */
Observable::Observable(const Object& aObject):
_subject(aObject),
_changed(false)
{
	_observers.setMemoryOwner(false);
}

void Observable::
addObserver(Object& aObserver)
{
	if (_observers.getIndex(&aObserver)== -1)
		_observers.append(&aObserver);
}

void Observable::
deleteObserver(Object& aObserver)
{
	_observers.remove(&aObserver);
}

void Observable::
notifyObservers(Event& aEvent)
{
	if (!_changed)
			return;
	clearChanged();

	for (int i =0; i< _observers.getSize(); i++){
		_observers.get(i)->update(_subject, aEvent);
	}

}
int Observable::
countObservers()
{
	return _observers.getSize();
}

void Observable::
deleteObservers()
{
	_observers.setSize(0);
}

void Observable::
setChanged()
{
	_changed=true;
}
void Observable::
clearChanged()
{
	_changed=false;
}
bool Observable::
hasChanged()
{
	return _changed;
}
