/*
 * Copyright (c) 2007, Stanford University. All rights reserved. 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions
 * are met: 
 *  - Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer. 
 *  - Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 *  - Neither the name of the Stanford University nor the names of its 
 *    contributors may be used to endorse or promote products derived 
 *    from this software without specific prior written permission. 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE. 
 */
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
