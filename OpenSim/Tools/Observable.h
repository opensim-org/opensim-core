#ifndef _Observable_h_
#define _Observable_h_
// Observable.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
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

/*  
 * Author:  
 */

// INCLUDES
#include "ArrayPtrs.h"

namespace OpenSim { 

class Event;
class Object;


// EXPORT LINE FOR MICROSOFT VISUAL C++
#ifdef WIN32
#ifndef SWIG
template class RDTOOLS_API ArrayPtrs<Object>;
#endif
#endif


#ifdef SWIG
	#ifdef RDTOOLS_API
		#undef RDTOOLS_API
		#define RDTOOLS_API
	#endif
#endif

//=============================================================================
//=============================================================================
/**
 */
class RDTOOLS_API Observable  
{

//=============================================================================
// DATA
//=============================================================================
private:
	/** Array of all registered observers to the object */
	ArrayPtrs<Object>	_observers;
	/** Subject being observed */
	const Object&			_subject;
	bool _changed;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	virtual ~Observable();
	Observable(const Object& aObject);
public:
	//--------------------------------------------------------------------------
	// Manage Observers
	//--------------------------------------------------------------------------
	void addObserver(Object& aObserver);
	void deleteObserver(Object& aObserver);
	void notifyObservers() const;
	/**
	 * Eventually observers will have to specify what event to observe so that they're not
	 * called unnecessarily. Will do this after the Event class hierarchy matures. For now 
	 * we'll observe everything.
	 */
	void addEventObserver(Object& aObserver, Event& aEvent) {};
	void notifyObservers(Event& aEvent);
	void deleteObservers();
	int countObservers();

	//--------------------------------------------------------------------------
	// Manage _changed flag
	//--------------------------------------------------------------------------
	void setChanged();
	void clearChanged();
	bool hasChanged();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
protected:

//=============================================================================
};	// END of class Observable

}; //namespace
//=============================================================================
//=============================================================================

#endif //__Observable_h__
