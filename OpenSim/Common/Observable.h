#ifndef _Observable_h_
#define _Observable_h_
// Observable.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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
template class OSIMCOMMON_API ArrayPtrs<Object>;
#endif
#endif


#ifdef SWIG
	#ifdef OSIMCOMMON_API
		#undef OSIMCOMMON_API
		#define OSIMCOMMON_API
	#endif
#endif

//=============================================================================
//=============================================================================
/**
 */
class OSIMCOMMON_API Observable  
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
