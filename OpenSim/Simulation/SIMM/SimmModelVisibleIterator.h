#ifndef __SimmModelVisibleIterator_h__
#define __SimmModelVisibleIterator_h__

// SimmModelVisibleIterator.h
// Authors: Ayman Habib
/* Copyright (c) 2005, Stanford University, Ayman Habib
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/VisibleObject.h>
#include <vector>

namespace OpenSim {

class SimmModel;

#ifdef SWIG
	#ifdef RDSIMULATION_API
		#undef RDSIMULATION_API
		#define RDSIMULATION_API
	#endif
#endif

/* -------------------------------------------------------------------------
	class SimmModelVisibleIterator
	Iterate through objects recursively collecting/returning objects that are visible
---------------------------------------------------------------------------- */
class RDSIMULATION_API SimmModelVisibleIterator {
private:
	// ---- data members:
	SimmModel&	_model;
	Object*	_traversalRoot;
	
	bool _traversalStarted;
	bool _traversalFinished;
		
public:
	// ---- construction/destruction:
	SimmModelVisibleIterator(SimmModel&);
	
	virtual ~SimmModelVisibleIterator();
	
	ArrayPtrs<VisibleObject>* getVisibleObjects(Object* traversalRoot=0);

	
	
}; // class SimmModelVisibleIterator

}; //namespace
#endif // __SimmModelVisibleIterator_h__
