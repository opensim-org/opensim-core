#ifndef __SimmModelIterator_h__
#define __SimmModelIterator_h__

// SimmModelIterator.h
// Authors: Kenny Smith
/* Copyright (c) 2005, Stanford University, Kenny Smith
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
#include <vector>

namespace OpenSim {

class SimmModel;
class SimmBody;
class SimmJoint;

#ifdef SWIG
	#ifdef RDSIMULATION_API
		#undef RDSIMULATION_API
		#define RDSIMULATION_API
	#endif
#endif

/* -------------------------------------------------------------------------
	class SimmModelIterator
	
	This class implements a depth-first traversal of the bodies in a
	SimmModel.  It was written to facilitate building a 3d scene graph
	for model viewing.  If our model traversal needs become more
	elaborate, then this class could be grown into a hierarchy of model
	traversal classes.
	
	NOTE: This iterator class does not support modifications to the model's
		bodies or joints while a traversal is in progress!
---------------------------------------------------------------------------- */
class RDSIMULATION_API SimmModelIterator {
private:
	// ---- data members:
	SimmModel&	_model;
	SimmBody*	_traversalRoot;
	
	bool _traversalStarted;
	bool _traversalFinished;
	
	typedef int joint_index;
	
	std::vector<joint_index> _stack; // a stack of joint indices holds the traversal state.
	
public:
	// ---- construction/destruction:
	SimmModelIterator(SimmModel&, SimmBody* traversalRoot = NULL);
	
	virtual ~SimmModelIterator();
	
	// ---- advance the traversal:
	SimmBody* getNextBody();
	
	// ---- query the current state of the traversal:
	SimmBody*  getCurrentBody();
	SimmJoint* getCurrentJoint();
	
	int		  getNumAncestors();
	SimmBody* getAncestor(int i = 0);
	
	// ---- reset the iterator to start over from the beginning (i.e. the root).
	void reset();
	
private:
	// ---- local routines:
	SimmBody*  determine_traversal_root(SimmBody* userSuppliedRoot);
	SimmJoint* get_joint(int i, bool allowRangeError = false);
	SimmBody*  get_top_child_body();
	SimmBody*  find_joint_with_parent(SimmBody* parent, int jointSearchStart);
	
}; // class SimmModelIterator

}; //namespace
#endif // __SimmModelIterator_h__
