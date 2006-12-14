// SimmModelIterator.cpp
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

#include "SimmModelIterator.h"

#include "SimmModel.h"
#include "SimmKinematicsEngine.h"
#include "SimmBody.h"
#include "SimmJoint.h"

using namespace OpenSim;
/* -------------------------------------------------------------------------
	SimmModelIterator::SimmModelIterator
---------------------------------------------------------------------------- */
SimmModelIterator::SimmModelIterator(SimmModel& model, SimmBody* traversalRoot) :
	_model(model),
	_traversalRoot(determine_traversal_root(traversalRoot)),
	_traversalStarted(false),
	_traversalFinished(false)
{
} // SimmModelIterator::SimmModelIterator

/* -------------------------------------------------------------------------
	SimmModelIterator::~SimmModelIterator
---------------------------------------------------------------------------- */
SimmModelIterator::~SimmModelIterator()
{
} // SimmModelIterator::~SimmModelIterator

/* -------------------------------------------------------------------------
	SimmModelIterator::getNextBody - this routine implements the depth-first
		traversal behavior.  Each time this routine is called, the traversal
		advances.
---------------------------------------------------------------------------- */
SimmBody* SimmModelIterator::getNextBody()
{
	SimmBody* nextBody = NULL;
	
	// If this is the VERY first time 'getNextBody' is called, then
	// we simply return the '_traversalRoot' body because it is by
	// definition the first body in a depth-first, preorder search.
	if ( ! _traversalStarted)
	{
		_traversalStarted = true;

		nextBody = _traversalRoot;
	}
	else if ( ! _traversalFinished)
	{
		// The 2nd time 'getNextBody' is called, we locate the 1st joint
		// with '_traversalRoot' as a parent and push it on the stack.
		bool secondTime = (_stack.size() == 0 &&
						 ! _traversalFinished && _traversalRoot != NULL);
		
		if (secondTime)
		{
			nextBody = find_joint_with_parent(_traversalRoot, 0);
		}
		else // it's the 3rd, 4th, 5th... call to this routine.
		{
			// First try to descend deeper (hence the name "depth first").
			if (_stack.size() > 0)
				nextBody = find_joint_with_parent(get_top_child_body(), 0);
			
			// If searching down the tree failed, then we search sideways for
			// peers, moving up the stack until we find one.
			while (nextBody == NULL && _stack.size() > 0)
			{
				SimmBody* parentBody = get_joint(_stack.back())->getParentBody();
				
				int jointSearchStart = _stack.back() + 1;
				
				_stack.pop_back();
				
				nextBody = find_joint_with_parent(parentBody, jointSearchStart);
			}
		}
	}
	
	if (nextBody == NULL)
		_traversalFinished = true;
		
	return nextBody;
	
} // SimmModelIterator::getNextBody

/* -------------------------------------------------------------------------
	SimmModelIterator::getCurrentBody
---------------------------------------------------------------------------- */
SimmBody* SimmModelIterator::getCurrentBody()
{
	SimmBody* body = NULL;
	
	if (_stack.size() > 0)
		body = get_joint(_stack.back())->getChildBody();
		
	else if (_traversalStarted && ! _traversalFinished)
		body = _traversalRoot;
		
	return body;
	
} // SimmModelIterator::getCurrentBody

/* -------------------------------------------------------------------------
	SimmModelIterator::getCurrentJoint
---------------------------------------------------------------------------- */
SimmJoint* SimmModelIterator::getCurrentJoint()
{
	SimmJoint* joint = NULL;
	
	if (_stack.size() > 0)
		joint = get_joint(_stack.back());
		
	return joint;
	
} // SimmModelIterator::getCurrentJoint

/* -------------------------------------------------------------------------
	SimmModelIterator::getNumAncestors
---------------------------------------------------------------------------- */
int SimmModelIterator::getNumAncestors()
{
	return _stack.size();
	
} // SimmModelIterator::getNumAncestors

/* -------------------------------------------------------------------------
	SimmModelIterator::getAncestor
---------------------------------------------------------------------------- */
SimmBody* SimmModelIterator::getAncestor(int i)
{
	SimmBody* body = NULL;
	
	if ((int) _stack.size() > i)
	{
		int j = _stack.size() - i;
		
		body = get_joint(_stack[j])->getParentBody();
	}	
	return body;
	
} // SimmModelIterator::getAncestor

/* -------------------------------------------------------------------------
	SimmModelIterator::reset
---------------------------------------------------------------------------- */
void SimmModelIterator::reset()
{
	_traversalStarted = false;
	_traversalFinished = false;
	_stack.clear();
	
} // SimmModelIterator::reset

/* -------------------------------------------------------------------------
	SimmModelIterator::determine_traversal_root - if the caller does not specify
		a traversal root body, then use the first ground body in the model.
---------------------------------------------------------------------------- */
SimmBody* SimmModelIterator::determine_traversal_root(SimmBody* userSuppliedRootBody)
{
	SimmBody* traversalRoot = userSuppliedRootBody;
	
	if (traversalRoot == NULL)
		traversalRoot = _model.getSimmKinematicsEngine().getGroundBodyPtr();
	
	assert(traversalRoot);
	
	return traversalRoot;
	
} // SimmModelIterator::determine_traversal_root

/* -------------------------------------------------------------------------
	SimmModelIterator::get_joint
---------------------------------------------------------------------------- */
SimmJoint* SimmModelIterator::get_joint(int i, bool allowRangeError)
{
	bool isInRange = true;
	
	if (allowRangeError)
	{
		isInRange = (i >= 0 && i < _model.getSimmKinematicsEngine().getNumJoints());
	}
	else
	{
		assert(i >= 0);
		assert(i < _model.getSimmKinematicsEngine().getNumJoints());
	}
	
	SimmJoint* joint = NULL;
	
	if (isInRange)
		joint = _model.getSimmKinematicsEngine().getJoint(i);
	
	if ( ! allowRangeError)
		assert(joint);
	
	return joint;
	
} // SimmModelIterator::get_joint

/* -------------------------------------------------------------------------
	SimmModelIterator::get_top_child_body
---------------------------------------------------------------------------- */
SimmBody* SimmModelIterator::get_top_child_body()
{
	SimmBody* topChild = NULL;
	
	if (_stack.size() > 0)
		topChild = get_joint(_stack.back())->getChildBody();
	
	return topChild;
	
} // SimmModelIterator::get_top_child_body

/* -------------------------------------------------------------------------
	SimmModelIterator::find_joint_with_parent
---------------------------------------------------------------------------- */
SimmBody* SimmModelIterator::find_joint_with_parent(
	SimmBody*	parent,
	int			jointSearchStart)
{
	SimmBody* jointChildBody = NULL;
	
	int numJoints = _model.getSimmKinematicsEngine().getNumJoints();
	
	int i = jointSearchStart;
	
	if (i >= 0 && i < numJoints)
		for (SimmJoint* joint = get_joint(i); joint; joint = get_joint(++i, true))
			if (joint->getParentBody() == parent)
				break;
	
	if (i >= 0 && i < numJoints)
	{
		_stack.push_back(i);

		jointChildBody = get_top_child_body();
	}
	
	return jointChildBody;
	
} // SimmModelIterator::find_joint_with_parent
