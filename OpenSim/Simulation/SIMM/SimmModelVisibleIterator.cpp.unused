// SimmModelVisibleIterator.cpp
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

#include "SimmModelVisibleIterator.h"
#include <OpenSim/Tools/Property.h>
#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/VisibleObject.h>


#include "SimmModel.h"


using namespace OpenSim;
/* -------------------------------------------------------------------------
	SimmModelVisibleIterator::SimmModelVisibleIterator
---------------------------------------------------------------------------- */
SimmModelVisibleIterator::SimmModelVisibleIterator(SimmModel& model) :
	_model(model),
	_traversalRoot(&model),
	_traversalStarted(false),
	_traversalFinished(false)
{
} // SimmModelVisibleIterator::SimmModelVisibleIterator

/* -------------------------------------------------------------------------
	SimmModelVisibleIterator::~SimmModelVisibleIterator
---------------------------------------------------------------------------- */
SimmModelVisibleIterator::~SimmModelVisibleIterator()
{
} // SimmModelVisibleIterator::~SimmModelVisibleIterator

/* -------------------------------------------------------------------------
	SimmModelVisibleIterator::getNextVisible - this routine cycles through visible objects
	in the model returning them in arbitrary order.
---------------------------------------------------------------------------- */
ArrayPtrs<VisibleObject>* SimmModelVisibleIterator::getVisibleObjects(Object* traversalRoot)
{
	if (traversalRoot == 0)
		traversalRoot = &_model;

	ArrayPtrs<VisibleObject>* collectVisibleObjects = new ArrayPtrs<VisibleObject>;
	collectVisibleObjects->setMemoryOwner(false);

	// Get properties of to pobject
	// if Object is an instance of a visible object then return it, otherwise go down the properties
	VisibleObject* vis;
	if(vis = dynamic_cast<VisibleObject*> (traversalRoot)){
		collectVisibleObjects->append(vis);
		return collectVisibleObjects;
	}
	PropertySet& rootProps = traversalRoot->getPropertySet();
	for(int i=0; i < rootProps.getSize(); i++){
		Property* nextProp = rootProps.get(i);
		switch(nextProp->getType()){
			case Property::Obj:
				{	// Recur
					Object& propObj = nextProp->getValueObj();
					collectVisibleObjects->append(*getVisibleObjects(&propObj));
					break;
				}
			case Property::ObjArray:
				{
					ArrayPtrs<Object>& propObjArray = nextProp->getValueObjArray();
					for(int j=0; j< propObjArray.getSize(); j++){
						collectVisibleObjects->append(*getVisibleObjects(propObjArray.get(j)));
					}
					break;
				}
			default:
				break;
		}
	}
	return collectVisibleObjects;
}
