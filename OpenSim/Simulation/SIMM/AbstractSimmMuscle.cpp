// AbstractSimmMuscle.cpp
// Author: Peter Loan
/*
 * Copyright (c) 2006, Stanford University. All rights reserved. 
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "AbstractSimmMuscle.h"
#include "AbstractCoordinate.h"
#include "SimmMuscleGroup.h"
#include "SimmMuscleViaPoint.h"
#include "SimmMuscleWrapPoint.h"
#include "WrapResult.h"
#include "AbstractModel.h"
#include "AbstractBody.h"
#include "AbstractDof.h"
#include "SimmMacros.h"
#include <OpenSim/Tools/Mtx.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/NatCubicSpline.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
AbstractSimmMuscle::AbstractSimmMuscle() :
   AbstractActuator(),
	_attachmentSetProp(PropertyObj("", SimmMusclePointSet())),
	_attachmentSet((SimmMusclePointSet&)_attachmentSetProp.getValueObj()),
 	_displayerProp(PropertyObj("", VisibleObject())),
   _displayer((VisibleObject&)_displayerProp.getValueObj()),
	_groupNames(_groupNamesProp.getValueStrArray()),
	_groups(NULL),
	_muscleWrapSetProp(PropertyObj("", MuscleWrapSet())),
	_muscleWrapSet((MuscleWrapSet&)_muscleWrapSetProp.getValueObj()),
   _muscleModelIndex(_muscleModelIndexProp.getValueInt()),
	_currentPath(NULL)
{
	setNull();
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
AbstractSimmMuscle::AbstractSimmMuscle(DOMElement *aElement) :
   AbstractActuator(aElement),
	_attachmentSetProp(PropertyObj("", SimmMusclePointSet())),
	_attachmentSet((SimmMusclePointSet&)_attachmentSetProp.getValueObj()),
 	_displayerProp(PropertyObj("", VisibleObject())),
   _displayer((VisibleObject&)_displayerProp.getValueObj()),
	_groupNames(_groupNamesProp.getValueStrArray()),
	_groups(NULL),
	_muscleWrapSetProp(PropertyObj("", MuscleWrapSet())),
	_muscleWrapSet((MuscleWrapSet&)_muscleWrapSetProp.getValueObj()),
   _muscleModelIndex(_muscleModelIndexProp.getValueInt()),
	_currentPath(NULL)
{
	setNull();
	setupProperties();
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
AbstractSimmMuscle::~AbstractSimmMuscle()
{
	VisibleObject* disp;
	if ((disp = getDisplayer())){
		 // Free up allocated geometry objects
		disp->freeGeometry();
	}
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMuscle AbstractSimmMuscle to be copied.
 */
AbstractSimmMuscle::AbstractSimmMuscle(const AbstractSimmMuscle &aMuscle) :
   AbstractActuator(aMuscle),
	_attachmentSetProp(PropertyObj("", SimmMusclePointSet())),
	_attachmentSet((SimmMusclePointSet&)_attachmentSetProp.getValueObj()),
 	_displayerProp(PropertyObj("", VisibleObject())),
   _displayer((VisibleObject&)_displayerProp.getValueObj()),
	_groupNames(_groupNamesProp.getValueStrArray()),
	_groups(NULL),
	_muscleWrapSetProp(PropertyObj("", MuscleWrapSet())),
	_muscleWrapSet((MuscleWrapSet&)_muscleWrapSetProp.getValueObj()),
   _muscleModelIndex(_muscleModelIndexProp.getValueInt()),
	_currentPath(NULL)
{
	setNull();
	setupProperties();
	copyData(aMuscle);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one AbstractSimmMuscle to another.
 *
 * @param aMuscle AbstractSimmMuscle to be copied.
 */
void AbstractSimmMuscle::copyData(const AbstractSimmMuscle &aMuscle)
{
	_attachmentSet = aMuscle._attachmentSet;
	_displayer = aMuscle._displayer;
	_groupNames = aMuscle._groupNames;
	_groups = aMuscle._groups;
	_muscleWrapSet = aMuscle._muscleWrapSet;
	_muscleModelIndex = aMuscle._muscleModelIndex;
	_preScaleLength = 0.0;
}

//_____________________________________________________________________________
/**
 * Set the data members of this AbstractSimmMuscle to their null values.
 */
void AbstractSimmMuscle::setNull()
{
	setType("AbstractSimmMuscle");

	_length = 0.0;
	_preScaleLength = 0.0;

	_currentPath.setSize(0);
	_pathValid = false;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void AbstractSimmMuscle::setupProperties()
{
	_attachmentSetProp.setName("SimmMusclePointSet");
	_propertySet.append(&_attachmentSetProp);

	_displayerProp.setName("display");
	_propertySet.append(&_displayerProp);

	_groupNamesProp.setName("groups");
	_propertySet.append(&_groupNamesProp);

	_muscleWrapSetProp.setName("MuscleWrapSet");
	_propertySet.append(&_muscleWrapSetProp);

	_muscleModelIndexProp.setName("muscle_model");
	_muscleModelIndexProp.setValue(4);
	_propertySet.append(&_muscleModelIndexProp);
}

//_____________________________________________________________________________
/**
 * Register types that are used by this class.
 */
void AbstractSimmMuscle::registerTypes()
{
	Object::RegisterType(SimmMusclePoint());
	Object::RegisterType(SimmMuscleViaPoint());
}

//_____________________________________________________________________________
/**
 * Perform set up functions after model has been deserialized or copied.
 *
 * @param aModel model containing this muscle.
 */
void AbstractSimmMuscle::setup(AbstractModel* aModel)
{
	int i;

	AbstractActuator::setup(aModel);

	for (i = 0; i < _attachmentSet.getSize(); i++){
		_attachmentSet.get(i)->setup(aModel, this);
		// Muscle points depend on the muscle itself
		getDisplayer()->addDependent(_attachmentSet.get(i)->getDisplayer());
	}

	_displayer.setOwner(this);

	_groups.setSize(0);
	for (i = 0; i < _groupNames.getSize(); i++)
		_groups.append(aModel->enterGroup(_groupNames[i]));

	for (i = 0; i < _muscleWrapSet.getSize(); i++)
		_muscleWrapSet[i]->setup(&aModel->getDynamicsEngine());

	calculatePath();
}
//_____________________________________________________________________________
/**
 * updateGeometrySize updates the size of the array of geometry items to be of the
 * correct size based on muscle changes
 * 
 */
void AbstractSimmMuscle::updateGeometrySize()
{
	int numberOfSegements = _displayer.countGeometry();

	// Track whether we're creating geometry from scratch or
	// just updating
	bool update = (numberOfSegements!=0);  
	int newNumberOfSegments=_currentPath.getSize()-1;
	// Keep track whether any Geometry needs to be created/deleted.
	bool segmentsChanged = (update && (newNumberOfSegments!= numberOfSegements));
	// update geom array to have correct number of entries
	if (!update || (update && segmentsChanged)){	
		if (newNumberOfSegments > numberOfSegements){ // add entries
			for(int segment = numberOfSegements; 
						segment < newNumberOfSegments; 
						segment++){
							Geometry *g = new LineGeometry();
							g->setFixed(false);
							_displayer.addGeometry(g);
						}

		}
		else {	// remove entries
			for(int segment = newNumberOfSegments; 
						segment < numberOfSegements; 
						segment++)
				_displayer.removeGeometry(_displayer.getGeometry(segment));
		}
	}
}
//_____________________________________________________________________________
/**
 * updateGeometryLocations updates the locations of the array of geometry items 
 * to be in the right place based on muscle changes
 * 
 */
void AbstractSimmMuscle::updateGeometryLocations()
{
	double globalLocation[3];
	double previousPointGlobalLocation[3];

	for (int i = 0; i < _currentPath.getSize(); i++){
		SimmMusclePoint* nextPoint = _attachmentSet.get(i);
		// xform point to global frame
		Array<double>& location=nextPoint->getAttachment();
		const AbstractBody* body = nextPoint->getBody();
		AbstractDynamicsEngine* engine = (const_cast<AbstractBody*> (body))->getDynamicsEngine();
		Transform xform = engine->getTransform(*body);
		if (i > 0){
			for(int j=0; j < 3; j++)
				previousPointGlobalLocation[j] = globalLocation[j];
		}
		engine->transformPosition(*body, location.get(), globalLocation);
		// Make a segment between globalLocation, previousPointGlobalLocation
		if (i > 0){
			// Geometry will be deleted when the object is deleted.
			LineGeometry *g = dynamic_cast<LineGeometry *>(_displayer.getGeometry(i-1));
			g->setPoints(previousPointGlobalLocation, globalLocation);
		}
	}
}
//_____________________________________________________________________________
/**
 * Update the geometric representation of the muscle.
 * The resulting geometry is maintained at the VisibleObject layer
 * 
 */
void AbstractSimmMuscle::updateGeometry()
{
	// Setup has been invoked already on muscle points so they are
	// in the right location, xform to global frame and create connectors
	// _allGeometry array already has the lastest geometry, should not need
	// to invalidate unless something changed (e.g. a viaPoint changed state).
	updateGeometrySize();
	updateGeometryLocations();
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
AbstractSimmMuscle& AbstractSimmMuscle::operator=(const AbstractSimmMuscle &aMuscle)
{
	// BASE CLASS
	AbstractActuator::operator=(aMuscle);

	copyData(aMuscle);

	return(*this);
}

//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
 * Perform computations that need to happen before the muscle is scaled.
 * For this object, that entails calculating and storing the musculotendon
 * length is the default body position.
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 */
void AbstractSimmMuscle::preScale(const ScaleSet& aScaleSet)
{
	_preScaleLength = getLength();
}

//_____________________________________________________________________________
/**
 * Scale the muscle based on XYZ scale factors for each body.
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 * @return Whether muscle was successfully scaled or not.
 */
void AbstractSimmMuscle::scale(const ScaleSet& aScaleSet)
{
	for (int i = 0; i < _attachmentSet.getSize(); i++)
	{
		const string& bodyName = _attachmentSet.get(i)->getBodyName();
		for (int j = 0; j < aScaleSet.getSize(); j++)
		{
			Scale *aScale = aScaleSet.get(j);
			if (bodyName == aScale->getSegmentName())
			{
				Array<double> scaleFactors(1.0, 3);
				aScale->getScaleFactors(scaleFactors);
				_attachmentSet.get(i)->scale(scaleFactors);
			}
		}
	}
}

//_____________________________________________________________________________
/**
 * Perform computations that need to happen after the muscle is scaled.
 * In most cases, these computations would be performed in the
 * classes descending from AbstractSimmMuscle, where the force-generating
 * properties are stored (except for the recalculation of the path, which
 * is handled here).
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 */
void AbstractSimmMuscle::postScale(const ScaleSet& aScaleSet)
{
	// Update Geometry to reflect scaling.
	// Done here since scale is invoked before bodies are scaled
	// so we may not have enough info to update (e.g. wrapping, via points)
	updateGeometry();	

	calculatePath();
}

//--------------------------------------------------------------------------
// COMPUTATIONS
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute values needed for calculating the muscle's actuation.
 * Right now this is only speed (contraction velocity; all other calculations
 * are handled by the derived classes.
 */
void AbstractSimmMuscle::computeActuation()
{
	double posRelative[3], velRelative[3];
	double posStart[3], posEnd[3], velStart[3], velEnd[3];
	SimmMusclePoint *start, *end;

	calculatePath();

	_speed = 0.0;

	AbstractDynamicsEngine &engine = _model->getDynamicsEngine();

	int i;
	for (i = 0; i < _currentPath.getSize() - 1; i++) {
		start = _currentPath[i];
		end = _currentPath[i+1];

		// Find the positions and velocities in the inertial frame.
		engine.getPosition(*(start->getBody()), start->getAttachment().get(), posStart);
		engine.getPosition(*(end->getBody()), end->getAttachment().get(), posEnd);
		engine.getVelocity(*(start->getBody()), start->getAttachment().get(), velStart);
		engine.getVelocity(*(end->getBody()), end->getAttachment().get(), velEnd);

		// Calculate the relative positions and velocities.
		for (int j = 0; j < 3; j++) {
			posRelative[j] = posEnd[j] - posStart[j];
			velRelative[j] = velEnd[j] - velStart[j];
		}

		// Normalize the vector from start to end.
		Mtx::Normalize(3, posRelative, posRelative);

		// Dot the relative velocity with the unit vector from start to end,
		// and add this speed to the running total.
		//
		_speed += (velRelative[0] * posRelative[0] +
			        velRelative[1] * posRelative[1] +
			        velRelative[2] * posRelative[2]);
	}
}

//=============================================================================
// LENGTH AND MOMENT ARM
//=============================================================================
//_____________________________________________________________________________
/**
 * Calculate the current path of the muscle.
 *
 */
void AbstractSimmMuscle::calculatePath()
{
	if (_pathValid == true)
		return;

	// empty out the current path
	_currentPath.setSize(0);

	// add the fixed and active via points to the path
	int i;
   for (i = 0; i < _attachmentSet.getSize(); i++) {
		if (_attachmentSet[i]->isActive())
			_currentPath.append(_attachmentSet[i]);
	}

   // use the current path so far to check for intersection
	// with wrap objects, which may add additional points to
	// the path
	applyWrapObjects();

	calculateLength();

	_pathValid = true;
}

//_____________________________________________________________________________
/**
 * Apply the wrap objects to the current muscle path.
 *
 */
void AbstractSimmMuscle::applyWrapObjects()
{
   int i, j, kk, pt1, pt2, maxIterations;
   int start, end, wrapStart, wrapEnd;
   double min_length_change, last_length;
   WrapResult best_wrap;
   SimmMusclePoint *smp, *emp;
   MuscleWrap* ws;
   AbstractWrapObject* wo;
   Array<int> result;
	Array<int> order;

	if (_muscleWrapSet.getSize() < 1)
		return;

	result.setSize(_muscleWrapSet.getSize());
	order.setSize(_muscleWrapSet.getSize());

   /* Set the initial order to be the order they are listed in the muscle. */
   for (i = 0; i < _muscleWrapSet.getSize(); i++)
      order[i] = i;

   /* If there is only one wrap object, calculate the wrapping only once.
    * If there are two or more objects, perform up to 8 iterations where
    * the result from one wrap object is used as the starting point for
    * the next wrap.
    */
   if (_muscleWrapSet.getSize() < 2)
      maxIterations = 1;
   else
      maxIterations = 8;

   for (kk = 0, last_length = 999999.999; kk < maxIterations; kk++)
   {
      for (i = 0; i < _muscleWrapSet.getSize(); i++)
      {
         result[i] = 0;
         ws = _muscleWrapSet.get(order[i]);
			wo = ws->getWrapObject();
         best_wrap.wrap_pts.setSize(0);
			min_length_change = rdMath::INFINITY;

         /* First remove this object's wrapping points from the current muscle path. */
         for (j = 0; j < _currentPath.getSize(); j++)
         {
				if (_currentPath.get(j) == &ws->getWrapPoint(0))
            {
					_currentPath.remove(j); // remove the first wrap point
					_currentPath.remove(j); // remove the second wrap point
               break;
            }
         }

         if (wo->getActive())
         {
            /* startPoint and endPoint in wrapStruct represent the user-defined
             * starting and ending points in the array of muscle points that should
             * be considered for wrapping. These indices take into account via points,
             * whether or not they are active. Thus they are indices into mp_orig[],
             * not mp[] (also, mp[] may contain wrapping points from previous wrap
             * objects, which would mess up the starting and ending indices). But the
             * goal is to find starting and ending indices in mp[] to consider for
             * wrapping over this wrap object. Here is how that is done:
             */

            /* 1. startPoint and endPoint are 1-based, so subtract 1 from them to get
             * indices into _attachmentSet. -1 (or any value less than 1) means use the first
             * (or last) point.
             */
				if (ws->getStartPoint() < 1)
               wrapStart = 0;
            else
               wrapStart = ws->getStartPoint() - 1;

            if (ws->getEndPoint() < 1)
               wrapEnd = _attachmentSet.getSize() - 1;
            else
               wrapEnd = ws->getEndPoint() - 1;

				/* 2. Scan forward from wrapStart in _attachmentSet to find the first point
				 * that is active. Store a pointer to it (smp).
				 */
				for (j = wrapStart; j <= wrapEnd; j++)
					if (_attachmentSet.get(j)->isActive())
						break;
				if (j > wrapEnd) // there are no active points in the muscle
					return;
				smp = _attachmentSet.get(j);

				/* 3. Scan backwards from wrapEnd in _attachmentSet to find the last
				 * point that is active. Store a pointer to it (emp).
				 */
				for (j = wrapEnd; j >= wrapStart; j--)
					if (_attachmentSet.get(j)->isActive())
						break;
				if (j < wrapStart) // there are no active points in the muscle
					return;
				emp = _attachmentSet.get(j);

				/* 3. Now find the indices of smp and emp in _currentPath.
				 */
				for (j = 0, start = -1, end = -1; j < _currentPath.getSize(); j++)
				{
					if (_currentPath.get(j) == smp)
						start = j;
					if (_currentPath.get(j) == emp)
						end = j;
				}
				if (start == -1 || end == -1) // this should never happen
					return;

				/* You now have indices into _currentPath (which is a list of all currently active points,
             * including wrap points) that represent the used-defined range of points to consider
             * for wrapping over this wrap object. Check each muscle segment in this range, choosing
             * the best wrap as the one that changes the muscle segment length the least:
             */
            for (pt1 = start; pt1 < end; pt1++)
            {
               pt2 = pt1 + 1;

               /* As long as the two points are not auto wrap points on the
                * same wrap object, check them for wrapping.
                */
					if (_currentPath.get(pt1)->getWrapObject() == NULL &&
						_currentPath.get(pt2)->getWrapObject() == NULL)
               {
                  WrapResult wr;

						wr.startPoint = pt1;
						wr.endPoint = pt2;

                  result[i] = wo->wrapMuscleSegment(*_currentPath.get(pt1), *_currentPath.get(pt2), *ws, wr);
                  if (result[i] == AbstractWrapObject::mandatoryWrap)
                  {
                     /* mandatoryWrap means the muscle line actually intersected
                      * the wrap object. In this case, you *must* choose this segment
                      * as the "best" one for wrapping. If the muscle has more than
                      * one segment that intersects the object, the first one is taken
                      * as the mandatory wrap (this is considered an ill-conditioned case).
                      */
                     best_wrap = wr;
							// store the best wrap in the muscleWrap for possible use next time
							ws->setPreviousWrap(wr);
                     break;
                  }
                  else if (result[i] == AbstractWrapObject::wrapped)
                  {
                     /* E_WRAPPED means the muscle segment was wrapped over the object,
                      * but you should consider the other segments as well to see if one
                      * wraps with a smaller length change.
                      */
                     double muscle_length_change = _calc_muscle_length_change(*wo, wr);
                     if (muscle_length_change < min_length_change)
                     {
                        best_wrap = wr;
								// store the best wrap in the muscleWrap for possible use next time
								ws->setPreviousWrap(wr);
                        min_length_change = muscle_length_change;
                     }
                     else
                     {
                        /* the wrap was not shorter than the current minimum,
                         * so just free the wrap points that were allocated.
                         */
								wr.wrap_pts.setSize(0);
                     }
                  }
                  else
                  {
                  }
               }
            }

            /* deallocate previous wrapping points if necessary: */
				ws->getWrapPoint(1).getWrapPath().setSize(0);

				if (best_wrap.wrap_pts.getSize() == 0)
            {
					if (ws) {
						ws->resetPreviousWrap();
                  ws->getWrapPoint(1).getWrapPath().setSize(0);
					}
            }
            else
            {
               /* if wrapping did occur, copy wrap information into the MuscleStruct */
               ws->getWrapPoint(0).getWrapPath().setSize(0);

					Array<SimmPoint>& wrapPath = ws->getWrapPoint(1).getWrapPath();
					wrapPath = best_wrap.wrap_pts;

					// In OpenSim, all conversion to/from the wrap object's reference
					// frame will be performed inside wrapMuscleSegment(). Thus, all
					// points in this function will be in their respective body
					// reference frames.
               // for (j = 0; j < wrapPath.getSize(); j++){
               //    convert_from_wrap_object_frame(wo, wrapPath.get(j));
               //    convert(ms->modelnum, wrapPath.get(j), wo->segment, ms->ground_segment);
               // }

					ws->getWrapPoint(0).setWrapLength(0.0);
					ws->getWrapPoint(1).setWrapLength(best_wrap.wrap_path_length);
					ws->getWrapPoint(0).setBody(*wo->getBody());
					ws->getWrapPoint(1).setBody(*wo->getBody());

					ws->getWrapPoint(0).setAttachment(best_wrap.r1);
					ws->getWrapPoint(1).setAttachment(best_wrap.r2);

               // Now insert the two new wrapping points into the mp[] array.
					_currentPath.insert(best_wrap.endPoint, &ws->getWrapPoint(0));
					_currentPath.insert(best_wrap.endPoint + 1, &ws->getWrapPoint(1));
            }
         }
      }

		calculateLength(); // length is stored in _length

      if (DABS(_length - last_length) < 0.0005)
      {
         break;
      }
      else
      {
         last_length = _length;
      }

      if (kk == 0 && _muscleWrapSet.getSize() > 1)
      {
         /* If the first wrap was a no wrap, and the second was a no wrap
          * because a point was inside the object, switch the order of
          * the first two objects and try again.
          */
			if (result[0] == AbstractWrapObject::noWrap && result[1] == AbstractWrapObject::insideRadius)
         {
            order[0] = 1;
            order[1] = 0;

            // remove wrap object 0 from the list of muscle points
				int index = 0;
            ws = _muscleWrapSet.get(index);
				for (j = 0; j < _currentPath.getSize(); j++)
				{
					if (_currentPath.get(j) == &ws->getWrapPoint(0))
					{
						_currentPath.remove(j); // remove the first wrap point
						_currentPath.remove(j); // remove the second wrap point
						break;
					}
				}
         }
      }
   }
}

/* -------------------------------------------------------------------------
   _calc_muscle_length_change - given the output of a successful muscle wrap
      over a wrap object, determine the percent change in length of the
      muscle segment incurred by wrapping.
---------------------------------------------------------------------------- */
double AbstractSimmMuscle::_calc_muscle_length_change(AbstractWrapObject& wo, WrapResult& wr)
{
	SimmMusclePoint* pt1 = _currentPath.get(wr.startPoint);
	SimmMusclePoint* pt2 = _currentPath.get(wr.endPoint);

   double straight_length = getModel()->getDynamicsEngine().calcDistance(*pt1->getBody(), pt1->getAttachment(),
		*pt2->getBody(), pt2->getAttachment());

	double* p1 = &pt1->getAttachment()[0];
	double* p2 = &pt2->getAttachment()[0];
   double wrap_length = getModel()->getDynamicsEngine().calcDistance(*pt1->getBody(), p1, *wo.getBody(), wr.r1);
   wrap_length += wr.wrap_path_length;
   wrap_length += getModel()->getDynamicsEngine().calcDistance(*wo.getBody(), wr.r2, *pt2->getBody(), p2);

   return wrap_length - straight_length; // return absolute diff, not relative
}

//_____________________________________________________________________________
/**
 * Calculate the current length of the muscle. This function assumes that the
 * path of the muscle has already been updated.
 *
 */
void AbstractSimmMuscle::calculateLength()
{
	_length = 0.0;

	AbstractDynamicsEngine &engine = _model->getDynamicsEngine();

	int i;

	for (i = 0; i < _currentPath.getSize() - 1; i++) {
		SimmMusclePoint* p1 = _currentPath[i];
		SimmMusclePoint* p2 = _currentPath[i+1];

      /* If both points are wrap points on the same wrap object, then this muscle
       * segment wraps over the surface of a wrap object, so just add in the
       * pre-calculated length.
       */
      if (p1->getWrapObject() && p2->getWrapObject() && p1->getWrapObject() == p2->getWrapObject()) {
			SimmMuscleWrapPoint* smwp = dynamic_cast<SimmMuscleWrapPoint*>(p2);
			if (smwp) {
				_length += smwp->getWrapLength();
				//printf("%d to %d (w)= %.6lf\n", i, i+1, smwp->getWrapLength());
			}
      } else {
         _length += engine.calcDistance(*p1->getBody(), p1->getAttachment(), *p2->getBody(), p2->getAttachment());
#if 0
			printf("%d to %d (-)= %.6lf (%.6lf %.6lf %.6lf) (%.6lf %.6lf %.6lf)\n", i, i+1, engine.calcDistance(*p1->getBody(), p1->getAttachment(), *p2->getBody(), p2->getAttachment()),
				p1->getAttachment()[0],
				p1->getAttachment()[1],
				p1->getAttachment()[2],
				p2->getAttachment()[0],
				p2->getAttachment()[1],
				p2->getAttachment()[2]);
#endif
      }
   }
	//printf("total length = %.12lf\n", _length);
}

//_____________________________________________________________________________
/**
 * Get the current length of the muscle.
 *
 * @return Current length of the muscle.
 */
double AbstractSimmMuscle::getLength()
{
	if (_pathValid == false)
		calculatePath();

	return _length;
}

//_____________________________________________________________________________
/**
 * Compute the muscle's moment arm for a coordinate, using the dl/dtheta
 * numerical technique. The length of the muscle is found at genc-delta,
 * genc, and genc+delta. A quadratic is fit to these three points, and
 * the derivative is found at genc. This is equal to the negative of the
 * moment arm. *
 * @param aCoord Coordinate for which to compute moment arm.
 * @return Moment arm value.
 */   
double AbstractSimmMuscle::getMomentArm(AbstractCoordinate& aCoord)
{
   double delta, originalValue, x[3], y[3];
	bool clampedSave, lockedSave;

	calculatePath();

	if (aCoord.getMotionType() == AbstractDof::Rotational)
		delta = 0.000873; /* 0.05 degrees * DEG_TO_RAD */
	else
		delta = 0.00005; /* model units, assume meters (TODO) */

	/* Unclamp and unlock the gencoord so you can change it by +/- delta
	 * without worrying about hitting the end of the range.
	 */
	clampedSave = aCoord.getClamped();
	lockedSave = aCoord.getLocked();
	aCoord.setClamped(false);
	aCoord.setLocked(false);
	originalValue = aCoord.getValue();

	// This code assumes that coordinate angles are in radians.
	x[1] = originalValue;
	y[1] = getLength();

	aCoord.setValue(originalValue - delta);
	x[0] = aCoord.getValue();
	y[0] = getLength();

	aCoord.setValue(originalValue + (2 * delta));
	x[2] = aCoord.getValue();
	y[2] = getLength();

	/* Put the gencoord back to its original value. */
	aCoord.setValue(originalValue);
	aCoord.setClamped(clampedSave);
	aCoord.setLocked(lockedSave);

	/* Make a spline function with room for 3 points, to hold the muscle
	 * length at the current coordinate value and at +/- delta from
	 * the current value.
	 */
	NatCubicSpline spline(3, x, y);

	/* The muscle's moment arm is the negative of the first derivative
	 * of this spline function at originalValue.
	 */
	return -spline.evaluate(1, originalValue, 1.0, 1.0);
}

double AbstractSimmMuscle::getSpeed() const
{
	return _speed;
}

//_____________________________________________________________________________
/**
 * Utility function to calculate the current pennation angle in a
 * muscle. Pennation angle increases as muscle fibers shorten. The implicit
 * modeling assumption is that muscles have constant volume.
 *
 * @param aFiberLength Current fiber length of muscle.
 * @param aOptimalFiberLength Optimal fiber length of muscle.
 * @param aInitialPennationAngle Pennation angle at optimal fiber length (in radians).
 * @return Current pennation angle (in radians).
 */
double AbstractSimmMuscle::calcPennation(double aFiberLength, double aOptimalFiberLength,
													  double aInitialPennationAngle) const
{
   double value = aOptimalFiberLength * sin(aInitialPennationAngle) / aFiberLength;

   if (value <= 0.0)
      return 0.0;
   else if (value >= 1.0)
		return rdMath::PI_2;
   else
      return asin(value);
}

//=============================================================================
// FORCE APPLICATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Apply the muscle's force at its points of attachment to the bodies.
 */
void AbstractSimmMuscle::apply()
{
	// NOTE: Force could be negative, in particular during CMC, when the optimizer is computing
	// gradients, it will setForce(+1) and setForce(-1) to compute the derivative with respect to force.
	if (fabs(getForce()) < TINY_NUMBER)
		return;

	AbstractDynamicsEngine &engine = _model->getDynamicsEngine();

	int i;
	SimmMusclePoint* start;
	SimmMusclePoint* end;
	const AbstractBody* startBody;
	const AbstractBody* endBody;
	for (i = 0; i < _currentPath.getSize() - 1; i++) {
		start = _currentPath[i];
		end = _currentPath[i+1];
		startBody = start->getBody();
		endBody = end->getBody();

		if (startBody != endBody)
		{
			double posStart[3], posEnd[3], forceVector[3], bodyForce[3];

			/* Find the positions of start and end in the inertial frame. */
			engine.getPosition(*(start->getBody()), start->getAttachment().get(), posStart);
			engine.getPosition(*(end->getBody()), end->getAttachment().get(), posEnd);

			/* Form a vector from start to end, in the inertial frame. */
			for (int j = 0; j < 3; j++)
				forceVector[j] = posEnd[j] - posStart[j];

			/* Normalize the vector from start to end. */
			Mtx::Normalize(3, forceVector, forceVector);

			double muscleForce = getForce();

			/* The force on the start body is in the direction of forceVector. */
			Mtx::Multiply(1, 3, forceVector, muscleForce, bodyForce);
			engine.applyForce(*(start->getBody()), start->getAttachment().get(), bodyForce);

			/* The force on the end body is in the opposite direction of forceVector. */
			Mtx::Multiply(1, 3, forceVector, -muscleForce, bodyForce);
			engine.applyForce(*(end->getBody()), end->getAttachment().get(), bodyForce);
		}
	}
}

//=============================================================================
// TESTING
//=============================================================================
void AbstractSimmMuscle::peteTest() const
{
	int i;

	cout << "Muscle: " << getName() << endl;
	for (i = 0; i < _attachmentSet.getSize(); i++)
		_attachmentSet.get(i)->peteTest();
	cout << "   groups: ";
	for (i = 0; i < _groupNames.getSize(); i++)
		cout << _groupNames[i] << " ";
	cout << endl;
	for (i = 0; i < _muscleWrapSet.getSize(); i++)
		_muscleWrapSet.get(i)->peteTest();
	cout << "   current length: " << _length << endl;
}
