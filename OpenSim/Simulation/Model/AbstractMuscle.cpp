// AbstractMuscle.cpp
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
#include "AbstractMuscle.h"
#include "AbstractCoordinate.h"
#include "MuscleViaPoint.h"
#include <OpenSim/Simulation/Wrap/MuscleWrapPoint.h>
#include <OpenSim/Simulation/Wrap/WrapResult.h>
#include <OpenSim/Simulation/Wrap/MuscleWrap.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include "Model.h"
#include "AbstractBody.h"
#include "AbstractDof.h"
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/NatCubicSpline.h>

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
AbstractMuscle::AbstractMuscle() :
   AbstractActuator(),
	_attachmentSetProp(PropertyObj("", MusclePointSet())),
	_attachmentSet((MusclePointSet&)_attachmentSetProp.getValueObj()),
 	_displayerProp(PropertyObj("", VisibleObject())),
   _displayer((VisibleObject&)_displayerProp.getValueObj()),
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
 * Destructor.
 */
AbstractMuscle::~AbstractMuscle()
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
 * @param aMuscle AbstractMuscle to be copied.
 */
AbstractMuscle::AbstractMuscle(const AbstractMuscle &aMuscle) :
   AbstractActuator(aMuscle),
	_attachmentSetProp(PropertyObj("", MusclePointSet())),
	_attachmentSet((MusclePointSet&)_attachmentSetProp.getValueObj()),
 	_displayerProp(PropertyObj("", VisibleObject())),
   _displayer((VisibleObject&)_displayerProp.getValueObj()),
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
 * Copy data members from one AbstractMuscle to another.
 *
 * @param aMuscle AbstractMuscle to be copied.
 */
void AbstractMuscle::copyData(const AbstractMuscle &aMuscle)
{
	_attachmentSet = aMuscle._attachmentSet;
	_displayer = aMuscle._displayer;
	_muscleWrapSet = aMuscle._muscleWrapSet;
	_muscleModelIndex = aMuscle._muscleModelIndex;
	_preScaleLength = 0.0;
	_pathValid = false;  // path should be recalculated because state may have changed
}

//_____________________________________________________________________________
/**
 * Set the data members of this AbstractMuscle to their null values.
 */
void AbstractMuscle::setNull()
{
	setType("AbstractMuscle");

	_length = 0.0;
	_preScaleLength = 0.0;

	_currentPath.setSize(0);
	_pathValid = false;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void AbstractMuscle::setupProperties()
{
	_attachmentSetProp.setName("MusclePointSet");
	_propertySet.append(&_attachmentSetProp);

	_displayerProp.setName("display");
	_propertySet.append(&_displayerProp);

	_muscleWrapSetProp.setName("MuscleWrapSet");
	_propertySet.append(&_muscleWrapSetProp);

	_muscleModelIndexProp.setName("muscle_model");
	_muscleModelIndexProp.setValue(4);
	_propertySet.append(&_muscleModelIndexProp, "Parameters");
}

//_____________________________________________________________________________
/**
 * Perform set up functions after model has been deserialized or copied.
 *
 * @param aModel model containing this muscle.
 */
void AbstractMuscle::setup(Model* aModel)
{
	int i;

	AbstractActuator::setup(aModel);

	nameAttachmentPoints(0);

	for (i = 0; i < _attachmentSet.getSize(); i++){
		_attachmentSet.get(i)->setup(aModel, this);
		// Muscle points depend on the muscle itself
		// Removing the dependency since muscle points now display as part of the
		// muscle itself, extracted directly from the set of line segments
		// representing the muscle path. -Ayman 02/07
		//getDisplayer()->addDependent(_attachmentSet.get(i)->getDisplayer());
	}

	_displayer.setOwner(this);

	for (i = 0; i < _muscleWrapSet.getSize(); i++)
		_muscleWrapSet[i]->setup(&aModel->getDynamicsEngine(), this);

	computePath();
}

//_____________________________________________________________________________
/**
 * Name the attachment points based on their position in the set.
 *
 * @param aStartingIndex the index of the first attachment point to name.
 */
void AbstractMuscle::nameAttachmentPoints(int aStartingIndex)
{
	char indx[5];
	for (int i = aStartingIndex; i < _attachmentSet.getSize(); i++)
	{
		sprintf(indx,"%d",i+1);
		_attachmentSet.get(i)->setName(getName() + "-P" + indx);
	}
}

const Array<MusclePoint*> AbstractMuscle::getCurrentPath()
{
	if (_pathValid == false)
		computePath();

	return _currentPath;
}

//_____________________________________________________________________________
/**
 * updateGeometrySize updates the size of the array of geometry items to be of the
 * correct size based on muscle changes
 * 
 */
void AbstractMuscle::updateGeometrySize()
{
	int numberOfSegements = _displayer.countGeometry();

	// Track whether we're creating geometry from scratch or
	// just updating
	bool update = (numberOfSegements!=0);  
	int newNumberOfSegments=_currentPath.getSize()-1;
	if (newNumberOfSegments <= 0)
		return;
	// update geom array to have correct number of entries
	if (!update || (update && (newNumberOfSegments != numberOfSegements))){	
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
			for(int segment = numberOfSegements-1;
						segment >= newNumberOfSegments; 
						segment--) // Remove back to front so no array packing is needed
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
void AbstractMuscle::updateGeometryLocations()
{
	double globalLocation[3];
	double previousPointGlobalLocation[3];

	for (int i = 0; i < _currentPath.getSize(); i++){
		MusclePoint* nextPoint = _currentPath.get(i);
		// xform point to global frame
		Array<double>& location=nextPoint->getAttachment();
		const AbstractBody* body = nextPoint->getBody();
		AbstractDynamicsEngine* engine = (const_cast<AbstractBody*> (body))->getDynamicsEngine();
		if (i > 0){
			for(int j=0; j < 3; j++)
				previousPointGlobalLocation[j] = globalLocation[j];
		}
		engine->transformPosition(*body, location.get(), globalLocation);
		// Make a segment between globalLocation, previousPointGlobalLocation
		if (i > 0){
			// Geometry will be deleted when the object is deleted.
			LineGeometry *g = static_cast<LineGeometry *>(_displayer.getGeometry(i-1));
			g->setPoints(previousPointGlobalLocation, globalLocation);
		}
	}
}
//_____________________________________________________________________________
/**
 * Update the geometric representation of the muscle.
 * The resulting geometry is maintained at the VisibleObject layer.
 * This function should not be made public. It is called internally
 * by computePath() only when the path has changed.
 * 
 */
void AbstractMuscle::updateGeometry()
{
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
AbstractMuscle& AbstractMuscle::operator=(const AbstractMuscle &aMuscle)
{
	// BASE CLASS
	AbstractActuator::operator=(aMuscle);

	copyData(aMuscle);

	return(*this);
}


//=============================================================================
// GENERIC NORMALIZED FORCE-LENGTH-VELOCIY PROPERTIES
//=============================================================================
//_____________________________________________________________________________
/**
 * Evaluate the normalized force-length-velicity curve for muscle.
 * A simple generic implementation is used here.  Derived classes should
 * override this method for more precise evaluation of the
 * force-length-velocity curve.
 *
 * @param aActivation Activation level of the muscle.  1.0 is full activation;
 * 0.0 is no activation.
 * @param aNormalizedLength Normalized length of the muscle fibers.  1.0 indicates
 * the muscle fibers are at their optimal length.  Lnorm = L / Lo.
 * @param aNormalizedVelocity Normalized shortening velocity of the muscle fibers.
 * Positive values indicate concentric contraction (shortening); negative values
 * indicate eccentric contraction (lengthening).  Normalized velocity is
 * the fiber shortening velocity divided by the maximum shortening velocity times
 * the optimal fiber length.  Vnorm = V / (Vmax*Lo).
 * @return Force normalized by the optimal force.
 */
double AbstractMuscle::
evaluateForceLengthVelocityCurve(double aActivation,double aNormalizedLength,double aNormalizedVelocity)
{
	// FORCE-LENGTH
	double fLength = exp(-17.33 * fabs(pow(aNormalizedLength-1.0,3)));

	// FORCE-VELOCITY
	double fVelocity = 1.8  -  1.8 / (1.0 + exp( (0.04 - aNormalizedVelocity)/0.18) );

	return aActivation * fLength * fVelocity;
}



//=============================================================================
// GET
//=============================================================================
//-----------------------------------------------------------------------------
// LENGTH
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute the total length of the muscle tendon complex.
 *
 * @return Total length of the muscle and tendon.
 */
double AbstractMuscle::getLength()
{
	if(_pathValid==false) computePath();

	return _length;
}
//_____________________________________________________________________________
/**
 * Get the length of the tendon.
 *
 * @param Current length of the tendon.
 */
double AbstractMuscle::getTendonLength()
{
	return getLength() - getFiberLengthAlongTendon();
}
//_____________________________________________________________________________
/**
 * Get the length of the muscle fiber(s) along the tendon.  This method
 * accounts for the pennation angle. 
 *
 * @param Current length of the muscle fiber(s) along the direction of
 * the tendon.
 */
double AbstractMuscle::getFiberLengthAlongTendon()
{
	return getFiberLength() * cos(getPennationAngle());
}

//-----------------------------------------------------------------------------
// FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute the force generated by the muscle fibers.  This accounts for
 * pennation angle.  That is, the fiber force is computed by dividing the
 * actuator force by the cosine of the pennation angle.
 *
 * @return Force in the muscle fibers.
 */
double AbstractMuscle::getFiberForce()
{
	double force;
	double cos_penang = cos(getPennationAngle());
	if(fabs(cos_penang) < rdMath::ZERO) {
		force = rdMath::NAN;
	} else {
		force = getForce() / cos_penang;
	}

	return force;
}
//_____________________________________________________________________________
/**
 * Get the active force generated by the muscle fibers.
 *
 * @param Current active force of the muscle fiber(s).
 */
double AbstractMuscle::getActiveFiberForce()
{
	return getFiberForce() - getPassiveFiberForce();
}
//_____________________________________________________________________________
/**
 * Get the active force generated by the muscle fibers along the direction
 * of the tendon.
 *
 * @param Current active force of the muscle fiber(s) along tendon.
 */
double AbstractMuscle::getActiveFiberForceAlongTendon()
{
	return getActiveFiberForce() * cos(getPennationAngle());
}
//_____________________________________________________________________________
/**
 * Get the passive force generated by the muscle fibers along the direction
 * of the tendon.
 *
 * @return Current passive force of the muscle fiber(s) along tendon.
 */
double AbstractMuscle::getPassiveFiberForceAlongTendon()
{
	return getPassiveFiberForce() * cos(getPennationAngle());
}


//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Add a new attachment point, with default location, to the muscle.
 *
 * @param aIndex The position in the attachmentSet to put the new point in.
 * @param aBody The body to attach the point to.
 */
MusclePoint* AbstractMuscle::addAttachmentPoint(int aIndex, AbstractBody& aBody)
{
	MusclePoint* newPoint = new MusclePoint();
	newPoint->setBody(aBody);
	Array<double>& location = newPoint->getAttachment();
	placeNewAttachment(location, aIndex, aBody);
	newPoint->setup(getModel(), this);
	_attachmentSet.insert(aIndex, newPoint);

	// rename the attachment points starting at this new one
	nameAttachmentPoints(aIndex);

	invalidatePath();

	return newPoint;
}

//_____________________________________________________________________________
/**
 * Determine an appropriate default XYZ location for a new attachment point.
 *
 * @param aOffset The XYZ location to be determined.
 * @param aIndex The position in the attachmentSet to put the new point in.
 * @param aBody The body to attach the point to.
 */
void AbstractMuscle::placeNewAttachment(Array<double>& aOffset, int aIndex, AbstractBody& aBody)
{
	// The location of the point is determined by moving a 'distance' from 'base' along
	// a vector from 'start' to 'end.' 'base' is the existing attachment point that is
	// in or closest to the index aIndex. 'start' and 'end' are existing attachment points--
	// which ones depends on where the new point is being added. 'distance' is 0.5 for points
	// added to the middle of a muscle (so the point appears halfway between the two adjacent
	// points), and 0.2 for points that are added to either end of the muscle. If there is only
	// one point in the muscle, the new point is put 0.01 units away in all three dimensions.
	if (_attachmentSet.getSize() > 1) {
	   int start, end, base;
	   double distance;
		if (aIndex == 0) {
			start = 1;
			end = 0;
			base = end;
			distance = 0.2;
		} else if (aIndex >= _attachmentSet.getSize()) {
			start = aIndex - 2;
			end = aIndex - 1;
			base = end;
			distance = 0.2;
		} else {
			start = aIndex;
			end = aIndex - 1;
			base = start;
			distance = 0.5;
		}
	   Array<double>& startPt = _attachmentSet.get(start)->getAttachment();
	   Array<double>& endPt = _attachmentSet.get(end)->getAttachment();
	   Array<double>& basePt = _attachmentSet.get(base)->getAttachment();
		Array<double> startPt2(0.0, 3);
		Array<double> endPt2(0.0, 3);
	   getModel()->getDynamicsEngine().transformPosition(*_attachmentSet.get(start)->getBody(), startPt, aBody, startPt2);
	   getModel()->getDynamicsEngine().transformPosition(*_attachmentSet.get(end)->getBody(), endPt, aBody, endPt2);
		aOffset[0] = basePt[0] + distance * (endPt2[0] - startPt2[0]);
		aOffset[1] = basePt[1] + distance * (endPt2[1] - startPt2[1]);
		aOffset[2] = basePt[2] + distance * (endPt2[2] - startPt2[2]);
	} else {
		int foo = 0;
		for (int i = 0; i < 3; i++) {
         aOffset[i] = _attachmentSet.get(foo)->getAttachment().get(i) + 0.01;
		}
	}
}

//_____________________________________________________________________________
/**
 * Delete an attachment point.
 *
 * @param aIndex The index of the point to delete
 */
bool AbstractMuscle::deleteAttachmentPoint(int aIndex)
{
	// An attachment point can be deleted only if there would remain
	// at least two other fixed points.
	int numOtherFixedPoints = 0;
	for (int i = 0; i < _attachmentSet.getSize(); i++) {
		if (i != aIndex) {
			if (!_attachmentSet.get(i)->isA("MuscleViaPoint"))
				numOtherFixedPoints++;
		}
	}

	if (numOtherFixedPoints >= 2) {
		_attachmentSet.remove(aIndex);

		// rename the attachment points starting at the deleted position
		nameAttachmentPoints(aIndex);

		invalidatePath();
		return true;
	}

	return false;
}

//_____________________________________________________________________________
/**
 * Create a new wrap instance and add it to the set.
 *
 * @param aWrapObject The wrap object to use in the new wrap instance.
 */
void AbstractMuscle::addMuscleWrap(AbstractWrapObject& aWrapObject)
{
	MuscleWrap* newWrap = new MuscleWrap();
	newWrap->setWrapObject(aWrapObject);
	newWrap->setMethod(MuscleWrap::hybrid);
	newWrap->setup(&getModel()->getDynamicsEngine(), this);
	_muscleWrapSet.append(newWrap);

	invalidatePath();
}

//_____________________________________________________________________________
/**
 * Move a wrap instance up in the list.
 *
 * @param aIndex The index of the wrap instance to move up.
 */
void AbstractMuscle::moveUpMuscleWrap(int aIndex)
{
	if (aIndex > 0) {
	   _muscleWrapSet.setMemoryOwner(false); // so wrap is not deleted by remove()
	   MuscleWrap* wrap = _muscleWrapSet.get(aIndex);
	   _muscleWrapSet.remove(aIndex);
	   _muscleWrapSet.insert(aIndex - 1, wrap);
	   _muscleWrapSet.setMemoryOwner(true);
	}

	invalidatePath();
}

//_____________________________________________________________________________
/**
 * Move a wrap instance down in the list.
 *
 * @param aIndex The index of the wrap instance to move down.
 */
void AbstractMuscle::moveDownMuscleWrap(int aIndex)
{
	if (aIndex < _muscleWrapSet.getSize() - 1) {
	   _muscleWrapSet.setMemoryOwner(false); // so wrap is not deleted by remove()
	   MuscleWrap* wrap = _muscleWrapSet.get(aIndex);
	   _muscleWrapSet.remove(aIndex);
	   _muscleWrapSet.insert(aIndex + 1, wrap);
	   _muscleWrapSet.setMemoryOwner(true);
	}

	invalidatePath();
}

//_____________________________________________________________________________
/**
 * Delete a wrap instance.
 *
 * @param aIndex The index of the wrap instance to delete.
 */
void AbstractMuscle::deleteMuscleWrap(int aIndex)
{
	_muscleWrapSet.remove(aIndex);

	invalidatePath();
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
void AbstractMuscle::preScale(const ScaleSet& aScaleSet)
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
void AbstractMuscle::scale(const ScaleSet& aScaleSet)
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
	invalidatePath();
}

//_____________________________________________________________________________
/**
 * Perform computations that need to happen after the muscle is scaled.
 * In most cases, these computations would be performed in the
 * classes descending from AbstractMuscle, where the force-generating
 * properties are stored (except for the recalculation of the path, which
 * is handled here).
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 */
void AbstractMuscle::postScale(const ScaleSet& aScaleSet)
{
	// Recalculate the muscle path. This will also update the geometry.
	// Done here since scale is invoked before bodies are scaled
	// so we may not have enough info to update (e.g. wrapping, via points)
	computePath();
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
void AbstractMuscle::computeActuation()
{
	double posRelative[3], velRelative[3];
	double posStartInertial[3], posEndInertial[3], velStartInertial[3], velEndInertial[3];
	double velStartLocal[3], velEndLocal[3], velStartMoving[3], velEndMoving[3];
	MusclePoint *start, *end;

	computePath();

	_speed = 0.0;

	AbstractDynamicsEngine &engine = _model->getDynamicsEngine();

	int i;
	for (i = 0; i < _currentPath.getSize() - 1; i++) {
		start = _currentPath[i];
		end = _currentPath[i+1];

		// Find the positions and velocities in the inertial frame.
		engine.getPosition(*(start->getBody()), start->getAttachment().get(), posStartInertial);
		engine.getPosition(*(end->getBody()), end->getAttachment().get(), posEndInertial);
		engine.getVelocity(*(start->getBody()), start->getAttachment().get(), velStartInertial);
		engine.getVelocity(*(end->getBody()), end->getAttachment().get(), velEndInertial);

		// The points might be moving in their local bodies' reference frames
		// (MovingMusclePoints and possibly MuscleWrapPoints) so find their
		// local velocities and transform them to the inertial frame.
		start->getVelocity(velStartLocal);
		end->getVelocity(velEndLocal);
		engine.transform(*(start->getBody()), velStartLocal, engine.getGroundBody(), velStartMoving);
		engine.transform(*(end->getBody()), velEndLocal, engine.getGroundBody(), velEndMoving);

		// Calculate the relative positions and velocities.
		for (int j = 0; j < 3; j++) {
			posRelative[j] = posEndInertial[j] - posStartInertial[j];
			velRelative[j] = (velEndInertial[j] + velEndMoving[j]) - (velStartInertial[j] + velStartMoving[j]);
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
// PATH, WRAPPING, AND MOMENT ARM
//=============================================================================
//_____________________________________________________________________________
/**
 * Calculate the current path of the muscle.
 *
 */
void AbstractMuscle::computePath()
{
	if (_pathValid == true)
		return;

	// empty out the current path
	_currentPath.setSize(0);

	// add the fixed and active via points to the path
	int i;
   for (i = 0; i < _attachmentSet.getSize(); i++) {
		_attachmentSet[i]->update();
		if (_attachmentSet[i]->isActive())
			_currentPath.append(_attachmentSet[i]);
	}

   // use the current path so far to check for intersection
	// with wrap objects, which may add additional points to
	// the path
	applyWrapObjects();

	calcLengthAfterPathComputation();

	// Update the geometry used to display the muscle.
	updateGeometry();

	_pathValid = true;
}

//_____________________________________________________________________________
/**
 * Apply the wrap objects to the current muscle path.
 */
void AbstractMuscle::applyWrapObjects()
{
	if (_muscleWrapSet.getSize() < 1)
		return;

   int i, j, kk, pt1, pt2, maxIterations;
   int start, end, wrapStart, wrapEnd;
   double min_length_change, last_length;
   WrapResult best_wrap;
   MusclePoint *smp, *emp;
   MuscleWrap* ws;
   AbstractWrapObject* wo;
   Array<int> result;
	Array<int> order;

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
					if (_currentPath.get(pt1)->getWrapObject() == NULL ||
						 _currentPath.get(pt2)->getWrapObject() == NULL ||
						 (_currentPath.get(pt1)->getWrapObject() != _currentPath.get(pt2)->getWrapObject()))
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

		calcLengthAfterPathComputation(); // length is stored in _length

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

//_____________________________________________________________________________
/**
 * _calc_muscle_length_change - given the output of a successful muscle wrap
 * over a wrap object, determine the percent change in length of the
 * muscle segment incurred by wrapping.
 */
double AbstractMuscle::_calc_muscle_length_change(AbstractWrapObject& wo, WrapResult& wr)
{
	MusclePoint* pt1 = _currentPath.get(wr.startPoint);
	MusclePoint* pt2 = _currentPath.get(wr.endPoint);

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
 * Compute the total length of the muscle (muscle fibers + tendon). This function
 * assumes that the path has already been updated.
 */
void AbstractMuscle::calcLengthAfterPathComputation()
{
	_length = 0.0;

	AbstractDynamicsEngine &engine = _model->getDynamicsEngine();

	int i;

	for (i = 0; i < _currentPath.getSize() - 1; i++) {
		MusclePoint* p1 = _currentPath[i];
		MusclePoint* p2 = _currentPath[i+1];

      /* If both points are wrap points on the same wrap object, then this muscle
       * segment wraps over the surface of a wrap object, so just add in the
       * pre-calculated length.
       */
      if (p1->getWrapObject() && p2->getWrapObject() && p1->getWrapObject() == p2->getWrapObject()) {
			MuscleWrapPoint* smwp = dynamic_cast<MuscleWrapPoint*>(p2);
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
 * Compute the muscle's moment arms for all generalized coordinates.
 *
 * @param rMomentArms Array of moments arms
 */   
void AbstractMuscle::computeMomentArms(Array<double> &rMomentArms)
{
	CoordinateSet *coordSet = _model->getDynamicsEngine().getCoordinateSet();

	int size = coordSet->getSize();
	rMomentArms.setSize(size);
	for(int i=0;i<size;i++) {
		AbstractCoordinate *coord = coordSet->get(i);
		if(coord==NULL) continue;
		rMomentArms[i] = computeMomentArm(*coord);
	}
}
//_____________________________________________________________________________
/**
 * Compute the muscle's moment arm for a coordinate, using the dl/dtheta
 * numerical technique. The length of the muscle is found at coordinate-delta,
 * coordinate, and coordinate+delta. A quadratic is fit to these three points, and
 * the derivative is found at the value of the coordinate. This is equal to the negative of the
 * moment arm.
 * @param aCoord Coordinate for which to compute moment arm.
 * @return Moment arm value.
 */   
double AbstractMuscle::computeMomentArm(AbstractCoordinate& aCoord)
{
   double delta, originalValue, x[3], y[3];
	bool clampedSave, lockedSave;

	computePath();

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

double AbstractMuscle::getShorteningSpeed()
{
	return _speed;
}

//_____________________________________________________________________________
/**
 * Utility function to calculate the current pennation angle in a
 * muscle. Pennation angle increases as muscle fibers shorten. The implicit
 * modeling assumption is that muscles have constant width.
 *
 * @param aFiberLength Current fiber length of muscle.
 * @param aOptimalFiberLength Optimal fiber length of muscle.
 * @param aInitialPennationAngle Pennation angle at optimal fiber length (in radians).
 * @return Current pennation angle (in radians).
 */
double AbstractMuscle::calcPennation(double aFiberLength, double aOptimalFiberLength,
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
void AbstractMuscle::apply()
{
	// NOTE: Force could be negative, in particular during CMC, when the optimizer is computing
	// gradients, it will setForce(+1) and setForce(-1) to compute the derivative with respect to force.
	if (fabs(getForce()) < TINY_NUMBER)
		return;

	AbstractDynamicsEngine &engine = _model->getDynamicsEngine();

	int i;
	MusclePoint* start;
	MusclePoint* end;
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

//_____________________________________________________________________________
/**
 * Update the visible object used to represent the muscle.
 */
void AbstractMuscle::updateDisplayer()
{
	computePath();
}
