#ifndef __AbstractSimmMuscle_h__
#define __AbstractSimmMuscle_h__

// AbstractSimmMuscle.h
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


// INCLUDE
#include <iostream>
#include <math.h>
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/PropertyInt.h>
#include <OpenSim/Tools/PropertyObj.h>
#include <OpenSim/Tools/ScaleSet.h>
#include <OpenSim/Tools/VisibleObject.h>
#include "AbstractActuator.h"
#include "SimmMusclePointSet.h"
#include "MuscleWrapSet.h"

#ifdef SWIG
	#ifdef RDSIMULATION_API
		#undef RDSIMULATION_API
		#define RDSIMULATION_API
	#endif
#endif

namespace OpenSim {

class SimmMuscleGroup;
class AbstractCoordinate;
class WrapResult;

//=============================================================================
//=============================================================================
/**
 * A base class representing a SIMM muscle. It adds data and methods to
 * AbstractActuator, but does not implement all of the necessary methods,
 * so it is abstract too.
 *
 * @author Peter Loan
 * @version 1.0
 */
class RDSIMULATION_API AbstractSimmMuscle : public AbstractActuator  
{
//=============================================================================
// DATA
//=============================================================================
protected:
	PropertyObj _attachmentSetProp;
	SimmMusclePointSet &_attachmentSet;

	// Support for Display
	PropertyObj _displayerProp;
	VisibleObject &_displayer;

	PropertyStrArray _groupNamesProp;
	Array<std::string> &_groupNames;
	Array<SimmMuscleGroup*> _groups;

	PropertyObj _muscleWrapSetProp;
	MuscleWrapSet &_muscleWrapSet;

	// muscle model index is not used in OpenSim, but is
	// stored here so that it can be written to a SIMM
	// muscle file if needed.
	PropertyInt _muscleModelIndexProp;
	int &_muscleModelIndex;

	// current length of muscle-tendon actuator
	double _length;

	// used for scaling tendon and fiber lengths
	double _preScaleLength;

	// current path = ordered array of currently active attachment points (fixed + via + wrap)
	Array<SimmMusclePoint*> _currentPath;

	// is current path valid?
	bool _pathValid;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	AbstractSimmMuscle();
	AbstractSimmMuscle(const AbstractSimmMuscle &aMuscle);
	virtual ~AbstractSimmMuscle();
	virtual Object* copy() const = 0;
	virtual void setup(AbstractModel *aModel);

	static AbstractSimmMuscle* safeDownCast(Object* aObject) { return dynamic_cast<AbstractSimmMuscle*>(aObject); }

#ifndef SWIG
	AbstractSimmMuscle& operator=(const AbstractSimmMuscle &aMuscle);
#endif
   void copyData(const AbstractSimmMuscle &aMuscle);
	const SimmMusclePointSet& getAttachmentSet() const { return _attachmentSet; }
	int getMuscleModelIndex() const { return _muscleModelIndex; }
	bool getMuscleModelIndexUseDefault() const { return _muscleModelIndexProp.getUseDefault(); }

	const Array<std::string>* getGroupNames() const { return &_groupNames; }

	// SCALING
	virtual void preScale(const ScaleSet& aScaleSet);
	virtual void scale(const ScaleSet& aScaleSet);
	virtual void postScale(const ScaleSet& aScaleSet);

	//=============================================================================
	// COMPUTATIONS
	//=============================================================================
	void calculatePath();
	void applyWrapObjects();
	double _calc_muscle_length_change(AbstractWrapObject& wo, WrapResult& wr);
	void calculateLength();
	double getLength();
	virtual double getMomentArm(AbstractCoordinate& aCoord);
	virtual double getSpeed() const;
	double calcPennation(double aFiberLength, double aOptimalFiberLength,
		double aInitialPennationAngle) const;
	virtual void computeActuation();
	void invalidatePath() { _pathValid = false; }

	//=============================================================================
	// FORCE APPLICATION
	//=============================================================================
	virtual void apply();

	virtual void peteTest() const;

	// Visible Object Support
	virtual VisibleObject* getDisplayer() const { return &_displayer; };
	// Update the geometry attached to the muscle (location of muscle points and connecting segments
	//  all in global/interial frame)
	virtual void updateGeometry();

	/* Register types to be used when reading an AbstractSimmMuscle object from xml file. */
	static void registerTypes();

private:
	void setNull();
	void setupProperties();
	void updateGeometrySize();
	void updateGeometryLocations();
//=============================================================================
};	// END of class AbstractSimmMuscle
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __AbstractSimmMuscle_h__


