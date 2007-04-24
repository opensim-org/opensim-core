#ifndef __AbstractMuscle_h__
#define __AbstractMuscle_h__

// AbstractMuscle.h
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Common/VisibleObject.h>
#include "AbstractActuator.h"
#include "MusclePointSet.h"
#include <OpenSim/Simulation/Wrap/MuscleWrapSet.h>

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {

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
class OSIMSIMULATION_API AbstractMuscle : public AbstractActuator  
{
//=============================================================================
// DATA
//=============================================================================
protected:
	PropertyObj _attachmentSetProp;
	MusclePointSet &_attachmentSet;

	// Support for Display
	PropertyObj _displayerProp;
	VisibleObject &_displayer;

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
	Array<MusclePoint*> _currentPath;

	// is current path valid?
	bool _pathValid;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	AbstractMuscle();
	AbstractMuscle(const AbstractMuscle &aMuscle);
	virtual ~AbstractMuscle();
	virtual Object* copy() const = 0;
	virtual void setup(Model *aModel);
#ifndef SWIG
	AbstractMuscle& operator=(const AbstractMuscle &aMuscle);
#endif
   void copyData(const AbstractMuscle &aMuscle);
	const MusclePointSet& getAttachmentSet() const { return _attachmentSet; }
	const Array<MusclePoint*> getCurrentPath();
	int getMuscleModelIndex() const { return _muscleModelIndex; }
	bool getMuscleModelIndexUseDefault() const { return _muscleModelIndexProp.getUseDefault(); }
	MuscleWrapSet& getWrapSet() { return _muscleWrapSet; }

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	MusclePoint* addAttachmentPoint(int aIndex, AbstractBody& aBody);
	void deleteAttachmentPoint(int aIndex);

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual double getPennationAngle() = 0;
	virtual double getPennationAngleAtOptimalFiberLength() = 0;
	virtual double getLength();
	virtual double getTendonLength();
	virtual double getFiberLength() = 0;
	virtual double getNormalizedFiberLength() = 0;
	virtual double getFiberLengthAlongTendon();
	/*
	virtual double getStrain();
	virtual double getTendonStrain();
	virtual double getFiberStrain();
	virtual double getFiberStrainAlongTendon();
	*/
	virtual double getShorteningSpeed();
	/*
	virtual double getTendonShorteningSpeed();
	virtual double getFiberShorteningSpeed();
	virtual double getFiberShorteningSpeedAlongTendon();
	*/
	virtual double getFiberForce();
	virtual double getActiveFiberForce();
	virtual double getPassiveFiberForce() = 0;
	virtual double getActiveFiberForceAlongTendon();
	virtual double getPassiveFiberForceAlongTendon();
	/*
	virtual double getTendonPower();
	virtual double getMusclePower();
	virtual double getPassiveMusclePower();
	virtual double getActiveMusclePower();
	*/
	//---------
	virtual void computeActuation();
	virtual double computeMomentArm(AbstractCoordinate& aCoord);
	virtual void computeMomentArms(Array<double> &rMomentArms);
	//virtual void computeMoments(Array<double> &rMoments);
	void computePath();
	void applyWrapObjects();
	double _calc_muscle_length_change(AbstractWrapObject& wo, WrapResult& wr);
	virtual void calcLengthAfterPathComputation();
	virtual double calcPennation(double aFiberLength, double aOptimalFiberLength,
		double aInitialPennationAngle) const;
	void invalidatePath() { _pathValid = false; }

	//--------------------------------------------------------------------------
	// SCALING
	//--------------------------------------------------------------------------
	virtual void preScale(const ScaleSet& aScaleSet);
	virtual void scale(const ScaleSet& aScaleSet);
	virtual void postScale(const ScaleSet& aScaleSet);

	//--------------------------------------------------------------------------
	// FORCE APPLICATION
	//--------------------------------------------------------------------------
	virtual void apply();
	virtual void peteTest() const;

	//--------------------------------------------------------------------------
	// Visible Object Support
	//--------------------------------------------------------------------------
	// Update the geometry attached to the muscle (location of muscle points and connecting segments
	//  all in global/interial frame)
	virtual void updateGeometry();
	virtual VisibleObject* getDisplayer() const { return &_displayer; };

	OPENSIM_DECLARE_DERIVED(AbstractMuscle, AbstractActuator);

private:
	void setNull();
	void setupProperties();
	void updateGeometrySize();
	void updateGeometryLocations();
	void nameAttachmentPoints(int aStartingIndex);

//=============================================================================
};	// END of class AbstractMuscle
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __AbstractMuscle_h__


