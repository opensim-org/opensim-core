#ifndef __AbstractMuscle_h__
#define __AbstractMuscle_h__

// AbstractMuscle.h
// Author: Peter Loan, Frank C. Anderson
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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
class AbstractWrapObject;

//=============================================================================
//=============================================================================
/**
 * A base class representing a muscle-tendon actuator. It adds data and methods
 * to AbstractActuator, but does not implement all of the necessary methods,
 * so it is abstract too. The path information for a muscle is contained
 * in this class, and the force-generating behavior should be defined in
 * the derived classes.
 *
 * @author Peter Loan
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMSIMULATION_API AbstractMuscle : public AbstractActuator  
{
//=============================================================================
// DATA
//=============================================================================
protected:
   // the set of attachment points defining the path of the muscle
	PropertyObj _attachmentSetProp;
	MusclePointSet &_attachmentSet;

	// used to display the muscle in the 3D window
	PropertyObj _displayerProp;
	VisibleObject &_displayer;

   // the wrap objects that are associated with this muscle
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
	void setName(const std::string &aName);
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
	// PATH GEOMETRY
	//--------------------------------------------------------------------------
	MusclePoint* addAttachmentPoint(int aIndex, AbstractBody& aBody);
	bool canDeleteAttachmentPoint(int aIndex);
	bool deleteAttachmentPoint(int aIndex);
	void addMuscleWrap(AbstractWrapObject& aWrapObject);
	void moveUpMuscleWrap(int aIndex);
	void moveDownMuscleWrap(int aIndex);
	void deleteMuscleWrap(int aIndex);

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
	virtual double getShorteningSpeed();
	virtual double getFiberForce();
	virtual double getActiveFiberForce();
	virtual double getPassiveFiberForce() = 0;
	virtual double getActiveFiberForceAlongTendon();
	virtual double getPassiveFiberForceAlongTendon();
	virtual double getMaxIsometricForce();
	virtual double getActivation() const = 0;

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void computeActuation();
	virtual double computeIsometricForce(double activation) = 0;
	virtual double computeIsokineticForceAssumingInfinitelyStiffTendon(double aActivation) = 0;
	virtual double computeMomentArm(AbstractCoordinate& aCoord);
	virtual void computeMomentArms(Array<double> &rMomentArms);
	virtual double
		evaluateForceLengthVelocityCurve(double aActivation, double aNormalizedLength, double aNormalizedVelocity);
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

	//--------------------------------------------------------------------------
	// Visible Object Support
	//--------------------------------------------------------------------------
	virtual VisibleObject* getDisplayer() const { return &_displayer; }
	virtual void updateDisplayer();

	OPENSIM_DECLARE_DERIVED(AbstractMuscle, AbstractActuator);

private:
	void setNull();
	void setupProperties();
	void updateGeometrySize();
	void updateGeometryLocations();
	void nameAttachmentPoints(int aStartingIndex);
   void placeNewAttachment(SimTK::Vec3& aOffset, int aIndex, AbstractBody& aBody);

protected:
	// Update the geometry attached to the muscle (location of muscle points and connecting segments
	//  all in global/interial frame)
	virtual void updateGeometry();

//=============================================================================
};	// END of class AbstractMuscle
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __AbstractMuscle_h__


