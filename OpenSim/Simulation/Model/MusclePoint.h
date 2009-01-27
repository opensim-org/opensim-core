#ifndef __MusclePoint_h__
#define __MusclePoint_h__

// MusclePoint.h
// Author: Peter Loan
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
#include <assert.h>
#include <iostream>
#include <string>
#include <math.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/Geometry.h>
#include <OpenSim/Common/VisibleObject.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyDblVec3.h>
#include <OpenSim/Common/PropertyStr.h>

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {

class AbstractBody;
class Model;
class AbstractMuscle;
class AbstractDynamicsEngine;
class AbstractWrapObject;

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM muscle point.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API MusclePoint : public Object  
{

//=============================================================================
// DATA
//=============================================================================
private:

protected:
   PropertyDblVec3 _attachmentProp;
   SimTK::Vec3 &_attachment;

	PropertyStr _bodyNameProp;
   std::string &_bodyName;

	// Support for Display
	PropertyObj _displayerProp;
	VisibleObject &_displayer;

	/* const*/ AbstractBody *_body; // Not const anymore since the body's displayer is not const

	AbstractMuscle* _muscle; // the muscle that owns this attachment point

	/** A temporary kluge until the default mechanism is working */
	static Geometry *_defaultGeometry;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	MusclePoint();
	MusclePoint(const MusclePoint &aPoint);
	virtual ~MusclePoint();
	virtual Object* copy() const;

#ifndef SWIG
	MusclePoint& operator=(const MusclePoint &aPoint);
#endif
   void copyData(const MusclePoint &aPoint);
	virtual void init(const MusclePoint& aPoint);

	const SimTK::Vec3& getAttachment() const { return _attachment; }
	SimTK::Vec3& getAttachment()  { return _attachment; }

	const double& getAttachmentCoord(int aXYZ) const { assert(aXYZ>=0 && aXYZ<=2); return _attachment[aXYZ]; }
	void setAttachmentCoord(int aXYZ, double aValue) { assert(aXYZ>=0 && aXYZ<=2); _attachment[aXYZ]=aValue; }
	// A variant that uses basic types for use by GUI
	void setAttachment(SimTK::Vec3& aAttachment);
   void setAttachment(int aCoordIndex, double aAttachment);
	void setAttachment(double pt[]){ // A variant that uses basic types for use by GUI
		setAttachment(SimTK::Vec3::updAs(pt));
	}
	AbstractBody* getBody() const { return _body; }
	void setBody(AbstractBody& aBody, bool preserveLocation = false);
	const std::string& getBodyName() const { return _bodyName; }
	AbstractMuscle* getMuscle() const { return _muscle; }


	virtual	void scale(const SimTK::Vec3& aScaleFactors);
	virtual bool isActive() const { return true; }
	virtual AbstractWrapObject* getWrapObject() const { return NULL; }
	virtual void setup(Model* aModel, AbstractMuscle* aMuscle);
	virtual void update() { }
	virtual void getVelocity(SimTK::Vec3& aVelocity);

	// Visible Object Support
	virtual VisibleObject* getDisplayer() const { return &_displayer; };
	virtual void updateGeometry();

	// Utility
	static MusclePoint* makeMusclePointOfType(MusclePoint* aPoint, const std::string& aNewTypeName);
	static void deleteMusclePoint(MusclePoint* aPoint) { if (aPoint) delete aPoint; }

	OPENSIM_DECLARE_DERIVED(MusclePoint, Object);
protected:

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class MusclePoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __MusclePoint_h__


