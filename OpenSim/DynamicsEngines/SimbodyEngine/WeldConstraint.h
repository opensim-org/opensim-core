#ifndef __WeldConstraint_h__
#define __WeldConstraint_h__

// WeldConstraint.h
// Author: Ajay Seth
/*
 * Copyright (c) 2008, Stanford University. All rights reserved. 
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
#include <string>
#include "osimSimbodyEngineDLL.h"
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Common/PropertyDblVec3.h>
#include <OpenSim/Common/Transform.h>
#include "Constraint.h"
#include "Body.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a Weld Constraint.  The underlying Constraint in Simbody
 * is a Constraint::Weld
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMBODYENGINE_API WeldConstraint : public Constraint  
{

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Specify first of two bodies welded together by the constraint. */
	PropertyStr _body1NameProp;
	std::string& _body1Name;

	/** Specify second of two bodies welded by the constraint. */
	PropertyStr _body2NameProp;
	std::string& _body2Name;

	/** Location of the weld in first body specified in body1 reference frame. */
	PropertyDblVec3 _locationInBody1Prop;
	SimTK::Vec3& _locationInBody1;

	/** Orientation of the weld axes on body1 specified in body1's
	reference frame.  Euler XYZ body-fixed rotation angles are used to express
	the orientation. */
	PropertyDblVec3 _orientationInBody1Prop;
	SimTK::Vec3& _orientationInBody1;

	/** Location of the weld in second body specified in body2 reference frame. */
	PropertyDblVec3 _locationInBody2Prop;
	SimTK::Vec3& _locationInBody2;

	/** Orientation of the weld axes on body2 specified in body2's
	reference frame.  Euler XYZ body-fixed rotation angles are used to express
	the orientation. */
	PropertyDblVec3 _orientationInBody2Prop;
	SimTK::Vec3& _orientationInBody2;

	/** First body weld constraint joins. */
	Body *_body1;

	/** Second body weld constraint joins. */
	Body *_body2;

//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION
	WeldConstraint();
	WeldConstraint(const WeldConstraint &aConstraint);
	virtual ~WeldConstraint();
	virtual Object* copy() const;
	WeldConstraint& operator=(const WeldConstraint &aConstraint);
	void copyData(const WeldConstraint &aConstraint);
	void setup(AbstractDynamicsEngine* aEngine);

	//SET 
	void setBody1ByName(std::string aBodyName);
	void setBody1WeldLocation(SimTK::Vec3 location, SimTK::Vec3 orientation=SimTK::Vec3(0));
	void setBody2ByName(std::string aBodyName);
	void setBody2WeldLocation(SimTK::Vec3 location, SimTK::Vec3 orientation=SimTK::Vec3(0));

private:
	void setNull();
	void setupProperties();
	void updateSimbody();
	friend class SimbodyEngine;

//=============================================================================
};	// END of class WeldConstraint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __WeldConstraint_h__


