#ifndef __CustomJoint_h__
#define __CustomJoint_h__
// CustomJoint.h
// Author: Frank C. Anderson, Ajay Seth
/*
 * Copyright (c)  2007, Stanford University. All rights reserved. 
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
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/Transform.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Simulation/Model/TransformAxisSet.h>
#include "Joint.h"
#include "Coordinate.h"

namespace OpenSim {

	class TransformAxis;
//=============================================================================
//=============================================================================
/**
 * A class implementing a custom joint.  The underlying joint in Simbody
 * is a custom mobilizer.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMSIMBODYENGINE_API CustomJoint : public Joint  
{

//=============================================================================
// DATA
//=============================================================================
protected:

	/** Set of transform axes defining how the child body moves with respect
	to the parent body as a function of the generalized coordinates.
	Up to six independent spacial transformations can be defined. */
	PropertyObj _transformAxisSetProp;
	TransformAxisSet &_transformAxisSet;

//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION
	CustomJoint();
	CustomJoint(const CustomJoint &aJoint);
	virtual ~CustomJoint();
	virtual Object* copy() const;
	CustomJoint& operator=(const CustomJoint &aJoint);
	void copyData(const CustomJoint &aJoint);
	void setup(AbstractDynamicsEngine* aEngine);

	// Transforms
	virtual TransformAxisSet* getTransformAxisSet() const { return &_transformAxisSet; }
	virtual const Transform& getForwardTransform();
	virtual const Transform& getInverseTransform();

	/** Connect the body to its parent by this CustomJoint in the underlying Simbody 
	    model. If the parent is not connected (does not have a valid MobilzedBodyIndex) 
		then throw an exception. It is up to the assembly routine to make sure it is
		connecting in a valid sequence - not the joint. */
	virtual void connectBody();

	// SCALE
	virtual void scale(const ScaleSet& aScaleSet);

	OPENSIM_DECLARE_DERIVED(CustomJoint, Joint);

private:
	void setNull();
	void setupProperties();
	void calcTransforms();
	void updateSimbody();
	void appendAxisCoordinateIndicesFunctionsForFunctionBasedMobilizer(CoordinateSet *coordinateSet,
		TransformAxis *transform,
		std::vector<SimTK::Vec3> &axes,
		std::vector<std::vector<int> > &coordinateIndices,
		std::vector<const SimTK::Function<1>*> &functions);

	friend class SimbodyEngine;
	friend class CoordinateCouplerConstraint;



//=============================================================================
};	// END of class CustomJoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __CustomJoint_h__


