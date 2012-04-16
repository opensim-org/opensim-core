#ifndef __SpatialTransform_h__
#define __SpatialTransform_h__

// SpatialTransform.h
// Author: Ajay Seth
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

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/Function.h>
#include "TransformAxis.h"

namespace OpenSim {

class CustomJoint;

//=============================================================================
//=============================================================================
/**
 * A class encapsulating the spatial transformation bewteen two bodies that 
 * defines the behaviour of a custom joint.
 *
 * @authors Ajay Seth
 * @version 1.0
 */

class OSIMSIMULATION_API SpatialTransform :	public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(SpatialTransform, Object);

private:
    static const int _numTransformAxes = 6;

protected:
	/** Define the individual transform axes (6) that specify the spatial transform. */
	PropertyObj _rotation1Prop;
	TransformAxis &_rotation1;

	PropertyObj _rotation2Prop;
	TransformAxis &_rotation2;

	PropertyObj _rotation3Prop;
	TransformAxis &_rotation3;

	PropertyObj _translation1Prop;
	TransformAxis &_translation1;

	PropertyObj _translation2Prop;
	TransformAxis &_translation2;

	PropertyObj _translation3Prop;
	TransformAxis &_translation3;

	CustomJoint *_owningJoint;

private:
	void setNull();
	void setupProperties();
	void constructTransformAxes();

public:
	SpatialTransform();
	SpatialTransform(const SpatialTransform& aSpatialTransform);
	~SpatialTransform(void);

	void copyData(const SpatialTransform &aSpatialTransform);

	// SETUP
	void setup(CustomJoint &aJoint);

	// Spatial Transform specific methods
	virtual OpenSim::Array<std::string> getCoordinateNames();
	virtual	std::vector<std::vector<int> > getCooridinateIndices();
	virtual std::vector<const SimTK::Function*> getFunctions();
	virtual std::vector<SimTK::Vec3> getAxes();

	// SCALE
	virtual void scale(const SimTK::Vec3 scaleFactors);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	TransformAxis& operator[](int aIndex) const;
	SpatialTransform& operator=(const SpatialTransform &aSpatialTransform);
#endif
	TransformAxis& getTransformAxis(int aIndex) const;
private:
	// Make sure axes are not parallel
	void constructIndepndentAxes(int nAxes, int startIndex);
	friend class CustomJoint;
//=============================================================================
};	// END of class SpatialTransform
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SpatialTransform_h__
