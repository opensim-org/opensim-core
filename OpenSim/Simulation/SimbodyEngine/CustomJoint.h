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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include "Joint.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a custom joint.  The underlying joint in Simbody
 * is a custom mobilizer.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMSIMULATION_API CustomJoint : public Joint  
{

//=============================================================================
// DATA
//=============================================================================
protected:

	/** Spatial transform  defining how the child body moves with respect
	to the parent body as a function of the generalized coordinates.
	Motion over 6 (independent) spatial axes must be defined. */
	PropertyObj _spatialTransformProp;
	SpatialTransform &_spatialTransform;

//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION
	CustomJoint();
	
	// Construct joint with supplied coordinates and transdorm axes
	CustomJoint(const std::string &name, Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
			 Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody,
			 SpatialTransform &aSpatialTransform, bool reverse=false);

	// Construct joint with default (empty) coordinates and axes
	CustomJoint(const std::string &name, Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
			 Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody, bool reverse=false);
	CustomJoint(const CustomJoint &aJoint);
	virtual ~CustomJoint();
	virtual Object* copy() const;
	CustomJoint& operator=(const CustomJoint &aJoint);
	void copyData(const CustomJoint &aJoint);


	virtual int numCoordinates() const {return _coordinateSet.getSize();};

	// Transforms
	virtual SpatialTransform& getSpatialTransform() const { return _spatialTransform; }

	// SCALE
	virtual void scale(const ScaleSet& aScaleSet);

	/** Override of the default implementation to account for versioning. */
	virtual void updateFromXMLNode();
	OPENSIM_DECLARE_DERIVED(CustomJoint, Joint);

protected:

	/** Construct coordinates according to the mobilities of the Joint */
	void constructCoordinates();
	virtual void setup(Model& aModel);
	virtual void createSystem(SimTK::MultibodySystem& system) const;

private:
	void setNull();
	void setupProperties();

//=============================================================================
};	// END of class CustomJoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __CustomJoint_h__


