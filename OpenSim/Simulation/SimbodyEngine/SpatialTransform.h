#ifndef OPENSIM_SPATIAL_TRANSFORM_H_
#define OPENSIM_SPATIAL_TRANSFORM_H_

// SpatialTransform.h
// Author: Ajay Seth
/*
 * Copyright (c)  2006-12, Stanford University. All rights reserved. 
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

//==============================================================================
//                           SPATIAL TRANSFORM
//==============================================================================
/**
 * A class encapsulating the spatial transformation bewteen two bodies that 
 * defines the behaviour of a custom joint.
 *
 * @authors Ajay Seth
 */

class OSIMSIMULATION_API SpatialTransform :	public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(SpatialTransform, Object);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
	/** Define the individual transform axes (6) that specify the spatial 
    transform; each is a TransformAxis object. **/
    OpenSim_DECLARE_PROPERTY(rotation1, TransformAxis,
		"3 Axes for rotations are listed first.");
	OpenSim_DECLARE_PROPERTY(rotation2, TransformAxis,
		"");
	OpenSim_DECLARE_PROPERTY(rotation3, TransformAxis,
		"");
	OpenSim_DECLARE_PROPERTY(translation1, TransformAxis,
		"3 Axes for translations are listed next.");
	OpenSim_DECLARE_PROPERTY(translation2, TransformAxis,
		"");
	OpenSim_DECLARE_PROPERTY(translation3, TransformAxis,
		"");
    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    SpatialTransform();

    // default destructor, copy constructor, copy assignment

	/** This tells the SpatialTransform the CustomJoint to which it belongs;
    this is not copied on copy construction or assignment. **/
	void setup(CustomJoint& owningJoint);

	/** Make sure axes are not parallel. **/
	void constructIndependentAxes(int nAxes, int startIndex);

	// Spatial Transform specific methods

    /** Construct a list of all unique coordinate names used by any of the
    contained TransformAxis objects. **/
	OpenSim::Array<std::string> getCoordinateNames() const;
    /** For each axis, construct a list of the coordinate indices that dictate
    motion along that axis. **/
	std::vector<std::vector<int> > getCoordinateIndices() const;
    /** Create a new SimTK::Function corresponding to each axis; these are
    heap allocated and it is up to the caller to delete them. **/
	std::vector<const SimTK::Function*> getFunctions() const;
    /** Get the axis direction associated with each TransformAxis. **/
	std::vector<SimTK::Vec3> getAxes() const;

	// SCALE
	void scale(const SimTK::Vec3 scaleFactors);

    /** Select one of the 6 axis, numbered 0-5 with rotation first, then
    translation. **/
	const TransformAxis& getTransformAxis(int whichAxis) const;
    /** Same, but returns a writable reference to the TransformAxis. **/
	TransformAxis& updTransformAxis(int whichAxis);

    #ifndef SWIG
    /** Same as getTransformAxis(). **/
	const TransformAxis& operator[](int whichAxis) const
    {   return getTransformAxis(whichAxis); }
    /** Same as updTransformAxis(). **/
	TransformAxis& operator[](int whichAxis) 
    {   return updTransformAxis(whichAxis); }
    #endif

private:
	void setNull();
	void constructProperties();

    static const int NumTransformAxes = 6;

//==============================================================================
};	// END of class SpatialTransform
//==============================================================================
//==============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_SPATIAL_TRANSFORM_H_
