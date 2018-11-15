#ifndef OPENSIM_SPATIAL_TRANSFORM_H_
#define OPENSIM_SPATIAL_TRANSFORM_H_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  SpatialTransform.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "TransformAxis.h"

namespace OpenSim {

class TransformAxis;
class CustomJoint;

//==============================================================================
//                           SPATIAL TRANSFORM
//==============================================================================
/**
 * A class encapsulating the spatial transformation between two bodies that 
 * defines the behavior of a custom joint.
 *
 * @authors Ajay Seth
 */

class OSIMSIMULATION_API SpatialTransform : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(SpatialTransform, Object);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
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

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    SpatialTransform();

    // default destructor, copy constructor, copy assignment

    /** This tells the SpatialTransform the CustomJoint to which it belongs;
    this is not copied on copy construction or assignment. **/
    void connectToJoint(CustomJoint& owningJoint);

    /** Make sure axes are not parallel. **/
    void constructIndependentAxes(int nAxes, int startIndex);

    // Spatial Transform specific methods

    /** Construct a list of all unique coordinate names used by any of the
    contained TransformAxis objects. **/
    OpenSim::Array<std::string> getCoordinateNames() const;
    /** For each axis, construct a list of the coordinate indices that dictate
    motion along that axis. **/
#ifndef SWIG
    std::vector<std::vector<int> > getCoordinateIndices() const;
#endif
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
};  // END of class SpatialTransform
//==============================================================================
//==============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_SPATIAL_TRANSFORM_H_
