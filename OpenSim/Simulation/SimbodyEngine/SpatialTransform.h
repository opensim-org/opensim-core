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
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
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
    /**
     * Define the six individual transform axes that specify the spatial
     * transform (rotational and translational). Each is a TransformAxis
     * object.
     */
    OpenSim_DECLARE_PROPERTY(rotation1, TransformAxis,
        "The first TransformAxis defining a rotation about an axis with "
        "to a set of generalized coordinates.");
    OpenSim_DECLARE_PROPERTY(rotation2, TransformAxis,
        "The second TransformAxis defining a rotation about an axis with "
        "to a set of generalized coordinates.");
    OpenSim_DECLARE_PROPERTY(rotation3, TransformAxis,
        "The third TransformAxis defining a rotation about an axis with "
        "to a set of generalized coordinates.");
    OpenSim_DECLARE_PROPERTY(translation1, TransformAxis,
        "The first TransformAxis defining a translation along an axis with "
        "to a set of generalized coordinates.");
    OpenSim_DECLARE_PROPERTY(translation2, TransformAxis,
        "The second TransformAxis defining a translation along an axis with "
        "to a set of generalized coordinates.");
    OpenSim_DECLARE_PROPERTY(translation3, TransformAxis,
        "The third TransformAxis defining a translation along an axis with "
        "to a set of generalized coordinates.");

//==============================================================================
// METHODS
//==============================================================================
    /**
     * Default constructor.
     */
    SpatialTransform();

    //** @name Accessors */
    // @{

    /**
     * Get one TransformAxis from an index in [0, 5], where rotation is first
     * (0, 1, 2), followed by translation (3, 4, 5). An exception is thrown if
     * the index is out of range.
     */
    const TransformAxis& getTransformAxis(int whichAxis) const;

    /**
     * Get a writable reference to one TransformAxis from an index in [0, 5],
     * where rotation is first (0, 1, 2), followed by translation (3, 4, 5). An
     * exception is thrown if the index is out of range.
     */
    TransformAxis& updTransformAxis(int whichAxis);

    /**
     * Set the TransformAxis at the specified index [0, 5]. An exception is
     * thrown if the index is out of range.
     */
    void setTransformAxis(int whichAxis, const TransformAxis& axis);

    #ifndef SWIG
    /// @copydoc getTransformAxis()
    const TransformAxis& operator[](int whichAxis) const;

    /// @copydoc updTransformAxis()
    TransformAxis& operator[](int whichAxis);

    /**
     * Construct a list of the coordinate indexes that dictate the motion of
     * each TransformAxis in this SpatialTransform.
     *
     * The outer vector is of size 6 (one entry per TransformAxis) and each
     * inner vector contains the indices of the coordinates (in the Model's
     * CoordinateSet) that affect motion along/about that axis. Therefore,
     * accessing the 3 coordinate of the first TransformAxis would be:
     *
     * @code{.cpp}
     * int coordinateIndex = getCoordinateIndices()[0][2];
     * @endcode
     */
    std::vector<std::vector<int>> getCoordinateIndices() const;
    #endif

    /**
     * Construct a list of all unique coordinate names used by any of the
     * contained TransformAxis objects.
     **/
    Array<std::string> getCoordinateNames() const;

    /**
     * Create a new SimTK::Function corresponding to each axis.
     *
     * @note These are heap allocated; it is up to the caller to delete them.
     */
    std::vector<const SimTK::Function*> getFunctions() const;

    /**
     * Get the axis direction associated with each TransformAxis.
     */
    std::vector<SimTK::Vec3> getAxes() const;

    /// @}

    //** @name Scaling */
    // @{

    /**
     * Scale the spatial transform functions of translations only.
     */
    void scale(const SimTK::Vec3 scaleFactors);

    // @}

    //** @name CustomJoint methods */
    // @{

    /** This tells the SpatialTransform the CustomJoint to which it belongs;
    this is not copied on copy construction or assignment. **/
    void connectToJoint(CustomJoint& owningJoint);


    /** Make sure axes are not parallel. **/
    void constructIndependentAxes(int nAxes, int startIndex);

    // @}

private:
    void constructProperties();

    // There are exactly six TransformAxis objects.
    static const int NumTransformAxes = 6;

};  // class SpatialTransform

} // namespace OpenSim

#endif // OPENSIM_SPATIAL_TRANSFORM_H_
