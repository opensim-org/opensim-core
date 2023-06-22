#ifndef OPENSIM_CONSTANT_CURVATURE_JOINT_H_
#define OPENSIM_CONSTANT_CURVATURE_JOINT_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  ConstantCurveJoint.h                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2023 Stanford University and the Authors                     *
 * Author(s): Keenon Werling                                                  *
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

// INCLUDE
#include "Joint.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**

A class implementing a ConstantCurvatureJoint joint. A ConstantCurvatureJoint
connects two bodies by a line segment of a fixed length. The endpoint of the
ConstantCurvatureJoint can be rotated by euler angles, and the offset is computed
as a function of the euler angles and the fixed length of the line segment.

This joint was originally designed as a lightweight way to model spine segments,
which can be approximated without individual link segments be instead using 3 of 
these joints in series.

@author Keenon Werling
*/
class OSIMSIMULATION_API ConstantCurvatureJoint : public Joint {
    OpenSim_DECLARE_CONCRETE_OBJECT(ConstantCurvatureJoint, Joint);

public:
    /** Indices of Coordinates for use as arguments to getCoordinate() and
        updCoordinate().

        <b>C++ example</b>
        \code{.cpp}
        const auto& rx = myConstantCurvatureJoint.
                         getCoordinate(ConstantCurvatureJoint::Coord::Rotation1X);
        \endcode

        <b>Python example</b>
        \code{.py}
        import opensim
        rx =
       myConstantCurvatureJoint.getCoordinate(opensim.ConstantCurvatureJoint.Coord_Rotation1X)
        \endcode

        <b>Java example</b>
        \code{.java}
        rx =
       myConstantCurvatureJoint.getCoordinate(ConstantCurvatureJoint.Coord.Rotation1X);
        \endcode

        <b>MATLAB example</b>
        \code{.m}
        rx = myConstantCurvatureJoint.get_coordinates(0);
        \endcode

        IMPORTANT WARNING!!!
        The joint has an **X-Z-Y** rotation ordering. We use this ordering because that means 
        the first two DOFs are rotation and translation, while the last DOF is merely a 
        "twist" of the segment without any translation. This is also for compatibility with
        the original implementation in Nimble Physics.
    */
    enum class Coord : unsigned {
        RotationX = 0u, ///< 0
        RotationZ = 1u, ///< 1
        RotationY = 2u, ///< 2
    };

private:
    /** Specify the Coordinates of the ConstantCurvatureJoint */
    CoordinateIndex rx{constructCoordinate(Coordinate::MotionType::Rotational,
            static_cast<unsigned>(Coord::RotationX))};
    CoordinateIndex rz{constructCoordinate(Coordinate::MotionType::Rotational,
            static_cast<unsigned>(Coord::RotationZ))};
    CoordinateIndex ry{constructCoordinate(Coordinate::MotionType::Rotational,
            static_cast<unsigned>(Coord::RotationY))};

public:
    //==============================================================================
    // PROPERTIES
    //==============================================================================
    OpenSim_DECLARE_PROPERTY(neutral_angle_x_z_y, SimTK::Vec3,
            "The neutral angle of the endpoint as a Vec3(rX, rY, rZ).");

    OpenSim_DECLARE_PROPERTY(length, double, "Length of line segment.");

    //=============================================================================
    // METHODS
    //=============================================================================
    // CONSTRUCTION
    ConstantCurvatureJoint();
    /** Convenience Joint like Constructor */
    ConstantCurvatureJoint(const std::string& name, const PhysicalFrame& parent,
            const PhysicalFrame& child, const SimTK::Vec3& neutralAngleXZY,
            const double length);

    /** Deprecated Joint Constructor
        NOTE(keenon): This constructor seems necessary to compile, but it has a 
        comment marking it deprecated in the EllipsoidJoint, so I copied that comment 
        over here as well. */
    ConstantCurvatureJoint(const std::string& name, const PhysicalFrame& parent,
            const SimTK::Vec3& locationInParent,
            const SimTK::Vec3& orientationInParent, const PhysicalFrame& child,
            const SimTK::Vec3& locationInChild,
            const SimTK::Vec3& orientationInChild,
            const SimTK::Vec3& neutralAngleXZY, const double length);

    // Set properties
    void setNeutralAngleXZY(const SimTK::Vec3& neutralAngleXZY);
    void setLength(const double length);

    /** Exposes getCoordinate() method defined in base class (overloaded below).
        @see Joint */
    using Joint::getCoordinate;

    /** Exposes updCoordinate() method defined in base class (overloaded below).
        @see Joint */
    using Joint::updCoordinate;

    /** Get a const reference to a Coordinate associated with this Joint.
        @see Coord */
    const Coordinate& getCoordinate(Coord idx) const {
        return get_coordinates(static_cast<unsigned>(idx));
    }

    /** Get a writable reference to a Coordinate associated with this Joint.
        @see Coord */
    Coordinate& updCoordinate(Coord idx) {
        return upd_coordinates(static_cast<unsigned>(idx));
    }

    // SCALE
    void extendScale(const SimTK::State& s, const ScaleSet& scaleSet) override;

    ////////////////////////////////////
    // Public, static math utility functions. These are public to faccilitate testing.
    ////////////////////////////////////

    /** This method will clamp an input set of joint angles q to the limits of the joint, and return the clamped vector */
    static SimTK::Vec3 clamp(const SimTK::Vec3& q);
    /** This method will convert a vector of X,Z,Y rotations into the corresponding SO3 rotation matrix */
    static SimTK::Rotation eulerXZYToMatrix(const SimTK::Vec3& _angle);
    /** This method will convert an SO3 rotation matrix into a corresponding vector of X,Z,Y rotations */
    static SimTK::Mat33 eulerXZYToMatrixGrad(const SimTK::Vec3& _angle, int index);
    /** This method will return the Jacobian of a pure Euler joint (following the XZY convention), where each 
        column gives the derivative of the spatial (SE3) coordinates for the joint transform wrt one degree of 
        freedom of the joint (so there are 3, and each is of dimension 6) */
    static SimTK::Mat63 getEulerJacobian(const SimTK::Vec3& q);
    /** This method will return the derivative of the matrix returned by getEulerJacobian, with respect to changes 
        to the `index` DOF of the joint. This is the same shape as the original matrix, because we take the 
        derivative of every entry of the matrix separately */
    static SimTK::Mat63 getEulerJacobianDerivWrtPos(const SimTK::Vec3& q, int index);
    /** This is much like getEulerJacobian(), because the rotational component is exactly the same, but the 
        translational component is now non-zero. This takes as input the length of the line segment, `d`. */
    static SimTK::Mat63 getConstantCurveJacobian(const SimTK::Vec3& pos, double d);
    /** This method will return the derivative of the matrix returned by getConstantCurveJacobian, with respect 
        to changes to the `index` DOF of the joint. This is the same shape as the original matrix, because we 
        take the derivative of every entry of the matrix separately */
    static SimTK::Mat63 getConstantCurveJacobianDerivWrtPosition(const SimTK::Vec3& pos, double d, int index);
    /** This method will return the derivative of the matrix returned by getConstantCurveJacobian, with respect 
        to time (changes to every element in `pos` at rate `dPos`). This is the same shape as the original matrix, 
        because we take the derivative of every entry of the matrix separately */
    static SimTK::Mat63 getConstantCurveJacobianDerivWrtTime(const SimTK::Vec3& pos, const SimTK::Vec3& dPos, double d);
    /** This computes a transform for a given DOF position (XZY euler rotation) and line segment length */
    static SimTK::Transform getTransform(SimTK::Vec3 pos, double d);

protected:
    // ModelComponent interface.
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendInitStateFromProperties(SimTK::State& s) const override;
    void extendSetPropertiesFromState(const SimTK::State& state) override;

    // Visual support in SimTK visualizer
    void generateDecorations(bool fixed, const ModelDisplayHints& hints,
            const SimTK::State& state,
            SimTK::Array_<SimTK::DecorativeGeometry>& geometryArray)
            const override;

private:
    void constructProperties();

    //=============================================================================
}; // END of class ConstantCurvatureJoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_CONSTANT_CURVATURE_JOINT_H_
