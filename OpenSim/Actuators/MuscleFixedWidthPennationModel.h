#ifndef OPENSIM_MUSCLEFIXEDWIDTHPENNATIONMODEL_H_
#define OPENSIM_MUSCLEFIXEDWIDTHPENNATIONMODEL_H_
/* -------------------------------------------------------------------------- *
 *                 OpenSim:  MuscleFixedWidthPennationModel.h                 *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Matthew Millard                                                 *
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
#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {
/** This is a muscle modeling utility class containing kinematic equations that
    describe the deformation of muscle fibers as they change length using a
    fixed-width-parallelogram pennation model. This pennation model makes
    several assumptions:
    \li Fibers are straight, parallel, of equal length, and coplanar.
    \li The area and height of the parallelogram remains constant.

    The parallelogram maintains a constant area and height by shearing as the
    muscle fibers change length, as shown in the figure below. The constant-area
    assumption is intended to mimic the constant-volume property of
    incompressible biological muscle. For details, please refer to Zajac (1989)
    and Millard et al. (2013).

    @param optimalFiberLength
        The optimal length of the muscle fibers (meters).
    @param optimalPennationAngle
        The angle between the tendon and fibers at optimal fiber length
        (radians).
    @param maximumPennationAngle
        The maximum pennation angle permitted (radians). This parameter is
        particularly useful for avoiding a pennation angle singularity at Pi/2
        radians.

    \image html fig_MuscleFixedWidthPennationModel.png

    <B>Conditions</B>
    \verbatim
    optimalFiberLength > 0
    0 <= optimalPennationAngle < Pi/2
    0 <= maximumPennationAngle <= Pi/2
    \endverbatim

    <B>Default Parameter Values</B>
    \verbatim
    optimalFiberLength ....... 0.1
    optimalPennationAngle .... 0.0
    maximumPennationAngle .... acos(0.1) = 84.3 degrees
    \endverbatim

    <B>Example</B>
    \code
    double optFibLen = 0.1;
    double optPenAng = SimTK::Pi/4.0;
    double maxPenAng = acos(0.001);
    MuscleFixedWidthPennationModel fibKin = MuscleFixedWidthPennationModel(optFibLen, optPenAng, maxPenAng);
    \endcode

    Note that this object should be updated through the set methods provided.
    These set methods will take care of rebuilding the object correctly. If you
    modify the properties directly, the object will not be rebuilt, and upon
    calling any function, an exception will be thrown because the object is
    out-of-date with its properties.

    <B>References</B>
    \li Zajac, F.E. (1989) %Muscle and tendon: properties, models, scaling, and
        application to biomechanics and motor control. Critical Reviews in
        Biomedical Engineering 17(4):359--411.
    \li Millard, M., Uchida, T., Seth, A., Delp, S.L. (2013) Flexing
        computational muscle: modeling and simulation of musculotendon dynamics.
        ASME Journal of Biomechanical Engineering 135(2):021005.
        http://dx.doi.org/10.1115/1.4023390.

    @author Matt Millard
*/
class OSIMACTUATORS_API MuscleFixedWidthPennationModel : public ModelComponent{
OpenSim_DECLARE_CONCRETE_OBJECT(MuscleFixedWidthPennationModel, ModelComponent);

public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(optimal_fiber_length, double,
        "Optimal length of the muscle fibers, in meters (overridden when this is a subcomponent of a Muscle)");
    OpenSim_DECLARE_PROPERTY(pennation_angle_at_optimal, double,
        "Angle between tendon and fibers at optimal fiber length, in radians (overridden when this is a subcomponent of a Muscle)");
    OpenSim_DECLARE_PROPERTY(maximum_pennation_angle, double,
        "Maximum pennation angle, in radians (overridden when this is a subcomponent of a Muscle)");

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** The default constructor creates a fixed-width-parallelogram pennation
    model with the default property values. */
    MuscleFixedWidthPennationModel();

    /** Creates a fixed-width-parallelogram pennation model using the provided
    parameters. */
    MuscleFixedWidthPennationModel(double optimalFiberLength,
                                   double optimalPennationAngle,
                                   double maximumPennationAngle);

    /** @returns The height of the fixed-width parallelogram. */
    double getParallelogramHeight() const;

    /** @returns The minimum possible fiber length. */
    double getMinimumFiberLength() const;

    /** @returns The minimum possible fiber length along the tendon. */
    double getMinimumFiberLengthAlongTendon() const;

    /** Calculates the pennation angle (the orientation of the parallelogram)
    given the fiber length. */
    double calcPennationAngle(double fiberLength) const;

    /** Calculates the length of the tendon given the cosine of the pennation
    angle, the length of the fiber, and the length of the entire musculotendon
    actuator. */
    double calcTendonLength(double cosPennationAngle,
                            double fiberLength,
                            double muscleLength) const;

    /** Calculates the length of the fiber projected onto the axis of the
    tendon. */
    double calcFiberLengthAlongTendon(double fiberLength,
                                      double cosPennationAngle) const;

    /** Calculates the angular velocity of the parallelogram (i.e., the time
    derivative of the pennation angle.
    @param tanPennationAngle
        The tangent of the pennation angle.
    @param fiberLength
        The length of the fiber (m).
    @param fiberVelocity
        The lengthening velocity of the fiber (m/s).
    @returns
        The angular velocity of the parallelogram (rad/s).
    */
    double calcPennationAngularVelocity(double tanPennationAngle,
                                        double fiberLength,
                                        double fiberVelocity) const;

    /**
    @param cosPennationAngle
        The cosine of the pennation angle.
    @param sinPennationAngle
        The sine of the pennation angle.
    @param pennationAngularVelocity
        The angular velocity of the parallelogram (rad/s).
    @param fiberLength
        The length of the fiber (m).
    @param fiberVelocity
        The lengthening velocity of the fiber (m/s).
    @param muscleVelocity
        The lengthening velocity of the muscle path (m/s).
    @returns
        The lengthening velocity of the tendon (m/s).
    */
    double calcTendonVelocity(double cosPennationAngle,
                              double sinPennationAngle,
                              double pennationAngularVelocity,
                              double fiberLength,
                              double fiberVelocity,
                              double muscleVelocity) const;

    /**
    @param fiberLength
        The length of the fiber (m).
    @param fiberVelocity
        The lengthening velocity of the fiber (m/s).
    @param sinPennationAngle
        The sine of the pennation angle.
    @param cosPennationAngle
        The cosine of the pennation angle.
    @param pennationAngularVelocity
        The angular velocity of the parallelogram (rad/s).
    @returns
        The lengthening velocity of the fiber projected onto the axis of the
        tendon (m/s).
    */
    double calcFiberVelocityAlongTendon(
                                    double fiberLength,
                                    double fiberVelocity,
                                    double sinPennationAngle,
                                    double cosPennationAngle,
                                    double pennationAngularVelocity) const;

    /**
    @param fiberLength
        The length of the fiber (m).
    @param fiberVelocity
        The lengthening velocity of the fiber (m/s).
    @param fiberAcceleration
        The lengthening acceleration of the fiber (m/s^2).
    @param sinPennationAngle
        The sine of the pennation angle.
    @param cosPennationAngle
        The cosine of the pennation angle.
    @param pennationAngularVelocity
        The angular velocity of the parallelogram (rad/s).
    @returns
        The angular acceleration of the parallelogram (rad/s^2).
    */
    double calcPennationAngularAcceleration(
                                    double fiberLength,
                                    double fiberVelocity,
                                    double fiberAcceleration,
                                    double sinPennationAngle,
                                    double cosPennationAngle,
                                    double pennationAngularVelocity) const;

    /**
    @param fiberLength
        The length of the fiber (m).
    @param fiberVelocity
        The lengthening velocity of the fiber (m/s).
    @param fiberAcceleration
        The lengthening acceleration of the fiber (m/s^2).
    @param sinPennationAngle
        The sine of the pennation angle.
    @param cosPennationAngle
        The cosine of the pennation angle.
    @param pennationAngularVelocity
        The angular velocity of the parallelogram (rad/s).
    @param pennationAngularAcceleration
        The angular acceleration of the parallelogram (rad/s^2).
    @returns
        The acceleration of the fiber projected onto the axis of the tendon
        (m/s^2).
    */
    double calcFiberAccelerationAlongTendon(
                                    double fiberLength,
                                    double fiberVelocity,
                                    double fiberAcceleration,
                                    double sinPennationAngle,
                                    double cosPennationAngle,
                                    double pennationAngularVelocity,
                                    double pennationAngularAcceleration) const;

    /** Calculates the partial derivative of the fiber length along the tendon
    with respect to the fiber length. */
    double calc_DFiberLengthAlongTendon_DfiberLength(
                                    double fiberLength,
                                    double sinPennationAngle,
                                    double cosPennationAngle,
                                    double DpennationAngle_DfiberLength) const;

    /** Calculates the partial derivative of the pennation angular velocity with
    respect to the fiber length. */
    double calc_DPennationAngularVelocity_DfiberLength(
                                    double fiberLength,
                                    double fiberVelocity,
                                    double sinPennationAngle,
                                    double cosPennationAngle,
                                    double pennationAngularVelocity,
                                    double DpennationAngle_DfiberLength) const;

    /** Calculates the partial derivative of the fiber velocity along the tendon
    with respect to the fiber length. */
    double calc_DFiberVelocityAlongTendon_DfiberLength(
                        double fiberLength,
                        double fiberVelocity,
                        double sinPennationAngle,
                        double cosPennationAngle,
                        double pennationAngularVelocity,
                        double DpennationAngle_DfiberLength,
                        double DpennationAngularVelocity_DfiberLength) const;

    /** Calculates the partial derivative of the pennation angle with respect to
    the fiber length. */
    double calc_DPennationAngle_DfiberLength(double fiberLength) const;

    /** Calculates the partial derivative of the tendon length with respect to
    the fiber length. */
    double calc_DTendonLength_DfiberLength(double fiberLength,
                                           double sinPennationAngle,
                                           double cosPennationAngle,
                                           double DpennationAngle_DfiberLength)
                                           const;

    /**
    @param muscleLength
        The length of the musculotendon actuator (m).
    @param tendonLength
        The length of the tendon (m).
    @return
        The length of the fiber (m).
    */
    double calcFiberLength(double muscleLength,
                           double tendonLength) const;

    /**
    @param cosPennationAngle
        The cosine of the pennation angle.
    @param muscleVelocity
        The lengthening velocity of the musculotendon actuator (m/s).
    @param tendonVelocity
        The lengthening velocity of the tendon (m/s).
    @return
        The lengthening velocity of the fiber (m/s).
    */
    double calcFiberVelocity(double cosPennationAngle,
                             double muscleVelocity,
                             double tendonVelocity) const;

protected:
    // Component interface.
    void extendFinalizeFromProperties() override;

private:
    void setNull();
    void constructProperties();

    double m_parallelogramHeight;
    double m_maximumSinPennation;
    double m_minimumFiberLength;
    double m_minimumFiberLengthAlongTendon;

    // Enforces a lower bound on the fiber length to avoid a numerical
    // singularity as the fiber length approaches zero.
    double clampFiberLength(double fiberLength) const;

    // These classes are friends because they call clampFiberLength().
    friend class Thelen2003Muscle;
    friend class Millard2012EquilibriumMuscle;
    friend class Millard2012AccelerationMuscle;
};

}
#endif //OPENSIM_MUSCLEFIXEDWIDTHPENNATIONMODEL_H_
