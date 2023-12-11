#ifndef OPENSIM_Millard2012EquilibriumMuscle_h__
#define OPENSIM_Millard2012EquilibriumMuscle_h__
/* -------------------------------------------------------------------------- *
 *                  OpenSim:  Millard2012EquilibriumMuscle.h                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Matthew Millard, Tom Uchida, Ajay Seth                          *
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
#include <simbody/internal/common.h>

// The parent class, Muscle.h, provides
//    1. max_isometric_force
//    2. optimal_fiber_length
//    3. tendon_slack_length
//    4. pennation_angle_at_optimal
//    5. max_contraction_velocity
//    6. ignore_tendon_compliance
//    7. ignore_activation_dynamics
#include <OpenSim/Simulation/Model/Muscle.h>

// Sub-models used by this muscle model
#include <OpenSim/Actuators/MuscleFirstOrderActivationDynamicModel.h>
#include <OpenSim/Actuators/MuscleFixedWidthPennationModel.h>
#include <OpenSim/Actuators/ActiveForceLengthCurve.h>
#include <OpenSim/Actuators/ForceVelocityCurve.h>
#include <OpenSim/Actuators/ForceVelocityInverseCurve.h>
#include <OpenSim/Actuators/FiberForceLengthCurve.h>
#include <OpenSim/Actuators/TendonForceLengthCurve.h>

#ifdef SWIG
    #ifdef OSIMACTUATORS_API
        #undef OSIMACTUATORS_API
        #define OSIMACTUATORS_API
    #endif
#endif

namespace OpenSim {

//==============================================================================
//                         Millard2012EquilibriumMuscle
//==============================================================================
/**
This class implements a configurable equilibrium muscle model, as described in
Millard et al.\ (2013). An equilibrium model assumes that the forces generated
by the fiber and tendon are equal:

\f[
 f_{ISO}\Big(\mathbf{a}(t) \mathbf{f}_L(\hat{l}_{CE}) \mathbf{f}_V(\hat{v}_{CE})
+ \mathbf{f}_{PE}(\hat{l}_{CE}) + \beta \hat{v}_{CE}\Big) \cos \phi
-  f_{ISO}\mathbf{f}_{SE}(\hat{l}_{T}) = 0
\f]

\image html fig_Millard2012EquilibriumMuscle.png

This model can be simulated in several configurations by adjusting three flags:

\li ignore_tendon_compliance: set to <I>true</I> to make the tendon rigid. This
assumption is usually reasonable for short tendons, and can result in a
performance improvement by eliminating high-frequency dynamics and removing the
fiber length from the state vector.

\li ignore_activation_dynamics: set to <I>true</I> to use the excitation input
as the activation signal. This results in faster simulations by reducing the
size of the state vector.

\li fiber_damping: set to a value greater than 0.001 to include fiber damping in
the model. The addition of damping reduces simulation time while allowing the
muscle model to be more physiological (it can have an activation of zero, its
active-force-length curve can go to zero, and its force-velocity curve can be
asymptotic).

<B>Elastic Tendon, No Fiber Damping</B>

The most typical configuration used in the literature is to simulate a muscle
with an elastic tendon, full fiber dynamics, and activation dynamics. The
resulting formulation suffers from three singularities: \f$\mathbf{a}(t)
\rightarrow 0\f$, \f$\phi \rightarrow 90^\circ\f$, and
\f$ \mathbf{f}_L(\hat{l}_{CE}) \rightarrow 0 \f$. These situations are all
handled in this model to ensure that it does not produce singularities and does
not result in intolerably long simulation times.

Numerical singularities arise from the manner in which the equilibrium equation
is rearranged to yield an ordinary differential equation (ODE). The above
equation is rearranged to isolate \f$ \mathbf{f}_V(\hat{v}_{CE}) \f$. We then
invert to solve for \f$ \hat{v}_{CE} \f$, which is then numerically integrated
during a simulation:
\f[
 \hat{v}_{CE} = \mathbf{f}_V ^{-1} \Big(
 \frac{ ( \mathbf{f}_{SE}(\hat{l}_{T}) ) /
 \cos \phi
  -  \mathbf{f}_{PE}(\hat{l}_{CE}) }
  { \mathbf{a}(t) \mathbf{f}_L(\hat{l}_{CE})} \Big)
\f]

The above equation becomes numerically stiff when terms in the denominator
approach zero (when \f$\mathbf{a}(t) \rightarrow 0\f$, \f$\phi
\rightarrow 90^\circ\f$, or \f$ \mathbf{f}_L(\hat{l}_{CE}) \rightarrow 0 \f$)
or, additionally, when the slope of \f$\mathbf{f}_V ^{-1}\f$ is steep (which
occurs at fiber velocities close to the maximum concentric and maximum
eccentric fiber velocities).

Singularities can be managed by ensuring that the muscle model is always
activated (\f$\mathbf{a}(t) > 0\f$), the fiber will stop contracting when a
pennation angle of 90 degrees is approached (\f$\phi < 90^\circ\f$), and the
fiber will also stop contracting as its length approaches a lower bound
(\f$ \hat{l}_{CE} > lowerbound\f$), which is typically around half the fiber's
resting length (to ensure \f$ \mathbf{f}_L(\hat{l}_{CE}) > 0 \f$). The fiber is
prevented from reaching unphysiological lengths or its maximum pennation angle
using a unilateral constraint. Additionally, the force-velocity curve is
modified so that it is invertible.

When an elastic tendon without fiber damping is selected, the minimum
active-force-length value is set to 0.1, the minimum permissible activation is
set to 0.01, and the maximum permissible pennation angle is set to acos(0.1) or
84.3 degrees. This is done as a convenience for the user to prevent the model
from taking an unreasonable amount of time to simulate.

<B>(Rigid Tendon) or (Elastic Tendon with Fiber Damping)</B>

Neither of these formulations has any singularities. The lower bound of the
active-force-length curve can be zero (min( \f$ \mathbf{f}_L(\hat{l}_{CE})) = 0
\f$), activation can be zero (i.e., the muscle can be turned off completely),
and the force-velocity curve need not be invertible.

The rigid tendon formulation removes the singularities by ignoring the
elasticity of the tendon. This assumption is reasonable for many muscles, but it
is up to the user to determine whether this assumption is valid.

The formulation that uses an elastic tendon with fiber damping removes
singularities by solving the equilibrium equation with Newton's method. This is
possible because the partial derivative of the equilibrium equation with respect
to fiber velocity is always positive if \f$ \beta > 0\f$ and, thus, Newton's
method can find a solution to the equilibrium equation.

When either of these singularity-free formulations is selected, the minimum
active-force-length value and the minimum permissible activation are set to
zero. This is done as a convenience for the user, as these changes make the
results of the model more realistic yet incur no performance penalty. The
maximum pennation angle is left as acos(0.1) or 84.3 degrees, as allowing higher
pennation angles results in an increasingly stiff fiber velocity state as
pennation angle increases.

<B>Usage</B>

This object should be updated through the <I>set</I> methods provided.

<B>Example</B>
@code
double maxIsometricForce  = 5000;   //N
double optimalFiberLength = 0.025;  //m
double tendonSlackLength  = 0.25;   //m
double pennationAngle     = 0.5;    //rad

bool ignoreTendonCompliance   = false;
bool ignoreActivationDynamics = false;
double dampingCoefficient     = 0.001;

Millard2012EquilibriumMuscle myMuscle("myMuscle",
                                      maxIsometricForce,
                                      optimalFiberLength,
                                      tendonSlackLength,
                                      pennationAngle);

myMuscle.setMuscleConfiguration(ignoreTendonCompliance,
                                ignoreActivationDynamics,
                                dampingCoefficient);
@endcode

Please refer to the doxygen for more information on the properties that are
objects themselves (MuscleFixedWidthPennationModel, ActiveForceLengthCurve,
FiberForceLengthCurve, TendonForceLengthCurve, and ForceVelocityInverseCurve).

<B>Reference</B>

Millard, M., Uchida, T., Seth, A., Delp, S.L. (2013) Flexing computational
muscle: modeling and simulation of musculotendon dynamics. ASME Journal of
Biomechanical Engineering 135(2):021005. http://dx.doi.org/10.1115/1.4023390.

@author Matt Millard
@author Tom Uchida
@author Ajay Seth
*/

class OSIMACTUATORS_API Millard2012EquilibriumMuscle : public Muscle {
OpenSim_DECLARE_CONCRETE_OBJECT(Millard2012EquilibriumMuscle, Muscle);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(fiber_damping, double,
        "The linear damping of the fiber.");
    OpenSim_DECLARE_PROPERTY(default_activation, double,
        "Assumed initial activation level if none is assigned.");
    OpenSim_DECLARE_PROPERTY(default_fiber_length, double,
        "Assumed initial fiber length if none is assigned.");
    OpenSim_DECLARE_PROPERTY(activation_time_constant, double,
        "Activation time constant (in seconds).");
    OpenSim_DECLARE_PROPERTY(deactivation_time_constant, double,
        "Deactivation time constant (in seconds).");
    OpenSim_DECLARE_PROPERTY(minimum_activation, double,
        "Activation lower bound.");
    OpenSim_DECLARE_PROPERTY(maximum_pennation_angle, double,
        "Maximum pennation angle (in radians).");
    OpenSim_DECLARE_UNNAMED_PROPERTY(ActiveForceLengthCurve,
        "Active-force-length curve.");
    OpenSim_DECLARE_UNNAMED_PROPERTY(ForceVelocityCurve,
        "Force-velocity curve.");
    OpenSim_DECLARE_UNNAMED_PROPERTY(FiberForceLengthCurve,
        "Passive-force-length curve.");
    OpenSim_DECLARE_UNNAMED_PROPERTY(TendonForceLengthCurve,
        "Tendon-force-length curve.");

//==============================================================================
// OUTPUTS
//==============================================================================
    OpenSim_DECLARE_OUTPUT(passive_fiber_elastic_force, double,
            getPassiveFiberElasticForce, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(passive_fiber_elastic_force_along_tendon, double,
            getPassiveFiberElasticForceAlongTendon, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(passive_fiber_damping_force, double,
            getPassiveFiberDampingForce, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(passive_fiber_damping_force_along_tendon, double,
            getPassiveFiberDampingForceAlongTendon, SimTK::Stage::Dynamics);

//==============================================================================
// CONSTRUCTORS
//==============================================================================
    /** Default constructor. Produces a non-functional empty muscle. */
    Millard2012EquilibriumMuscle();

    /** Constructs a functional muscle using default curves and activation model
    parameters. The tendon is assumed to be elastic, full fiber dynamics are
    solved, and activation dynamics are included.
        @param aName The name of the muscle.
        @param aMaxIsometricForce The force generated by the muscle when fully
    activated at its optimal resting length with a contraction velocity of zero.
        @param aOptimalFiberLength The optimal length of the muscle fiber.
        @param aTendonSlackLength The resting length of the tendon.
        @param aPennationAngle The angle of the fiber (in radians) relative to
    the tendon when the fiber is at its optimal resting length. */
    Millard2012EquilibriumMuscle(const  std::string &aName,
                                 double aMaxIsometricForce,
                                 double aOptimalFiberLength,
                                 double aTendonSlackLength,
                                 double aPennationAngle);

//==============================================================================
// GET METHODS
//==============================================================================
    /** @returns A boolean indicating whether fiber damping is being used. */
    bool getUseFiberDamping() const;

    /** @returns The fiber damping coefficient. */
    double getFiberDamping() const;

    /** @returns The default activation level that is used as an initial
    condition if none is provided by the user. */
    double getDefaultActivation() const;

    /** @returns The default fiber length that is used as an initial condition
    if none is provided by the user. */
    double getDefaultFiberLength() const;

    /** @returns The activation time constant (in seconds). */
    double getActivationTimeConstant() const;

    /** @returns The deactivation time constant (in seconds). */
    double getDeactivationTimeConstant() const;

    /** @returns The minimum activation level permitted by the muscle model.
    Note that this equilibrium model, like all equilibrium models, has a
    singularity when activation approaches 0, which means that a non-zero lower
    bound is required. */
    double getMinimumActivation() const;

    /** @returns The ActiveForceLengthCurve used by this model. */
    const ActiveForceLengthCurve& getActiveForceLengthCurve() const;
    /** @returns The ForceVelocityCurve used by this model. */
    const ForceVelocityCurve& getForceVelocityCurve() const;
    /** @returns The FiberForceLengthCurve used by this model. */
    const FiberForceLengthCurve& getFiberForceLengthCurve() const;
    /** @returns The TendonForceLengthCurve used by this model. */
    const TendonForceLengthCurve& getTendonForceLengthCurve() const;

    /** @returns The MuscleFixedWidthPennationModel owned by this model. */
    const MuscleFixedWidthPennationModel& getPennationModel() const;

    /** @returns The MuscleFirstOrderActivationDynamicModel owned by this
    model. */
    const MuscleFirstOrderActivationDynamicModel& getActivationModel() const;

    /** @returns The minimum fiber length, which is the maximum of two values:
    the smallest fiber length allowed by the pennation model, and the minimum
    fiber length on the active-force-length curve. When the fiber reaches this
    length, it is constrained to this value until the fiber velocity becomes
    positive. */
    double getMinimumFiberLength() const;

    /** @returns The minimum fiber length along the tendon, which is the maximum
    of two values: the smallest fiber length along the tendon permitted by the
    pennation model, and the minimum fiber length along the tendon on the
    active-force-length curve. When the fiber length reaches this value, it is
    constrained to this length along the tendon until the fiber velocity becomes
    positive. */
    double getMinimumFiberLengthAlongTendon() const;

    /** @param s The state of the system.
        @returns The normalized force term associated with the tendon element,
    \f$\mathbf{f}_{SE}(\hat{l}_{T})\f$, in the equilibrium equation. */
    double getTendonForceMultiplier(SimTK::State& s) const;

    /** @returns The stiffness of the muscle fibers along the tendon (N/m). */
    double getFiberStiffnessAlongTendon(const SimTK::State& s) const;

    /** @param s The state of the system.
    @returns The velocity of the fiber (m/s). */
    double getFiberVelocity(const SimTK::State& s) const;

    /** @param s The state of the system.
    @returns The time derivative of activation. */
    double getActivationDerivative(const SimTK::State& s) const;

    /** get the portion of the passive fiber force generated by the elastic
        element only (N) */
    double getPassiveFiberElasticForce(const SimTK::State& s) const;
    /** get the portion of the passive fiber force generated by the elastic
        element only, projected onto the tendon direction (N) */
    double getPassiveFiberElasticForceAlongTendon(const SimTK::State& s) const;
    /** get the portion of the passive fiber force generated by the damping
        element only (N) */
    double getPassiveFiberDampingForce(const SimTK::State& s) const;
    /** get the portion of the passive fiber force generated by the damping
        element only, projected onto the tendon direction (N) */
    double getPassiveFiberDampingForceAlongTendon(const SimTK::State& s) const;

//==============================================================================
// SET METHODS
//==============================================================================
    /** @param ignoreTendonCompliance Use a rigid (true) or elastic tendon.
        @param ignoreActivationDynamics Treat the excitation input as the
    activation signal (true) or use a first-order activation dynamic model.
        @param dampingCoefficient Specify the amount of damping to include in
    the model (must be either 0 or greater than 0.001). */
    void setMuscleConfiguration(bool ignoreTendonCompliance,
                                bool ignoreActivationDynamics,
                                double dampingCoefficient);

    /** @param dampingCoefficient Define the fiber damping coefficient. */
    void setFiberDamping(double dampingCoefficient);

    /** @param activation The default activation level that is used to
    initialize the muscle. */
    void setDefaultActivation(double activation);

    /** @param s The state of the system.
        @param activation The desired activation level. */
    void setActivation(SimTK::State& s, double activation) const override;

    /** @param fiberLength The default fiber length that is used to initialize
    the muscle. */
    void setDefaultFiberLength(double fiberLength);

    /** @param activationTimeConstant The activation time constant (in
    seconds). */
    void setActivationTimeConstant(double activationTimeConstant);

    /** @param deactivationTimeConstant The deactivation time constant (in
    seconds). */
    void setDeactivationTimeConstant(double deactivationTimeConstant);

    /** @param minimumActivation The minimum permissible activation level. */
    void setMinimumActivation(double minimumActivation);

    /** @param aActiveForceLengthCurve The ActiveForceLengthCurve used by the
    muscle model to scale active fiber force as a function of fiber length. */
    void setActiveForceLengthCurve(
        ActiveForceLengthCurve& aActiveForceLengthCurve);

    /** @param aForceVelocityCurve The ForceVelocityCurve used by the muscle
    model to calculate the derivative of fiber length. */
    void setForceVelocityCurve(ForceVelocityCurve& aForceVelocityCurve);

    /** @param aFiberForceLengthCurve The FiberForceLengthCurve used by the
    muscle model to calculate the passive force the muscle fiber generates as a
    function of fiber length. */
    void setFiberForceLengthCurve(
        FiberForceLengthCurve& aFiberForceLengthCurve);
    
    /** @param aTendonForceLengthCurve The TendonForceLengthCurve used by the
    muscle model to calculate the force exerted by the tendon as a function of
    tendon length. */
    void setTendonForceLengthCurve(
        TendonForceLengthCurve& aTendonForceLengthCurve);

    /** @param[out] s The state of the system.
        @param fiberLength The desired fiber length (m). */
    void setFiberLength(SimTK::State& s, double fiberLength) const;

//==============================================================================
// MUSCLE.H INTERFACE
//==============================================================================
    /** @param[in] s The state of the system.
        @returns The tensile force the muscle is generating (N). */
    double computeActuation(const SimTK::State& s) const override final;

    /** Computes the fiber length such that the fiber and tendon are developing
    the same force, distributing the velocity of the entire musculotendon
    actuator between the fiber and tendon according to their relative
    stiffnesses.
        @param[in,out] s The state of the system.
        @throws MuscleCannotEquilibrate
    */
    void computeInitialFiberEquilibrium(SimTK::State& s) const override {
        computeFiberEquilibrium(s, false);
    }

    /** Computes the fiber length such that the fiber and tendon are developing
        the same force, either assuming muscle-tendon velocity as provided
        by the state or zero as designated by the useZeroVelocity flag.
        @param[in,out] s         The state of the system.
        @param solveForVelocity  Flag indicating to solve for fiber velocity,
                                 which by default is false (zero fiber-velocity)
        @throws MuscleCannotEquilibrate
    */
    void computeFiberEquilibrium(SimTK::State& s, 
                                 bool solveForVelocity = false) const;

//==============================================================================
// DEPRECATED
//==============================================================================
    ///@cond DEPRECATED
    /*  Once the ignore_tendon_compliance flag is implemented correctly, get rid
    of this method as it duplicates code in calcMuscleLengthInfo,
    calcFiberVelocityInfo, and calcMuscleDynamicsInfo.
        @param activation of the muscle [0-1]
        @param fiberLength in (m)
        @param fiberVelocity in (m/s)
        @returns the force component generated by the fiber that is associated
    only with activation (the parallel element is not included) */
    double calcActiveFiberForceAlongTendon(double activation,
                                           double fiberLength,
                                           double fiberVelocity) const;

    /*  @returns a vector of the fiber state that will cause this muscle to
    reproduce the supplied boundary conditions
        \li 0: activation
        \li 1: normalized fiber length (m/lopt)
        \li 2: pennation angle (rad)
        \li 3: normalized fiber velocity ( (m/s) / (vmax * lopt))
        @param lengthMT the length of the entire musculotendon path (m)
        @param velocityMT the velocity of the entire musculotendon path (m)
        @param tendonForce the tensile (+) force applied by the tendon (N)
        @param dTendonForceDT the first time derivative of the force applied by
    the tendon (N/s) */
    SimTK::Vec4 calcFiberStateGivenBoundaryCond(double lengthMT,
                                                double velocityMT,
                                                double tendonForce,
                                                double dTendonForceDT) const;

    /*  @returns the active fiber force generated by a rigid tendon equilibrium
    muscle model
        @param s the state of the system
        @param aActivation the activation of the muscle */
    double calcInextensibleTendonActiveFiberForce(SimTK::State& s,
                                        double aActivation) const override
        final;
    ///@endcond

    /** Adjust the properties of the muscle after the model has been scaled. The
        optimal fiber length and tendon slack length are each multiplied by the
        ratio of the current path length and the path length before scaling. */
    void extendPostScale(const SimTK::State& s,
                         const ScaleSet& scaleSet) override;

//==============================================================================
// PROTECTED METHODS
//==============================================================================
protected:

    /** Gets the derivative of an actuator state by index.
        @param s The state.
        @param aStateName The name of the state to get.
        @return The value of the state derivative. */
    double getStateVariableDeriv(const SimTK::State& s,
                                 const std::string &aStateName) const;

    /** Sets the derivative of an actuator state specified by name.
        @param s The state.
        @param aStateName The name of the state to set.
        @param aValue The value to which the state should be set. */
    void setStateVariableDeriv(const SimTK::State& s,
                               const std::string &aStateName,
                               double aValue) const;

//==============================================================================
// MUSCLE INTERFACE REQUIREMENTS
//==============================================================================
    /** Calculate the position-related values associated with the muscle state
    (fiber and tendon lengths, normalized lengths, pennation angle, etc.). */
    void calcMuscleLengthInfo(const SimTK::State& s,
                              MuscleLengthInfo& mli) const override;

    /** Calculate the velocity-related values associated with the muscle state
    (fiber and tendon velocities, normalized velocities, pennation angular
    velocity, etc.). */
    void calcFiberVelocityInfo(const SimTK::State& s,
                               FiberVelocityInfo& fvi) const override;

    /** Calculate the dynamics-related values associated with the muscle state
    (from the active- and passive-force-length curves, the force-velocity curve,
    and the tendon-force-length curve). The last entry is a SimTK::Vector
    containing the passive conservative (elastic) fiber force and the passive
    non-conservative (damping) fiber force. */
    void calcMuscleDynamicsInfo(const SimTK::State& s,
                                MuscleDynamicsInfo& mdi) const override;

    /** Calculate the potential energy values associated with the muscle */
    void  calcMusclePotentialEnergyInfo(const SimTK::State& s, 
            MusclePotentialEnergyInfo& mpei) const override;

//==============================================================================
// MODELCOMPONENT INTERFACE REQUIREMENTS
//==============================================================================
    /** Sets up the ModelComponent from the model, if necessary */
    void extendConnectToModel(Model& model) override;

    /** Creates the ModelComponent so that it can be used in simulation */
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    /** Initializes the state of the ModelComponent */
    void extendInitStateFromProperties(SimTK::State& s) const override;

    /** Sets the default state for the ModelComponent */
    void extendSetPropertiesFromState(const SimTK::State& s) override;

    /** Computes state variable derivatives */
    void computeStateVariableDerivatives(const SimTK::State& s) const override;

private:
    // The name used to access the activation state.
    static const std::string STATE_ACTIVATION_NAME;
    // The name used to access the fiber length state.
    static const std::string STATE_FIBER_LENGTH_NAME;

    // Indicates whether fiber damping is included in the model (false if
    // dampingCoefficient < 0.001).
    bool use_fiber_damping;

    void setNull();
    void constructProperties();

    // Rebuilds muscle model if any of its properties have changed.
    void extendFinalizeFromProperties() override;

    /*  @param fiso the maximum isometric force the fiber can generate
        @param ftendon the current tendon load
        @param cosPhi the cosine of the pennation angle
        @param fal the fiber active-force-length multiplier
        @param fv the fiber force-velocity multiplier
        @param fpe the fiber force-length multiplier
        @param dlceN the normalized fiber velocity
        @param cosphi the cosine of the pennation angle
        @returns activation required to realize ftendon */
    double calcActivation(double fiso,
                          double ftendon,
                          double cosPhi,
                          double fal,
                          double fv,
                          double fpe,
                          double dlceN) const;

    /*  @param fiberForce the force, in Newtons, developed by the fiber
        @param fiberStiffness the stiffness, in N/m, of the fiber
        @param lce the fiber length
        @param sinphi the sine of the pennation angle
        @param cosphi the cosine of the pennation angle
        @returns the partial derivative of fiber force along the tendon with
    respect to small changes in fiber length (in the direction of the fiber) */
    double calc_DFiberForceAT_DFiberLength(double fiberForce,
                                           double fiberStiffness,
                                           double lce,
                                           double sinPhi,
                                           double cosPhi) const;

    /*  @param dFm_d_lce stiffness of the fiber in the direction of the fiber
        @param sinphi the sine of the pennation angle
        @param cosphi the cosine of the pennation angle
        @param lce the fiber length
        @returns the stiffness of the fiber in the direction of the tendon */
    double calc_DFiberForceAT_DFiberLengthAT(double dFm_d_lce,
                                             double sinPhi,
                                             double cosPhi,
                                             double lce) const;

    /*  @param dFt_d_tl the partial derivative of tendon force w.r.t. small
    changes in tendon length (i.e., tendon stiffness), in (N/m)
        @param lce the fiber length
        @param sinphi the sine of the pennation angle
        @param cosphi the cosine of the pennation angle
        @returns the partial derivative of tendon force with respect to small
    changes in fiber length */
    double calc_DTendonForce_DFiberLength(double dFt_d_tl,
                                          double lce,
                                          double sinphi,
                                          double cosphi) const;

//==============================================================================
// PRIVATE UTILITY CLASS MEMBERS
//==============================================================================

    // Subcomponents owned by the muscle. The properties of these subcomponents
    // are set in extendFinalizeFromProperties() from the properties of the
    // muscle.
    MemberSubcomponentIndex penMdlIdx{
      constructSubcomponent<MuscleFixedWidthPennationModel>("penMdl") };
    MemberSubcomponentIndex actMdlIdx{
      constructSubcomponent<MuscleFirstOrderActivationDynamicModel>("actMdl") };

    // Singularity-free inverse of ForceVelocityCurve.
    ForceVelocityInverseCurve fvInvCurve;

    // Here, I'm using the 'm_' to prevent me from trashing this variable with a
    // poorly chosen local variable.
    double m_minimumFiberLength;
    double m_minimumFiberLengthAlongTendon;

    // Returns true if the fiber length is currently shorter than the minimum
    // value allowed by the pennation model and the active force length curve
    bool isFiberStateClamped(double lce, double dlceN) const;

    // Returns the maximum of the minimum fiber length and the current fiber
    // length.
    double clampFiberLength(double lce) const;

    struct MuscleStateEstimate {
        double solutionError = SimTK::NaN;
        double fiberLength = SimTK::NaN;
        double tendonVelocity = SimTK::NaN;
        double tendonForce = SimTK::NaN;

        enum class Status {
            Success_Converged,
            Warning_FiberAtLowerBound,
            Failure_MaxIterationsReached
        } status = Status::Failure_MaxIterationsReached;
    };

    /* Solves fiber length and velocity to satisfy the equilibrium equations.
    The velocity of the entire musculotendon actuator is shared between the
    tendon and the fiber based on their relative mechanical stiffnesses.

    @param aActivation the initial activation of the muscle
    @param pathLength length of the whole musculotendon actuator
    @param pathLengtheningSpeed lengthening speed of the muscle path
    @param aSolTolerance the desired relative tolerance of the equilibrium
           solution
    @param aMaxIterations the maximum number of Newton steps allowed before we
           give up attempting to initialize the model
    @param staticSolution set to true to calculate the static equilibrium
           solution, setting fiber and tendon velocities to zero
    */
    MuscleStateEstimate estimateMuscleFiberState(const double aActivation,
                                 const double pathLength,
                                 const double pathLengtheningSpeed,
                                 const double aSolTolerance,
                                 const int aMaxIterations,
                                 bool staticSolution=false) const;

};
} //end of namespace OpenSim

#endif // OPENSIM_Millard2012EquilibriumMuscle_h__
