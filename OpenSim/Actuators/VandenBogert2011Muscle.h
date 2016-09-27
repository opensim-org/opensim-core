#ifndef OPENSIM_VANDENBOGERT2011MUSCLE_H_
#define OPENSIM_VANDENBOGERT2011MUSCLE_H_

/* -------------------------------------------------------------------------- *
*                 OpenSim:  VandenBogert2011Muscle.h                         *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2016 Stanford University and the Authors                     *
* Author(s): Brad Humphreys, Chris Dembia, Antoine van den Bogert            *
*                                                                            *
* Licensed under the Apache License, Version 2.0 (the "License"); you may    *
* not use this file except in compliance with the License. You may obtain a  *
* copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
*                                                                            *
* Unless required by applicable law or agreed to in writing, software        *
* distributed under the License is distributed on an "AS IS" BASIS,          *
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
* See the License for the specific language governing permissions and        *
        * limitations under the License.                                     *
* -------------------------------------------------------------------------- */

#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <simbody/internal/common.h>

/*
The parent class, Muscle.h, provides
    1. max_isometric_force
    2. optimal_fiber_length
    3. tendon_slack_length
    4. pennation_angle_at_optimal
    5. max_contraction_velocity
*/
#include <OpenSim/Simulation/Model/Muscle.h>

namespace OpenSim {
/**
This class implements a muscle with implicit dynamics capabilities as described
 in van den Bogert et al[1].

 The muscle is comprised of 4 components and is show below:
- CE: Contractile Element
- DE: Damping Element
- PE: Parallel Elastic Element
- SE: Series Elastic Element (tendon)

\image html fig_VandenBogertMuscle.png



The dimensions are:
 - \f$h\f$ : Muscle height
 - \f$ l_{m} \f$ : Length of the muscle contractile and parallel elastic element (also know as the
 "muscle fiber length")
 - \f$ l_{mt} \f$ -  Muscle and tendon fiber length
 - \f$ l_{p} \f$ - Length of the fiber projected on to the line of action
 of the tendon (also the length of the damping elements)
 - \f$ l_{t} \f$ - Length of the tendon
 - \f$ \phi \f$ - Pennation angle of the muscle fiber


<B>Nomenclature </B> <BR>
 Symbols with the a \f$ \bar{bar} \f$ indicate that the quantity has been normalized by a constant parameter.  For forces, this will all ways be \f$F_{m,opt}\f$.  For velocities and lengths this will be \f$l_{m_opt}\f$ (velocities will therefore have units of "optimal lengths per second")

<table>
<tr>
<td colspan=3>
</td>
|Symbol | Description | Units |
|--------|---------|----------|
|a       | Activation (State Variable)      | unitless        |
|\f$b_{de}\f$       | Damping, % of \f$F_{m,opt}\f$ per 1 optimal length per second   (Parameter: fiber_damping)  | N/m/s  |
|\f$A_{hill} \f$       | Hill Constant (Parameter: force_velocity_hill_constant)| unitless   |
|\f$ F_{se} \f$| Force in the Series Elastic Element (Calculated Value) | N |
|\f$ F_{ce} \f$| Force in the Contractile Element  (Calculated Value) | N |
|\f$ F_{de} \f$| Force in the Damping Element  (Calculated Value) |N|
|\f$ F_{m,opt} \f$| Force in the contractile element under isometric conditions, at optimal length  (Parameter: max_isometric_force)  |N|
|\f$ k_{pe} \f$| Parallel elastic element linear stiffness  (Derived Parameter) | N/m |
|\f$ k_{pe,quad} \f$| Parallel elastic element linear stiffness (Derived Parameter) | \f$ N/m^{2} \f$|
|\f$ k_{se} \f$| Series  elastic element (tendon) linear stiffness  (Derived Parameter) | N/m |
|\f$ k_{se,quad} \f$| Series elastic element (tendon) linear stiffness (Derived Parameter) | \f$ N/m^{2} \f$|
|\f$ \lambda \f$| Activation-Velocity Parameter (Derived Parameter)| unitless |
|\f$ \lambda_{m} \f$|Activation-Velocity Parameter Slope (Parameter: activation_velocity_slope)| unitless |
|\f$ \lambda_{o} \f$|Lambda value at zero Activation (Parameter: activation_velocity_intercept)| unitless |
 |\f$ \phi_{ref} \f$|Pennation at optimal fiber length (Parameter: pennation_at_optimal_fiber_length) |rad|
|\f$ \bar{l_{m,slack}} \f$ | length of muscle when parallel elastic element is slack (Parameter: fiber_slack_length_norm)| unitless |
|\f$ l_{t,slack} \f$ | Slack Length of the tendon (Parameter: tendon_slack_length )| m |
|\f$ \tau_{a} \f$| Activation Time Constant (Parameter: activation_time_constant)| 1/s |
|\f$ \tau_{d} \f$ | Deactivation Time Constant (Parameter: deactivation_time_constant) |1/s|
|\f$ u_{max} \f$ | Strain in tendon at \f$F_{m,opt}\f$  (Parameter: tendon_strain_at_max_iso_force) | m/m |
|\f$ \bar{v_{max}} \f$ | Normalized maximum lengthining (concentric) velocity (Parameter: force_velocity_max_lengthening_force_norm) | unitless|
|w | force-length width (Parameter: active_force_length_curve_width); This is the kShapeActive parameter in Thelen2003Muscle| unitless |
 </tr>
</table>

------------------------------------------------------------------------------
 <B>Force Balance</B> <BR>
 The implicit muscle determines it's operating state by calculating the forces
 which result in the tendon and muscle complexes being balanced (equivalent):
 \f[
 F_{se} - (F_{ce} + F_{pe})\cos\phi - F_{de} = 0
 \f]


 <B>Series Elastic Element Force, \f$ F_{se} \f$  </B> <BR>
 This is the elastic force in the tendon and is a function of the length of the
 tendon (\f$ l_{mt} - l_{m} \cos\phi \f$) which is :
  \f[
 F_{se} = \mathbf{f_{se}} (l_{mt} - l_{m} \cos\phi)
 \f]


 <B>Contractile Element Force, \f$ F_{ce} \f$ </B> <BR>
 The contractile element force is:
 \f[
 F_{ce} = a*F_{m,opt} \mathbf{f_{l}}(l_{m}) \mathbf{f_{v}} (\dot{l_{m}})
 \f]
 \f$ \mathbf{f_{l}} \f$  and  \f$ \mathbf{f_{v}} \f$ are the muscle force-length and force-velocity relationships and are detailed below.


 <B>Parallel Elastic Element Force, \f$ F_{pe} \f$ </B> <BR>
 The elastic force in the muscle fiber is a function of the length of the contractile element:
  \f[
 F_{pe} = \mathbf{f_{pe}}(l_{m})
 \f]


 <B>ParallelDamping Element Force, \f$ F_{de} \f$ </B> <BR>
 A dissipative force in the muscle fiber is modeled as a function of the velocity of the velocity of
 the muscle fiber projected along the line of action of the tendon:
 \f[
 F_{de}=\mathbf{f_{de}}(\dot{l_{m}})
\f]

 ------------------------------------------------------------------------------
 <B>Muscle State Equation</B> <BR>
 The state equation for the muscle is then:

 \f[
 \mathbf{f_{se}} (l_{mt} - l_{m} \cos\phi)

 - [aF_{m,opt} \mathbf{f_{l}}(l_{m}) \mathbf{f_{v}} (\dot{l_{m}})

 + \mathbf{f_{pe}}(l_{ce})]
 \cos\phi
 - \mathbf{f_{de}}(\dot{l_{m}})
 =0
 \f]

 The muscle length state variable is then chosen:

\f[
 x_{1}= l_{m}\cos\phi  = l_{p}
 \f]

 (This differs from other muscle models that use a state variable of \f$ l_{m} \f$.)

  Due to the \f$ l_{m} \f$ and \f$ \dot{l_{m}}\f$ terms in the force-length and force-velocity relationship,
  along with the \f$ \cos\phi \f$ term, intermediate equations are needed to convert from the state variable \f$ x_{1} \f$ to
  \f$ \bar{l_{p}} \f$.



  To remove the pennation angle \f$ \phi \f$ as a variable, a constant muscle volume assumption is used[2]:
 \f[
  l_{m} \sin\phi = l_{m,ref} \sin\phi_{ref} = h
 \f]

  This muscle uses the reference length of \f$ l_{m,ref}=l_{m,opt} \f$.  \f$ h \f$ is referred to as the muscle height (a constant).  \f$ \phi_{ref} \f$ and \f$ l_{ce,ref} \f$ are the pennation angle at a reference fiber length respectively.  If the pennation is less than 0.01 radians, the projected fiber length equals the fiber length (if pennation is zero, the muscle volume would be zero).

  An implicit first order differential equation of state \f$ x_{1} \f$ can then be formed using the following two equations:
 \f[
  l_{m} = ({l_{p}}^2 + h^{2})^{0.5} = ({x_{1}}^2 + h^{2})^{0.5}
 \f]
\f[
 \cos\phi=\frac {l_{p}}{l_{m}}=\frac {x_{1}}{l_{m}}
 \f]


Muscle activation, \f$ a \f$ is a function of the control signal (neural excitation), is modeled[3] as:

 \f[
  \dot{a}=(u-a)    (\frac{u}{\tau_{a}} + \frac{1-u}{\tau_{d}})
\f]
This is a first order differential equation and therefore the muscle has a second state variable \f$ x_{2}=a \f$.


 Implicit formulation of muscles have the following advantages:<BR>
 1) This muscle does not have a singularity at zero activation,  \f$ a=0\f$  <BR>
 2) This muscle does not have a singularity at pennation=90 deg,  \f$ \phi=\pi \f$ <BR>
 3) This muscle has continuous derivatives.  This allows for use of gradient based solvers such as those commonly used in optimization. <BR>

 The above singularities can cause very small time stepping by integrators when they are approached in simulation. <BR>

  In the formulation of each of the force components, normalized values are used to help insure numerical stability and to allow comparison to standard muscle curves.  Also, the implementation of the \f$ x_{1} \f$ state variable uses a normalization:
\f[
 x_{1}= \frac{l_{m}\cos\phi} {l_{m,opt}} = \frac {l_{p}} {l_{m,opt}} = \bar{l_{p}}
 \f]

------------------------------------------------------------------------------

 <B> Force Length Relationship, \f$ f_{l} \f$</B> <BR>
 The force-length relationship of the muscle is modeled with a Gaussian curve [4]:
 \f[
 \mathbf{f_{l}} = e^{-d^{2}}
 \f]

 where:
 \f[
 d=\frac{l_{m}-l_{m,opt}}{w \cdot l_{m,opt}}
\f]


 \image html fig_VandenBogert2011Muscle_fl.png


 <B>Force Velocity Relationship, \f$ f_{v} \f$</B> <BR>
 The force-velocity relationship of the muscle is broken into two regimes:

 For Muscle Shortening (\f$ \dot{l_{m}}<0 \f$), the Hill[5] concentric model is utilized:
 \f[
 \mathbf{f_{v}}=  \frac {\lambda v_{max} +\dot{l_{m}}}
 {\lambda v_{max} -\frac{\dot{l_{m}}}{A_{Hill}}}
\f]

  For muscle elongating (\f$ \dot{l_{m}}>=0 \f$), the Katz[6] eccentric model is utilized:
 \f[
\mathbf{f_{v}} = \frac{\bar{f_{v,max}} * \dot{l_{m}} +c}
{ \dot{l_{m}} +c}
 \f]

 where:
  \f[
c=\frac{\lambda \cdot v_{max} A_{hill} (f_{v,max}-1)}
{A_{hill}+1}
 \f]

 The parameter \f$ \lambda \f$ in the force-velocity relationship provides a scalar to adjust the maximum lengthening velocity based upon muscle activation[7]:
   \f[
     \lambda = \lambda_{m}a + \lambda_{o}
  \f]


 \image html fig_VandenBogert2011Muscle_fv.png


  <B>Parallel Elastic Element Force, \f$ F_{pe} \f$</B> <BR>

 Calculation of the force in the parallel elastic element are expressed in terms of normalized force:
 \f[
 F_{pe} = F_{m,opt} \cdot \bar{F_{pe}}
 \f]

 Elongation of the parallel element from it's slack length (and relative to fiber's optimal length):
 \f[
 \bar{dl_{m}} = \frac {l_{m}} {l_{m,opt}}  -  \frac {l_{m,slack}} {l_{m,opt}} = \bar{l_{m}} -  \bar{l_{m,slack}}
 \f]

 If the parallel element's length is less than it's slack length (\f$ l_{m}<l_{m,slack} \f$, so \f$ \bar{dl_{m}} < 0 \f$) a light linear stiffnesss of 1 N/m is used:
  \f[
 \bar{k_{pe}} = 1\frac{N}{m}  \frac{l_{m,opt}} {F_{m,opt}}
 \f]
  \f[
 \bar{F_{pe}} = \bar{k_{pe}} \bar{dl_{m}}
 \f]

 If the parallel element's length is greater than it's slack length (\f$ l_{m}>l_{m,slack} \f$, so \f$ \bar{dl_{m}} > 0 \f$) a quadratic stiffness is added:
   \f[
 \bar{k_{pe,quad}} = \frac{1}{w^{2}}
 \f]
 This insures that \f$ F_{pe} = F_{m,opt} \f$ when \f$ l_{m}=l_{m,opt}(1+w)\f$
  \f[
 \bar{F_{pe}} = \bar{k_{pe}} \bar{dl_{m}} + \bar{k_{pe,quad}} \bar{dl_{m}}^{2}
 \f]


 \image html fig_VandenBogert2011Muscle_PeeForce.png

 <B> Series Elastic Element (Tendon) Force, \f$ F_{se} \f$ </B> <BR>


The length of the tendon is:
 \f[
 l_{t} = l_{mt} - l_{p}
 \f]

 The elongation of the tendon is then
 \f[
 dl_{t}=l_{t}-l_{t,slack}
 \f]

  The force in the tendon is:
   \f[
 F_{se} = k_{se}dl_{t} + k_{se,quad}(dl_{t})^{2}
 \f]

If the tendon is not being stretched, a very light spring rate of 1N/m is used:
  \f[
 k_{se} = 1
 \f]

 If the tendon is being stretched an additional quadratic term is used so that
 force in the tendon: \f$F_{se} = F_{m,opt}\f$  when the strain in the tendon = \f$u_{max}\f$:

 \f[
 k_{se,quad} = \frac{F_{m,opt}}{(u_{max}l_{t,slack})^{2}}
 \f]



To normalize the force (note that due to the quadratic terms, non-normalized lengths must be used):
 \f[
 \bar{F_{se}} = \frac{1}{F_{m,opt}} =\frac{dl_{t}}{F_{m,opt}} + \frac{1}{(u_{max} dl_{t})^{2}}
 \f]

  \image html fig_VandenBogert2011Muscle_SeeForce.png

 <B>Parralel Damping Element Force, \f$  F_{de} \f$ </B> <BR>

\f$  F_{de} \f$ is the viscous damping parallel to the contractile element. The damping is set such that it equals \f$b_{de}\f$ percent of \f$F_{m,opt}\f$ when the muscle velocity is 1 optimal length per second:

  \f[
 F_{de}  = b_{de} F_{m,opt} \frac {\dot{l_p}}{l_{m_opt}}
 \f]

  or in normalized terms:

 \f[
 \bar{F_{de}}  = b_{de} \bar{\dot{l_p}}
 \f]

 This damping is present in the model to provide numerical stability when activation of the muscle is zero.


------------------------------------------------------------------------------
  <B>References </B> <BR>
 [1] van den Bogert A, Blana D, Heinrich D, Implicit methods for efficient musculoskeletal simulation and optimal control. Procedia IUTAM 2011; 2:297-316 <BR>
 [2] Otten E. Concepts and models of functional architecture in skeletal muscle. Exerc Sport Sci Rev 1988; 16:89-137 <BR>
 [3] He J, Levine WS, Loeb GE. Feedback gains for correcting small perturbations to standing posture. IEEE Trans Auto Control 1991; 36:322-32. <BR>
 [4] Walker S, Schrodt I, Segment lengths and thin filament periods in skeletal muscle fibers of the rhesus monkey and the human. The Anatom Rec 1974; 178:63-81 <BR>
 [5] Hill A, The heat of shortening and the dynamic constants of Muscle. Proc Royal Society 1938 126 <BR>
 [6] Katz B, The relation between force and speed in muscular contraction. J Physiol 1939; 96:45-64 <BR>
 [7] Chow J, Darling W, The maximum shortening velocity of muscle should be scaled with activation. J Appl Physiol 1999; 86(3):1025-31 <BR>

 */
    //TODO:  Add detailed muscle discription

// Derive new class from Muscle.h
class OSIMACTUATORS_API VandenBogert2011Muscle : public Muscle {
        OpenSim_DECLARE_CONCRETE_OBJECT(VandenBogert2011Muscle, Muscle);

public:
//=============================================================================
// PROPERTIES
//=============================================================================

        //OpenSim_DECLARE_PROPERTY(max_isometric_force, double,
        //  F_{max} (N) "Maximum isometric force that the fibers can generate");
        // Comes from parent class, Muscle.h

        OpenSim_DECLARE_PROPERTY(tendon_strain_at_max_iso_force, double,
        "tendon strain at maximum isometric muscle force");
        //u_{max} (dimensionless) strain in the series elastic element at load
        // of maxIsometricForce

        OpenSim_DECLARE_PROPERTY(active_force_length_curve_width, double,
        "force-length shape factor");
        //W (dimensionless) width parameter of the force-length relationship of
        // the muscle fiber

        OpenSim_DECLARE_PROPERTY(force_velocity_hill_constant, double,
        "force-velocity shape factor");
        //AHill (dimensionless) Hill parameter of the force-velocity
        // relationship

        OpenSim_DECLARE_PROPERTY(force_velocity_max_lengthening_force_norm, double,
        "maximum normalized lengthening force");
        //FV_{max} (dimensionless) maximal eccentric force

        //OpenSim_DECLARE_PROPERTY(optimal_fiber_length, double,
        //    "Optimal Length of Contractile Element"
        //Lceopt (m)
        // Comes from parent class, Muscle.h

        OpenSim_DECLARE_PROPERTY(fiber_damping, double,
        "The linear damping of the fiber");
        //b (s/m) damping coefficient of damper parallel to the fiber
        //      (normalized to maxIsometricForce)

        OpenSim_DECLARE_PROPERTY(fiber_slack_length_norm, double,
                                 "(dimensionless) slack length of the parallel "
                                         "elastic element, divided by Lceopt");
        //L_{slack,fiber}(dimensionless) slack length of the fiber (PEE)


        OpenSim_DECLARE_PROPERTY(activation_time_constant, double,
        "Activation time(s)");
        //T_{act} (s) Activation time

        OpenSim_DECLARE_PROPERTY(deactivation_time_constant, double,
        "Deactivation time(s)");
        //T_{deact} (s) Deactivation time

        OpenSim_DECLARE_PROPERTY(pennation_at_optimal_fiber_length, double,
                                 "pennation at optimal fiber length "
                                         "equilibrium");

        OpenSim_DECLARE_PROPERTY(default_activation, double,
                                 "default activation");

        OpenSim_DECLARE_PROPERTY(default_fiber_length, double,
                                 "Assumed initial fiber length if none is "
                                         "assigned.");


//=============================================================================
// Construction
//=============================================================================
        /** Constructs a functional muscle using default parameters. */
      explicit  VandenBogert2011Muscle();

        /** Constructs a functional muscle using default curves and activation
        model parameters.
        @param name The name of the muscle.
        @param maxIsometricForce The force generated by the muscle when fully
        activated at its optimal resting length with a contraction velocity of
        zero.
        @param optimalFiberLength The optimal length of the muscle fiber.
        @param tendonSlackLength The resting length of the tendon.
        @param pennationAngle The angle of the fiber (in radians) relative to
        the tendon when the fiber is at its optimal resting length. */
        VandenBogert2011Muscle(const std::string &name,
        double maxIsometricForce,
        double optimalFiberLength,
        double tendonSlackLength,
        double pennationAngle);



//-------------------------------------------------------------------------
// GET & SET Properties
//-------------------------------------------------------------------------
        /** @param tendon_strain_at_max_iso_force The tendon strain at a load of Fmax*/
        void setFMaxTendonStrain(double tendon_strain_at_max_iso_force);
        /** @returns The tendon strain at a load of Fmax*/
        double getFMaxTendonStrain() const;

        /** @param flWidth The width parameter (W) of the force-length relationship of
        the muscle fiber (dimensionless)*/
        void setFlWidth(double flWidth);
        /** @returns The width parameter (W) of the force-length relationship of
        the muscle fiber (dimensionless)*/
        double getFlWidth() const;

        /** @param fvAHill The Hill parameter of the force-velocity relationship
        (dimensionless)*/
        void setFvAHill(double fvAHill);
        /** @returns The Hill parameter of the force-velocity relationship
        (dimensionless)*/
        double getFvAHill() const;

        /** @param fvMaxMultiplier The maximal eccentric force multiplier FV_{max}
        (dimensionless) */
        void setFvmaxMultiplier(double fvMaxMultiplier);
        /** @returns The maximal eccentric force multiplier FV_{max}
        (dimensionless) */
        double getFvmaxMultiplier() const;

        /** @param dampingCoefficient The damping coefficient of damper parallel to the fiber
        (normalized to maxIsometricForce), b (s/m)*/
        void setDampingCoefficient(double dampingCoefficient);
        /** @returns The damping coefficient of damper parallel to the fiber
        (normalized to maxIsometricForce), b (s/m)*/
        double getDampingCoefficient() const;

        /** @param normFiberSlackLength The slack length of the parallel elastic element, divided by
        fiber optimal length, L_{slack,fiber} (dimensionless)*/
        void setNormFiberSlackLength(double normFiberSlackLength);
        /** @returns The slack length of the parallel elastic element, divided
        by fiber optimal length, L_{slack,fiber} (dimensionless)*/
        double getNormFiberSlackLength() const;

        /** @param activTimeConstant The activation time constant, T_{act} (s)*/
        void setActivTimeConstant(double activTimeConstant);
        /** @returns The activation time constant, T_{act} (s)*/
        double getActivTimeConstant() const;

        /** @param deactivationTimeConstant The deactivation time constant, T_{deact} (s)*/
        void setDeactivationTimeConstant(double deactivationTimeConstant);
        /** @returns The deactivation time constant, T_{deact} (s)*/
        double getDeactivationTimeConstant() const;

        /** @param pennAtOptFiberLength The pennation angel at optimal fiber length,
        phi_{opt} (rad)*/
        void setPennAtOptFiberLength(double pennAtOptFiberLength);
        /** @returns The pennation angel at optimal fiber length,
        phi_{opt} (rad)*/
        double getPennAtOptFiberLength() const;

        /** @param defaultActivation The default muscle activation, (dimensionless)*/
        void setDefaultActivation(double defaultActivation);
        /** @returns The default muscle activation, (dimensionless)*/
        double getDefaultActivation() const;

        /** @param fiberLength The default muscle fiber length, ()*/
        void setDefaultFiberLength(double fiberLength);
        /** @returns The default muscle fiber length, ()*/
        double getDefaultFiberLength() const;


        /**@param s The state of the system.
        @param activation The muscle activation, (dimensionless)*/
        void setActivation(SimTK::State& s,double activation) const override;
        /**@param s The state of the system.
        @param fiberLength The muscle fiber length, ()*/
        void setFiberLength(SimTK::State& s,double fiberLength) const;

        /* Calculate the muscle implicit residual.  Returns a state Vec2
        containing the force residual and activation residual.  When these
        values are equal to zero, the muscle equations are "balanced" and the
        muscle is in a valid state.
                @param s The state of the system.
                @param projFibVelNorm_guess The muscle fiber velocity, projected
                in line with tendon and normalized by the optimal fiber length,
                (dimensionless)
                @param activdot_guess The muscle activation time derivative,
                (1/s)
                @param excitation The muscle excitation
                @returns Muscle residual vector:
                        \li 0: force (N)
                        \li 1: activation (unitless)*/
        SimTK::Vec2 getResidual(const SimTK::State& s,
                                double projFibVelNorm_guess,
                                double activdot_guess, double excitation) const;


        //Hacks because cache variables not implemented yet

        /**@param s The state of the system.
           @returns The Fiber Length of the muscle.*/
        double  getFiberLength(const SimTK::State& s) const; //BTH
        /**@param s The state of the system.
           @returns The activation of the muscle.*/
        double  getActivation(const SimTK::State& s) const override;  // BTH


        //Temporary struct for troubleshooting
        struct ImplicitResidual {
            double forceResidual = SimTK::NaN;
            double activResidual = SimTK::NaN;
            double forceTendon = SimTK::NaN;
            SimTK::Mat22 df_dy = {SimTK::NaN,SimTK::NaN,SimTK::NaN,
                                  SimTK::NaN,SimTK::NaN,SimTK::NaN};
            SimTK::Mat22 df_dydot = {SimTK::NaN,SimTK::NaN,SimTK::NaN,
                                     SimTK::NaN,SimTK::NaN,SimTK::NaN};
            SimTK::Vec2 df_du = {SimTK::NaN,SimTK::NaN};
            double df_dmuscleLength = SimTK::NaN;
            double F1 = SimTK::NaN;
            double F2 = SimTK::NaN;
            double F3 = SimTK::NaN;
            double F4 = SimTK::NaN;
            double F5 = SimTK::NaN; };

//==============================================================================
// MUSCLE.H INTERFACE
//==============================================================================

        double computeActuation(const SimTK::State& s) const override final;

        void computeInitialFiberEquilibrium(SimTK::State& s) const override;

        void computeFiberEquilibriumAtZeroVelocity(SimTK::State& s) const
                override;


//=============================================================================
// COMPUTATION
//=============================================================================

    // TODO: These should move to protected at some point

        // Methods to calculate muscle residual:


        /** Calculates the muscle residuals.
                @param muscleLength Length of muscle (m)
                @param projFibLengthNorm Length of the muscle fiber projected inline
                        with tendon and normalized by the optimal fiber length (m/m)
                @param activ The muscle activation
                @param projFibVelNorm Velocity of the muscle fiber projected
                        inline with tendon and normalized by the optimal fiber length (1/s)
                @param activdot Time derivative of the muscle activation
                @param excitation The muscle control excitation
                @param returnJacobians If = true, calculate and return the
                        Jacobains. default is =  false, Jacobians are not calculated
                @returns TBD for now see ImplicitResidual structure*/
        ImplicitResidual calcImplicitResidual(double muscleLength,
                                              double projFibLengthNorm,
                                              double activ,
                                              double projFibVelNorm,
                                              double activdot,
                                              double excitation,
                                              bool returnJacobians=false) const;

        /** Calculates the muscle residuals. (State equation form for vector
        manipulation)
                @param y vector of:
                        \li 0: projected fiber length
                        \li 1: muscle activation
                @param ydot_guess Vector of the guess for the derivative of y
                @param muscleLength Length of muscle (m)
                @param excitation Muscle control excitation
                @param returnJacobians If = true, calculate and return the
                        Jacobains. default is =  false, Jacobians are not calculated
                @returns TBD for now see ImplicitResidual structure*/
        ImplicitResidual calcImplicitResidual(SimTK::Vec2 y,
                                              SimTK::Vec2 ydot_guess,
                                              double muscleLength,
                                              double excitation,
                                              bool returnJacobians=false) const;

        /** Calculates the muscle residuals. (method for getting from
        state)
                @param s The state of the system. (provides projected fiber
                        length and muscle activation)
                @param projFibVel_guess Guess of the fiber velocity
                        (projected in-line with the tendon)
                @param activdot_guess Guess of the time derivative of activation
                @param excitation Muscle control excitation
                @param returnJacobians If = true, calculate and return the
                        Jacobains. default is = false, Jacobians are not calculated
                @returns TBD for now see ImplicitResidual structure*/
        ImplicitResidual calcImplicitResidual(const SimTK::State& s,
                                              double projFibVel_guess,
                                              double activdot_guess,
                                              double excitation,
                                              bool returnJacobians=false) const;

        //Convience methods for converting between muscle length, projected
        // muscle length, and normalized values.  Need to do this because I am
        // sticking with the convention that external to the muscle,
        // muscle length is the state variable.  Inside this muscle's code it
        // is using the projected muscle length.

        //TODO: Maybe cache variables?

        /** @param fiberLength The fiber length
            @param argsAreNormalized T: The values are normalized by the muscle's
                optimal length, F: Not normalized
            @returns The projected fiber length
                (will be normalized if argsAreNormalized = true)*/
        double fiberLengthToProjectedLength (double fiberLength ,
                                             bool argsAreNormalized=false) const;


        /**@param projFibLen The projected (inline with tendon) length
            @param argsAreNormalized T: The values provided are normalized by the
            muscle's optimal length, F: Not normalized
            @returns The fiber length
            (will be normalized if argsAreNormalized = true)*/
        double projFibLenToFiberLength (double projFibLen,
                                        bool argsAreNormalized=false) const;

        /**@param fiberVelocity Velocity of the fiber
           @param fiberLength Fiber Length
           @param projFiberLength Projected fiber velocity (in-line with tendon)
           @param argsAreNormalized T: The values provided are normalized by the
           muscle's optimal length, F: Not normalized
           @returns The projected fiber velocity
                (will be normalized if argsAreNormalized = true)*/
        double fiberVelocityToProjFibVel (double fiberVelocity,
                                          double fiberLength,
                                          double projFiberLength,
                                          bool argsAreNormalized=false) const;

        /**@param fiberVelocity Velocity of the fiber
           @param fiberLength Fiber Length
           @param argsAreNormalized T: The values provided are normalized by the
           muscle's optimal length, false: Not normalized
           @returns The projected fiber velocity
           (will be normalized if argsAreNormalized = true)*/
        double fiberVelocityToProjFibVel (double fiberVelocity,
                                          double fiberLength,
                                          bool argsAreNormalized=false) const;

        /**@param projFibVel Projected velocity of the fiber
           @param fiberLength Fiber Length
           @param projFiberLength Projected fiber velocity (in-line with tendon)
           @param argsAreNormalized true: The values provided are normalized by the
           muscle's optimal length, false: Not normalized
           @returns The fiber velocity
           (will be normalized if argsAreNormalized = true)*/
        double projFibVelToFiberVelocity(double projFibVel, double fiberLength,
                                         double projFiberLength,
                                         bool argsAreNormalized=false) const;

        /**@param projFibVel Projected velocity of the fiber
           @param projFiberLength Projected fiber velocity (in-line with tendon)
           @param argsAreNormalized true: The values provided are normalized by the
           muscle's optimal length, false: Not normalized
           @returns The fiber velocity
          (will be normalized if argsAreNormalized = true)*/
        double projFibVelToFiberVelocity(double projFibVel,
                                         double projFiberLength,
                                         bool argsAreNormalized=false) const;


        //During tests we want to insure that the anlytical jacobians are
        // correct.  To do this we can compare them to finite difference
        //calcualted jacobians.

        /** Calculate Jacobian by finite difference
              @param y Vector of:
                        \li 0: projected fiber length
                        \li 1: muscle activation
              @param ydot Vector of:
                        \li 0: projected fiber velocity
                        \li 1: muscle activation time derivative
              @param muscleLength The muscle length
              @param excitation The muscle control excitation
              @param h The peturbation size of the finite step
              @returns TBD for now see ImplicitResidual structure*/
        ImplicitResidual calcJacobianByFiniteDiff(SimTK::Vec2 y,
                                                  SimTK::Vec2 ydot,
                                                  double muscleLength,
                                                  double excitation, double stepSize ) const;
        /** Calculate Jacobian by finite difference
              @param muscleLength Muscle Length
              @param projFibLenNorm The projected fiber length (inline with
              tendon) and normalized by muscle optimal length.
              @param activ The muscle activation
              @param projFibVelNorm The projected fiber velocity (inline with
              tendon) and normalized by muscle optimal length.
              @param activdot The time derivative of the muscle activation
              @param excitation The muscle control excitation
              @param stepSize The peturbation size of the finite step
              @returns TBD for now see ImplicitResidual structure*/
        ImplicitResidual calcJacobianByFiniteDiff(double muscleLength,
                                                  double projFibLenNorm,
                                                  double activ,
                                                  double projFibVelNorm,
                                                  double activdot, double excitation,
                                                  double stepSize) const;




        /** Calculate the muscle static equilibrium.  The muscle length is
            static, the fiber velocity is zero, and the activation derivative is
            zero.
                @param muscleLength The muscle length
                @param activ The muscle activation
                @returns Vector:
                        \li 0: projected fiber length normalized
                        \li 1: muscle force */
        SimTK::Vec2 calcFiberStaticEquilbirum(double muscleLength,
                                              double activ) const;

        /** Calculate the muscle residual at static equilibrium.  The muscle
        length is  static, the fiber velocity is zero, and the activation
         derivative is zero.
                @param projFibLen The projected fiber length
                @param muscleLength The muscle length
                @param activ The muscle activation
        @returns Vector:
                \li 0: muscle force residual
                \li 1: derivative of residual wrt activation
                \li 2: muscle force*/
        SimTK::Vec3 calcFiberStaticEquilibResidual(double projFibLen,
                                                   double muscleLength,
                                                   double activ) const;



        //TODO: Need doxygen
        SimTK::Vec2 calcSolveMuscle(const SimTK::State& s,
                                    double activ,
                                    SimTK::Vector yDotInitialGuess)
                                const;



//=============================================================================
// PROTECTED METHODS
//=============================================================================
    protected:



//==============================================================================
// MUSCLE INTERFACE REQUIREMENTS
//==============================================================================

    /* Not implmented (yet)
        void calcMuscleLengthInfo(const SimTK::State& s,
                                  MuscleLengthInfo& mli) const override;


        void calcFiberVelocityInfo(const SimTK::State& s,
                                   FiberVelocityInfo& fvi) const override;


        void calcMuscleDynamicsInfo(const SimTK::State& s,
                                    MuscleDynamicsInfo& mdi) const override;


        void  calcMusclePotentialEnergyInfo(const SimTK::State& s,
                                            MusclePotentialEnergyInfo& mpei)
                                            const override;
     */

//==============================================================================
// MODELCOMPONENT INTERFACE REQUIREMENTS
//==============================================================================

        /** Sets up the ModelComponent from the model, if necessary */
        //void extendConnectToModel(Model& model) override;
        //Removed per Chris Dembia comments

        /** add new dynamical states to the multibody system corresponding
            to this muscle */
        void extendAddToSystem(SimTK::MultibodySystem &system) const override;

        /** initialize muscle state variables from properties. For example, any
            properties that contain default state values */
        void extendInitStateFromProperties(SimTK::State &s) const override;

        /** use the current values in the state to update any properties such as
            default values for state variables */
        void extendSetPropertiesFromState(const SimTK::State &s) override;

        /** Computes state variable derivatives */
        void computeStateVariableDerivatives(const SimTK::State& s)
                const override;



//=============================================================================
// DATA
//=============================================================================



    private:
        /** construct the new properties and set their default values */
        void constructProperties();

    };  // END of class VandenBogert2011Muscle
} // end of namespace OpenSim
#endif // OPENSIM_THELEN_2003_MUSCLE_H_
