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

/*
#ifdef SWIG
#ifdef OSIMACTUATORS_API
        #undef OSIMACTUATORS_API
        #define OSIMACTUATORS_API
    #endif
#endif
*/


namespace OpenSim {
/**
This class implements a muscle with implicit dynamics capabilities as described
 in van den Bogert et al. \(2011).
 */
    //TODO:  Add detailed muscle discription

// Derive new class from Muscle.h
class OSIMACTUATORS_API VandenBogert2011Muscle : public Muscle {
//TODO Do I need: class OSIMACTUATORS_API VandenBogert2011Muscle......
        OpenSim_DECLARE_CONCRETE_OBJECT(VandenBogert2011Muscle, Muscle);

public:
//=============================================================================
// PROPERTIES
//=============================================================================

        //OpenSim_DECLARE_PROPERTY(max_isometric_force, double,
        //  F_{max} (N) "Maximum isometric force that the fibers can generate");
        // Comes from parent class, Muscle.h

        OpenSim_DECLARE_PROPERTY(fMaxTendonStrain, double,
        "tendon strain at maximum isometric muscle force");
        //u_{max} (dimensionless) strain in the series elastic element at load
        // of maxIsometricForce

        OpenSim_DECLARE_PROPERTY(fl_width, double,
        "force-length shape factor");
        //W (dimensionless) width parameter of the force-length relationship of
        // the muscle fiber

        OpenSim_DECLARE_PROPERTY(fv_AHill, double,
        "force-velocity shape factor");
        //AHill (dimensionless) Hill parameter of the force-velocity
        // relationship

        OpenSim_DECLARE_PROPERTY(fv_maxMultiplier, double,
        "maximum normalized lengthening force");
        //FV_{max} (dimensionless) maximal eccentric force

        //OpenSim_DECLARE_PROPERTY(optimal_fiber_length, double,
        //    "Optimal Length of Contractile Element"
        //Lceopt (m)
        // Comes from parent class, Muscle.h

        OpenSim_DECLARE_PROPERTY(dampingCoefficient, double,
        "The linear damping of the fiber");
        //b (s/m) damping coefficient of damper parallel to the fiber
        //      (normalized to maxIsometricForce)

        OpenSim_DECLARE_PROPERTY(normFiberSlackLength, double,
                                 "(dimensionless) slack length of the parallel "
                                         "elastic element, divided by Lceopt");
        //L_{slack,fiber}(dimensionless) slack length of the fiber (PEE)


        OpenSim_DECLARE_PROPERTY(activTimeConstant, double,
        "Activation time(s)");
        //T_{act} (s) Activation time

        OpenSim_DECLARE_PROPERTY(deactivTimeConstant, double,
        "Deactivation time(s)");
        //T_{deact} (s) Deactivation time

        OpenSim_DECLARE_PROPERTY(pennAtOptFiberLength, double,
                                 "pennation at optimal fiber length "
                                         "equilibrium");

        OpenSim_DECLARE_PROPERTY(defaultActivation, double,
                                 "default activation");

        OpenSim_DECLARE_PROPERTY(default_fiber_length, double,
                                 "Assumed initial fiber length if none is "
                                         "assigned.");


//=============================================================================
// Construction
//=============================================================================
        /**Default constructor: produces a non-functional empty muscle*/
        VandenBogert2011Muscle();

        /** Constructs a functional muscle using default parameters.
         @param name The name of the muscle.*/
        VandenBogert2011Muscle(const std::string& name);


// TODO: Add MaxIsometricForce, OptimalFiberLength, TendonSlackLength,
//      PennationAngle  to constructor


//-------------------------------------------------------------------------
// GET & SET Properties
//-------------------------------------------------------------------------
        /** @param the tendon strain at a load of Fmax*/
        void setFMaxTendonStrain(double fMaxTendonStrain);
        /** @returns the tendon strain at a load of Fmax*/
        double getFMaxTendonStrain() const;

        /** @param the width parameter (W) of the force-length relationship of
        the muscle fiber (dimensionless)*/
        void setFlWidth(double flWidth);
        /** @returns the width parameter (W) of the force-length relationship of
        the muscle fiber (dimensionless)*/
        double getFlWidth() const;

        /** @param the Hill parameter of the force-velocity relationship
        (dimensionless)*/
        void setFvAHill(double fvAHill);
        /** @returns the Hill parameter of the force-velocity relationship
        (dimensionless)*/
        double getFvAHill() const;

        /** @param the maximal eccentric force multiplier FV_{max}
        (dimensionless) */
        void setFvmaxMultiplier(double fvMaxMultiplier);
        /** @returns the maximal eccentric force multiplier FV_{max}
        (dimensionless) */
        double getFvmaxMultiplier() const;

        /** @param the  damping coefficient of damper parallel to the fiber
        (normalized to maxIsometricForce), b (s/m)*/
        void setDampingCoefficient(double dampingCoefficient);
        /** @returns the  damping coefficient of damper parallel to the fiber
        (normalized to maxIsometricForce), b (s/m)*/
        double getDampingCoefficient() const;

        /** @param the slack length of the parallel elastic element, divided by
        fiber optimal length, L_{slack,fiber} (dimensionless)*/
        void setNormFiberSlackLength(double normFiberSlackLength);
        /** @returns the slack length of the parallel elastic element, divided
        by fiber optimal length, L_{slack,fiber} (dimensionless)*/
        double getNormFiberSlackLength() const;

        /** @param the activation time constant, T_{act} (s)*/
        void setActivTimeConstant(double activTimeConstant);
        /** @returns the activation time constant, T_{act} (s)*/
        double getActivTimeConstant() const;

        /** @param the deactivation time constant, T_{deact} (s)*/
        void setDeactivTimeConstant(double deactivTimeConstant);
        /** @returns the deactivation time constant, T_{deact} (s)*/
        double getDeactivTimeConstant() const;

        /** @param the pennation angel at optimal fiber length,
        phi_{opt} (rad)*/
        void setPennAtOptFiberLength(double pennAtOptFiberLength);
        /** @returns the pennation angel at optimal fiber length,
        phi_{opt} (rad)*/
        double getPennAtOptFiberLength() const;

        /** @param the default muscle activation, (dimensionless)*/
        void setDefaultActivation(double defaultActivation);
        /** @returns the default muscle activation, (dimensionless)*/
        double getDefaultActivation() const;

        /** @param the default muscle fiber length, ()*/
        void setDefaultFiberLength(double fiberLength);
        /** @returns the default muscle fiber length,, ()*/
        double getDefaultFiberLength() const;


        /**@param s The state of the system.
        @param the muscle activation, (dimensionless)*/
        void setActivation(SimTK::State& s,double activation) const override;
        /**@param s The state of the system.
        @param fiberLength the muscle fiber length, ()*/
        void setFiberLength(SimTK::State& s,double fiberLength) const;

        /* Calculate the muscle implicit residual.  Returns a state Vec2
        containing the force residual and activation residual.  When these
        value are equal to zero, the muscle equations are "balanced" and the
         muscle  is muscle is in a valid state.
                @param s The state of the system.
                @param projFibVelNorm_guess The muscle fiber velocity, projected
                in line with tendon and normalized by the optimal fiber length,
                (dimensionless)
                @param activdot_guess The muscle activation time derivative,
                (1/s)
                @returns a vector of the muscle residual:
                        \li 0: force (N)
                        \li 1: activation (unitless)*/
        SimTK::Vec2 getResidual(const SimTK::State& s,
                                double projFibVelNorm_guess,
                                double activdot_guess, double u) const;


        //Hacks because cache variables not implemented yet

        /**@param The state of the system.
           @returns The Fiber Length of the muscle.*/
        double  getFiberLength(SimTK::State& s) const;
        /**@param The state of the system.
           @returns The activation of the muscle.*/
        double  getActivation(SimTK::State& s) const;


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
        /** @param[in] s The state of the system.
            @returns The tensile force the muscle is generating (N). */
        double computeActuation(const SimTK::State& s) const override final;

        /** Computes the fiber length such that the fiber and tendon are
        developing the same force, distributing the velocity of the entire
        musculotendon actuator between the fiber and tendon according to their
        relative stiffnesses.
                @param[in,out] s The state of the system. */
        void computeInitialFiberEquilibrium(SimTK::State& s) const override;

        /** Computes the fiber length such that the fiber and tendon are
        developing the same force, assuming velocities are zero. This is a
        static equilibrium version of computeInitialFiberEquilibrium(). By
        setting velocities to zero, we obtain a reasonable and robust solution
        that provides a rough solution for fiber length.
                @param[in,out] s The state of the system. */
        void computeFiberEquilibriumAtZeroVelocity(SimTK::State& s) const
                override;


//=============================================================================
// COMPUTATION
//=============================================================================

    // TODO: These should move to protected at some point

        // Methods to calculate muscle residual:


        /** Calculates the muscle residuals.
                @param muscleLength length of muscle (N)
                @param projFibLength length of the muscle fiber projected inline
                        with tendon (m)
                @param activ muscle activation
                @param projFiberVelocity velocity of the muscle fiber projected
                        inline with tendon (m/s)
                @param activ_dot time derivative of the muslce activation
                @param u muscle control (excitation)
                @param returnJacobians if=1, calculate and return the
                        Jacobains. default is =0, no Jacobians not calculated
                @returns TBD for now see ImplicitResidual structure*/
        ImplicitResidual calcImplicitResidual(double muslceLength,
                                              double projFiberLength,
                                              double activ,
                                              double projFiberVelocity,
                                              double activ_dot,
                                              double u,
                                              int returnJacobians) const;
        /** Calculates the muscle residuals. (State equation form for vector
        manipulation)
                @param y vector of:
                        \li 0: projected fiber length
                        \li 1: muscle activation
                @param ydot_guess Vector of the guess for the derivative of y
                @param muscleLength length of muscle (N)
                @param u muscle control (excitation)
                @param returnJacobians if=1, calculate and return the
                        Jacobains. default is =0, no Jacobians not calculated
                @returns TBD for now see ImplicitResidual structure*/
        ImplicitResidual calcImplicitResidual(SimTK::Vec2 y,
                                              SimTK::Vec2 ydot_guess,
                                              double muscleLength,
                                              double u,
                                              int returnJacobians) const;

        /** Calculates the muscle residuals. (method for getting from
        state)
                @param s The state of the system. (provides projected fiber
                        length and muscle activation)
                @param projFibVel_guess Guess of the fiber velocity
                        (projected in-line with the tendon)
                @param activdot_guess Guess of the time derivative of activation
                @param u muscle control (excitation)
                @param returnJacobians if=1, calculate and return the
                        Jacobains. default is =0, no Jacobians not calculated
                @returns TBD for now see ImplicitResidual structure*/
        ImplicitResidual calcImplicitResidual(const SimTK::State& s,
                                              double projFibVel_guess,
                                              double activdot_guess,
                                              double u,
                                              int returnJacobians);

        //Convience methods for converting between muscle length, projected
        // muscle length, and normalized values.  Need to do this because I am
        // sticking with the convention that external to the muscle,
        // muscle length is the state variable.  Inside this muscle's code it
        // is using the projected muscle length.

        //TODO: Maybe cache variables?

        /** @param fiberLength The fiber length
            @param normalizedValues T: The values are normalized by the muscle's
                optimal length, F: Not normalized
            @returns The projected fiber length
                (will be nomralized if normalizedValue=T)*/
        double fiberLengthToProjectedLength (double fiberLength ,
                                             bool normalizedValues) const;


        /** @param projFibLength The projected (inline with tendon) length
            @param normalizedValues T: The values provided are normalized by the
            muscle's optimal length, F: Not normalized
            @returns The fiber length
            (will be nomralized if normalizedValue=T)*/
        double projFibLenToFiberLength (double projFibLen,
                                        bool normalizedValues) const;

        /**@param fiberVelocity Velocity of the fiber
           @param fiberLength Fiber Length
           @param projFiberLength projected fiber velocity (in-line with tendon)
           @param normalizedValues T: The values provided are normalized by the
           muscle's optimal length, F: Not normalized
           @returns The projected fiber velocity
                (will be nomralized if normalizedValue=T)*/
        double fiberVelocityToProjFibVel (double fiberVelocity,
                                          double fiberLength,
                                          double projFiberLength,
                                          bool normalizedValues) const;

        /**@param fiberVelocity Velocity of the fiber
           @param fiberLength Fiber Length
           @param normalizedValues T: The values provided are normalized by the
           muscle's optimal length, F: Not normalized
           @returns The projected fiber velocity
           (will be nomralized if normalizedValue=T)*/
        double fiberVelocityToProjFibVel (double fiberVelocity,
                                          double fiberLength,
                                          bool normalizedValues) const;

        /**@param projFibVel Projected velocity of the fiber
           @param fiberLength Fiber Length
           @param projFiberLength projected fiber velocity (in-line with tendon)
           @param normalizedValues T: The values provided are normalized by the
           muscle's optimal length, F: Not normalized
           @returns The fiber velocity
           (will be nomralized if normalizedValue=T)*/
        double projFibVelToFiberVelocity(double projFibVel, double fiberLength,
                                         double projFiberLength,
                                         bool normalizedValues) const;

        /**@param projFibVel Projected velocity of the fiber
           @param projFiberLength projected fiber velocity (in-line with tendon)
           @param normalizedValues T: The values provided are normalized by the
           muscle's optimal length, F: Not normalized
           @returns The fiber velocity
          (will be nomralized if normalizedValue=T)*/
        double projFibVelToFiberVelocity(double projFibVel,
                                         double projFiberLength,
                                         bool normalizedValues) const;


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
              @param u The muscle control (excitation)
              @param h The peturbation size of the finite step
              @returns TBD for now see ImplicitResidual structure*/
        ImplicitResidual calcJacobianByFiniteDiff(SimTK::Vec2 y,
                                                  SimTK::Vec2 ydot,
                                                  double muscleLength,
                                                  double u, double h ) const;
        /** Calculate Jacobian by finite difference
              @param muscleLength Muscle Length
              @param projFibLenNorm The projected fiber length (inline with
              tendon) and normalized by muscle optimal length.
              @param activ The muscle activation
              @param projFibVelNorm The projected fiber velocity (inline with
              tendon) and normalized by muscle optimal length.
              @param activdot The time derivative of the muscle activation
              @param u The muscle control (excitation)
              @param h The peturbation size of the finite step
              @returns TBD for now see ImplicitResidual structure*/
        ImplicitResidual calcJacobianByFiniteDiff(double muscleLength,
                                                  double projFibLenNorm,
                                                  double activ,
                                                  double projFibVelNorm,
                                                  double activdot, double u,
                                                  double h) const;




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




        SimTK::Vector calcSolveMuscle(
                SimTK::State& s, double activ, SimTK::Vector yDotInitialGuess)
                                const;

        class ImplicitSystemDerivativeSolver : public SimTK::OptimizerSystem {};

        //Hacks for troubleshooting Mat22:
        SimTK::Mat22  fixMat22(SimTK::Mat22 matIn,SimTK::Mat22 matFixed) const;
        SimTK::Mat33 quickMat33() const;
        SimTK::Mat22 quickMat22() const;
        SimTK::Vec4 quickVec4() const;
        SimTK::Vec4 flattenMat22(SimTK::Mat22 matIn) const;



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
        void extendConnectToModel(Model& model) override;

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
