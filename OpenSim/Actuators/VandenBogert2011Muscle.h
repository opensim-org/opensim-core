#ifndef OPENSIM_VANDENBOGERT2011MUSCLE_H_
#define OPENSIM_VANDENBOGERT2011MUSCLE_H_
/* -------------------------------------------------------------------------- *
 *                 OpenSim:  VandenBogert2011Muscle.h                         *
 * -------------------------------------------------------------------------- */


// INCLUDE
#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <simbody/internal/common.h>
#include <OpenSim/Simulation/Model/Muscle.h>



namespace OpenSim {

// Derive new class from Muscle.h
class OSIMACTUATORS_API VandenBogert2011Muscle : public Muscle {    //TODO Do I need: class OSIMACTUATORS_API VandenBogert2011Muscle......
        OpenSim_DECLARE_CONCRETE_OBJECT(VandenBogert2011Muscle, Muscle);

public:
//=============================================================================
// PROPERTIES
//=============================================================================
/*
The parent class, Muscle.h, provides
    1. max_isometric_force
    2. optimal_fiber_length
    3. tendon_slack_length
    4. pennation_angle_at_optimal
    5. max_contraction_velocity
*/

//OpenSim_DECLARE_PROPERTY(max_isometric_force, double,  <-----Comes from parent class, Muscle.h
//    "Maximum isometric force that the fibers can generate");
// F_{max} (N)

        OpenSim_DECLARE_PROPERTY(fMaxTendonStrain, double,
        "tendon strain at maximum isometric muscle force");
        //u_{max} (dimensionless) strain in the series elastic element at load of maxIsometricForce

        OpenSim_DECLARE_PROPERTY(fl_width, double,
        "force-length shape factor");
        //W (dimensionless) width parameter of the force-length relationship of the muscle fiber

        OpenSim_DECLARE_PROPERTY(fv_AHill, double,
        "force-velocity shape factor");
        //AHill (dimensionless) Hill parameter of the force-velocity relationship

        OpenSim_DECLARE_PROPERTY(fv_maxMultiplier, double,
        "maximum normalized lengthening force");
        //FV_{max} (dimensionless) maximal eccentric force

        //OpenSim_DECLARE_PROPERTY(optimal_fiber_length, double,    <-----Comes from parent class, Muscle.h
        //    "Optimal Length of Contractile Element"                                               //Lceopt (m)

        OpenSim_DECLARE_PROPERTY(dampingCoefficient, double,
        "The linear damping of the fiber");
        //b (s/m) damping coefficient of damper parallel to the fiber (normalized to maxIsometricForce)

        OpenSim_DECLARE_PROPERTY(normFiberSlackLength, double,
                                 "(dimensionless) slack length of the parallel elastic element, divided by Lceopt");
        //L_{slack,fiber}(dimensionless) slack length of the fiber (PEE)


        OpenSim_DECLARE_PROPERTY(activTimeConstant, double,
        "Activation time(s)");
        //T_{act} (s) Activation time

        OpenSim_DECLARE_PROPERTY(deactivTimeConstant, double,
        "Deactivation time(s)");
        //T_{deact} (s) Deactivation time

        OpenSim_DECLARE_PROPERTY(pennAtOptFiberLength, double,
                                 "pennation at optimal fiber length equilbrium");

        OpenSim_DECLARE_PROPERTY(defaultActivation, double,
                                 "default activation");

        OpenSim_DECLARE_PROPERTY(default_fiber_length, double,
                                 "Assumed initial fiber length if none is assigned.");


//=============================================================================
// Construction
//=============================================================================
        /**Default constructor: produces a non-functional empty muscle*/
        VandenBogert2011Muscle();



        VandenBogert2011Muscle(const std::string& name);

        // TODO: Uses default destructor, copy constructor, copy assignment operator?


//-------------------------------------------------------------------------
// GET & SET Properties
//-------------------------------------------------------------------------
        // Properties
        void setFMaxTendonStrain(double fMaxTendonStrain);
        double getFMaxTendonStrain() const;

        void setFlWidth(double flWidth);
        double getFlWidth() const;

        void setFvAHill(double fvAHill);
        double getFvAHill() const;

        void setFvmaxMultiplier(double fvMaxMultiplier);
        double getFvmaxMultiplier() const;

        void setDampingCoefficient(double dampingCoefficient);
        double getDampingCoefficient() const;

        void setNormFiberSlackLength(double normFiberSlackLength);
        double getNormFiberSlackLength() const;

        void setActivTimeConstant(double activTimeConstant);
        double getActivTimeConstant() const;

        void setDeactivTimeConstant(double deactivTimeConstant);
        double getDeactivTimeConstant() const;

        void setPennAtOptFiberLength(double pennAtOptFiberLength);
        double getPennAtOptFiberLength() const;

        void setDefaultActivation(double defaultActivation);
        double getDefaultActivation() const;

        void setDefaultFiberLength(double fiberLength);
        double getDefaultFiberLength() const;

        void setActivation(SimTK::State& s,double activation) const override;
        void setFiberLength(SimTK::State& s,double fiberLength) const;


        SimTK::Vec2 getResidual(const SimTK::State& s, double projFibVelNorm_guess, double activdot_guess, double u) const;

        //Hacks because cache variables not implmented yet
        double  getFiberLength(SimTK::State& s) const;
        double  getActivation(SimTK::State& s) const;



        struct ImplicitResidual {
            double forceResidual = SimTK::NaN;
            double activResidual = SimTK::NaN;
            double forceTendon = SimTK::NaN;
            SimTK::Mat22 df_dy = {SimTK::NaN,SimTK::NaN,SimTK::NaN,SimTK::NaN,SimTK::NaN,SimTK::NaN};
            SimTK::Mat22 df_dydot = {SimTK::NaN,SimTK::NaN,SimTK::NaN,SimTK::NaN,SimTK::NaN,SimTK::NaN};
            SimTK::Vec2 df_du = {SimTK::NaN,SimTK::NaN};
            double df_dmuscleLength = SimTK::NaN;
            double F1 = SimTK::NaN;
            double F2 = SimTK::NaN;
            double F3 = SimTK::NaN;
            double F4 = SimTK::NaN;
            double F5 = SimTK::NaN;



        };

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

        //ImplicitResults calcImplicitResidual(double muslceLength, double fiberLength, double activ, double fiberVelocity, double activ_dot, double u, int returnJacobians) const;
        ImplicitResidual calcImplicitResidual(double muslceLength, double projFiberLength, double activ, double projFiberVelocity, double activ_dot, double u, int returnJacobians) const;
        ImplicitResidual calcImplicitResidual(SimTK::Vec2 y,SimTK::Vec2 ydot_guess, double muscleLength, double u, int returnJacobians) const;
        ImplicitResidual calcImplicitResidual(const SimTK::State& s, double projFibVel_guess, double activdot_guess, double u, int returnJacobians) const;


        double fiberLengthToProjectedLength (double fiberLength , bool normalizedValues) const;
        double projFibLenToFiberLength (double projFibLen, bool normalizedValues) const;

        double fiberVelocityToProjFibVel (double fiberVelocity, double fiberLength, double projFiberLength, bool normalizedValues) const;
        double fiberVelocityToProjFibVel (double fiberVelocity, double fiberLength,  bool normalizedValues) const;

        double projFibVelToFiberVelocity(double projFibVel, double fiberLength, double projFiberLength, bool normalizedValues) const;
        double projFibVelToFiberVelocity(double projFibVel, double projFiberLength, bool normalizedValues) const;





        ImplicitResidual calcJacobianByFiniteDiff(SimTK::Vec2 y,SimTK::Vec2 ydot, double muscleLength, double u, double h ) const;
        ImplicitResidual calcJacobianByFiniteDiff(double muscleLength, double projFibLenNorm, double activ,
                                                  double projFibVelNorm, double activdot, double u,
                                                  double h) const;


        SimTK::Vec2 calcFiberStaticEquilbirum(double muscleLength, double activ) const;
        SimTK::Vec3 calcFiberStaticEquilibResidual(double projFibLen, double muscleLength, double activ) const;



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

    /* Not implmented
        void calcMuscleLengthInfo(const SimTK::State& s,
                                  MuscleLengthInfo& mli) const override;


        void calcFiberVelocityInfo(const SimTK::State& s,
                                   FiberVelocityInfo& fvi) const override;


        void calcMuscleDynamicsInfo(const SimTK::State& s,
                                    MuscleDynamicsInfo& mdi) const override;


        void  calcMusclePotentialEnergyInfo(const SimTK::State& s,
                                            MusclePotentialEnergyInfo& mpei) const override;
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
        void computeStateVariableDerivatives(const SimTK::State& s) const override;



        // this is a hack to work around not using mdi/mli......
        //SimTK::Vec2 getStateVariables(const SimTK::State& s);


//=============================================================================
// DATA
//=============================================================================



    private:
        /** construct the new properties and set their default values */
        void constructProperties();

    };  // END of class VandenBogert2011Muscle
} // end of namespace OpenSim
#endif // OPENSIM_THELEN_2003_MUSCLE_H_
