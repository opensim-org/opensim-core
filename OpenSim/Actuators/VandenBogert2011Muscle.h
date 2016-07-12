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
class VandenBogert2011Muscle : public Muscle {    //TODO Do I need: class OSIMACTUATORS_API VandenBogert2011Muscle......
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
        // phi_opt


//==============================================================================
// PUBLIC METHODS
//==============================================================================
        double computeActuation(const SimTK::State& s) const override;
        void computeInitialFiberEquilibrium(SimTK::State& s) const override;
        void setActivation(SimTK::State& s,double activation) const override;

//=============================================================================
// Construction
//=============================================================================
        /**Default constructor: produces a non-functional empty muscle*/
        VandenBogert2011Muscle();
        explicit VandenBogert2011Muscle(const std::string &name);


        // TODO: Uses default (compiler-generated) destructor, copy constructor, copy assignment operator?


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

        //struct ImplicitResults;

        struct ImplicitResidual {
            double forceResidual = SimTK::NaN;
            double activResidual = SimTK::NaN;
            double forceTendon = SimTK::NaN;
            SimTK::Mat23 df_dy = {SimTK::NaN,SimTK::NaN,SimTK::NaN,SimTK::NaN,SimTK::NaN,SimTK::NaN};
            SimTK::Mat23 df_dydot = {SimTK::NaN,SimTK::NaN,SimTK::NaN,SimTK::NaN,SimTK::NaN,SimTK::NaN};
            double df_du = SimTK::NaN;
            double F1 = SimTK::NaN;
            double F2 = SimTK::NaN;
            double F3 = SimTK::NaN;
            double F4 = SimTK::NaN;
            double F5 = SimTK::NaN;



        };


//=============================================================================
// COMPUTATION
//=============================================================================

        //ImplicitResults calcImplicitResidual(double muslceLength, double fiberLength, double activ, double fiberVelocity, double activ_dot, double u, int returnJacobians) const;
        ImplicitResidual calcImplicitResidual(double muslceLength, double projFiberLength, double activ, double projFiberVelocity, double activ_dot, double u, int returnJacobians) const;
        ImplicitResidual calcImplicitResidual(SimTK::Vec3 y,SimTK::Vec3 ydot, double u, int returnJacobians=0) const;

        ImplicitResidual    calcJacobianByFiniteDiff(SimTK::Vec3 y,SimTK::Vec3 ydot, double u, double h=1e-7 ) const;

    protected:
//=============================================================================
// PROTECTED METHODS
//=============================================================================



        // Model Component Interface
        /** add new dynamical states to the multibody system corresponding
            to this muscle */
        void extendAddToSystem(SimTK::MultibodySystem &system) const override;

        /** initialize muscle state variables from properties. For example, any
            properties that contain default state values */
        void extendInitStateFromProperties(SimTK::State &s) const override;

        /** use the current values in the state to update any properties such as
            default values for state variables */
        void extendSetPropertiesFromState(const SimTK::State &s) override;






//=============================================================================
// DATA
//=============================================================================



    private:
        /** construct the new properties and set their default values */
        void constructProperties();

    };  // END of class VandenBogert2011Muscle
} // end of namespace OpenSim
#endif // OPENSIM_THELEN_2003_MUSCLE_H_
