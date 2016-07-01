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
//    "Maximum isometric force that the fibers can generate");                                  //Fmax (N)

        OpenSim_DECLARE_PROPERTY(strain_at_max_iso_force_SEE, double,
        "Strain in the series elastic element at load of Fmax");                                    //umax (dimensionless)

        OpenSim_DECLARE_PROPERTY(fl_width, double,
        "Width parameter of the force-length relationship of the contractile element");             //W (dimensionless)

        OpenSim_DECLARE_PROPERTY(fv_AHill, double,
        "Hill parameter of the force-velocity relationship");                                       //AHill

        OpenSim_DECLARE_PROPERTY(fv_max_multiplier, double,
        "Maximal eccentric force multiplier");                                                      //FVmax

        //OpenSim_DECLARE_PROPERTY(optimal_fiber_length, double,    <-----Comes from parent class, Muscle.h
        //    "Optimal Length of Contractile Element");                                                 //Lceopt (m)

        OpenSim_DECLARE_PROPERTY(dampingCoeff_Pee, double,
        "Damping coefficient of damper parallel to the CE (normalized to Fmax)");                   //b (s/m)

        //OpenSim_DECLARE_PROPERTY(tendon_slack_length, double,     <-----Comes from parent class, Muscle.h
        //   "Slack length of the series elastic element");                                            //SEELslack (m)


        OpenSim_DECLARE_PROPERTY(length_slack_Pee, double,
                                 "(dimensionless) slack length of the parallel elastic element, divided by Lceopt");                   //b (s/m)




        OpenSim_DECLARE_PROPERTY(t_act, double,
        "Activation time(s)");                         //Tact (s)

        OpenSim_DECLARE_PROPERTY(t_deact, double,
        "Deactivation time(s)");                         //Tdeact (s)

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
        void setStrainAtMaxIsoForceSee(double StrainAtMaxIsoForceSee);
        double getStrainAtMaxIsoForceSee() const;

        void setFlWidth(double FlWidth);
        double getFlWidth() const;

        void setFvAHill(double FvAHill);
        double getFvAHill() const;

        void setFvMaxMultiplier(double FvMaxMultiplier);
        double getFvMaxMultiplier() const;

        void setDampingCoeffPee(double DampingCoeffPee);
        double getDampingCoeffPee() const;

        void setLengthSlackPee(double LengthSlackPee);
        double getLengthSlackPee() const;

        void setTact(double Tact);
        double getTact() const;

        void setTdeact(double Tdeact);
        double getTdeact() const;

/** default values for states
        double getDefaultActiveMotorUnits() const;
        void setDefaultActiveMotorUnits(double activeMotorUnits);
        double getDefaultFatiguedMotorUnits() const;
        void setDefaultFatiguedMotorUnits(double fatiguedMotorUnits);*/

//=============================================================================
// COMPUTATION
//=============================================================================

//std::array<double, 3> calcImplicitResidual(const SimTK::State& s) const;
        SimTK::Vec3 calcImplicitResidual(double Lm, double Lce, double a, double Lcedot, double adot, double u) const;

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

    private:
        /** construct the new properties and set their default values */
        void constructProperties();

    };  // END of class VandenBogert2011Muscle
} // end of namespace OpenSim
#endif // OPENSIM_THELEN_2003_MUSCLE_H_
