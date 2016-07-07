/* -------------------------------------------------------------------------- *
 *                      OpenSim:  VandenBogert.cpp                            *
 * -------------------------------------------------------------------------- */


//=============================================================================
// INCLUDES
//=============================================================================
#include "VandenBogert2011Muscle.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/*
 * Default constructor
 */
VandenBogert2011Muscle::VandenBogert2011Muscle()
{
    constructProperties();
}

//_____________________________________________________________________________
/*
 * Constructor.
 */
VandenBogert2011Muscle::VandenBogert2011Muscle(const std::string &name)
{
    constructProperties();
}

//_____________________________________________________________________________
/*
 * Construct and initialize properties.
 * All properties are added to the property set. Once added, they can be
 * read in and written to files.
 */
void VandenBogert2011Muscle::constructProperties()
{
    setAuthors("A. van den Bogert, B. Humphreys");
    //constructProperty_max_isometric_force(1000);
    constructProperty_strain_at_max_iso_force_SEE(0.04);
    constructProperty_fl_width(0.63);
    constructProperty_fv_AHill(0.25);
    constructProperty_fv_max_multiplier(1.4);
    //constructProperty_optimal_fiber_length(0.1);
    constructProperty_dampingCoeff_Pee(0.01);
    constructProperty_length_slack_Pee(1.0);
    //constructProperty_tendon_slack_length(0.2);
    constructProperty_t_act(0.01);
    constructProperty_t_deact(0.04);

}

// Define new states and their derivatives in the underlying system
void VandenBogert2011Muscle::extendAddToSystem(SimTK::MultibodySystem& system) const
{

    // No States to add, yet
    /* Allow Millard2012EquilibriumMuscle to add its states, before extending
    Super::extendAddToSystem(system);

    // Now add the states necessary to implement the fatigable behavior
    addStateVariable("target_activation");
    addStateVariable("active_motor_units");
    addStateVariable("fatigued_motor_units");
    // and their corresponding derivatives
    addCacheVariable("target_activation_deriv", 0.0, SimTK::Stage::Dynamics);
    addCacheVariable("active_motor_units_deriv", 0.0, SimTK::Stage::Dynamics);
    addCacheVariable("fatigued_motor_units_deriv", 0.0, SimTK::Stage::Dynamics);*/
}

void VandenBogert2011Muscle::extendInitStateFromProperties(SimTK::State& s) const
{
    // No States Yet
    /*Super::extendInitStateFromProperties(s);
    setTargetActivation(s, getDefaultTargetActivation());
    setActiveMotorUnits(s, getDefaultActiveMotorUnits());
    setFatiguedMotorUnits(s, getDefaultFatiguedMotorUnits());*/
}

void VandenBogert2011Muscle::extendSetPropertiesFromState(const SimTK::State& s)
{
    // No States Yet
    /*Super::extendSetPropertiesFromState(s);
    setDefaultTargetActivation(getTargetActivation(s));
    setDefaultActiveMotorUnits(getActiveMotorUnits(s));
    setDefaultFatiguedMotorUnits(getFatiguedMotorUnits(s));*/
}

//--------------------------------------------------------------------------
// GET & SET Properties
//--------------------------------------------------------------------------
void VandenBogert2011Muscle::setStrainAtMaxIsoForceSee(double StrainAtMaxIsoForceSee) {
        set_strain_at_max_iso_force_SEE(StrainAtMaxIsoForceSee);
    }
double VandenBogert2011Muscle::getStrainAtMaxIsoForceSee() const
    { return get_strain_at_max_iso_force_SEE(); }

void VandenBogert2011Muscle::setFlWidth(double FlWidth)
    { set_fl_width(FlWidth); }
double VandenBogert2011Muscle::getFlWidth() const
    { return get_fl_width(); }

void VandenBogert2011Muscle::setFvAHill(double FvAHill)
    { set_fv_AHill(FvAHill); }
double VandenBogert2011Muscle::getFvAHill() const
    { return get_fv_AHill(); }

void VandenBogert2011Muscle::setFvMaxMultiplier(double FvMaxMultiplier)
    { set_fv_max_multiplier(FvMaxMultiplier); }
double VandenBogert2011Muscle::getFvMaxMultiplier() const
    { return get_fv_max_multiplier(); }

void VandenBogert2011Muscle::setDampingCoeffPee(double DampingCoeffPee)
    { set_dampingCoeff_Pee(DampingCoeffPee); }
double VandenBogert2011Muscle::getDampingCoeffPee() const
    { return get_dampingCoeff_Pee(); }

void VandenBogert2011Muscle::setLengthSlackPee(double LengthSlackPee)
    { set_length_slack_Pee(LengthSlackPee); }
double VandenBogert2011Muscle::getLengthSlackPee() const
    { return get_length_slack_Pee(); }

void VandenBogert2011Muscle::setTact(double Tact)
    { set_t_act(Tact); }
double VandenBogert2011Muscle::getTact() const
    { return get_t_act(); }

void VandenBogert2011Muscle::setTdeact(double Tdeact)
    { set_t_deact(Tdeact); }
double VandenBogert2011Muscle::getTdeact() const
    { return get_t_deact(); }




//--------------------------------------------------------------------------
// GET & SET States and their derivatives
//--------------------------------------------------------------------------

//No states yet
/*double FatigableMuscle::getTargetActivation(const SimTK::State& s) const
{   return getStateVariableValue(s, "target_activation"); }

void FatigableMuscle::setTargetActivation(SimTK::State& s,
                                          double fatiguedAct) const
{   setStateVariableValue(s, "target_activation", fatiguedAct); }

double FatigableMuscle::getTargetActivationDeriv(const SimTK::State& s) const
{   return getStateVariableDerivativeValue(s, "target_activation"); }

void FatigableMuscle::setTargetActivationDeriv(const SimTK::State& s,
                                               double fatiguedActDeriv) const
{   setStateVariableDerivativeValue(s, "target_activation", fatiguedActDeriv); }

double FatigableMuscle::getActiveMotorUnits(const SimTK::State& s) const
{   return getStateVariableValue(s, "active_motor_units"); }

void FatigableMuscle::setActiveMotorUnits(SimTK::State& s,
                                          double activeMotorUnits) const
{   setStateVariableValue(s, "active_motor_units", activeMotorUnits); }

double FatigableMuscle::getActiveMotorUnitsDeriv(const SimTK::State& s) const
{   return getStateVariableDerivativeValue(s, "active_motor_units"); }

void FatigableMuscle::setActiveMotorUnitsDeriv(const SimTK::State& s,
                                               double activeMotorUnitsDeriv) const
{   setStateVariableDerivativeValue(s, "active_motor_units", activeMotorUnitsDeriv); }

double FatigableMuscle::getFatiguedMotorUnits(const SimTK::State& s) const
{   return getStateVariableValue(s, "fatigued_motor_units"); }

void FatigableMuscle::setFatiguedMotorUnits(SimTK::State& s,
                                            double fatiguedMotorUnits) const
{   setStateVariableValue(s, "fatigued_motor_units", fatiguedMotorUnits); }

double FatigableMuscle::getFatiguedMotorUnitsDeriv(const SimTK::State& s) const
{    return getStateVariableDerivativeValue(s, "fatigued_motor_units"); }

void FatigableMuscle::setFatiguedMotorUnitsDeriv(const SimTK::State& s,
                                                 double fatiguedMotorUnitsDeriv) const
{   setStateVariableDerivativeValue(s, "fatigued_motor_units", fatiguedMotorUnitsDeriv);
 }*/





//==============================================================================
// Muscle.h Interface
//==============================================================================


double  VandenBogert2011Muscle::computeActuation(const SimTK::State& s) const
{return( 0.0);}

void VandenBogert2011Muscle::setActivation(SimTK::State& s,double activation) const
{}

void VandenBogert2011Muscle::computeInitialFiberEquilibrium(SimTK::State& s) const
{}




//=============================================================================
// COMPUTATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the derivatives of the muscle states.
 *
 * @param s  system state
 */


//array<double,3> calcImplicitResidual(const SimTK::State& s) const

//SimTK::Vec3 VandenBogert2011Muscle::calcImplicitResidual(double Lm, double Lce, double a, double Lcedot, double adot, double u, int returnJacobians) const {


VandenBogert2011Muscle::ImplicitResults VandenBogert2011Muscle::calcImplicitResidual(double Lm, double Lce, double a, double Lcedot, double adot, double u, int returnJacobians) const {

    // Later when using state variables:
    //State Variables
    //double Lm = getLength(s);
    //double Lce = getNormalizedFiberLength(s);  //Ton: Lce is dimensionless, it is the muscle fiber length divided by Lceopt
    //double a = getActivation(s);
    //double Lcedot = getNormalizedFiberVelocity(s);

    // Parameters
    double Fmax = getMaxIsometricForce();  //Maximum isometric force that the fibers can generate
    double umax = getStrainAtMaxIsoForceSee(); //Strain in the series elastic element at load of Fmax
    double W = getFlWidth();   //Width parameter of the force-length relationship of the contractile element
    double AHill = getFvAHill();        //Hill parameter of the force-velocity relationship
    double FVmax = getFvMaxMultiplier(); //Maximal eccentric force multiplier
    double Lceopt = getOptimalFiberLength();  //Optimal Length of Contractile Element
    double b = getDampingCoeffPee();     //Damping coefficient of damper parallel to the CE (normalized to Fmax)
    double SEELslack = getTendonSlackLength(); //Slack length of the series elastic element
    double PEELslack = getLengthSlackPee();  //Slack length of the parallel elastic element, divided by Lceopt
    double Tact = getTact();  // Activation time(s)
    double Tdeact = getTdeact();  // Deactivation time(s)

    // constants derived from the muscle parameters
    double Vmax = 10 * Lceopt;                 //Maximum shortening velocity (m/s) is 10 fiber lengths per sec
    double d1 = Vmax * AHill * (FVmax - 1) / (AHill + 1);  // parameter in the eccentric force-velocity equation


    double kPEE2 = 1 / pow(W, 2);                       // PEE quadratic stiffness, so Fpee = Fmax when Lce = Lce*(1+W)
    double kSEE2 = 1 / pow(SEELslack * umax, 2);  // SEE quadratic stiffness, so Fsee = Fmax at strain of umax


    // Jacobian Matrices
    SimTK::Mat23 dfdy;
    SimTK::Mat23 dfdydot;




    //----------------F1 is the normalized isometric force-length relationship at maximum activation--------------//
    double ff = (Lce - 1.0) / W;   // [dimensionless]
    double F1 = exp(pow(-ff, 2));        // Gaussian force-length curve

    double dF1_dLce = 0;
    if (returnJacobians) {
        double dF1_dLce = -2.0 * ff * F1 / W;
    }

    double F2 = 0.0;



    //----------------- F2 is the dimensionless force-velocity relationship --------------------------------//
    double dF2_dLcedot = 0;
    if (Lcedot < 0) {
        //concentric contraction
        ff = Vmax - Lcedot / AHill;
        F2 = (Vmax + Lcedot) / ff;
        if (returnJacobians) {
            double dF2_dLcedot = (1.0 + F2 / AHill) / ff;
        }
    }
    else {
        //eccentric contraction
        double c = Vmax * AHill * (FVmax - 1.0) / (AHill + 1.0); // parameter in the eccentric force-velocity equation
        ff = Lcedot + c;
        F2 = (FVmax * Lcedot + c) / ff;
        if (returnJacobians) {
            double dF2_dLcedot = (FVmax - F2) / ff;
        }
    }



    //----------F3 is the dimensionless PEE force (in units of Fmax)-------------//
    double kPEE = 1.0 / Fmax * Lceopt;      // stiffness of the linear term is 1 N/m, convert to Fmax/Lceopt units
    ff = (Lce - PEELslack);             // elongation of PEE, relative to Lceopt
    double F3 = kPEE * ff;                // low stiffness linear term
    double dF3_dLce = kPEE;
    if (ff > 0) {
        //add quadratic term for positive elongation
        F3 = F3 + kPEE2 * pow(ff, 2);
        if (returnJacobians) {
            double F3_dLce = dF3_dLce + 2 * kPEE2 * ff;
        }
    }




    //---------------F4 is the dimensionless SEE force (in units of Fmax)----------//
    double kSEE = 1.0 / Fmax;                     // stiffness of the linear term is 1 N/m, convert to Fmax/m
    ff = Lm - Lce * Lceopt - SEELslack;  // elongation of SEE, in meters
    double F4 = kSEE * ff;              //  low stiffness linear term
    double dF4_dLce = 0;
    double dF4_dLm = 0;
    if (returnJacobians) {
        double dF4_dLce = -kSEE * Lceopt;
        double dF4_dLm = kSEE;
    }
    if (ff > 0) {
        // add quadratic term for positive deformation
        F4 = F4 + kSEE2 * pow(ff, 2);
        if (returnJacobians) {
            double dF4_dLce = dF4_dLce - 2 * kSEE2 * Lceopt * ff;
            double dF4_dLm = dF4_dLm + 2 * kSEE2 * ff;
        }
    }



    //-------------- F5 is viscous damping parallel to the CE (0.001 of Fmax at 1 Lceopt/s) to  -----------//
    // ensure that df/dLcedot is never zero
    double F5 = 0.001 * Lcedot;    // TODO:  is 0.001 suppossed to b?
    double dF5_dLcedot = 0.001;

    //The muscle dynamics equation: f = Fsee - Fce - Fpee - Fdamping = 0
    double muscleForceResidual = F4 - a * F1 * F2 - F3 - F5;


    if (returnJacobians) {
        dfdy[0][2] = -F1 * F2;                              // df/da
        dfdy[0][1] = dF4_dLce - a * dF1_dLce * F2 - dF3_dLce;    // df/dLce
        dfdydot[0][1] = -a * F1 * dF2_dLcedot - dF5_dLcedot;        // df/dLcedot
        double df_dLm = dF4_dLm;
        //dfdy[1][1] = -(d / L) * df_dLm;                      // derivative of f with respect to muscle length
        dfdy[0][0] = -df_dLm;
    }



    // ---------------- Force in SEE is Fmax*F4 ---------------------------- //
    double Fsee = Fmax * F4;

    // TODO: I don't think we need to implement dFsee_.....
    /*if (nargout > 1)
        dFsee_dLm  = Fmax * dF4_dLm;
    dFsee_dLce = Fmax * dF4_dLce;
    dFsee_dy   = [0;0;(-d/L)*dFsee_dLm; 0;dFsee_dLce;0;0];  % Fsee is a function of y(3) & y(5)
    end*/



    //----------------------Activation dynamics equation-------------------//
    double activationResidual = adot - (u - a) * (u / Tact + (1 - u) / Tdeact);
    if (returnJacobians) {
        dfdy[1][2] = (u / Tact + (1 - u) / Tdeact);
        double dfdu = -(u / Tact + (1 - u) / Tdeact) - (u - a) * (1 / Tact - 1 / Tdeact);
        dfdydot[1][2] = 1;
    }



    VandenBogert2011Muscle::ImplicitResults Results;

    Results.forceResidual=muscleForceResidual;
    Results.activationResidual=activationResidual;
    Results.forceSee=Fsee;
    Results.df_dy=dfdy;
    Results.df_dydot=dfdydot;


return Results;

}
