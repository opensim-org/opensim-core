/* -------------------------------------------------------------------------- *
 *                      OpenSim:  VandenBogert.cpp                            *
 * -------------------------------------------------------------------------- */


//=============================================================================
// INCLUDES
//=============================================================================
#include "VandenBogert2011Muscle.h"
#include <OpenSim/Simulation/Model/Model.h>


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
    finalizeFromProperties();
}
//_____________________________________________________________________________
/*
 * Constructor.
 */
VandenBogert2011Muscle::VandenBogert2011Muscle
        (const std::string& name,  double maxIsometricForce,
                 double optimalFiberLength,double tendonSlackLength,
                 double pennationAngle)
{ ;
    constructProperties();
    setName(name);
    setMaxIsometricForce(maxIsometricForce);
    setOptimalFiberLength(optimalFiberLength);
    setTendonSlackLength(tendonSlackLength);
    setPennationAngleAtOptimalFiberLength(pennationAngle);
    finalizeFromProperties();
}



//_____________________________________________________________________________
/*
 * Construct and initialize properties.
 * All properties are added to the property set. Once added, they can be
 * read in and written to files.
 */
void VandenBogert2011Muscle::constructProperties()
{
    setAuthors("A. van den Bogert, B. Humphreys, C. Dembia");
    setReferences("van den Bogert A, Blana D, Heinrich D, Implicit methods for efficient musculoskeletal simulation and optimal control. Procedia IUTAM 2011; 2:297-316");

    constructProperty_tendon_strain_at_max_iso_force(0.033);
    constructProperty_active_force_length_curve_width(0.63);
    constructProperty_force_velocity_hill_constant(0.25);
    constructProperty_force_velocity_max_lengthening_force_norm(1.5);
    constructProperty_fiber_damping(0.01);
    constructProperty_fiber_slack_length_norm(1.0);
    constructProperty_activation_time_constant(0.01);
    constructProperty_deactivation_time_constant(0.04);
    constructProperty_pennation_at_optimal_fiber_length(0);
    constructProperty_default_activation(0.1);
    constructProperty_default_fiber_length(1.0);

    // These come from muscle.h
    //constructProperty_max_isometric_force(1000);
    //constructProperty_optimal_fiber_length(0.1);
    //constructProperty_tendon_slack_length(0.2);
    //constructProperty_max_contraction_velocity(10*optimal_fiber_length);
}




void VandenBogert2011Muscle::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();

}

//--------------------------------------------------------------------------
// GET & SET Properties
//--------------------------------------------------------------------------
void VandenBogert2011Muscle::setFMaxTendonStrain(double fMaxTendonStrain) {
    set_tendon_strain_at_max_iso_force(fMaxTendonStrain); }
double VandenBogert2011Muscle::getFMaxTendonStrain() const {
     return get_tendon_strain_at_max_iso_force(); }

void VandenBogert2011Muscle::setFlWidth(double flWidth) {
     set_active_force_length_curve_width(flWidth); }
double VandenBogert2011Muscle::getFlWidth() const
    { return get_active_force_length_curve_width(); }

void VandenBogert2011Muscle::setFvAHill(double fvAHill) {
    set_force_velocity_hill_constant(fvAHill); }
double VandenBogert2011Muscle::getFvAHill() const {
     return get_force_velocity_hill_constant(); }

void VandenBogert2011Muscle::setFvmaxMultiplier(double fvMaxMultiplier) {
     set_force_velocity_max_lengthening_force_norm(fvMaxMultiplier); }
double VandenBogert2011Muscle::getFvmaxMultiplier() const {
     return get_force_velocity_max_lengthening_force_norm(); }

void VandenBogert2011Muscle::setDampingCoefficient(double dampingCoefficient) {
     set_fiber_damping(dampingCoefficient); }
double VandenBogert2011Muscle::getDampingCoefficient() const {
     return get_fiber_damping(); }

void VandenBogert2011Muscle::setNormFiberSlackLength(double
                                                     normFiberSlackLength) {
     set_fiber_slack_length_norm(normFiberSlackLength); }
double VandenBogert2011Muscle::getNormFiberSlackLength() const {
     return get_fiber_slack_length_norm(); }

void VandenBogert2011Muscle::setActivTimeConstant(double activTimeConstant) {
     set_activation_time_constant(activTimeConstant); }
double VandenBogert2011Muscle::getActivTimeConstant() const {
     return get_activation_time_constant(); }

void VandenBogert2011Muscle::setDeactivationTimeConstant(double
                                                    deactivationTimeConstant) {
     set_deactivation_time_constant(deactivationTimeConstant); }
double VandenBogert2011Muscle::getDeactivationTimeConstant() const {
     return get_deactivation_time_constant(); }

void VandenBogert2011Muscle::setPennAtOptFiberLength(double
                                                     pennAtOptFiberLength) {
    set_pennation_at_optimal_fiber_length(pennAtOptFiberLength); }
double VandenBogert2011Muscle::getPennAtOptFiberLength() const {
    return get_pennation_at_optimal_fiber_length(); }


void VandenBogert2011Muscle::setDefaultActivation(double
                                                     defaultActivation) {
    set_default_activation(defaultActivation); }
double VandenBogert2011Muscle::getDefaultActivation() const {
    return get_default_activation(); }

void VandenBogert2011Muscle::setDefaultFiberLength(double fiberLength)
{
    set_default_fiber_length(fiberLength);
}

double VandenBogert2011Muscle::getDefaultFiberLength() const
{   return get_default_fiber_length(); }


void VandenBogert2011Muscle::setProjFiberLengthNorm(SimTK::State& s, double projFibLenNorm)
            const
{
    //In other muscles this is setFiberLength
        setStateVariableValue(s, "projected_fiber_length_normalized",projFibLenNorm);
        markCacheVariableInvalid(s,"lengthInfo");
        markCacheVariableInvalid(s,"velInfo");
        markCacheVariableInvalid(s,"dynamicsInfo");
    }


// Set the activation
void VandenBogert2011Muscle::setActivation(SimTK::State& s,double activation)
const
//TODO: Add clamping?  Add ignore_activation_dynamics
{
    setStateVariableValue(s, "activation", activation);

    //markCacheVariableInvalid(s,"velInfo");
    //markCacheVariableInvalid(s,"dynamicsInfo");
}

// Get the Residual of muscle
// TODO:  This breaks naming convention in that it does not get an already
// evaluated constant, it actually performs the calculation.  Without using a
// state variable or adding to the cache variables, I think I have to do it this
// way?
SimTK::Vec2 VandenBogert2011Muscle::getResidual(const SimTK::State& s,
                                                double projFibVelNorm_guess,
                                                double activdot_guess,
                                                double excitation)
                                                const
{
    ImplicitResidual residualStruct = calcImplicitResidual(s,
                                                           projFibVelNorm_guess,
                                                           activdot_guess,
                                                           excitation, 0);

    SimTK::Vec2 residual = {residualStruct.forceResidual,
                            residualStruct.activResidual};
    return(residual);
}



//These are hacks because I am not using muscle.h mli cache variables, yet:
double  VandenBogert2011Muscle::getProjFiberLengthNorm(const SimTK::State& s) const
{
    return getStateVariableValue(s, "projected_fiber_length_normalized");
}
double  VandenBogert2011Muscle::getActivation(const SimTK::State& s) const
{
    return getStateVariableValue(s, "activation");
}



//==============================================================================
// Muscle.h Interface
//==============================================================================


double  VandenBogert2011Muscle::computeActuation(const SimTK::State& s) const
{
    const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
    setActuation(s, mdi.tendonForce);
    return mdi.tendonForce;
}



void VandenBogert2011Muscle::computeInitialFiberEquilibrium(SimTK::State& s)
const
{
    // TODO:  this is the same code as computeFiberEquilibriumAtZeroVelocity
    //Need to update for v~=0
    //Let's start with the default activation and the fiberVelocity=0

    computeFiberEquilibriumAtZeroVelocity(s);

}



void VandenBogert2011Muscle::
computeFiberEquilibriumAtZeroVelocity(SimTK::State& s) const
{

    double activation = getActivation(s);
    double projFibLenNorm = getProjFiberLengthNorm(s);
    SimTK::Vec2 lenghAndForce = calcFiberStaticEquilbirum(projFibLenNorm,
                                                        activation);

    setActuation(s, lenghAndForce[1]);
    setProjFiberLengthNorm(s,lenghAndForce[0]);

    //TODO:  Millard Muscle handles non-convergence here and zero muscle length.
    //     Need to consider adding.

}


void VandenBogert2011Muscle::
postScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
    //TODO:  I have not really thought through if there are special cases for this muscle
    GeometryPath& path = upd_GeometryPath();
    path.postScale(s, aScaleSet);

    if (path.getPreScaleLength(s) > 0.0) {
        double scaleFactor = getLength(s) / path.getPreScaleLength(s);
        upd_optimal_fiber_length() *= scaleFactor;
        upd_tendon_slack_length() *= scaleFactor;
        path.setPreScaleLength(s, 0.0);
    }
}

//==============================================================================
// MODELCOMPONENT INTERFACE REQUIREMENTS
//==============================================================================


void VandenBogert2011Muscle::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model);
}


// Define new states and their derivatives in the underlying system
void VandenBogert2011Muscle::extendAddToSystem(SimTK::MultibodySystem& system)
const
{

    Super::extendAddToSystem(system);

    SimTK_ASSERT(isObjectUpToDateWithProperties(),
                 "VandenBogert2011Muscle: Muscle properties not up-to-date");

    addStateVariable("projected_fiber_length_normalized");
    addStateVariable("activation");

}

void VandenBogert2011Muscle::extendInitStateFromProperties(SimTK::State& s)
const
{
    Super::extendInitStateFromProperties(s);

    setActivation(s, getDefaultActivation());

    double projFiberLength = fiberLengthToProjectedLength(getDefaultFiberLength(), false);
    setProjFiberLengthNorm(s, projFiberLength/getOptimalFiberLength());
}

void VandenBogert2011Muscle::extendSetPropertiesFromState(const SimTK::State& s)
{
    Super::extendSetPropertiesFromState(s);

    setDefaultActivation(getStateVariableValue(s,"activation"));
    setDefaultFiberLength(getStateVariableValue(s,"projected_fiber_length_normalized"));

}


void VandenBogert2011Muscle::computeStateVariableDerivatives(const SimTK::State& s) const
{

    double excitation = getControl(s);
    // For now we will start with static guess
    // TODO: figure out how to use previous state as guess
    SimTK::Vector ydotInitialGuess(2);
    ydotInitialGuess[0] = 0.0;
    ydotInitialGuess[1] = 0.0;
    double projFibVelNorm = calcSolveMuscle(s, excitation, ydotInitialGuess);
    setStateVariableDerivativeValue(s, "projected_fiber_length_normalized", projFibVelNorm );


    double adot =getActivationDerivative(s);
    setStateVariableDerivativeValue(s, "activation",  adot);



}


//=============================================================================
// COMPUTATION
//=============================================================================



class ImplicitSystemForwardEulerStep : public SimTK::OptimizerSystem {
public:

    //TODO:  Add back in option to use NR solver (and perform benchmark)
    /* Constructor class. Parameters passed are accessed in the objectiveFunc()
     * class. */
    ImplicitSystemForwardEulerStep(int numParameters, SimTK::State& s, double& u): SimTK::OptimizerSystem(numParameters), numControls(numParameters), si(s), excitation(u)
    {

        //VandenBogert2011Muscle::ImplicitResidual  Results1 = VandenBogert2011Muscle::calcImplicitResidual(s,0.0,0.0,0.0,0);

    }

    int objectiveFunc(const SimTK::Vector &new_yDotGuess, bool new_params,
                      SimTK::Real& f) const override {
        // No objective function
        f = 0;
        return 0;
    }

    int constraintFunc(const SimTK::Vector &new_yDotGuess, bool newParams,
                       SimTK::Vector& constraints)  const override {
        //The constraints function is the residuals

        double projFibVelNorm_guess = new_yDotGuess[0];
        double activdot = 0;  //BTH - big hack here.  Remove this

        VandenBogert2011Muscle::ImplicitResidual results = p_muscle->calcImplicitResidual(si,projFibVelNorm_guess,activdot,excitation,0);


        constraints[0] = results.forceResidual;
        //constraints[1] = results.activResidual;

        return 0;
    }

private:
    int numControls;
    SimTK::State& si;
    double excitation;
    SimTK::ReferencePtr<VandenBogert2011Muscle> p_muscle;
};



double VandenBogert2011Muscle::fiberLengthToProjectedLength
        (double fiberLength , bool argsAreNormalized) const {

    /* When argsAreNormalized= True the function expects normalized values and
     * returns normalized values.  The function though is internally structured
     * to utilize non-normalized values (therefore for it will convert
     * normalized arguments to non-normalized values, perform calculations,
     * and do then convert the return values to normalized
     * when argsAreNormalized= True.)*/

    double pennAtOptFiberLength = getPennAtOptFiberLength();
    double projFibLen = 0;


    // If there is pennation, compute fiberLength and muscleWidth from state s using the constant volume constraint: Lce*sin(p) = Lceopt*sin(popt)
    // Lce is dimensionless (normalized to Lceopt)
    // If pennation is zero, we can't do this because volume is zero, and fiberLength = projFibLen
    if (pennAtOptFiberLength < 0.01) return fiberLength;

    //else:
    double optimalFiberLength = getOptimalFiberLength();

    if (argsAreNormalized) fiberLength = fiberLength*optimalFiberLength;

    double muscleWidth = sin(pennAtOptFiberLength);
    //b is the fixed distance of the fiber perpindicular to the tendon (width)

    /*TODO: muscleWidth as cache variable
     * recalculated. Should be moved info muscleLengthInfo cache */

    // The fiber can not be shorter than b; if it is, the projected length
    // equation below will return a complex value.  It also physically can
    // not happen (the fiber being shorter than the fixed width)
    if (fiberLength >= muscleWidth) {
        projFibLen = sqrt(pow(fiberLength, 2) - pow(muscleWidth, 2));
    } else {
        projFibLen = SimTK::NaN;
    }  //TODO: Doing this for now, but need to
    // come up with a clamping scheme (Millard has one)

    if (argsAreNormalized) projFibLen = projFibLen/optimalFiberLength;


    return projFibLen;
};


//------------------------------------------------------------------------------
double VandenBogert2011Muscle::projFibLenToFiberLength (double projFibLen,
                                                        bool argsAreNormalized)
                                                        const {

    /* When argsAreNormalized= True the function expects normalized values and
     * returns normalized values.  The function though is internally structured
     * to utilize non-normalized values (therefore for it will convert
     * normalized arguments to non-normalized values, perform calculations,
     * and do then convert the return values to normalized
     * when argsAreNormalized= True.)*/


    double pennAtOptFiberLength = getPennAtOptFiberLength();
    double fiberLength = 0;

    double optimalFiberLength = getOptimalFiberLength();

    if (argsAreNormalized) projFibLen = projFibLen *
                                        optimalFiberLength;  //Convert from Norm


    // If pennation is zero, we can't do this because volume is zero, and fiberLength = projFibLen
    if (pennAtOptFiberLength < 0.01) {
        fiberLength = projFibLen;
    } else {
        double static muscleWidth = sin(pennAtOptFiberLength);
        fiberLength = sqrt(pow(projFibLen, 2) + pow(muscleWidth, 2));
    }

    //Convert back to Norm if needed
    if (argsAreNormalized) fiberLength = fiberLength / optimalFiberLength;

    return fiberLength;
}

//------------------------------------------------------------------------------
double VandenBogert2011Muscle::fiberVelocityToProjFibVel
        (double fiberVelocity, double fiberLength, double projFiberLength,
         bool argsAreNormalized) const {

    /* When argsAreNormalized= True the function expects normalized values and
     * returns normalized values.  The function though is internally structured
     * to utilize non-normalized values (therefore for it will convert
     * normalized arguments to non-normalized values, perform calculations,
     * and do then convert the return values to normalized
     * when argsAreNormalized= True.)*/

    // Note that this normalized by fiber optimal length (not by max velocity)
    double optimalFiberLength = getOptimalFiberLength();

    if (argsAreNormalized) {
        fiberLength = fiberLength * getOptimalFiberLength();
        fiberVelocity = fiberVelocity * optimalFiberLength;
        projFiberLength = projFiberLength * optimalFiberLength;
    }

    double projFibVel = 0;

    if (fiberVelocity != 0) {
        projFibVel = fiberVelocity / cos(projFiberLength / fiberLength);

    } else {
        projFibVel = 0; }

    if (argsAreNormalized) projFibVel = projFibVel / optimalFiberLength;

    return projFibVel;
};

//------------------------------------------------------------------------------
double VandenBogert2011Muscle::fiberVelocityToProjFibVel
        (double fiberVelocity, double fiberLength, bool argsAreNormalized)
                const {

    //overload method when projFiberLength is not known/provided

    double projFiberLength = fiberLengthToProjectedLength(fiberLength,
                                                          argsAreNormalized);
    double projFiberVelocity = fiberVelocityToProjFibVel (fiberVelocity,
                                                          fiberLength,
                                                          projFiberLength,
                                                          argsAreNormalized);
    return projFiberVelocity;
};


//------------------------------------------------------------------------------
double VandenBogert2011Muscle::projFibVelToFiberVelocity(double projFibVel,
                                                         double fiberLength,
                                                         double projFiberLength,
                                                         bool argsAreNormalized)
                                                         const {

    /* When argsAreNormalized= True the function expects normalized values and
 * returns normalized values.  The function though is internally structured
 * to utilize non-normalized values (therefore for it will convert
 * normalized arguments to non-normalized values, perform calculations,
 * and do then convert the return values to normalized
 * when argsAreNormalized= True.)*/

// Note that this normalized by fiber optimal length (not by max velocity)
    double optimalFiberLength = getOptimalFiberLength();

    if (argsAreNormalized) {
        fiberLength = fiberLength * getOptimalFiberLength();
        projFibVel = projFibVel * optimalFiberLength;
        projFiberLength = projFiberLength * optimalFiberLength;
    }

    double fiberVelocity = 0;

    if (projFibVel != 0) {
        fiberVelocity = projFibVel * cos(projFiberLength / fiberLength);
    } else {
        fiberVelocity = 0; }

    if (argsAreNormalized) fiberVelocity = fiberVelocity / optimalFiberLength;

    return fiberVelocity;}

//------------------------------------------------------------------------------
double VandenBogert2011Muscle::projFibVelToFiberVelocity
        (double projFibVel, double projFibLength, bool argsAreNormalized)
                        const {

    //overload method when fiberLength is not known/provided

    double fiberLength = projFibLenToFiberLength(fiberLength, argsAreNormalized);
    double fiberVelocity = projFibVelToFiberVelocity(projFibVel, fiberLength,
                                                     projFibLength,
                                                     argsAreNormalized);

    return fiberVelocity;
};



//------------------------------------------------------------------------------
VandenBogert2011Muscle::ImplicitResidual
VandenBogert2011Muscle::calcImplicitResidual(const SimTK::State& s,
                                             double projFibVelNorm_guess,
                                             double activdot_guess,
                                             double excitation,
                                             bool returnJacobians)
const
{
    // Overload method for state as parameters

    VandenBogert2011Muscle::ImplicitResidual results = calcImplicitResidual(
            getLength(s), getProjFiberLengthNorm(s), getActivation(s), projFibVelNorm_guess,
            activdot_guess, excitation, returnJacobians);

    return results;
}


//------------------------------------------------------------------------------
VandenBogert2011Muscle::ImplicitResidual
VandenBogert2011Muscle::calcImplicitResidual(SimTK::Vec2 y,
                                             SimTK::Vec2 ydot_guess,
                                             double muscleLength,
                                             double excitation, bool returnJacobians)
const
{
    // Overload method for state vectors as parameters
    VandenBogert2011Muscle::ImplicitResidual results = calcImplicitResidual(
            muscleLength, y[0], y[1], ydot_guess[0], ydot_guess[1], excitation,
            returnJacobians);
    return results;

}

//------------------------------------------------------------------------------
VandenBogert2011Muscle::ImplicitResidual VandenBogert2011Muscle::
calcImplicitResidual(double muscleLength, double projFibLenNorm, double activ,
                     double projFibVelNorm, double activdot, double excitation,
                     bool returnJacobians) const {


    //TODO: Match symbols to doxygen diagram and Add symbolic equations comments
    //TODO: May want to make this into a separate function

    // -------------------------Parameters----------------------------//
    //F_{iso} Maximum isometric force that the fibers can generate
    double maxIsoForce = getMaxIsometricForce();

    //u_{max} (dimensionless) strain in the series elastic element at load of
    //      maxIsometricForce
    double fMaxTendonStrain = getFMaxTendonStrain(); //Strain in the series
    //      elastic element at load of Fmax

    //W (dimensionless) width parameter of the force-length relationship of
    //      the muscle fiber
    double fl_width = getFlWidth();

    //AHill (dimensionless) Hill parameter of the force-velocity relationship
    double fv_AHill = getFvAHill();   //Hill parameter of the f-v relationship

    //FV_{max} (dimensionless) maximal eccentric force
    double fv_maxMultiplier = getFvmaxMultiplier();
    //Maximal eccentric force multiplier

    // L_{opt}  (m) Optimal Length of Contractile Element;
    double optFiberLength = getOptimalFiberLength();
    //Optimal Length of Contractile Element

    //b (s/m) damping coefficient of damper parallel to the fiber
    // (normalized to maxIsometricForce)
    double dampingCoeff = getDampingCoefficient();

    //L_{slack,fiber} Slack length of the parallel elastic element, divided by
    //          Lceopt
    double fiberSlackLengthNorm = getNormFiberSlackLength();

    //L_{slack,tendon} (m) slack length of the tendon
    double tendonSlackLength = getTendonSlackLength();

    //T_{act} (s) Activation time
    double activTimeConstant = getActivTimeConstant();

    //T_{deact} (s) Deactivation time
    double deactivationTimeConstant = getDeactivationTimeConstant();

    //phi_{opt} pennation at Optimal Fiber Length
    double pennAtOptFiberLength = getPennAtOptFiberLength();


    // constants derived from the muscle parameters


    // Jacobian Matrices
    SimTK::Mat22 df_dy;
    SimTK::Mat22 df_dydot;


    //-------Convert projFiberLength & Velocity to fiberLength & Velocity------/
    double cosPenn;
    double dcosPenn_dprojFibLen;
    double fiberLengthNorm;
    double dfiberLength_dprojFibLen;

    fiberLengthNorm=projFibLenToFiberLength(projFibLenNorm,true);

    if (pennAtOptFiberLength<0.01)  {
        // If pennation is zero, we can't do this because volume is zero,
        //      and fiberLength ~ projFiberLength
        cosPenn = 1.0;
        dfiberLength_dprojFibLen = 1;
        dcosPenn_dprojFibLen = 0;
    } else {
        double b = sin(pennAtOptFiberLength);
        cosPenn = projFibLenNorm/fiberLengthNorm;
        dfiberLength_dprojFibLen = cosPenn;
        dcosPenn_dprojFibLen = pow(b,2) / pow(fiberLengthNorm,3);
    }


    // Compute fiberVelocity and its derivatives wrt projFibLen and projFibVel
    double fiberLengtheningVelocityNorm = projFibVelNorm*cosPenn;
    double dfiberLengtheningVelocityNorm_dprojFibVelNorm = cosPenn;
    double dfiberLengtheningVelocityNorm_dprojFibLenNorm =
            projFibVelNorm * dcosPenn_dprojFibLen;


    //---F1 is the normalized isometric force-length relationship at maximum
    //                                               activation--------------//
    double fiberExp = (fiberLengthNorm - 1.0) / fl_width;   // [dimensionless]
    double F1 = exp(-pow(fiberExp, 2));        // Gaussian force-length curve

    double dF1_dfiberLengthNorm = 0;

    getDebugLevel();

    if (returnJacobians) {
        dF1_dfiberLengthNorm = -2.0 * fiberExp * F1 / fl_width;
        double dF1_dprojFibLenNorm =
                dF1_dfiberLengthNorm * dfiberLength_dprojFibLen;
    }



    //-------- F2 is the dimensionless force-velocity relationship -------//
    double F2;
    double dF2_dfiberLengtheningVelocityNorm;
    double dF2_dactiv;
    double dF2_dprojFibVelNorm;
    double dF2_dprojFibLenNorm;
    double df_dmuscleLength;

    // Chow/Darling Vel-Activation Relationship //TODO:  Add full reference
    //double lambda = 0.5025 + 0.5341*activ;
    //double  dlambda_da = 0.5341;
    double lambda = 1;   //Turn it off for now as it seems to cause an issue
                        // with negative force large concentric vel
    double dlambda_da = 0;

    if (fiberLengtheningVelocityNorm < 0) {
        //Hill's equation for concentric contraction
        // F2 = (V_{max} + V_{fiber}) / (V_{max} - V_{fiber}/a_{Hill})

        double hillDenom = (lambda*getMaxContractionVelocity() - fiberLengtheningVelocityNorm/fv_AHill);
        F2 = (lambda*getMaxContractionVelocity()+ fiberLengtheningVelocityNorm) / hillDenom;

        if (returnJacobians) {
            dF2_dfiberLengtheningVelocityNorm  = (1.0 + F2 / fv_AHill) / hillDenom;
            dF2_dactiv = - dlambda_da * getMaxContractionVelocity() * fiberLengtheningVelocityNorm *
                    (1.0 + 1.0/fv_AHill) / pow(hillDenom,2);
        }
    } else {
        //Katz Curve for eccentric contraction
        // c is Katz Constant
        double c3 = getMaxContractionVelocity()* fv_AHill * (fv_maxMultiplier - 1.0) /
                (fv_AHill + 1.0); // parameter in the eccentric f-v equation
        double c = lambda*c3;
        //F2 = (g_{max} * V_{fiber} + c) / (V_{fiber} + c)
        double katzDenom = (fiberLengtheningVelocityNorm  + c);
        F2 = (fv_maxMultiplier * fiberLengtheningVelocityNorm  + c) / katzDenom ;
        if (returnJacobians) {
            dF2_dfiberLengtheningVelocityNorm = (fv_maxMultiplier - F2) / katzDenom ;
            dF2_dactiv = dlambda_da * c3 * fiberLengtheningVelocityNorm *
                    (1-fv_maxMultiplier) / pow(katzDenom,2);
        }
    }
    if (returnJacobians){
        dF2_dprojFibVelNorm =
                dF2_dfiberLengtheningVelocityNorm * dfiberLengtheningVelocityNorm_dprojFibVelNorm;
        dF2_dprojFibLenNorm =
                dF2_dfiberLengtheningVelocityNorm * dfiberLengtheningVelocityNorm_dprojFibLenNorm;
    }

    //------F3 is the dimensionless fiber (PEE) force (in units of Fmax)------//
    double dF3_dprojFibLenNorm;

    // stiffness of the linear term is 1 N/m, convert to Fmax/Lceopt units
    double kPEENorm = 1.0 / maxIsoForce * optFiberLength;
    // elongation of fiber (PEE), relative to Lceopt
    double elongationFiberNorm = (fiberLengthNorm - fiberSlackLengthNorm);
    double F3 = kPEENorm * elongationFiberNorm;
    // low stiffness linear term
    double dF3_dfiberLengthNorm = kPEENorm;
    double kPEE2Norm =0;
    if (elongationFiberNorm > 0) {
        kPEE2Norm = 1 / pow(fl_width, 2);  //Fiber (PEE) quadratic stiffness,
        //          so Fpee = Fmax when Lce = Lce*(1+W)
        //add quadratic term for positive elongation
        F3 = F3 + kPEE2Norm * pow(elongationFiberNorm, 2);
        if (returnJacobians) {
            dF3_dfiberLengthNorm =
                    dF3_dfiberLengthNorm + 2 * kPEE2Norm * elongationFiberNorm;
        }
    }
    if (returnJacobians) {
        dF3_dprojFibLenNorm = dF3_dfiberLengthNorm * dfiberLength_dprojFibLen;
    }


    //--------F4 is the dimensionless SEE force (in units of Fmax)----------//


    // stiffness of the linear term is 1 N/m, convert to Fmax/m (so normalized
    // by Fmax)
    double kSEE = 1.0 / maxIsoForce;

    // elongation of tendon (SEE), in meters
    double elongationTendon = muscleLength - projFibLenNorm *
                                             optFiberLength - tendonSlackLength;
    //  low stiffness linear term
    double F4 = kSEE * elongationTendon;
    double dF4_dmuscleLength;
    double dF4_dprojFibLenNorm;
    if (returnJacobians) {
         dF4_dprojFibLenNorm = -kSEE * optFiberLength;
         dF4_dmuscleLength = kSEE;
    }

    double kSEE2 = SimTK::NaN;
    if (elongationTendon > 0) {
        // Tendon (SEE) quadratic stiffness, so Fsee = Fmax at strain of umax
        // This is normalized by Fmax
        kSEE2 = 1 / pow(tendonSlackLength * fMaxTendonStrain, 2);

        // add quadratic term for positive deformation
        F4 = F4 + (kSEE2 * pow(elongationTendon, 2));
        if (returnJacobians) {
             dF4_dprojFibLenNorm = dF4_dprojFibLenNorm - 2 * kSEE2 *
                                             optFiberLength* elongationTendon;
             dF4_dmuscleLength = dF4_dmuscleLength + 2 * kSEE2 *
                                                             elongationTendon;
        }
    }


    //-- F5 is viscous damping parallel to the CE (0.001 of Fmax at 1 Lceopt/s)
    // to  ensure that df/dLcedot is never zero-----------//
    double F5 = dampingCoeff * projFibVelNorm ;
    double dF5_dprojFibVelNorm  = dampingCoeff;


    // ---------Calculate the Muscle Force Residual ---------------------- //
    //The muscle dynamics equation: f = Fsee - (a*Fce - Fpee)*cos(Penn) -
    //                                                          Fdamping = 0
    //   (Reminder the units of fRes are (N/N), needs to be *Fmax to get to N)
    double fRes = F4 - (activ * F1 * F2 + F3)*cosPenn - F5;



if (getDebugLevel()>0){
    cout << "-------------------------" << endl;
    cout << "activ: " << activ << endl;
    cout << "F1 (FL): " << F1 << endl;
    cout << "F2 (FV): " << F2 << endl;
    cout << "F3 (PEE) : " << F3 << endl;
    cout << "F4 (SEE): " << F4 << endl;
    cout << "F5 (Damping): " << F5 << endl;
    cout << "fRes: " << fRes << endl;
    cout << "------------------" << endl;
    cout << "cosPenn: " << cosPenn << endl;
    cout << "muscleLength: " << muscleLength << endl;
    cout << " SEE" << endl;
    cout << "   kSEE: " << kSEE << endl;
    cout << "   kSEE2: " << kSEE2 << endl;
    cout << "   elongationTendon: " << elongationTendon << endl;
    cout << "   tendonSlackLength: " << tendonSlackLength << endl;
    cout << " PEE" << endl;
    cout << "   fiberLengthNorm: " << fiberLengthNorm << endl;
    cout << "   optFiberLength: " << optFiberLength << endl;
    cout << "   projFibLenNorm: " << projFibLenNorm << endl;
    cout << "   kPEENorm: " << kPEENorm << endl;
    cout << "   kPEE2Norm: " << kPEE2Norm << endl;
    cout << "   elongationFiberNorm: " <<  elongationFiberNorm << endl;
    cout << "" << endl;
}

    // --------------- Force in tendon (SEE) is maxIsoForce*F4 -------------- //
    double Fsee = maxIsoForce * F4;

    // TODO: I don't think we need to implement dFsee_.....
    /*if (nargout > 1)
        dFsee_dLm  = Fmax * dF4_dLm;
    dFsee_dLce = Fmax * dF4_dLce;
    dFsee_dy   = [0;0;(-d/L)*dFsee_dLm; 0;dFsee_dLce;0;0];
     % Fsee is a function of y(3) & y(5)
    end*/


    //----------------------Activation dynamics equation-------------------//

    double activationResidual = activdot - calcActivationDerivative(activ,excitation);
    SimTK::Vec2 df_du;
    double dActRes_dactiv = 0;
    double dActRes_dactivdot = 0;

    if (returnJacobians) {
        dActRes_dactiv = (excitation / activTimeConstant + (1 - excitation) / deactivationTimeConstant);
        dActRes_dactivdot = 1;

        df_du[0] = 0;
        df_du[1] = -(excitation / activTimeConstant + (1 - excitation) / deactivationTimeConstant )
                       - (excitation - activ) * (1 / activTimeConstant - 1 / deactivationTimeConstant );
    }



    //---------------------Assemble Jacobians---------------------------//
    if (returnJacobians) {
        double dfRes_dactiv = -(F1*F2 + activ*F1*dF2_dactiv )*cosPenn;
        double dfRes_dprojFibLengthNorm = dF4_dprojFibLenNorm -
                      (activ*(dF1_dfiberLengthNorm*F2 + F1*dF2_dprojFibLenNorm) +
                       dF3_dprojFibLenNorm) * cosPenn - (activ*F1*F2 + F3) *
                        dcosPenn_dprojFibLen;
        double dfRes_dprojFibVelNorm =
                - activ*F1*dF2_dprojFibVelNorm - dF5_dprojFibVelNorm;
        double dfRes_dmuscleLength = dF4_dmuscleLength;


        //y=(projFiberLength,activation)  <-States

        //df_fy is 2x2:  Where columns are states:
        //          [projFiberLengthNorm, activation]
        //              Rows are:
        //          [force residual; Activation Residual]

        // Row 1 - df_dy  (force)
        df_dy[0][0] = dfRes_dprojFibLengthNorm;
        df_dy[0][1] = dfRes_dactiv;
        // Row 2 - df_dy (activation)
        df_dy[1][0] = 0;
        df_dy[1][1] = dActRes_dactiv ;


        // Row 1 - df_dydot  (force)
        df_dydot[0][0] = dfRes_dprojFibVelNorm;
        df_dydot[0][1] = 0;
        // Row 2 - df_dydot (activation)
        df_dydot[1][0] = 0;
        df_dydot[1][1] = dActRes_dactivdot;

        df_dmuscleLength = dfRes_dmuscleLength;
    }



    VandenBogert2011Muscle::ImplicitResidual results;

    results.forceResidual = fRes;
    results.activResidual = activationResidual;
    results.forceTendon = Fsee;
    results.df_dy = df_dy;
    results.df_dydot = df_dydot;
    results.df_du = df_du;
    results.df_dmuscleLength = df_dmuscleLength;
    results.F1 = F1;    //Output force components for troubleshooting
    results.F2 = F2;
    results.F3 = F3;
    results.F4 = F4;
    results.F5 = F5;

return results; }



double VandenBogert2011Muscle::
calcActivationDerivative(double activation, double excitation) const
{

    double activationDerivative=(excitation - activation) *
    (excitation / getActivTimeConstant() + (1 - excitation) / getDeactivationTimeConstant());

    return activationDerivative;
}


double VandenBogert2011Muscle::
getActivationDerivative(const SimTK::State& s) const
{
    double activationDerivative =
            calcActivationDerivative(getActivation(s),getExcitation(s));

    return activationDerivative;
}





//==============================================================================
// MUSCLE INTERFACE REQUIREMENTS -- MUSCLE LENGTH INFO
//==============================================================================
void VandenBogert2011Muscle::calcMuscleLengthInfo(const SimTK::State& s,
                                                        MuscleLengthInfo& mli) const
{
    // Get musculotendon actuator properties.
    //double maxIsoForce    = getMaxIsometricForce();
    double optFiberLength = getOptimalFiberLength();
    double tendonSlackLen = getTendonSlackLength();

    try {

        double projFibLengthNorm = getStateVariableValue(s, "projected_fiber_length_normalized");

        mli.normFiberLength = projFibLenToFiberLength(projFibLengthNorm,true);
        mli.fiberLength = mli.normFiberLength * optFiberLength ;

        mli.pennationAngle    = penMdl.calcPennationAngle(mli.fiberLength);
        mli.cosPennationAngle = cos(mli.pennationAngle);
        mli.sinPennationAngle = sin(mli.pennationAngle);
        mli.fiberLengthAlongTendon = mli.fiberLength * mli.cosPennationAngle;

        // Necessary even for the rigid tendon, as it might have gone slack.
        mli.tendonLength      = penMdl.calcTendonLength(mli.cosPennationAngle,
                                                        mli.fiberLength, getLength(s));
        mli.normTendonLength  = mli.tendonLength / tendonSlackLen;
        mli.tendonStrain      = mli.normTendonLength - 1.0;

        mli.fiberPassiveForceLengthMultiplier =
                fpeCurve.calcValue(mli.normFiberLength);
        mli.fiberActiveForceLengthMultiplier =
                falCurve.calcValue(mli.normFiberLength);

    } catch(const std::exception &x) {
        std::string msg = "Exception caught in Millard2012EquilibriumMuscle::"
                                  "calcMuscleLengthInfo from " + getName() + "\n"
                          + x.what();
        throw OpenSim::Exception(msg);
    }
}





 double VandenBogert2011Muscle::getProjFiberVelNorm(const SimTK::State& s) const
 {
     double excitation = getExcitation(s);
     SimTK::Vector projFibVelNormGuess; //BTH - Should make this use the current state derivative value if possible
     projFibVelNormGuess[0]=0;
     double projFiberVelocityNorm = calcSolveMuscle(s, excitation, projFibVelNormGuess);

     return projFiberVelocityNorm;
 }





//------------------------------------------------------------------------------
VandenBogert2011Muscle::ImplicitResidual
VandenBogert2011Muscle::calcJacobianByFiniteDiff(double muscleLength,
                                                 double projFibLenNorm,
                                                 double activ,
                                                 double projFibVelNorm,
                                                 double activdot, double excitation,
                                                 double stepSize) const
{
    SimTK::Vec2 y;
    SimTK::Vec2 ydot;
    y[0]=projFibLenNorm;
    y[1]=activ;
    ydot[0]=projFibVelNorm;
    ydot[1]=activdot;

    // Overload method for state vectors as parameters
    VandenBogert2011Muscle::ImplicitResidual results =
            calcJacobianByFiniteDiff(y, ydot, muscleLength, excitation, stepSize );

    return results;
}


//------------------------------------------------------------------------------
VandenBogert2011Muscle::ImplicitResidual    VandenBogert2011Muscle::
calcJacobianByFiniteDiff(SimTK::Vec2 y, SimTK::Vec2 ydot, double muscleLength,
                         double excitation, double stepSize ) const {



    //TODO: In the optimizer class, you do not have to supply the Jacobian
    // function.  The must mean there is code in Simbody that does finite diff.
    // Should probably look at calling that in place of this code.

    // Jacobian Matrices
    SimTK::Mat22 df_dy;
    SimTK::Mat22 df_dydot;
    SimTK::Vec2 df_du;

    VandenBogert2011Muscle::ImplicitResidual opPoint;
    opPoint = calcImplicitResidual (y,ydot,muscleLength,excitation,0);
    double opForceResidual = opPoint.forceResidual;
    double opActivResidual = opPoint.activResidual;

    VandenBogert2011Muscle::ImplicitResidual del;

    //----------df_dy------------//
    SimTK::Vec2 dh = {stepSize,0};
    del = calcImplicitResidual(y+dh,ydot,muscleLength,excitation,0);
    df_dy[0][0] = (del.forceResidual-opForceResidual)/stepSize;
    df_dy[1][0] = (del.activResidual-opActivResidual)/stepSize;

    dh = {0,stepSize};
    del = calcImplicitResidual(y+dh,ydot,muscleLength,excitation,0);
    df_dy[1][0] = (del.forceResidual-opForceResidual)/stepSize;
    df_dy[1][1] = (del.activResidual-opActivResidual)/stepSize;

    //----------df_dydot------------//
    dh = {stepSize,0};
    del = calcImplicitResidual(y,ydot+dh,muscleLength,excitation,0);
    df_dydot[0][0] = (del.forceResidual-opForceResidual)/stepSize;
    df_dydot[1][0] = (del.activResidual-opActivResidual)/stepSize;

    dh = {0,stepSize};
    del = calcImplicitResidual(y,ydot+dh,muscleLength,excitation,0);
    df_dydot[1][0] = (del.forceResidual-opForceResidual)/stepSize;
    df_dydot[1][1] = (del.activResidual-opActivResidual)/stepSize;

    //----------df_du----------------//
    del = calcImplicitResidual(y,ydot,muscleLength,excitation+stepSize,0);
    df_du[0] = (del.forceResidual-opForceResidual)/stepSize;
    df_du[1] = (del.activResidual-opActivResidual)/stepSize;

/*
    for (int i =0; i<=1; i=i+1) {
        yTemp[i]=y[i]+h;
        del = calcImplicitResidual(yTemp,ydot,u,0);
        df_dy[0][i]=(del.forceResidual-opForceResidual)/h; //force residual
        df_dy[1][i]=(del.activResidual-opActivResidual)/h; //activ residual


        ydotTemp[i]=y[i]+h;
        del = calcImplicitResidual(y,ydotTemp,u,0);

        df_dydot[0][i]=(del.forceResidual-opForceResidual)/h; //force residual
        df_dydot[1][i]=(del.activResidual-opActivResidual)/h; //activ residual

        yTemp=y;
        ydotTemp=ydot;

    }*/


    //del = calcImplicitResidual(y,ydot,u+h,0);
    //double df_du = (del.forceResidual-opForceResidual)/h;

    opPoint.df_dy = df_dy;
    opPoint.df_dydot = df_dydot;
    opPoint.df_du = df_du;

    return opPoint;
}


//------------------------------------------------------------------------------
// Just a quick convenience method to get the force residual
// under static conditions
SimTK::Vec3 VandenBogert2011Muscle::calcFiberStaticEquilibResidual(
        double projFibLenNorm, double muscleLength, double activ) const
{
    VandenBogert2011Muscle::ImplicitResidual results = calcImplicitResidual(
            muscleLength, projFibLenNorm, activ, 0.0, 0.0, activ, 1.0);

    SimTK::Vec3 resAndDerivative;

    resAndDerivative[0] = results.forceResidual;
    resAndDerivative[1] = results.df_dy[0][0];
    resAndDerivative[2] = results.forceTendon;

    return resAndDerivative;
}

//------------------------------------------------------------------------------
SimTK::Vec2 VandenBogert2011Muscle::calcFiberStaticEquilbirum(
        double muscleLength, double activ) const {

    // This code assumes that the muscle lengthing speed is 0.
    // It utilizes a Newton-Raphson Method to root solve to obtain the
    //fiber length.


    //TODO: Code is not optimized.  Specifically the number of calls to
    //calcImplicitResidual can be reduced

    //TODO: Generalize with a Lambda function (will need help with that).
    //TODO: calcImplicitResidual really only needs to calculate df_ds
    //       (single element) for this function


    double tol = 1e-8; //TODO : Look at changing to proportional of eps
    double a = 0;
    double b = 10;  //10 muscle lengths (more will slow convergence by adding steps)

    double x = (a + b) / 2.0;
    double dx = 2 * tol;

    int neval = 0;

    SimTK::Vec3 forceResAndDerivative;

    //Perform a Newton Line Search
    while ((abs(dx) >= tol) && (neval < 100)) {

        neval++;

        // Set a to be lower value and b to be upper value
        a = min(a, b);
        b = max(a, b);


        forceResAndDerivative =
                calcFiberStaticEquilibResidual(x, muscleLength, activ);
        double fx = forceResAndDerivative[0];

        //After the 1st iteration, use the new guess as a new upper or lower bound
        if (neval > 1) {
            forceResAndDerivative =
                    calcFiberStaticEquilibResidual(a, muscleLength, activ);
            double funcA = forceResAndDerivative[0];

            if ((funcA * fx) > 0) {
                a = x;
            } else {
                b = x;
            }

            forceResAndDerivative =
                    calcFiberStaticEquilibResidual(x, muscleLength, activ);
            double dfx = forceResAndDerivative[1];
            //double forceRes = forceResAndDerivative[0];

            dx = -fx / dfx;
            double xdx = x - dx;

            bool inInterval = ((xdx >= a) && (xdx <= b));

            forceResAndDerivative =
                    calcFiberStaticEquilibResidual(xdx, muscleLength, activ);
            bool largeDeriv = abs(forceResAndDerivative[0]) > (0.5 * abs(fx));

            if (~inInterval || largeDeriv) {
                x = (a + b) / 2;
                dx = (a - b) / 2;
            } else {
                x = xdx;
            }

            // TODO:  Need to handle condition when number of loop iterations reaches
            // neval limit. See Millard2012 Muscle Error Handling
        }
    }
    SimTK::Vec2 vout;
    vout[0] = x;   //projFiberLengthNorm
    vout[1] = forceResAndDerivative[2];  // muscleForce
    return vout;
};


//------------------------------------------------------------------------------
//Calculate the ydot values to drive the residuals to 0 and "balance" the muscle

double VandenBogert2011Muscle::calcSolveMuscle(const SimTK::State& s,
          double excitation, SimTK::Vector projFibVelNormGuess) const {

//SimTK::Vector VandenBogert2011Muscle::calcSolveMuscle(const SimTK::State& s,
//         double activ, SimTK::Vector yDotInitialGuess)  {

    SimTK::State stemp=s;

    //ImplicitSystemForwardEulerStep sys(2, stemp, activ);
    ImplicitSystemForwardEulerStep sys(1, stemp, excitation);

    //TODO:  Need come up with reasonable bounds
    //SimTK::Vector lower_bounds(2);
    SimTK::Vector lower_bounds(1);
    lower_bounds[0] = -SimTK::Infinity;
    //lower_bounds[1] = -SimTK::Infinity;

    //SimTK::Vector upper_bounds(2);
    SimTK::Vector upper_bounds(1);
    upper_bounds[0] = SimTK::Infinity;
    //upper_bounds[1] = SimTK::Infinity;

    sys.setParameterLimits(lower_bounds, upper_bounds);


    SimTK::Optimizer opt(sys, SimTK::InteriorPoint); //Create the optimizer

    // Specify settings for the optimizer
    opt.setConvergenceTolerance(0.1);
    opt.useNumericalGradient(true, 1e-5);
    opt.useNumericalJacobian(true);
    opt.setMaxIterations(100);
    opt.setLimitedMemoryHistory(500);

    opt.optimize(projFibVelNormGuess);  // Optimize


    double projFibVelNorm=projFibVelNormGuess[0];
    //ydot[1]=yDotInitialGuess[1];

    return projFibVelNorm;
};











