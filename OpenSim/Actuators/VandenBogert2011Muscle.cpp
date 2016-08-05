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
}

//_____________________________________________________________________________
/*
 * Constructor.
 */

VandenBogert2011Muscle::VandenBogert2011Muscle(const std::string& aName)
{
    setName(aName);
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
    setAuthors("A. van den Bogert, B. Humphreys, C. Dembia");
    //constructProperty_max_isometric_force(1000);
    constructProperty_fMaxTendonStrain(0.033);
    constructProperty_fl_width(0.63);
    constructProperty_fv_AHill(0.25);
    constructProperty_fv_maxMultiplier(1.5);
    //constructProperty_optimal_fiber_length(0.1);
    constructProperty_dampingCoefficient(0.01);
    constructProperty_normFiberSlackLength(1.0);
    //constructProperty_tendon_slack_length(0.2);
    constructProperty_activTimeConstant(0.01);
    constructProperty_deactivTimeConstant(0.04);
    constructProperty_pennAtOptFiberLength(0);
    constructProperty_defaultActivation(0.1);
    constructProperty_default_fiber_length(1.0);
}


//--------------------------------------------------------------------------
// GET & SET Properties
//--------------------------------------------------------------------------


void VandenBogert2011Muscle::setFMaxTendonStrain(double fMaxTendonStrain) {
    set_fMaxTendonStrain(fMaxTendonStrain); }
double VandenBogert2011Muscle::getFMaxTendonStrain() const {
     return get_fMaxTendonStrain(); }

void VandenBogert2011Muscle::setFlWidth(double flWidth) {
     set_fl_width(flWidth); }
double VandenBogert2011Muscle::getFlWidth() const
    { return get_fl_width(); }

void VandenBogert2011Muscle::setFvAHill(double fvAHill) {
    set_fv_AHill(fvAHill); }
double VandenBogert2011Muscle::getFvAHill() const {
     return get_fv_AHill(); }

void VandenBogert2011Muscle::setFvmaxMultiplier(double fvMaxMultiplier) {
     set_fv_maxMultiplier(fvMaxMultiplier); }
double VandenBogert2011Muscle::getFvmaxMultiplier() const {
     return get_fv_maxMultiplier(); }

void VandenBogert2011Muscle::setDampingCoefficient(double dampingCoefficient) {
     set_dampingCoefficient(dampingCoefficient); }
double VandenBogert2011Muscle::getDampingCoefficient() const {
     return get_dampingCoefficient(); }

void VandenBogert2011Muscle::setNormFiberSlackLength(double
                                                     normFiberSlackLength) {
     set_normFiberSlackLength(normFiberSlackLength); }
double VandenBogert2011Muscle::getNormFiberSlackLength() const {
     return get_normFiberSlackLength(); }

void VandenBogert2011Muscle::setActivTimeConstant(double activTimeConstant) {
     set_activTimeConstant(activTimeConstant); }
double VandenBogert2011Muscle::getActivTimeConstant() const {
     return get_activTimeConstant(); }

void VandenBogert2011Muscle::setDeactivTimeConstant(double
                                                    deactivTimeConstant) {
     set_deactivTimeConstant(deactivTimeConstant); }
double VandenBogert2011Muscle::getDeactivTimeConstant() const {
     return get_deactivTimeConstant(); }

void VandenBogert2011Muscle::setPennAtOptFiberLength(double
                                                     pennAtOptFiberLength) {
    set_pennAtOptFiberLength(pennAtOptFiberLength); }
double VandenBogert2011Muscle::getPennAtOptFiberLength() const {
    return get_pennAtOptFiberLength(); }


void VandenBogert2011Muscle::setDefaultActivation(double
                                                     defaultActivation) {
    set_defaultActivation(defaultActivation); }
double VandenBogert2011Muscle::getDefaultActivation() const {
    return get_defaultActivation(); }

void VandenBogert2011Muscle::setDefaultFiberLength(double fiberLength)
{
    set_default_fiber_length(fiberLength);
}

double VandenBogert2011Muscle::getDefaultFiberLength() const
{   return get_default_fiber_length(); }


void VandenBogert2011Muscle::setFiberLength(SimTK::State& s, double fiberLength)
            const
{

        setStateVariableValue(s, "fiber_length",fiberLength);
        //markCacheVariableInvalid(s,"lengthInfo");
        //markCacheVariableInvalid(s,"velInfo");
        //markCacheVariableInvalid(s,"dynamicsInfo");
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
                                                double activdot_guess, double u)
                                                const
{
    ImplicitResidual residualStruct = calcImplicitResidual(s,
                                                           projFibVelNorm_guess,
                                                           activdot_guess,
                                                           u, 0);

    SimTK::Vec2 residual = {residualStruct.forceResidual,
                            residualStruct.activResidual};
    return(residual);
}



//These are hacks because I am not using muscle.h mli cache variables, yet:
double  VandenBogert2011Muscle::getFiberLength(SimTK::State& s) const
{
    double fiberLength=getStateVariableValue(s, "fiber_length");
    return fiberLength;
}
double  VandenBogert2011Muscle::getActivation(SimTK::State& s) const
{
    double activ=getStateVariableValue(s, "activation");
    return activ;
}



//==============================================================================
// Muscle.h Interface
//==============================================================================


double  VandenBogert2011Muscle::computeActuation(const SimTK::State& s) const
{
    double activ=getActivation(s);
    // For now we will start with static guess
    // TODO: figure out how to use previous state as guess
    SimTK::Vector ydotInitialGuess(2);
    ydotInitialGuess[0]=0.0;
    ydotInitialGuess[1]=0.0;
    SimTK::Vec2 sdot=calcSolveMuscle(s, activ, ydotInitialGuess);

    VandenBogert2011Muscle::ImplicitResidual Result= calcImplicitResidual(s,
                                          sdot[0],
                                          sdot[1],
                                          activ,
                                          0);

    //TODO: Could probably add a check here that residuals =0
    double actuation=Result.forceTendon;

    return(actuation);
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
    double activation = getDefaultActivation();
    SimTK::Vec2 lenghAndForce=calcFiberStaticEquilbirum(getLength(s),
                                                        activation);

    double projFibLenNorm=lenghAndForce[0];
    double tendonForce=lenghAndForce[1];

    //TODO:  Millard Muscle handles non-convergence here and zero muscle length.
    //     Need to consider adding.

    setActuation(s, tendonForce);
    double fiberLength =
            projFibLenToFiberLength(projFibLenNorm,1)*getOptimalFiberLength();
    setStateVariableValue(s, "fiber_length", fiberLength);
    setStateVariableValue(s, "activation",  activation);

}
// I am cheating here.  Instead of using mli and mdi caches
/*
SimTK::Vec2 VandenBogert2011Muscle::getStateVariables(const SimTK::State& s) {
    double fiberLength = getStateVariableValue(s, "fiber_length");
    double activ = getStateVariableValue(s, "activation");
    SimTK::Vec2 stateVariables = {fiberLength, activ};
    return stateVariables;};

*/


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

    addStateVariable("fiber_length");
    addStateVariable("activation");

}

void VandenBogert2011Muscle::extendInitStateFromProperties(SimTK::State& s)
const
{
    Super::extendInitStateFromProperties(s);

        setActivation(s, getDefaultActivation());
        setFiberLength(s, getDefaultFiberLength());
}

void VandenBogert2011Muscle::extendSetPropertiesFromState(const SimTK::State& s)
{
    Super::extendSetPropertiesFromState(s);

        setDefaultActivation(getStateVariableValue(s,"activation"));
        setDefaultFiberLength(getStateVariableValue(s,"fiber_length"));

}


void VandenBogert2011Muscle::
computeStateVariableDerivatives(const SimTK::State& s) const
{

    double activ=getActivation(s);
    // For now we will start with static guess
    // TODO: figure out how to use previous state as guess
    SimTK::Vector ydotInitialGuess(2);
    ydotInitialGuess[0]=0.0;
    ydotInitialGuess[1]=0.0;
    SimTK::Vec2 sdot=calcSolveMuscle(s, activ, ydotInitialGuess);

    setStateVariableDerivativeValue(s, "fiber_length", sdot[0]);
    setStateVariableDerivativeValue(s, "activation", sdot[1]);


}


//=============================================================================
// COMPUTATION
//=============================================================================
double VandenBogert2011Muscle::fiberLengthToProjectedLength
        (double fiberLength , bool normalizedValues=0) const {

    double pennAtOptFiberLength = getPennAtOptFiberLength();
    double projFibLen = 0;

    if (pennAtOptFiberLength < 0.01) {
        projFibLen = fiberLength;
    }

    else {

        double optimalFiberLength=getOptimalFiberLength();

        if (normalizedValues) {fiberLength=fiberLength*optimalFiberLength;};


        double muscleWidth = sin(
                pennAtOptFiberLength);  //b is the fixed distance of the fiber
        //perpindicular to the tendon (width)

        /*TODO: muscleWidth as cache variable
         * recalculated. Should be moved info muslceLengthInfo cache */

        // The fiber can not be shorter than b; if it is, the projected length
        // equation below will return a complex value.  It also physically can
        // not happen (the fiber being shorter than the fixed width)
        if (fiberLength >= muscleWidth) {
            projFibLen = sqrt(pow(fiberLength, 2) - pow(muscleWidth, 2));
        }
        else {
            projFibLen = SimTK::NaN;
        }  //TODO: Doing this for now, but need to
        // come up with a clamping scheme (Millard has one)

        if (normalizedValues) {projFibLen=projFibLen/optimalFiberLength;};


    };
    return projFibLen;
};


//------------------------------------------------------------------------------
double VandenBogert2011Muscle::projFibLenToFiberLength (double projFibLen,
                                                        bool normalizedValues=0)
                                                        const
{
    double pennAtOptFiberLength=getPennAtOptFiberLength();
    double fiberLength=0;

    double optimalFiberLength=getOptimalFiberLength();

    if (normalizedValues){
        projFibLen=projFibLen * optimalFiberLength;  //Convert from Norm
    }

    if (pennAtOptFiberLength < 0.01) {
        fiberLength = projFibLen; }
    else {
        double static muscleWidth = sin(pennAtOptFiberLength);
        fiberLength = sqrt(pow(projFibLen, 2) + pow(muscleWidth, 2));}

    if (normalizedValues){                      //Convert back to Norm if needed
        fiberLength=fiberLength / optimalFiberLength;
    }

    return fiberLength;
}

//------------------------------------------------------------------------------
double VandenBogert2011Muscle::fiberVelocityToProjFibVel
        (double fiberVelocity, double fiberLength, double projFiberLength,
         bool normalizedValues=0) const
{
    // Note that this normalized by fiber optimal length (not by max velocity)
    double optimalFiberLength = getOptimalFiberLength();

    if (normalizedValues) {
        fiberLength = fiberLength * getOptimalFiberLength();
        fiberVelocity = fiberVelocity * optimalFiberLength;
        projFiberLength = projFiberLength * optimalFiberLength;
    }

    double projFibVel = 0;

    if (fiberVelocity != 0) {
        projFibVel = fiberVelocity / cos(projFiberLength / fiberLength);
    }
    else { projFibVel = 0; }

    if (normalizedValues) {projFibVel
                                   = projFibVel / optimalFiberLength; };

    return projFibVel;
};

//------------------------------------------------------------------------------
double VandenBogert2011Muscle::fiberVelocityToProjFibVel
        (double fiberVelocity, double fiberLength, bool normalizedValues=0)
                const {

    //overload method when projFiberLength is not known/provided

    double projFiberLength=fiberLengthToProjectedLength(fiberLength,
                                                        normalizedValues);
    double projFiberVelocity = fiberVelocityToProjFibVel (fiberVelocity,
                                                          fiberLength,
                                                          projFiberLength,
                                                          normalizedValues);
    return projFiberVelocity;
};


//------------------------------------------------------------------------------
double VandenBogert2011Muscle::projFibVelToFiberVelocity(double projFibVel,
                                                         double fiberLength,
                                                         double projFiberLength,
                                                         bool normalizedValues=0)
                                                         const
{
// Note that this normalized by fiber optimal length (not by max velocity)
    double optimalFiberLength = getOptimalFiberLength();

    if (normalizedValues)
    {
        fiberLength = fiberLength * getOptimalFiberLength();
        projFibVel = projFibVel * optimalFiberLength;
        projFiberLength = projFiberLength * optimalFiberLength;
    }

    double fiberVelocity = 0;

    if (projFibVel != 0)
    {
        fiberVelocity = projFibVel * cos(projFiberLength / fiberLength);
    }
    else { fiberVelocity = 0; }

    if (normalizedValues)
    {
        fiberVelocity = fiberVelocity / optimalFiberLength;
    };

    return fiberVelocity;}

//------------------------------------------------------------------------------
double VandenBogert2011Muscle::projFibVelToFiberVelocity
        (double projFibVel, double projFibLength, bool normalizedValues=0)
                        const {

    //overload method when fiberLength is not known/provided

    double fiberLength=projFibLenToFiberLength(fiberLength ,normalizedValues);
    double fiberVelocity = projFibVelToFiberVelocity(projFibVel, fiberLength,
                                                     projFibLength,
                                                     normalizedValues);

    return fiberVelocity;
};



//------------------------------------------------------------------------------
VandenBogert2011Muscle::ImplicitResidual
VandenBogert2011Muscle::calcImplicitResidual(const SimTK::State& s,
                                             double projFibVelNorm_guess,
                                             double activdot_guess,
                                             double u, int returnJacobians)


{
    // Overload method for state as parameters

    double muscleLength = getLength(s);

    //double activ =getActivations(s); //Does not work because info not implemented
    //double activ = getStateVariableValues(s,"activation");  // does not work
    //double fiberLength = getStateVariableValues(s,"fiber_length");

    SimTK::Vector stateVec =getStateVariableValues(s);
    double fiberLength =stateVec[0];
    double activ =stateVec[1];


    double projFibLenNorm =
            fiberLengthToProjectedLength(fiberLength/getOptimalFiberLength(),1);

    VandenBogert2011Muscle::ImplicitResidual Results= calcImplicitResidual(
            muscleLength, projFibLenNorm, activ, projFibVelNorm_guess,
            activdot_guess, u, 0);

    return Results;
}


//------------------------------------------------------------------------------
VandenBogert2011Muscle::ImplicitResidual
VandenBogert2011Muscle::calcImplicitResidual(SimTK::Vec2 y,
                                             SimTK::Vec2 ydot_guess,
                                             double muscleLength,
                                             double u, int returnJacobians=0)
const
{
    // Overload method for state vectors as parameters
    VandenBogert2011Muscle::ImplicitResidual Results= calcImplicitResidual(
            muscleLength, y[0], y[1], ydot_guess[0], ydot_guess[1], u,
            returnJacobians);
    return Results;

}

//------------------------------------------------------------------------------
VandenBogert2011Muscle::ImplicitResidual VandenBogert2011Muscle::
calcImplicitResidual(double muscleLength, double projFibLenNorm, double activ,
                     double projFibVelNorm, double activdot, double u,
                     int returnJacobians=0) const {


    //TODO: Match symbols to doxygen diagram and Add symbolic equations comments
    //TODO: May want to make this into a seperate function

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
    double fv_AHill= getFvAHill();   //Hill parameter of the f-v relationship

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
    double fiberSlackLengthNorm= getNormFiberSlackLength();

    //L_{slack,tendon} (m) slack length of the tendon
    double tendonSlackLength = getTendonSlackLength();

    //T_{act} (s) Activation time
    double activTimeConstant = getActivTimeConstant();

    //T_{deact} (s) Deactivation time
    double deactivTimeConstant = getDeactivTimeConstant();

    //phi_{opt} pennation at Optimal Fiber Length
    double pennAtOptFiberLength = getPennAtOptFiberLength();


    // constants derived from the muscle parameters
    double vMaxNorm = 10 * optFiberLength;
    //Maximum shortening velocity in optimal fiber lengths per sec

    // Jacobian Matrices
    SimTK::Mat22 df_dy;
    SimTK::Mat22 df_dydot;


    //-------Convert projFiberLength & Velocity to fiberLength & Velocity------/
    double cosPenn;
    double dcosPenn_dprojFibLen;
    double fiberLengthNorm;
    double dfiberLength_dprojFibLen;
    if (pennAtOptFiberLength<0.01)  {
        // If pennation is zero, we can't do this because volume is zero,
        //      and fiberLength ~ projFiberLength
        cosPenn=1.0;
        fiberLengthNorm=projFibLenNorm;
        dfiberLength_dprojFibLen=1;
        dcosPenn_dprojFibLen=0;
    }
    else {
        double b=sin(pennAtOptFiberLength);
        fiberLengthNorm=sqrt(pow(projFibLenNorm,2) + pow(b,2));
        cosPenn = projFibLenNorm/fiberLengthNorm;
        dfiberLength_dprojFibLen=cosPenn;
        dcosPenn_dprojFibLen=pow(b,2) / pow(fiberLengthNorm,3);

    }

    // Compute fiberVelocity and its derivatives wrt projFibLen and projFibVel
    double fiberVelocityNorm = projFibVelNorm*cosPenn;
    double dfiberVelocityNorm_dprojFibVelNorm = cosPenn;
    double dfiberVelocityNorm_dprojFibLenNorm =
            projFibVelNorm * dcosPenn_dprojFibLen;


    //---F1 is the normalized isometric force-length relationship at maximum
    //                                               activation--------------//
    double fiberExp = (fiberLengthNorm - 1.0) / fl_width;   // [dimensionless]
    double F1 = exp(-pow(fiberExp, 2));        // Gaussian force-length curve

    double dF1_dfiberLengthNorm = 0;
    if (returnJacobians) {
        dF1_dfiberLengthNorm = -2.0 * fiberExp * F1 / fl_width;
        double dF1_dprojFibLenNorm =
                dF1_dfiberLengthNorm * dfiberLength_dprojFibLen;
    }



    //-------- F2 is the dimensionless force-velocity relationship -------//
    double F2;
    double dF2_dfiberVelocityNorm;
    double dF2_dactiv;
    double dF2_dprojFibVelNorm;
    double dF2_dprojFibLenNorm;
    double df_dmuscleLength;

    // Chow/Darling Vel-Activation Relationship //TODO:  Add full reference
    //double lambda = 0.5025 + 0.5341*activ;
    //double  dlambda_da = 0.5341;
    double lambda = 1;   //Turn it off for now as it seems to cause an issue
                        // with negative force large concentric vel
    double dlambda_da =0;

    if (fiberVelocityNorm < 0) {
        //Hill's equation for concentric contraction
        // F2 = (V_{max} + V_{fiber}) / (V_{max} - V_{fiber}/a_{Hill})

        double hillDenom = (lambda*vMaxNorm - fiberVelocityNorm/fv_AHill);
        F2 = (lambda*vMaxNorm + fiberVelocityNorm) / hillDenom;

        if (returnJacobians) {
            dF2_dfiberVelocityNorm  = (1.0 + F2 / fv_AHill) / hillDenom;
            dF2_dactiv = - dlambda_da * vMaxNorm * fiberVelocityNorm *
                    (1.0 + 1.0/fv_AHill) / pow(hillDenom,2);
        }
    }
    else {
        //Katz Curve for eccentric contraction
        // c is Katz Constant
        double c3 = vMaxNorm * fv_AHill * (fv_maxMultiplier - 1.0) /
                (fv_AHill + 1.0); // parameter in the eccentric f-v equation
        double c = lambda*c3;
        //F2 = (g_{max} * V_{fiber} + c) / (V_{fiber} + c)
        double katzDenom = (fiberVelocityNorm  + c);
        F2 = (fv_maxMultiplier * fiberVelocityNorm  + c) / katzDenom ;
        if (returnJacobians) {
            dF2_dfiberVelocityNorm = (fv_maxMultiplier - F2) / katzDenom ;
            dF2_dactiv = dlambda_da * c3 * fiberVelocityNorm *
                    (1-fv_maxMultiplier) / pow(katzDenom,2);
        }
    }
    if (returnJacobians){
        dF2_dprojFibVelNorm =
                dF2_dfiberVelocityNorm * dfiberVelocityNorm_dprojFibVelNorm;
        dF2_dprojFibLenNorm =
                dF2_dfiberVelocityNorm * dfiberVelocityNorm_dprojFibLenNorm;
    }


    //------F3 is the dimensionless fiber (PEE) force (in units of Fmax)------//



    double dF3_dprojFibLenNorm;

    // stiffness of the linear term is 1 N/m, convert to Fmax/Lceopt units
    double kPEE = 1.0 / maxIsoForce * optFiberLength;
    // elongation of fiber (PEE), relative to Lceopt
    double kPEE2Norm = 1 / pow(fl_width, 2);  //Fiber (PEE) quadratic stiffness,
    //          so Fpee = Fmax when Lce = Lce*(1+W)
    double elongationFiberNorm = (fiberLengthNorm - fiberSlackLengthNorm);
    double F3 = kPEE * elongationFiberNorm;
    // low stiffness linear term
    double dF3_dfiberLengthNorm = kPEE;
    if (elongationFiberNorm > 0) {
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

    // Tendon (SEE) quadratic stiffness, so Fsee = Fmax at strain of umax
    // This is normalized by Fmax
    double kSEE2 = maxIsoForce / pow(tendonSlackLength * fMaxTendonStrain, 2);

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
    if (elongationTendon > 0) {
        // add quadratic term for positive deformation
        F4 = F4 + (kSEE2 * pow(elongationTendon, 2))/maxIsoForce;
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
    //TODO:  Should this be: dampingCoeff * fiberVelocity  (not proj)?
    //TODO:  Why are damping coeff used instead of damping ratio?
    double dF5_dprojFibVelNorm  = dampingCoeff;


    // ---------Calculate the Muscle Force Residual ---------------------- //
    //The muscle dynamics equation: f = Fsee - (a*Fce - Fpee)*cos(Penn) -
    //                                                          Fdamping = 0

    double fRes = F4 - (activ * F1 * F2 + F3)*cosPenn - F5;


// This is just here for troubleshooting
if (returnJacobians==2){
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
    cout << "   kPEE: " << kPEE << endl;
    cout << "   kPEE2Norm: " << kPEE2Norm << endl;
    cout << "   elongationFiberNorm: " <<  elongationFiberNorm << endl;
    cout << "" << endl;};

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

    double activationResidual = activdot - (u - activ) *
                    (u / activTimeConstant + (1 - u) / deactivTimeConstant );
    SimTK::Vec2 df_du;
    double dActRes_dactiv=0;
    double dActRes_dactivdot = 0;

    if (returnJacobians) {
        dActRes_dactiv= (u / activTimeConstant + (1 - u) / deactivTimeConstant);

        dActRes_dactivdot = 1;


        df_du[0]=0;
        df_du[1] = -(u / activTimeConstant + (1 - u) / deactivTimeConstant )
                       - (u - activ) * (1 / activTimeConstant - 1 /
                                                deactivTimeConstant );
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



    VandenBogert2011Muscle::ImplicitResidual Results;

    Results.forceResidual=fRes;
    Results.activResidual=activationResidual;
    Results.forceTendon=Fsee;
    Results.df_dy=df_dy;
    Results.df_dydot=df_dydot;
    Results.df_du=df_du;
    Results.df_dmuscleLength=df_dmuscleLength;
    Results.F1=F1;    //Output force components for troubleshooting
    Results.F2=F2;
    Results.F3=F3;
    Results.F4=F4;
    Results.F5=F5;



return Results; }






//------------------------------------------------------------------------------
VandenBogert2011Muscle::ImplicitResidual
VandenBogert2011Muscle::calcJacobianByFiniteDiff(double muscleLength,
                                                 double projFibLenNorm,
                                                 double activ,
                                                 double projFibVelNorm,
                                                 double activdot, double u,
                                                 double h) const
{
    SimTK::Vec2 y;
    SimTK::Vec2 ydot;
    y[0]=projFibLenNorm;
    y[1]=activ;
    ydot[0]=projFibVelNorm;
    ydot[1]=activdot;

    // Overload method for state vectors as parameters
    VandenBogert2011Muscle::ImplicitResidual Results =
            calcJacobianByFiniteDiff(y, ydot, muscleLength, u, h );

    return Results;
}


//------------------------------------------------------------------------------
VandenBogert2011Muscle::ImplicitResidual    VandenBogert2011Muscle::
calcJacobianByFiniteDiff(SimTK::Vec2 y, SimTK::Vec2 ydot, double muscleLength,
                         double u, double h ) const {



    //TODO: In the optimizer class, you do not have to supply the Jacobian
    // function.  The must mean there is code in Simbody that does finite diff.
    // Should probably look at calling that in place of this code.

    // Jacobian Matrices
    SimTK::Mat22 df_dy;
    SimTK::Mat22 df_dydot;
    SimTK::Vec2 df_du;

    VandenBogert2011Muscle::ImplicitResidual opPoint;
    opPoint=calcImplicitResidual (y,ydot,muscleLength,u,0);
    double opForceResidual=opPoint.forceResidual;
    double opActivResidual=opPoint.activResidual;

    VandenBogert2011Muscle::ImplicitResidual del;

    //----------df_dy------------//
    SimTK::Vec2 dh = {h,0};
    del = calcImplicitResidual(y+dh,ydot,muscleLength,u,0);
    df_dy[0][0]= (del.forceResidual-opForceResidual)/h;
    df_dy[1][0]= (del.activResidual-opActivResidual)/h;

    dh = {0,h};
    del = calcImplicitResidual(y+dh,ydot,muscleLength,u,0);
    df_dy[1][0]= (del.forceResidual-opForceResidual)/h;
    df_dy[1][1]= (del.activResidual-opActivResidual)/h;

    //----------df_dydot------------//
    dh = {h,0};
    del = calcImplicitResidual(y,ydot+dh,muscleLength,u,0);
    df_dydot[0][0]= (del.forceResidual-opForceResidual)/h;
    df_dydot[1][0]= (del.activResidual-opActivResidual)/h;

    dh = {0,h};
    del = calcImplicitResidual(y,ydot+dh,muscleLength,u,0);
    df_dydot[1][0]= (del.forceResidual-opForceResidual)/h;
    df_dydot[1][1]= (del.activResidual-opActivResidual)/h;

    //----------df_du----------------//
    del = calcImplicitResidual(y,ydot,muscleLength,u+h,0);
    df_du[0]= (del.forceResidual-opForceResidual)/h;
    df_du[1]= (del.activResidual-opActivResidual)/h;

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

    opPoint.df_dy=df_dy;
    opPoint.df_dydot=df_dydot;
    opPoint.df_du=df_du;

    return opPoint;

}


//------------------------------------------------------------------------------
SimTK::Vec3 VandenBogert2011Muscle::calcFiberStaticEquilibResidual(
        double projFibLen, double muscleLength, double activ) const
{
// Just a quick convience method to get the force residual
// under static conditions

    VandenBogert2011Muscle::ImplicitResidual Results = calcImplicitResidual(
            muscleLength, projFibLen, activ, 0.0, 0.0, activ, 1.0);

    SimTK::Vec3 resAndDerivative;

    resAndDerivative[0]=Results.forceResidual;
    resAndDerivative[1]=Results.df_dy[0][0];
    resAndDerivative[2]=Results.forceTendon;

    return resAndDerivative;

}

//------------------------------------------------------------------------------
SimTK::Vec2 VandenBogert2011Muscle::calcFiberStaticEquilbirum(
        double muscleLength, double activ) const {

    // This code assumes that the muscle lengthing speed is 0.


    //TODO: Code is not optimized.  Specifically the number of calls to
    //calcImplicitResidual can be reduced

    //TODO: Generalize with a Lambda function (will need help with that).
    //TODO: calcImplicitResidual really only needs to calculate df_ds
    //       (single element) for this function


double tol=1e-8; //TODO : Look at changing to propotional of eps
double a=0;
double b=10;  //10 muscle lengths (more will slow convergence by adding steps)

double x=(a+b)/2.0;
double dx=2*tol;

int neval=0;

SimTK::Vec3 forceResAndDerivative;

    //Perform a Newton Line Search
while ((abs(dx)>=tol) && (neval<100)) {

    neval++;

    // Set a to be lower value and b to be upper value
    a=min(a,b);
    b=max(a,b);


    forceResAndDerivative =
            calcFiberStaticEquilibResidual(x, muscleLength, activ);
    double fx = forceResAndDerivative[0];

    //After the 1st iteration, use the new guess as a new upper or lower bound
    if (neval>1) {
        forceResAndDerivative =
                calcFiberStaticEquilibResidual(a, muscleLength, activ);
        double funcA = forceResAndDerivative[0];

        if ((funcA *fx)>0){
            a = x;}
        else { b = x;}}

    forceResAndDerivative =
            calcFiberStaticEquilibResidual(x, muscleLength, activ);
    double dfx= forceResAndDerivative[1];
    //double forceRes=forceResAndDerivative[0];

    dx =-fx/dfx;
    double xdx=x-dx;

    bool inInterval=((xdx>=a) && (xdx<=b));

    forceResAndDerivative =
            calcFiberStaticEquilibResidual(xdx, muscleLength, activ);
    bool largeDeriv=abs(forceResAndDerivative[0])> (0.5*abs(fx));

    if (~inInterval || largeDeriv) {
        x=(a+b)/2;
        dx=(a-b)/2;}
    else
        {x=xdx;};


    };

    // TODO:  Need to handle condition when number of loop iterations reaches
    // neval limit. See Millard2012 Muscle Error Handling

    SimTK::Vec2 vout;
    vout[0]=x;   //projFiberLengthNorm
    vout[1]=forceResAndDerivative[2];  // muscleForce
    return vout;
}




//------------------------------------------------------------------------------
//Calculate the ydot values to drive the residuals to 0 and "balance" the muscle
SimTK::Vector VandenBogert2011Muscle::calcSolveMuscle(SimTK::State& s,
         double activ, SimTK::Vector yDotInitialGuess) const {


    ImplicitSystemDerivativeSolver sys(2, s, activ);

    //TODO:  Need come up with reasonable bounds
    SimTK::Vector lower_bounds(2);
    lower_bounds[0] = -SimTK::Infinity;
    lower_bounds[1] = -SimTK::Infinity;

    SimTK::Vector upper_bounds(2);
    upper_bounds[0] = SimTK::Infinity;
    upper_bounds[1] = SimTK::Infinity;

    sys.setParameterLimits(lower_bounds, upper_bounds);

    SimTK::Optimizer opt(sys, SimTK::InteriorPoint); //Create the optimizer

    // Specify settings for the optimizer
    opt.setConvergenceTolerance(0.1);
    opt.useNumericalGradient(true, 1e-5);
    opt.useNumericalJacobian(true);
    opt.setMaxIterations(100);
    opt.setLimitedMemoryHistory(500);

    opt.optimize(yDotInitialGuess);  // Optimize

    return yDotInitialGuess;
}

//------------------------------------------------------------------------------
//Create a optimization solver to balance the muscle
class ImplicitSystemDerivativeSolver : public SimTK::OptimizerSystem {
public:

    //TODO:  Add back in option to use NR solver (and perform benchmark)
    /* Constructor class. Parameters passed are accessed in the objectiveFunc()
     * class. */
    ImplicitSystemDerivativeSolver(int numParameters, SimTK::State& s,
                                   double& u):
            OptimizerSystem(numParameters),
            numControls(numParameters),
            si(s),
            excitation(u)
    {}

    int objectiveFunc(const SimTK::Vector &new_yDotGuess, bool new_params,
                      SimTK::Real& f) const override {
        // No objective function
        f = 0;
        return 0;
    }

    int constraintFunc(const SimTK::Vector &new_yDotGuess, bool newParams,
                       SimTK::Vector& constraints)  override {
        //The constraints function is the residuals

        double projFibVelNorm_guess=new_yDotGuess[0];
        double activdot_guess=new_yDotGuess[1];
        VandenBogert2011Muscle::ImplicitResidual
                Results = VandenBogert2011Muscle::calcImplicitResidual
                (si,projFibVelNorm_guess,activdot_guess,excitation,0);

        constraints[0]=Results.forceResidual;
        constraints[1]=Results.activResidual;

        return 0;
    }

private:
    int numControls;
    SimTK::State& si;
    double& excitation;
};



 SimTK::Mat22  VandenBogert2011Muscle::fixMat22(SimTK::Mat22 matIn,
                                                SimTK::Mat22 matFixed) const{
     for (int m =0; m<=1; m=m+1) {
         for (int n =0; n<=1; n=n+1) {
             matFixed[m][n]=matIn[m][n];
         }}
     return matFixed;
 }

SimTK::Mat33 VandenBogert2011Muscle::quickMat33() const {
    SimTK::Mat33 m;
    m[1][1]=1;
    return m;
}

SimTK::Mat22 VandenBogert2011Muscle::quickMat22() const {
    SimTK::Mat22 m;
    m[1][1]=2;
    return m;
}


SimTK::Vec4 VandenBogert2011Muscle::quickVec4() const {
    SimTK::Vec4 m;
    m[1]=2;
    return m;
}

SimTK::Vec4 VandenBogert2011Muscle::flattenMat22(SimTK::Mat22 matIn) const {
    SimTK::Vec4 vecOut;
    vecOut[0]=matIn[0][0];
    vecOut[1]=matIn[0][1];
    vecOut[2]=matIn[1][0];
    vecOut[3]=matIn[1][1];
    return vecOut;
}
