/* -------------------------------------------------------------------------- *
 *                  OpenSim:  testMuscleMetabolicsProbes.cpp                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2013 Stanford University and the Authors                *
 * Author(s): Thomas Uchida                                                   *
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


//==============================================================================
// Tests for the Umberger2010 and Bhargava2004 muscle metabolics probes.
//
// A. We compare results from the Umberger2010 probe to those published by
//    Umberger et al. (Figure 1 in [1]). This test is unaffected by the later
//    revisions made by Umberger [2]. The muscle model consists only of the
//    contractile element described by van Soest and Bobbert [3], with some
//    modifications (described on pp. 100-101 of [1]).
//
// B. We attach both probes to a Millard2012Equilibrium muscle and confirm that
//    reasonable results are reported in a variety of scenarios.
//
// References:
// 1. Umberger, B.R., Gerritsen, K.G.M., Martin, P.E. (2003) A model of human
//    muscle energy expenditure. Computer Methods in Biomechanics and Biomedical
//    Engineering 6(2):99-111.
// 2. Umberger, B.R. (2010) Stance and swing phase costs in human walking.
//    Journal of the Royal Society Interface 7(50):1329-1340.
// 3. van Soest, A.J., Bobbert, M.F. (1993) The contribution of muscle
//    properties in the control of explosive movements. Biological Cybernetics
//    69(3):195-204.
//==============================================================================


#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Actuators/osimActuators.h>
#include <OpenSim/Actuators/ZerothOrderMuscleActivationDynamics.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Simulation/Model/Umberger2010MuscleMetabolicsProbe.h>
//#include <OpenSim/Analyses/ForceReporter.h>
//#include <OpenSim/Analyses/ProbeReporter.h>
//#include <OpenSim/Simulation/Model/Bhargava2004MuscleMetabolicsProbe.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

//#define DISPLAY_RESULTS
#define DISPLAY_ERRORS

using namespace OpenSim;
using namespace SimTK;
using namespace std;


//==============================================================================
//                              EXCITATION GETTER
//==============================================================================
// Supplies muscle excitation to the ZerothOrderMuscleActivationDynamics model
// used by UmbergerMuscle.
class MyExcitationGetter : public MuscleActivationDynamics::ExcitationGetter {
public:
    MyExcitationGetter(Muscle& muscle) : _muscle(muscle) {}

    double getExcitation(const SimTK::State& s) const OVERRIDE_11
    { return _muscle.getControl(s); }

private:
    const Muscle& _muscle;
};


//==============================================================================
//                               UMBERGER MUSCLE
//==============================================================================
// Muscle model used by Umberger et al., which is a slightly modified version of
// the contractile element described by van Soest and Bobbert.
class UmbergerMuscle : public Muscle {
OpenSim_DECLARE_CONCRETE_OBJECT(UmbergerMuscle, Muscle);
public:
    OpenSim_DECLARE_PROPERTY(width, double,
        "Normalized width of the active-force-length curve.");
    OpenSim_DECLARE_PROPERTY(Arel, double,
        "Arel = A_Hill * fiberActiveForceLengthMultiplier / maxIsometricForce");
    OpenSim_DECLARE_PROPERTY(Brel, double,
        "Brel = B_Hill / optimalFiberLength");
    OpenSim_DECLARE_PROPERTY(FmaxEccentric, double,
        "Asymptote on the eccentric side of the force-velocity curve.");
    OpenSim_DECLARE_UNNAMED_PROPERTY(ZerothOrderMuscleActivationDynamics,
        "Activation dynamic model that simply sets activation to excitation.");

    //--------------------------------------------------------------------------
    // CONSTRUCTOR
    //--------------------------------------------------------------------------
    UmbergerMuscle(const std::string& muscleName, double maxIsometricForce,
                   double optimalFiberLength, double width, double Arel,
                   double Brel, double FmaxEccentric)
    {
        setName(muscleName);
        setMaxIsometricForce(maxIsometricForce);
        setOptimalFiberLength(optimalFiberLength);
        setMaxContractionVelocity(Brel/Arel);

        constructProperty_width(width);
        constructProperty_Arel(Arel);
        constructProperty_Brel(Brel);
        constructProperty_FmaxEccentric(FmaxEccentric);

        constructProperty_ZerothOrderMuscleActivationDynamics(
            ZerothOrderMuscleActivationDynamics());
        upd_ZerothOrderMuscleActivationDynamics().setExcitationGetter(
            new MyExcitationGetter(*this));
    }

    //--------------------------------------------------------------------------
    // MODELCOMPONENT INTERFACE
    //--------------------------------------------------------------------------
    void connectToModel(Model& model) OVERRIDE_11
    {
        ZerothOrderMuscleActivationDynamics &zomad =
            upd_ZerothOrderMuscleActivationDynamics();
        includeAsSubComponent(&zomad);
        Super::connectToModel(model);
    }

    void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11
    {
        Super::addToSystem(system);
        addStateVariable(stateName_fiberLength,   SimTK::Stage::Position);
        addStateVariable(stateName_fiberVelocity, SimTK::Stage::Dynamics);
    }

    void initStateFromProperties(SimTK::State& s) const OVERRIDE_11
    {
        Super::initStateFromProperties(s);
        setFiberLength(s, getOptimalFiberLength());
    }

    void setPropertiesFromState(const SimTK::State& s) OVERRIDE_11
    {
        Super::setPropertiesFromState(s);
        setOptimalFiberLength(getFiberLength(s));
    }

    SimTK::Vector computeStateVariableDerivatives(const SimTK::State& s) const
        OVERRIDE_11
    {
        // This implementation is not intended for use in dynamic simulations.
        SimTK::Vector derivs = Super::computeStateVariableDerivatives(s);
        const int n = derivs.size();
        derivs.resizeKeep(n + getNumStateVariables());
        for (int i=0; i<getNumStateVariables(); ++i)
            derivs[n+i] = 0;
        return derivs;
    }

    //--------------------------------------------------------------------------
    // MUSCLE INTERFACE
    //--------------------------------------------------------------------------
    void computeInitialFiberEquilibrium(SimTK::State& s) const OVERRIDE_11 {}

    void setActivation(SimTK::State& s, double activation) const OVERRIDE_11
    { get_ZerothOrderMuscleActivationDynamics().setActivation(s,activation); }

    double computeActuation(const SimTK::State& s) const OVERRIDE_11
    { return get_ZerothOrderMuscleActivationDynamics().getActivation(s); }

    // Calculate position-level variables.
    void calcMuscleLengthInfo(const SimTK::State& s, MuscleLengthInfo& mli)
        const OVERRIDE_11
    {
        mli.fiberLength            = getStateVariable(s, stateName_fiberLength);
        mli.fiberLengthAlongTendon = mli.fiberLength;
        mli.normFiberLength        = mli.fiberLength / getOptimalFiberLength();
        mli.tendonLength           = 0;
        mli.normTendonLength       = 0;
        mli.tendonStrain           = 0;
        mli.pennationAngle         = 0;
        mli.cosPennationAngle      = 1;
        mli.sinPennationAngle      = 0;
        mli.fiberPassiveForceLengthMultiplier = 0;

        // The fiberActiveForceLengthMultiplier (referred to as 'Fisom' in [3])
        // is the proportion of maxIsometricForce that would be delivered
        // isometrically at maximal activation. Fisom=1 if Lce=Lceopt.
        if (mli.fiberLength < (1 - get_width()) * getOptimalFiberLength() ||
            mli.fiberLength > (1 + get_width()) * getOptimalFiberLength())
            mli.fiberActiveForceLengthMultiplier = 0;
        else {
            double c  = -1.0 / (get_width() * get_width());
            double t1 = mli.fiberLength / getOptimalFiberLength();
            mli.fiberActiveForceLengthMultiplier = c*t1*(t1-2) + c + 1;
        }
    }

    // Calculate velocity-level variables.
    void calcFiberVelocityInfo(const SimTK::State& s, FiberVelocityInfo& fvi)
        const OVERRIDE_11
    {
        fvi.fiberVelocity = getStateVariable(s, stateName_fiberVelocity);
        fvi.fiberVelocityAlongTendon = fvi.fiberVelocity;
        fvi.normFiberVelocity        = fvi.fiberVelocity /
                        (getMaxContractionVelocity() * getOptimalFiberLength());

        fvi.pennationAngularVelocity     = 0;
        fvi.tendonVelocity               = 0;
        fvi.normTendonVelocity           = 0;
        fvi.fiberForceVelocityMultiplier = 1;
    }

    // Calculate dynamics-level variables.
    void calcMuscleDynamicsInfo(const SimTK::State& s, MuscleDynamicsInfo& mdi)
        const OVERRIDE_11
    {
        mdi.activation = computeActuation(s);

        // These expressions were obtained by solving the 'Vce' equations in [3]
        // for force F, then applying the modifications described in [1].
        // Negative fiber velocity corresponds to concentric contraction.
        double ArelStar = pow(mdi.activation,-0.3) * get_Arel();
        if (getFiberVelocity(s) <= 0) {
            double v  = max(getFiberVelocity(s),
                        -getMaxContractionVelocity() * getOptimalFiberLength());
            double t1 = get_Brel() * getOptimalFiberLength();
            mdi.fiberForce = (t1*getActiveForceLengthMultiplier(s) + ArelStar*v)
                             / (t1 - v);
        } else {
            double c2 = -get_FmaxEccentric() / mdi.activation;
            double c3 = (get_FmaxEccentric()-1) * get_Brel() / (mdi.activation *
                            2 * (getActiveForceLengthMultiplier(s) + ArelStar));
            double c1 = (get_FmaxEccentric()-1) * c3 / mdi.activation;
            mdi.fiberForce = -(getOptimalFiberLength() * (c1 + c2*c3)
                               + c2*getFiberVelocity(s)) /
                             (getFiberVelocity(s) + c3*getOptimalFiberLength());
        }
        mdi.fiberForce *= getMaxIsometricForce() * mdi.activation;

        mdi.fiberForceAlongTendon = mdi.fiberForce;
        mdi.normFiberForce        = mdi.fiberForce / getMaxIsometricForce();
        mdi.activeFiberForce      = mdi.fiberForce;
        mdi.passiveFiberForce     = 0;
        mdi.tendonForce           = mdi.fiberForce;
        mdi.normTendonForce       = mdi.normFiberForce;
        mdi.fiberStiffness        = 0;
        mdi.fiberStiffnessAlongTendon = 0;
        mdi.tendonStiffness       = 0;
        mdi.muscleStiffness       = 0;
        mdi.fiberActivePower      = 0;
        mdi.fiberPassivePower     = 0;
        mdi.tendonPower           = 0;
        mdi.musclePower           = 0;
    }

    //--------------------------------------------------------------------------
    // SET FIBER LENGTH AND VELOCITY
    //--------------------------------------------------------------------------
    void setFiberLength(SimTK::State& s, double fiberLength) const
    { setStateVariable(s, stateName_fiberLength, fiberLength); }

    void setNormFiberVelocity(SimTK::State& s, double normFiberVelocity) const
    {
        setStateVariable(s, stateName_fiberVelocity, normFiberVelocity *
                         getMaxContractionVelocity() * getOptimalFiberLength());
    }

private:
    static const std::string stateName_fiberLength;
    static const std::string stateName_fiberVelocity;
};
const std::string UmbergerMuscle::stateName_fiberLength   = "fiber_length";
const std::string UmbergerMuscle::stateName_fiberVelocity = "fiber_velocity";


//==============================================================================
//                    CONSTANT EXCITATION MUSCLE CONTROLLER
//==============================================================================
// Simple controller to maintain muscle excitation at a constant value. The
// Umberger2010 probe depends on both excitation and activation.
class ConstantExcitationMuscleController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(ConstantExcitationMuscleController, Controller);
public:
    ConstantExcitationMuscleController(double u) : _u(u) {}

    void computeControls(const SimTK::State& s, SimTK::Vector &controls) const
        OVERRIDE_11
    { controls[0] = _u; }

private:
    double _u;
};


//==============================================================================
//                 COMPARE UMBERGER PROBE TO PUBLISHED RESULTS
//==============================================================================
// Normalized force, mechanical power, and total energy liberation rate are
// calculated as functions of normalized shortening velocity for the soleus and
// rectus femoris muscles. These data are then compared to either analytical
// expressions or polynomials fit to the results published by Umberger et al.
const int numPoints = 21;
void generateMuscleData(const std::string& muscleName,
                        double  maxIsometricForce,
                        double  optimalFiberLength,
                        double  width,
                        double  Arel,
                        double  Brel,
                        double  FmaxEccentric,
                        double  slowTwitchRatio,
                        double  muscleMass,
                        double* normalizedForce,
                        double* mechanicalPower,
                        double* totalEnergyRate)
{
    // Create OpenSim model.
    Model model;
    model.setName("testModel_"+muscleName);
    OpenSim::Body& ground = model.getGroundBody();

    // Create block. The length and velocity of the muscle will be specified, so
    // the properties of the block are inconsequential.
    const double blockMass       = 1.0;
    const double blockSideLength = 0.1;
    Inertia blockInertia = blockMass * Inertia::brick(Vec3(blockSideLength/2));
    OpenSim::Body *block = new OpenSim::Body("block", blockMass, Vec3(0),
                                             blockInertia);

    // Create slider joint between ground and block.
    OpenSim::SliderJoint prismatic("prismatic", ground, Vec3(0), Vec3(0),
                                                *block, Vec3(0), Vec3(0));
    CoordinateSet& prisCoordSet = prismatic.upd_CoordinateSet();
    prisCoordSet[0].setName("xTranslation");
    model.addBody(block);

    // Create muscle attached to ground and block.
    UmbergerMuscle *muscle = new UmbergerMuscle(muscleName, maxIsometricForce,
        optimalFiberLength, width, Arel, Brel, FmaxEccentric);
    muscle->addNewPathPoint("muscle-ground", ground, Vec3(0));
    muscle->addNewPathPoint("muscle-block",  *block, Vec3(0));
    model.addForce(muscle);

    // Attach muscle controller.
    const double constantActivation = 1.0;
    ConstantExcitationMuscleController* controller =
        new ConstantExcitationMuscleController(constantActivation);
    controller->setActuators(model.updActuators());
    model.addController(controller);

    // Attach Umberger probes. Must call addProbe() before addMuscle().
    Umberger2010MuscleMetabolicsProbe* mechanicalPowerProbe =
        new Umberger2010MuscleMetabolicsProbe(false, false, false, true);
    model.addProbe(mechanicalPowerProbe);
    mechanicalPowerProbe->setName("mechanicalPowerProbe");
    mechanicalPowerProbe->set_probe_operation("value");
    mechanicalPowerProbe->addMuscle(muscleName, slowTwitchRatio, muscleMass);

    Umberger2010MuscleMetabolicsProbe* totalEnergyRateProbe =
        new Umberger2010MuscleMetabolicsProbe(true, true, false, true);
    model.addProbe(totalEnergyRateProbe);
    totalEnergyRateProbe->setName("totalEnergyRateProbe");
    totalEnergyRateProbe->set_probe_operation("value");
    totalEnergyRateProbe->addMuscle(muscleName, slowTwitchRatio, muscleMass);
    totalEnergyRateProbe->set_aerobic_factor(1.0);

    // Initialize.
    SimTK::State& state = model.initSystem();
    muscle->setFiberLength(state, muscle->getOptimalFiberLength());

    // Calculate normalized force, mechanical power [W/kg], and total energy
    // liberation rate [W/kg] at numPoints evenly-spaced normalized shortening
    // velocities between 0 and 1.
    for (int i=0; i<numPoints; ++i) {
        state.setTime(i);
        muscle->setNormFiberVelocity(state, -(double)i/(numPoints-1));
        model.getMultibodySystem().realize(state, SimTK::Stage::Report);
        normalizedForce[i] = muscle->getFiberForce(state)
                             / muscle->getMaxIsometricForce();
        mechanicalPower[i] = mechanicalPowerProbe->computeProbeInputs(state)[0]
                             / muscleMass;
        totalEnergyRate[i] = totalEnergyRateProbe->computeProbeInputs(state)[0]
                             / muscleMass;
    }

    // Print for debugging.
    #ifdef DISPLAY_RESULTS
    const int w = 16;
    cout << "\n\nResults for " << muscleName << " shortening test" << endl;
    for (int i=0; i<4*w; ++i) {cout << "=";} cout << endl;
    cout << setw(w) << "Velocity"
         << setw(w) << "Force"
         << setw(w) << "Mech power"
         << setw(w) << "Total rate" << endl;
    for (int i=0; i<4*w; ++i) {cout << "=";} cout << endl;
    for (int i=0; i<numPoints; ++i)
        cout << setw(w) << -(double)i/(numPoints-1)
             << setw(w) << normalizedForce[i]
             << setw(w) << mechanicalPower[i]
             << setw(w) << totalEnergyRate[i] << endl;
    #endif
}

void compareUmbergerProbeToPublishedResults()
{
    //--------------------------------------------------------------------------
    // Generate data for soleus muscle.
    //--------------------------------------------------------------------------
    const double sol_maxIsometricForce  = 3127;
    const double sol_optimalFiberLength = 0.055;
    const double sol_Arel               = 0.18;
    const double sol_Brel               = 2.16;
    const double sol_muscleMass         = 0.805;
    double sol_normalizedForce[numPoints];
    double sol_mechanicalPower[numPoints];
    double sol_totalEnergyRate[numPoints];
    generateMuscleData("soleus", sol_maxIsometricForce, sol_optimalFiberLength,
                       0.80, sol_Arel, sol_Brel, 1.5, 0.8, sol_muscleMass,
                       &sol_normalizedForce[0], &sol_mechanicalPower[0],
                       &sol_totalEnergyRate[0]);

    //--------------------------------------------------------------------------
    // Compare data for soleus muscle to published results.
    //--------------------------------------------------------------------------
    #ifdef DISPLAY_ERRORS
    const int w = 16;
    cout << "\n\nAbsolute errors for soleus shortening test" << endl;
    for (int i=0; i<4*w; ++i) {cout << "=";} cout << endl;
    cout << setw(w) << "Velocity"
         << setw(w) << "e(Force)"
         << setw(w) << "e(Mech power)"
         << setw(w) << "e(Total rate)" << endl;
    for (int i=0; i<4*w; ++i) {cout << "=";} cout << endl;
    #endif

    for (int i=0; i<numPoints; ++i) {
        double vNorm = -(double)i/(numPoints-1);

        // Normalized force should be within roundoff error of this analytical
        // expression.
        double forceExpected = sol_Arel * (1 + vNorm) / (sol_Arel - vNorm);
        ASSERT_EQUAL(forceExpected, sol_normalizedForce[i],
            100*SimTK::SignificantReal, __FILE__, __LINE__,
            "testMuscleMetabolicsProbes: error in soleus normalized force.");

        // Mechanical power should be within roundoff error of this analytical
        // expression.
        double v_mps = vNorm * (sol_Brel/sol_Arel) * sol_optimalFiberLength;
        double powerExpected = -forceExpected * sol_maxIsometricForce * v_mps
                               / sol_muscleMass;
        ASSERT_EQUAL(powerExpected, sol_mechanicalPower[i],
            100*SimTK::SignificantReal, __FILE__, __LINE__,
            "testMuscleMetabolicsProbes: error in soleus mechanical power.");

        // Polynomials have been fit to the published results for total energy
        // liberation rate. The maximum absolute error is less than 1.0 [W/kg]
        // at all normalized velocities in [0,1]. Using Horner's method.
        double rateExpected = SimTK::NaN;
        if (vNorm >= -0.4) {
            // 5th-order polynomial gives maximum absolute error of 0.794.
            rateExpected = ((((-54693.72433*vNorm - 73876.93352)*vNorm
                           - 41460.1508)*vNorm - 13203.6807)*vNorm
                           - 2682.925993)*vNorm + 51.39363659;
        } else {
            // 3rd-order polynomial gives maximum absolute error of 0.477.
            rateExpected = ((-187.6077328*vNorm - 553.9650747)*vNorm
                           - 194.9721769)*vNorm + 332.9105995;
        }
        ASSERT_EQUAL(rateExpected, sol_totalEnergyRate[i], 1.0,
            "testMuscleMetabolicsProbes: error in soleus total energy rate.");

        #ifdef DISPLAY_ERRORS
        cout << setw(w) << -(double)i/(numPoints-1)
             << setw(w) << fabs(sol_normalizedForce[i] - forceExpected)
             << setw(w) << fabs(sol_mechanicalPower[i] - powerExpected)
             << setw(w) << fabs(sol_totalEnergyRate[i] - rateExpected) << endl;
        #endif
    }

    //--------------------------------------------------------------------------
    // Generate data for rectus femoris muscle.
    //--------------------------------------------------------------------------
    const double rec_maxIsometricForce  = 1118;
    const double rec_optimalFiberLength = 0.084;
    const double rec_Arel               = 0.36;
    const double rec_Brel               = 4.32;
    const double rec_muscleMass         = 0.4275;
    double rec_normalizedForce[numPoints];
    double rec_mechanicalPower[numPoints];
    double rec_totalEnergyRate[numPoints];
    generateMuscleData("rectus femoris", rec_maxIsometricForce,
                       rec_optimalFiberLength, 0.76, rec_Arel, rec_Brel, 1.5,
                       0.35, rec_muscleMass, &rec_normalizedForce[0],
                       &rec_mechanicalPower[0], &rec_totalEnergyRate[0]);

    //--------------------------------------------------------------------------
    // Compare data for rectus femoris muscle to published results.
    //--------------------------------------------------------------------------
    #ifdef DISPLAY_ERRORS
    cout << "\n\nAbsolute errors for rectus femoris shortening test" << endl;
    for (int i=0; i<4*w; ++i) {cout << "=";} cout << endl;
    cout << setw(w) << "Velocity"
         << setw(w) << "e(Force)"
         << setw(w) << "e(Mech power)"
         << setw(w) << "e(Total rate)" << endl;
    for (int i=0; i<4*w; ++i) {cout << "=";} cout << endl;
    #endif

    for (int i=0; i<numPoints; ++i) {
        double vNorm = -(double)i/(numPoints-1);

        // Normalized force should be within roundoff error of this analytical
        // expression.
        double forceExpected = rec_Arel * (1 + vNorm) / (rec_Arel - vNorm);
        ASSERT_EQUAL(forceExpected, rec_normalizedForce[i],
            100*SimTK::SignificantReal, __FILE__, __LINE__,
            "testMuscleMetabolicsProbes: error in rectus normalized force.");

        // Mechanical power should be within roundoff error of this analytical
        // expression.
        double v_mps = vNorm * (rec_Brel/rec_Arel) * rec_optimalFiberLength;
        double powerExpected = -forceExpected * rec_maxIsometricForce * v_mps
                               / rec_muscleMass;
        ASSERT_EQUAL(powerExpected, rec_mechanicalPower[i],
            100*SimTK::SignificantReal, __FILE__, __LINE__,
            "testMuscleMetabolicsProbes: error in rectus mechanical power.");

        // Polynomials have been fit to the published results for total energy
        // liberation rate. The maximum absolute error is less than 1.0 [W/kg]
        // at all normalized velocities in [0,1]. Using Horner's method.
        double rateExpected = SimTK::NaN;
        if (vNorm >= -0.4) {
            // 4th-order polynomial gives maximum absolute error of 0.829.
            rateExpected = (((-10181.27442*vNorm - 13652.40864)*vNorm
                           - 8368.687834)*vNorm - 2747.369194)*vNorm
                           + 109.028979;
        } else {
            // 3rd-order polynomial gives maximum absolute error of 0.811.
            rateExpected = ((-404.1161485*vNorm - 1267.967225)*vNorm
                           - 744.3435788)*vNorm + 362.7505521;
        }
        ASSERT_EQUAL(rateExpected, rec_totalEnergyRate[i], 1.0,
            "testMuscleMetabolicsProbes: error in rectus total energy rate.");

        #ifdef DISPLAY_ERRORS
        cout << setw(w) << -(double)i/(numPoints-1)
             << setw(w) << fabs(rec_normalizedForce[i] - forceExpected)
             << setw(w) << fabs(rec_mechanicalPower[i] - powerExpected)
             << setw(w) << fabs(rec_totalEnergyRate[i] - rateExpected) << endl;
        #endif
    }
}


//==============================================================================
//                  TEST UMBERGER USING MILLARD2012EQUILIBRIUM
//==============================================================================
void testUmbergerUsingMillard()
{
    // TODO
}


//==============================================================================
//                  TEST BHARGAVA USING MILLARD2012EQUILIBRIUM
//==============================================================================
void testBhargavaUsingMillard()
{
    // TODO
}


//==============================================================================
//                                     MAIN
//==============================================================================
int main()
{
    SimTK::Array_<std::string> failures;

    try { compareUmbergerProbeToPublishedResults();
        cout << "\ncompareUmbergerProbeToPublishedResults test passed" << endl;
    } catch (const OpenSim::Exception& e) {
        e.print(cerr);
        failures.push_back("compareUmbergerProbeToPublishedResults");
    }

    try { testUmbergerUsingMillard();
        cout << "testUmbergerUsingMillard test passed" << endl;
    } catch (const OpenSim::Exception& e) {
        e.print(cerr);
        failures.push_back("testUmbergerUsingMillard");
    }

    try { testBhargavaUsingMillard();
        cout << "testBhargavaUsingMillard test passed" << endl;
    } catch (const OpenSim::Exception& e) {
        e.print(cerr);
        failures.push_back("testBhargavaUsingMillard");
    }

    printf("\n\n");
    cout <<"************************************************************"<<endl;
    cout <<"************************************************************"<<endl;

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "testMuscleMetabolicsProbes Done" << endl;
    return 0;
}
