/* -------------------------------------------------------------------------- *
 *                  OpenSim:  testMuscleMetabolicsProbes.cpp                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
// B. Several Umberger2010 and Bhargava2004 probes are created in a variety of
//    configurations. These probes are then attached to Millard2012Equilibrium
//    muscles for basic functionality testing.
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
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Simulation/Model/Umberger2010MuscleMetabolicsProbe.h>
#include <OpenSim/Simulation/Model/Bhargava2004MuscleMetabolicsProbe.h>
#include <OpenSim/Analyses/ProbeReporter.h>
#include <OpenSim/Analyses/MuscleAnalysis.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

// The zeroth-order muscle activation dynamics model can be used only once the
// API supports subcomponents that have states.
//#define USE_ACTIVATION_DYNAMICS_MODEL
#ifdef  USE_ACTIVATION_DYNAMICS_MODEL
#include <OpenSim/Actuators/ZerothOrderMuscleActivationDynamics.h>
#endif

const bool DISPLAY_PROBE_OUTPUTS      = false;
const bool DISPLAY_ERROR_CALCULATIONS = false;
const bool OUTPUT_FILES               = false;

using namespace OpenSim;
using namespace SimTK;
using namespace std;


#ifdef USE_ACTIVATION_DYNAMICS_MODEL
//==============================================================================
//                              EXCITATION GETTER
//==============================================================================
// Supplies muscle excitation to the ZerothOrderMuscleActivationDynamics model
// used by UmbergerMuscle.
class MyExcitationGetter : public MuscleActivationDynamics::ExcitationGetter {
public:
    MyExcitationGetter(Muscle& muscle) : _muscle(muscle) {}

    double getExcitation(const SimTK::State& s) const override
    { return _muscle.getControl(s); }

private:
    const Muscle& _muscle;
};
#endif


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

    #ifdef USE_ACTIVATION_DYNAMICS_MODEL
    OpenSim_DECLARE_UNNAMED_PROPERTY(ZerothOrderMuscleActivationDynamics,
        "Activation dynamic model that simply sets activation to excitation.");
    #endif

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

        #ifdef USE_ACTIVATION_DYNAMICS_MODEL
        constructProperty_ZerothOrderMuscleActivationDynamics(
            ZerothOrderMuscleActivationDynamics());
        upd_ZerothOrderMuscleActivationDynamics().setExcitationGetter(
            new MyExcitationGetter(*this));
        #endif
    }

    //--------------------------------------------------------------------------
    // SET MUSCLE STATES
    //--------------------------------------------------------------------------
    void setFiberLength(SimTK::State& s, double fiberLength) const
    {
        setStateVariableValue(s, stateName_fiberLength, fiberLength);
        markCacheVariableInvalid(s, "lengthInfo");
        markCacheVariableInvalid(s, "velInfo");
        markCacheVariableInvalid(s, "dynamicsInfo");
    }

    void setNormFiberVelocity(SimTK::State& s, double normFiberVelocity) const
    {
        setStateVariableValue(s, stateName_fiberVelocity, normFiberVelocity *
                         getMaxContractionVelocity() * getOptimalFiberLength());
        markCacheVariableInvalid(s, "velInfo");
        markCacheVariableInvalid(s, "dynamicsInfo");
    }

    //--------------------------------------------------------------------------
    // MODELCOMPONENT INTERFACE
    //--------------------------------------------------------------------------
    void extendConnectToModel(Model& model) override
    {
        Super::extendConnectToModel(model);
        #ifdef USE_ACTIVATION_DYNAMICS_MODEL
        ZerothOrderMuscleActivationDynamics &zomad =
            upd_ZerothOrderMuscleActivationDynamics();
        includeAsSubComponent(&zomad);
        #endif
    }

    void extendAddToSystem(SimTK::MultibodySystem& system) const override
    {
        Super::extendAddToSystem(system);
        addStateVariable(stateName_fiberLength);
        addStateVariable(stateName_fiberVelocity);
    }

    void extendInitStateFromProperties(SimTK::State& s) const override
    {
        Super::extendInitStateFromProperties(s);
        setFiberLength(s, getOptimalFiberLength());
    }

    void extendSetPropertiesFromState(const SimTK::State& s) override
    {
        Super::extendSetPropertiesFromState(s);
        setOptimalFiberLength(getFiberLength(s));
    }

    void computeStateVariableDerivatives(const SimTK::State& s) const override
    {
        // This implementation is not intended for use in dynamic simulations.
        /*const int n = */getNumStateVariables();
        setStateVariableDerivativeValue(s, stateName_fiberLength, 0.0);
        setStateVariableDerivativeValue(s, stateName_fiberVelocity, 0.0);
    }

    //--------------------------------------------------------------------------
    // MUSCLE INTERFACE
    //--------------------------------------------------------------------------
    void computeInitialFiberEquilibrium(SimTK::State& s) const override {}

    void setActivation(SimTK::State& s, double activation) const override
    {
        #ifdef USE_ACTIVATION_DYNAMICS_MODEL
        get_ZerothOrderMuscleActivationDynamics().setActivation(s, activation);
        #else
        setExcitation(s, activation);
        #endif
    }

    double computeActuation(const SimTK::State& s) const override
    {
        const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
        setActuation(s, mdi.tendonForce);
        return mdi.tendonForce;
    }

    // Calculate position-level variables.
    void calcMuscleLengthInfo(const SimTK::State& s, MuscleLengthInfo& mli)
        const override
    {
        mli.fiberLength            = getStateVariableValue(s, stateName_fiberLength);
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
        const override
    {
        fvi.fiberVelocity = getStateVariableValue(s, stateName_fiberVelocity);
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
        const override
    {
        #ifdef USE_ACTIVATION_DYNAMICS_MODEL
        mdi.activation =
            get_ZerothOrderMuscleActivationDynamics().getActivation(s);
        #else
        mdi.activation = getExcitation(s);
        #endif

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

private:
    static const std::string stateName_fiberLength;
    static const std::string stateName_fiberVelocity;
};
const std::string UmbergerMuscle::stateName_fiberLength   = "fiber_length";
const std::string UmbergerMuscle::stateName_fiberVelocity = "fiber_velocity";


//==============================================================================
//                    CONSTANT-EXCITATION MUSCLE CONTROLLER
//==============================================================================
// Simple controller to maintain all muscle excitations at the same constant
// value. The metabolic probes depend on both excitation and activation.
class ConstantExcitationMuscleController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(ConstantExcitationMuscleController, Controller);
public:
    ConstantExcitationMuscleController(double u) : _u(u) {}

    void computeControls(const SimTK::State& s, SimTK::Vector &controls) const
        override
    {
        for (int i=0; i<_model->getMuscles().getSize(); ++i)
            controls[i] = _u;
    }

    void setConstantExcitation(double u) { _u = u; }

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
void generateUmbergerMuscleData(const std::string& muscleName,
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
    Ground& ground = model.updGround();

    // Create block. The length and velocity of the muscle will be specified, so
    // the properties of the block are irrelevant.
    const double blockMass       = 1.0;
    const double blockSideLength = 0.1;
    Inertia blockInertia = blockMass * Inertia::brick(Vec3(blockSideLength/2));
    OpenSim::Body *block = new OpenSim::Body("block", blockMass, Vec3(0),
                                             blockInertia);

    // Create slider joint between ground and block.
    SliderJoint* prismatic = new SliderJoint("prismatic", ground, Vec3(0), Vec3(0),
                                                *block, Vec3(0), Vec3(0));
    prismatic->updCoordinate().setName("xTranslation");
    model.addBody(block);
    model.addJoint(prismatic);

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

    model.setup();
    mechanicalPowerProbe->setName("mechanicalPowerProbe");
    mechanicalPowerProbe->setOperation("value");
    mechanicalPowerProbe->addMuscle(muscleName, slowTwitchRatio, muscleMass);

    Umberger2010MuscleMetabolicsProbe* totalEnergyRateProbe =
        new Umberger2010MuscleMetabolicsProbe(true, true, false, true);
    model.addProbe(totalEnergyRateProbe);
    totalEnergyRateProbe->setName("totalEnergyRateProbe");
    totalEnergyRateProbe->setOperation("value");
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
    if (DISPLAY_PROBE_OUTPUTS) {
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
    }
}

void compareUmbergerProbeToPublishedResults()
{
    //--------------------------------------------------------------------------
    // Generate data for soleus muscle.
    //--------------------------------------------------------------------------
    cout << "- generating data for soleus muscle" << endl;
    const double sol_maxIsometricForce  = 3127;
    const double sol_optimalFiberLength = 0.055;
    const double sol_Arel               = 0.18;
    const double sol_Brel               = 2.16;
    const double sol_muscleMass         = 0.805;
    double sol_normalizedForce[numPoints];
    double sol_mechanicalPower[numPoints];
    double sol_totalEnergyRate[numPoints];
    generateUmbergerMuscleData("soleus", sol_maxIsometricForce,
        sol_optimalFiberLength, 0.80, sol_Arel, sol_Brel, 1.5, 0.8,
        sol_muscleMass, &sol_normalizedForce[0], &sol_mechanicalPower[0],
        &sol_totalEnergyRate[0]);

    //--------------------------------------------------------------------------
    // Compare data for soleus muscle to published results.
    //--------------------------------------------------------------------------
    cout << "- comparing to published results" << endl;
    const int w = 16;
    if (DISPLAY_ERROR_CALCULATIONS) {
        cout << "\n\nAbsolute errors for soleus shortening test" << endl;
        for (int i=0; i<4*w; ++i) {cout << "=";} cout << endl;
        cout << setw(w) << "Velocity"
             << setw(w) << "e(Force)"
             << setw(w) << "e(Mech power)"
             << setw(w) << "e(Total rate)" << endl;
        for (int i=0; i<4*w; ++i) {cout << "=";} cout << endl;
    }

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

        if (DISPLAY_ERROR_CALCULATIONS) {
            cout << setw(w) << -(double)i/(numPoints-1)
                 << setw(w) << fabs(sol_normalizedForce[i] - forceExpected)
                 << setw(w) << fabs(sol_mechanicalPower[i] - powerExpected)
                 << setw(w) << fabs(sol_totalEnergyRate[i] - rateExpected)
                 << endl;
        }
    }

    //--------------------------------------------------------------------------
    // Generate data for rectus femoris muscle.
    //--------------------------------------------------------------------------
    cout << "- generating data for rectus femoris muscle" << endl;
    const double rec_maxIsometricForce  = 1118;
    const double rec_optimalFiberLength = 0.084;
    const double rec_Arel               = 0.36;
    const double rec_Brel               = 4.32;
    const double rec_muscleMass         = 0.4275;
    double rec_normalizedForce[numPoints];
    double rec_mechanicalPower[numPoints];
    double rec_totalEnergyRate[numPoints];
    generateUmbergerMuscleData("rectus femoris", rec_maxIsometricForce,
        rec_optimalFiberLength, 0.76, rec_Arel, rec_Brel, 1.5, 0.35,
        rec_muscleMass, &rec_normalizedForce[0], &rec_mechanicalPower[0],
        &rec_totalEnergyRate[0]);

    //--------------------------------------------------------------------------
    // Compare data for rectus femoris muscle to published results.
    //--------------------------------------------------------------------------
    cout << "- comparing to published results" << endl;
    if (DISPLAY_ERROR_CALCULATIONS) {
        cout << "\n\nAbsolute errors for rectus femoris shortening test" << endl;
        for (int i=0; i<4*w; ++i) {cout << "=";} cout << endl;
        cout << setw(w) << "Velocity"
             << setw(w) << "e(Force)"
             << setw(w) << "e(Mech power)"
             << setw(w) << "e(Total rate)" << endl;
        for (int i=0; i<4*w; ++i) {cout << "=";} cout << endl;
    }

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

        if (DISPLAY_ERROR_CALCULATIONS) {
            cout << setw(w) << -(double)i/(numPoints-1)
                 << setw(w) << fabs(rec_normalizedForce[i] - forceExpected)
                 << setw(w) << fabs(rec_mechanicalPower[i] - powerExpected)
                 << setw(w) << fabs(rec_totalEnergyRate[i] - rateExpected)
                 << endl;
        }
    }
}


//==============================================================================
//      TEST UMBERGER AND BHARGAVA PROBES USING MILLARD EQUILIBRIUM MUSCLE
//==============================================================================
// Builds an OpenSim model consisting of two Millard2012Equilibrium muscles,
// attaches several Umberger2010 and Bhargava2004 muscle metabolics probes, and
// confirms that the probes are functioning properly:
//   - probes and muscles can be added and removed
//   - muscle mass parameter is correctly handled
//   - probe components are correctly reported individually and combined
//   - mechanical work rate is calculated correctly
//   - total energy at final time equals integral of total rate
//   - multiple muscles are correctly handled
//   - less energy is liberated with lower activation
Storage simulateModel(Model& model, double t0, double t1)
{
    // Initialize model and state.
    cout << "- initializing" << endl;
    SimTK::State& state = model.initSystem();

    for (int i=0; i<model.getMuscles().getSize(); ++i)
        model.getMuscles().get(i).setIgnoreActivationDynamics(state, true);
    model.getMultibodySystem().realize(state, SimTK::Stage::Dynamics);
    model.equilibrateMuscles(state);

    // Prepare integrator.
    const double integrationAccuracy = 1.0e-8;
    SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    integrator.setAccuracy(integrationAccuracy);

    Manager manager(model, integrator);
    state.setTime(t0);

    // Simulate.
    const clock_t tStart = clock();
    cout << "- integrating from " << t0 << " to " << t1 << "s" << endl;
    manager.integrate(state, t1);
    cout << "- simulation complete (" << (double)(clock()-tStart)/CLOCKS_PER_SEC
         << " seconds elapsed)" << endl;

    return manager.getStateStorage();
}

void testProbesUsingMillardMuscleSimulation()
{
    //--------------------------------------------------------------------------
    // Build an OpenSim model consisting of two Millard2012Equilibrium muscles.
    //--------------------------------------------------------------------------
    Model model;
    model.setName("testModel_metabolics");
    Ground& ground = model.updGround();

    // Create block.
    const double blockMass       = 1.0;
    const double blockSideLength = 0.1;
    Inertia blockInertia = blockMass * Inertia::brick(Vec3(blockSideLength/2));
    OpenSim::Body *block = new OpenSim::Body("block", blockMass, Vec3(0),
                                             blockInertia);
    block->attachGeometry(new Mesh("block.vtp"));

    // Create slider joint between ground and block.
    SliderJoint* prismatic = new SliderJoint("prismatic", ground, Vec3(0), Vec3(0),
                                                *block, Vec3(0), Vec3(0));
    auto& prisCoord = prismatic->updCoordinate();
    prisCoord.setName("xTranslation");
    prisCoord.setRangeMin(-1);
    prisCoord.setRangeMax(1);

    // Prescribe motion.
    Sine motion(0.1, SimTK::Pi, 0);
    prisCoord.setPrescribedFunction(motion);
    prisCoord.setDefaultIsPrescribed(true);
    model.addBody(block);
    model.addJoint(prismatic);

    // Create muscles attached to ground and block.
    //                    _______
    //                   |       |
    // x---[ muscle1 ]---| block |---[ muscle2 ]---x
    //                   |_______|
    //
    //                       0 --> +
    const double optimalFiberLength = 0.1;
    const double tendonSlackLength  = 0.2;
    const double anchorDistance     = optimalFiberLength + tendonSlackLength
                                      + blockSideLength/2;
    double desiredActivation        = 1.0;

    Millard2012EquilibriumMuscle *muscle1 = new Millard2012EquilibriumMuscle(
        "muscle1", 100, optimalFiberLength, tendonSlackLength, 0);
    muscle1->addNewPathPoint("m1_ground", ground, Vec3(-anchorDistance,0,0));
    muscle1->addNewPathPoint("m1_block",  *block, Vec3(-blockSideLength/2,0,0));
    muscle1->setDefaultActivation(desiredActivation);
    model.addForce(muscle1);

    Millard2012EquilibriumMuscle *muscle2 = new Millard2012EquilibriumMuscle(
        "muscle2", 100, optimalFiberLength, tendonSlackLength, 0);
    muscle2->addNewPathPoint("m2_ground", ground, Vec3(anchorDistance,0,0));
    muscle2->addNewPathPoint("m2_block",  *block, Vec3(blockSideLength/2,0,0));
    muscle2->setDefaultActivation(desiredActivation);
    model.addForce(muscle2);

    // Attach muscle controller.
    ConstantExcitationMuscleController* controller =
        new ConstantExcitationMuscleController(desiredActivation);
    controller->setActuators(model.updActuators());
    model.addController(controller);

    //--------------------------------------------------------------------------
    // Configuration tests for Umberger2010MuscleMetabolicsProbe. Ensure probes
    // and muscles can be added and removed, and that the muscle mass parameter
    // is correctly handled.
    //--------------------------------------------------------------------------
    cout << "- running tests for adding, configuring, and removing the "
         << "Umberger2010 probe" << endl;

    // Add a test probe to the model. This probe will eventually be removed.
    Umberger2010MuscleMetabolicsProbe* umbergerTest = new
        Umberger2010MuscleMetabolicsProbe(true, true, true, true);
    model.addProbe(umbergerTest);
    ASSERT(model.getNumProbes()==1, __FILE__, __LINE__,
        "Umberger2010MuscleMetabolicsProbe could not be added to the model.");

    // Add a muscle to the probe without providing the muscle mass.
    umbergerTest->addMuscle(muscle1->getName(), 0.6);
    model.setup();
    ASSERT(umbergerTest->getNumMetabolicMuscles()==1, __FILE__, __LINE__,
        "Muscle could not be added to Umberger2010MuscleMetabolicsProbe.");
    ASSERT(!umbergerTest->isUsingProvidedMass(muscle1->getName()), __FILE__,
        __LINE__, "Umberger probe should not be using provided muscle mass.");

    // Change the muscle mass calculation.
    umbergerTest->useProvidedMass(muscle1->getName(), 1.0);
    ASSERT(umbergerTest->isUsingProvidedMass(muscle1->getName()), __FILE__,
        __LINE__, "Umberger probe should be using provided muscle mass.");
    umbergerTest->useCalculatedMass(muscle1->getName());
    ASSERT(!umbergerTest->isUsingProvidedMass(muscle1->getName()), __FILE__,
        __LINE__, "Umberger probe should not be using provided muscle mass.");

    // Add another muscle to the probe, this time providing the muscle mass.
    umbergerTest->addMuscle(muscle2->getName(), 0.6, 1.0);
    model.setup();
    ASSERT(umbergerTest->isUsingProvidedMass(muscle2->getName()), __FILE__,
        __LINE__, "Umberger probe should be using provided muscle mass.");

    // Remove a muscle from the probe.
    umbergerTest->removeMuscle(muscle1->getName());
    model.setup();
    ASSERT(umbergerTest->getNumMetabolicMuscles()==1, __FILE__, __LINE__,
        "Muscle could not be removed from Umberger2010MuscleMetabolicsProbe.");

    // Remove the probe from the model.
    model.removeProbe(umbergerTest);
    model.setup();
    ASSERT(model.getNumProbes()==0, __FILE__, __LINE__,
        "Umberger2010MuscleMetabolicsProbe could not be removed from the model.");

    //--------------------------------------------------------------------------
    // Configuration tests for Bhargava2004MuscleMetabolicsProbe. Ensure probes
    // and muscles can be added and removed, and that the muscle mass parameter
    // is correctly handled.
    //--------------------------------------------------------------------------
    cout << "- running tests for adding, configuring, and removing the "
         << "Bhargava2004 probe\n" << endl;

    // Add a test probe to the model. This probe will eventually be removed.
    Bhargava2004MuscleMetabolicsProbe* bhargavaTest = new
        Bhargava2004MuscleMetabolicsProbe(true, true, true, true, true);
    model.addProbe(bhargavaTest);
    ASSERT(model.getNumProbes()==1, __FILE__, __LINE__,
        "Bhargava2004MuscleMetabolicsProbe could not be added to the model.");

    // Add a muscle to the probe without providing the muscle mass.
    bhargavaTest->addMuscle(muscle1->getName(), 0.6, 40, 133, 74, 111);
    model.setup();
    ASSERT(bhargavaTest->getNumMetabolicMuscles()==1, __FILE__, __LINE__,
        "Muscle could not be added to Bhargava2004MuscleMetabolicsProbe.");
    ASSERT(!bhargavaTest->isUsingProvidedMass(muscle1->getName()), __FILE__,
        __LINE__, "Bhargava probe should not be using provided muscle mass.");

    // Change the muscle mass calculation.
    bhargavaTest->useProvidedMass(muscle1->getName(), 1.0);
    ASSERT(bhargavaTest->isUsingProvidedMass(muscle1->getName()), __FILE__,
        __LINE__, "Bhargava probe should be using provided muscle mass.");
    bhargavaTest->useCalculatedMass(muscle1->getName());
    ASSERT(!bhargavaTest->isUsingProvidedMass(muscle1->getName()), __FILE__,
        __LINE__, "Bhargava probe should not be using provided muscle mass.");

    // Add another muscle to the probe, this time providing the muscle mass.
    bhargavaTest->addMuscle(muscle1->getName(), 0.6, 40, 133, 74, 111, 1.0);
    model.setup();
    ASSERT(bhargavaTest->isUsingProvidedMass(muscle1->getName()), __FILE__,
        __LINE__, "Bhargava probe should be using provided muscle mass.");

    // Remove a muscle from the probe.
    bhargavaTest->removeMuscle(muscle1->getName());
    model.setup();
    ASSERT(bhargavaTest->getNumMetabolicMuscles()==1, __FILE__, __LINE__,
        "Muscle could not be removed from Bhargava2004MuscleMetabolicsProbe.");

    // Remove the probe from the model.
    model.removeProbe(bhargavaTest);
    model.setup();
    ASSERT(model.getNumProbes()==0, __FILE__, __LINE__,
        "Bhargava2004MuscleMetabolicsProbe could not be removed from the model.");

    //--------------------------------------------------------------------------
    // Attach Umberger2010 and Bhargava2004 muscle metabolics probes.
    //--------------------------------------------------------------------------
    int probeCounter = 0;       // Number of probes attached.
    int extraColumns = 0;       // Number of columns expected in storage file is
                                // probeCounter + extraColumns.
    const int w = 4;

    // Attach Umberger2010 probes to record individual heat rate and mechanical
    // power components at each point in time for muscle1.
    Umberger2010MuscleMetabolicsProbe* umbergerActMaint_rate_m1 = new
        Umberger2010MuscleMetabolicsProbe(true, false, false, false);
    model.addProbe(umbergerActMaint_rate_m1);
    umbergerActMaint_rate_m1->setName("umbergerActMaint_rate_m1");
    umbergerActMaint_rate_m1->setOperation("value");
    umbergerActMaint_rate_m1->addMuscle(muscle1->getName(), 0.5);
    cout << setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
         << "activation and maintenance heat rate (muscle 1)" << endl;

    Umberger2010MuscleMetabolicsProbe* umbergerShorten_rate_m1 = new
        Umberger2010MuscleMetabolicsProbe(false, true, false, false);
    model.addProbe(umbergerShorten_rate_m1);
    umbergerShorten_rate_m1->setName("umbergerShorten_rate_m1");
    umbergerShorten_rate_m1->setOperation("value");
    umbergerShorten_rate_m1->addMuscle(muscle1->getName(), 0.5);
    cout << setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
         << "shortening and lengthening heat rate (muscle 1)" << endl;

    Umberger2010MuscleMetabolicsProbe* umbergerBasal_rate_m1 = new
        Umberger2010MuscleMetabolicsProbe(false, false, true, false);
    model.addProbe(umbergerBasal_rate_m1);
    umbergerBasal_rate_m1->setName("umbergerBasal_rate_m1");
    umbergerBasal_rate_m1->setOperation("value");
    umbergerBasal_rate_m1->addMuscle(muscle1->getName(), 0.5);
    cout << setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
         << "basal heat rate (muscle 1)" << endl;

    Umberger2010MuscleMetabolicsProbe* umbergerMechWork_rate_m1 = new
        Umberger2010MuscleMetabolicsProbe(false, false, false, true);
    model.addProbe(umbergerMechWork_rate_m1);
    umbergerMechWork_rate_m1->setName("umbergerMechWork_rate_m1");
    umbergerMechWork_rate_m1->setOperation("value");
    umbergerMechWork_rate_m1->addMuscle(muscle1->getName(), 0.5);
    cout << setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
         << "mechanical power (muscle 1)" << endl;

    // Attach Umberger2010 probe to record total rate of energy liberation at
    // each point in time for muscle1.
    Umberger2010MuscleMetabolicsProbe* umbergerTotal_rate_m1 = new
        Umberger2010MuscleMetabolicsProbe(true, true, true, true);
    model.addProbe(umbergerTotal_rate_m1);
    umbergerTotal_rate_m1->setName("umbergerTotal_rate_m1");
    umbergerTotal_rate_m1->setOperation("value");
    umbergerTotal_rate_m1->addMuscle(muscle1->getName(), 0.5);
    cout << setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
         << "total rate of energy liberation (muscle 1)" << endl;

    // Attach Umberger2010 probes to record total energy liberation over the
    // entire simulation for (a) muscle1, (b) muscle2, (c) total for both
    // muscles, and (d) total for both muscles with all components reported.
    Umberger2010MuscleMetabolicsProbe* umbergerTotal_m1 = new
        Umberger2010MuscleMetabolicsProbe(true, true, true, true);
    model.addProbe(umbergerTotal_m1);
    umbergerTotal_m1->setName("umbergerTotal_m1");
    umbergerTotal_m1->setOperation("integrate");
    umbergerTotal_m1->setInitialConditions(Vector(Vec1(0)));
    umbergerTotal_m1->addMuscle(muscle1->getName(), 0.5);
    cout << setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
         << "total energy liberation (muscle 1)" << endl;

    Umberger2010MuscleMetabolicsProbe* umbergerTotal_m2 = new
        Umberger2010MuscleMetabolicsProbe(true, true, true, true);
    model.addProbe(umbergerTotal_m2);
    umbergerTotal_m2->setName("umbergerTotal_m2");
    umbergerTotal_m2->setOperation("integrate");
    umbergerTotal_m2->setInitialConditions(Vector(Vec1(0)));
    umbergerTotal_m2->addMuscle(muscle2->getName(), 0.5);
    cout << setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
         << "total energy liberation (muscle 2)" << endl;

    Umberger2010MuscleMetabolicsProbe* umbergerTotal_both = new
        Umberger2010MuscleMetabolicsProbe(true, true, true, true);
    model.addProbe(umbergerTotal_both);
    umbergerTotal_both->setName("umbergerTotal_both");
    umbergerTotal_both->setOperation("integrate");
    umbergerTotal_both->setInitialConditions(Vector(Vec1(0)));
    umbergerTotal_both->addMuscle(muscle1->getName(), 0.5);
    umbergerTotal_both->addMuscle(muscle2->getName(), 0.5);
    cout << setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
         << "total energy liberation (both muscles, total only)" << endl;

    Umberger2010MuscleMetabolicsProbe* umbergerTotalAllPieces_both = new
        Umberger2010MuscleMetabolicsProbe(true, true, true, true);
    model.addProbe(umbergerTotalAllPieces_both);
    umbergerTotalAllPieces_both->setName("umbergerTotalAllPieces_both");
    umbergerTotalAllPieces_both->setOperation("integrate");
    umbergerTotalAllPieces_both->set_report_total_metabolics_only(false);
    umbergerTotalAllPieces_both->setInitialConditions(Vector(Vec4(0)));
    umbergerTotalAllPieces_both->addMuscle(muscle1->getName(), 0.5);
    umbergerTotalAllPieces_both->addMuscle(muscle2->getName(), 0.5);
    cout << setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
         << "total energy liberation (both muscles, all components)" << endl;
    extraColumns += 3;

    // Attach Bhargava2004 probes to record individual heat rate and mechanical
    // power components at each point in time for muscle1.
    Bhargava2004MuscleMetabolicsProbe* bhargavaAct_rate_m1 = new
        Bhargava2004MuscleMetabolicsProbe(true, false, false, false, false);
    model.addProbe(bhargavaAct_rate_m1);
    bhargavaAct_rate_m1->setName("bhargavaAct_rate_m1");
    bhargavaAct_rate_m1->setOperation("value");
    bhargavaAct_rate_m1->addMuscle(muscle1->getName(), 0.5, 40, 133, 74, 111);
    cout << setw(w) << ++probeCounter << ") Added Bhargava2004 probe: "
         << "activation heat rate (muscle 1)" << endl;

    Bhargava2004MuscleMetabolicsProbe* bhargavaMaint_rate_m1 = new
        Bhargava2004MuscleMetabolicsProbe(false, true, false, false, false);
    model.addProbe(bhargavaMaint_rate_m1);
    bhargavaMaint_rate_m1->setName("bhargavaMaint_rate_m1");
    bhargavaMaint_rate_m1->setOperation("value");
    bhargavaMaint_rate_m1->addMuscle(muscle1->getName(), 0.5, 40, 133, 74, 111);
    cout << setw(w) << ++probeCounter << ") Added Bhargava2004 probe: "
         << "maintenance heat rate (muscle 1)" << endl;

    Bhargava2004MuscleMetabolicsProbe* bhargavaShorten_rate_m1 = new
        Bhargava2004MuscleMetabolicsProbe(false, false, true, false, false);
    model.addProbe(bhargavaShorten_rate_m1);
    bhargavaShorten_rate_m1->setName("bhargavaShorten_rate_m1");
    bhargavaShorten_rate_m1->setOperation("value");
    bhargavaShorten_rate_m1->addMuscle(muscle1->getName(), 0.5, 40, 133, 74, 111);
    cout << setw(w) << ++probeCounter << ") Added Bhargava2004 probe: "
         << "shortening and lengthening heat rate (muscle 1)" << endl;

    Bhargava2004MuscleMetabolicsProbe* bhargavaBasal_rate_m1 = new
        Bhargava2004MuscleMetabolicsProbe(false, false, false, true, false);
    model.addProbe(bhargavaBasal_rate_m1);
    bhargavaBasal_rate_m1->setName("bhargavaBasal_rate_m1");
    bhargavaBasal_rate_m1->setOperation("value");
    bhargavaBasal_rate_m1->addMuscle(muscle1->getName(), 0.5, 40, 133, 74, 111);
    cout << setw(w) << ++probeCounter << ") Added Bhargava2004 probe: "
         << "basal heat rate (muscle 1)" << endl;

    Bhargava2004MuscleMetabolicsProbe* bhargavaMechWork_rate_m1 = new
        Bhargava2004MuscleMetabolicsProbe(false, false, false, false, true);
    model.addProbe(bhargavaMechWork_rate_m1);
    bhargavaMechWork_rate_m1->setName("bhargavaMechWork_rate_m1");
    bhargavaMechWork_rate_m1->setOperation("value");
    bhargavaMechWork_rate_m1->addMuscle(muscle1->getName(), 0.5, 40, 133, 74, 111);
    cout << setw(w) << ++probeCounter << ") Added Bhargava2004 probe: "
         << "mechanical power (muscle 1)" << endl;

    // Attach Bhargava2004 probe to record total rate of energy liberation at
    // each point in time for muscle1.
    Bhargava2004MuscleMetabolicsProbe* bhargavaTotal_rate_m1 = new
        Bhargava2004MuscleMetabolicsProbe(true, true, true, true, true);
    model.addProbe(bhargavaTotal_rate_m1);
    bhargavaTotal_rate_m1->setName("bhargavaTotal_rate_m1");
    bhargavaTotal_rate_m1->setOperation("value");
    bhargavaTotal_rate_m1->addMuscle(muscle1->getName(), 0.5, 40, 133, 74, 111);
    cout << setw(w) << ++probeCounter << ") Added Bhargava2004 probe: "
         << "total rate of energy liberation (muscle 1)" << endl;

    // Attach Bhargava2004 probes to record total energy liberation over the
    // entire simulation for (a) muscle1, (b) muscle2, and (c) total for both
    // muscles, and (d) total for both muscles with all components reported.
    Bhargava2004MuscleMetabolicsProbe* bhargavaTotal_m1 = new
        Bhargava2004MuscleMetabolicsProbe(true, true, true, true, true);
    model.addProbe(bhargavaTotal_m1);
    bhargavaTotal_m1->setName("bhargavaTotal_m1");
    bhargavaTotal_m1->setOperation("integrate");
    bhargavaTotal_m1->setInitialConditions(Vector(Vec1(0)));
    bhargavaTotal_m1->addMuscle(muscle1->getName(), 0.5, 40, 133, 74, 111);
    cout << setw(w) << ++probeCounter << ") Added Bhargava2004 probe: "
         << "total energy liberation (muscle 1)" << endl;

    Bhargava2004MuscleMetabolicsProbe* bhargavaTotal_m2 = new
        Bhargava2004MuscleMetabolicsProbe(true, true, true, true, true);
    model.addProbe(bhargavaTotal_m2);
    bhargavaTotal_m2->setName("bhargavaTotal_m2");
    bhargavaTotal_m2->setOperation("integrate");
    bhargavaTotal_m2->setInitialConditions(Vector(Vec1(0)));
    bhargavaTotal_m2->addMuscle(muscle2->getName(), 0.5, 40, 133, 74, 111);
    cout << setw(w) << ++probeCounter << ") Added Bhargava2004 probe: "
         << "total energy liberation (muscle 2)" << endl;

    Bhargava2004MuscleMetabolicsProbe* bhargavaTotal_both = new
        Bhargava2004MuscleMetabolicsProbe(true, true, true, true, true);
    model.addProbe(bhargavaTotal_both);
    bhargavaTotal_both->setName("bhargavaTotal_both");
    bhargavaTotal_both->setOperation("integrate");
    bhargavaTotal_both->setInitialConditions(Vector(Vec1(0)));
    bhargavaTotal_both->addMuscle(muscle1->getName(), 0.5, 40, 133, 74, 111);
    bhargavaTotal_both->addMuscle(muscle2->getName(), 0.5, 40, 133, 74, 111);
    cout << setw(w) << ++probeCounter << ") Added Bhargava2004 probe: "
         << "total energy liberation (both muscles, total only)" << endl;

    Bhargava2004MuscleMetabolicsProbe* bhargavaTotalAllPieces_both = new
        Bhargava2004MuscleMetabolicsProbe(true, true, true, true, true);
    model.addProbe(bhargavaTotalAllPieces_both);
    bhargavaTotalAllPieces_both->setName("bhargavaTotalAllPieces_both");
    bhargavaTotalAllPieces_both->setOperation("integrate");
    bhargavaTotalAllPieces_both->set_report_total_metabolics_only(false);
    bhargavaTotalAllPieces_both->setInitialConditions(Vector(Vec4(0)));
    bhargavaTotalAllPieces_both->addMuscle(muscle1->getName(),0.5,40,133,74,111);
    bhargavaTotalAllPieces_both->addMuscle(muscle2->getName(),0.5,40,133,74,111);
    cout << setw(w) << ++probeCounter << ") Added Bhargava2004 probe: "
         << "total energy liberation (both muscles, all components)" << endl;
    extraColumns += 3;

    // Attach reporters.
    ProbeReporter* probeReporter = new ProbeReporter(&model);
    model.addAnalysis(probeReporter);
    MuscleAnalysis* muscleAnalysis = new MuscleAnalysis(&model);
    model.addAnalysis(muscleAnalysis);

    // Print the model.
    model.finalizeFromProperties();
    printf("\n"); model.printBasicInfo(); printf("\n");
    const std::string baseFilename = "testMuscleMetabolicsProbes";
    if (OUTPUT_FILES) {
        const std::string fname = baseFilename + "Model.osim";
        model.print(fname);
        cout << "+ saved model file: " << fname << endl;
    }

    //--------------------------------------------------------------------------
    // Run simulation.
    //--------------------------------------------------------------------------
    const double t0 = 0.0;
    const double t1 = 2.0;
    auto stateStorage = simulateModel(model, t0, t1);

    // Output results files.
    if (OUTPUT_FILES) {
        std::string fname = baseFilename + "_states.sto";
        stateStorage.print(fname);
        cout << "+ saved state storage file: " << fname << endl;

        fname = baseFilename + "_probes.sto";
        probeReporter->getProbeStorage().print(fname);
        cout << "+ saved probe storage file: " << fname << endl;

        fname = baseFilename + "_activeFiberForce.sto";
        muscleAnalysis->getActiveFiberForceStorage()->print(fname);
        cout << "+ saved active fiber force storage file: " << fname << endl;

        fname = baseFilename + "_fiberVelocity.sto";
        muscleAnalysis->getFiberVelocityStorage()->print(fname);
        cout << "+ saved fiber velocity storage file: " << fname << endl;
    }

    // Store column indices.
    const Storage& probeStorage = probeReporter->getProbeStorage();
    const int numProbeOutputs = probeStorage.getColumnLabels().getSize()-1;
    ASSERT(numProbeOutputs == probeCounter+extraColumns, __FILE__, __LINE__,
        "Incorrect number of columns in probe storage.");

    std::map<std::string, int> probeCol;
    probeCol["umbActMaint_rate_m1"] = probeStorage
      .getColumnIndicesForIdentifier("umbergerActMaint_rate_m1_TOTAL")[0]-1;
    probeCol["umbShorten_rate_m1"] = probeStorage
      .getColumnIndicesForIdentifier("umbergerShorten_rate_m1_TOTAL")[0]-1;
    probeCol["umbBasal_rate_m1"] = probeStorage
      .getColumnIndicesForIdentifier("umbergerBasal_rate_m1_TOTAL")[0]-1;
    probeCol["umbMechWork_rate_m1"] = probeStorage
      .getColumnIndicesForIdentifier("umbergerMechWork_rate_m1_TOTAL")[0]-1;
    probeCol["umbTotal_rate_m1"] = probeStorage
      .getColumnIndicesForIdentifier("umbergerTotal_rate_m1_TOTAL")[0]-1;
    probeCol["umbTotal_m1"] = probeStorage
      .getColumnIndicesForIdentifier("umbergerTotal_m1_TOTAL")[0]-1;
    probeCol["umbTotal_m2"] = probeStorage
      .getColumnIndicesForIdentifier("umbergerTotal_m2_TOTAL")[0]-1;
    probeCol["umbTotal_both"] = probeStorage
      .getColumnIndicesForIdentifier("umbergerTotal_both_TOTAL")[0]-1;
    probeCol["umbTotalAllPieces_both_total"] = probeStorage
      .getColumnIndicesForIdentifier("umbergerTotalAllPieces_both_TOTAL")[0]-1;
    probeCol["umbTotalAllPieces_both_basal"] = probeStorage
      .getColumnIndicesForIdentifier("umbergerTotalAllPieces_both_BASAL")[0]-1;
    probeCol["umbTotalAllPieces_both_muscle1"] = probeStorage
      .getColumnIndicesForIdentifier("umbergerTotalAllPieces_both_muscle1")[0]-1;
    probeCol["umbTotalAllPieces_both_muscle2"] = probeStorage
      .getColumnIndicesForIdentifier("umbergerTotalAllPieces_both_muscle2")[0]-1;

    probeCol["bhaAct_rate_m1"] = probeStorage
      .getColumnIndicesForIdentifier("bhargavaAct_rate_m1_TOTAL")[0]-1;
    probeCol["bhaMaint_rate_m1"] = probeStorage
      .getColumnIndicesForIdentifier("bhargavaMaint_rate_m1_TOTAL")[0]-1;
    probeCol["bhaShorten_rate_m1"] = probeStorage
      .getColumnIndicesForIdentifier("bhargavaShorten_rate_m1_TOTAL")[0]-1;
    probeCol["bhaBasal_rate_m1"] = probeStorage
      .getColumnIndicesForIdentifier("bhargavaBasal_rate_m1_TOTAL")[0]-1;
    probeCol["bhaMechWork_rate_m1"] = probeStorage
      .getColumnIndicesForIdentifier("bhargavaMechWork_rate_m1_TOTAL")[0]-1;
    probeCol["bhaTotal_rate_m1"] = probeStorage
      .getColumnIndicesForIdentifier("bhargavaTotal_rate_m1_TOTAL")[0]-1;
    probeCol["bhaTotal_m1"] = probeStorage
      .getColumnIndicesForIdentifier("bhargavaTotal_m1_TOTAL")[0]-1;
    probeCol["bhaTotal_m2"] = probeStorage
      .getColumnIndicesForIdentifier("bhargavaTotal_m2_TOTAL")[0]-1;
    probeCol["bhaTotal_both"] = probeStorage
      .getColumnIndicesForIdentifier("bhargavaTotal_both_TOTAL")[0]-1;
    probeCol["bhaTotalAllPieces_both_total"] = probeStorage
      .getColumnIndicesForIdentifier("bhargavaTotalAllPieces_both_TOTAL")[0]-1;
    probeCol["bhaTotalAllPieces_both_basal"] = probeStorage
      .getColumnIndicesForIdentifier("bhargavaTotalAllPieces_both_BASAL")[0]-1;
    probeCol["bhaTotalAllPieces_both_muscle1"] = probeStorage
      .getColumnIndicesForIdentifier("bhargavaTotalAllPieces_both_muscle1")[0]-1;
    probeCol["bhaTotalAllPieces_both_muscle2"] = probeStorage
      .getColumnIndicesForIdentifier("bhargavaTotalAllPieces_both_muscle2")[0]-1;

    Storage* forceStorage = muscleAnalysis->getActiveFiberForceStorage();
    const int numForceOutputs = forceStorage->getColumnLabels().getSize()-1;
    ASSERT(numForceOutputs == 2, __FILE__, __LINE__,
        "Incorrect number of columns in active fiber force storage.");
    const int idx_force_muscle1 = forceStorage
        ->getColumnIndicesForIdentifier(muscle1->getName())[0]-1;

    Storage* fibVelStorage = muscleAnalysis->getFiberVelocityStorage();
    const int numFibVelOutputs = fibVelStorage->getColumnLabels().getSize()-1;
    ASSERT(numFibVelOutputs == 2, __FILE__, __LINE__,
        "Incorrect number of columns in fiber velocity storage.");
    const int idx_fibVel_muscle1 = fibVelStorage
        ->getColumnIndicesForIdentifier(muscle1->getName())[0]-1;

    //--------------------------------------------------------------------------
    // Check instantaneous power results at numPoints evenly-spaced instants.
    //--------------------------------------------------------------------------
    cout << "- checking instantaneous power results" << endl;
    const int numPoints = 21;
    for (int i=0; i<numPoints; ++i) {
        double t = t0 + (double)i/(numPoints-1)*(t1-t0);

        // Get data from probe at current time.
        Array<double> probeData;
        probeData.setSize(numProbeOutputs);
        probeStorage.getDataAtTime(t, numProbeOutputs, probeData);

        // Output from the probes reporting individual heat rates and mechanical
        // power must sum to the output from the probe reporting the total rate
        // of energy liberation.
        ASSERT_EQUAL(probeData[probeCol["umbActMaint_rate_m1"]]
                       + probeData[probeCol["umbShorten_rate_m1"]]
                       + probeData[probeCol["umbBasal_rate_m1"]]
                       + probeData[probeCol["umbMechWork_rate_m1"]],
                     probeData[probeCol["umbTotal_rate_m1"]],
                     100*SimTK::SignificantReal, __FILE__, __LINE__,
            "Umberger2010: wrong sum of individual rates and mechanical power.");

        ASSERT_EQUAL(probeData[probeCol["bhaAct_rate_m1"]]
                       + probeData[probeCol["bhaMaint_rate_m1"]]
                       + probeData[probeCol["bhaShorten_rate_m1"]]
                       + probeData[probeCol["bhaBasal_rate_m1"]]
                       + probeData[probeCol["bhaMechWork_rate_m1"]],
                     probeData[probeCol["bhaTotal_rate_m1"]],
                     100*SimTK::SignificantReal, __FILE__, __LINE__,
            "Bhargava2004: wrong sum of individual rates and mechanical power.");

        // Total rate of energy liberation reported must not depend on whether
        // the individual components are reported as well.
        ASSERT_EQUAL(probeData[probeCol["umbTotal_both"]],
                     probeData[probeCol["umbTotalAllPieces_both_total"]],
                     100*SimTK::SignificantReal, __FILE__, __LINE__,
            "Umberger2010: total heat rate changes if components are reported.");

        ASSERT_EQUAL(probeData[probeCol["bhaTotal_both"]],
                     probeData[probeCol["bhaTotalAllPieces_both_total"]],
                     100*SimTK::SignificantReal, __FILE__, __LINE__,
            "Bhargava2004: total heat rate changes if components are reported.");

        // Mechanical work rates should agree with fiber velocity and active
        // fiber force data.
        Array<double> forceData;
        forceData.setSize(numForceOutputs);
        forceStorage->getDataAtTime(t, numForceOutputs, forceData);

        Array<double> fibVelData;
        fibVelData.setSize(numFibVelOutputs);
        fibVelStorage->getDataAtTime(t, numFibVelOutputs, fibVelData);

        const double powerExpected = -forceData[idx_force_muscle1]
                                     * fibVelData[idx_fibVel_muscle1];
        ASSERT_EQUAL(probeData[probeCol["umbMechWork_rate_m1"]], powerExpected,
                     1.0e-2, __FILE__, __LINE__,
            "Umberger2010: mechanical power disagrees with muscle analysis.");

        ASSERT_EQUAL(probeData[probeCol["bhaMechWork_rate_m1"]], powerExpected,
                     1.0e-2, __FILE__, __LINE__,
            "Bhargava2004: mechanical power disagrees with muscle analysis.");
    }

    //--------------------------------------------------------------------------
    // Integrate rates and check total energy liberation results at time t1.
    //--------------------------------------------------------------------------
    std::unique_ptr<Storage> probeStorageInt{probeStorage.integrate(t0, t1)};
    if (OUTPUT_FILES) {
        std::string fname = baseFilename + "_probesInteg.sto";
        probeStorageInt->print(fname);
        cout << "+ saved integrated probe storage file: " << fname << endl;
    }
    cout << "- checking total energy liberation results" << endl;

    // Get data from probes at final time.
    Array<double> probeData_t1;
    probeData_t1.setSize(numProbeOutputs);
    probeStorage.getDataAtTime(t1, numProbeOutputs, probeData_t1);

    Array<double> probeDataInt;
    probeDataInt.setSize(numProbeOutputs);
    probeStorageInt->getDataAtTime(t1, numProbeOutputs, probeDataInt);

    // Total energy at final time must equal integral of total rate.
    ASSERT_EQUAL(probeData_t1[probeCol["umbTotal_m1"]],
                 probeDataInt[probeCol["umbTotal_rate_m1"]],
                 1.0e-2, __FILE__, __LINE__,
        "Umberger2010: integral of total rate differs from final total energy.");

    ASSERT_EQUAL(probeData_t1[probeCol["bhaTotal_m1"]],
                 probeDataInt[probeCol["bhaTotal_rate_m1"]],
                 1.0e-2, __FILE__, __LINE__,
        "Bhargava2004: integral of total rate differs from final total energy.");

    // Check reporting of metabolic probe components: Umberger2010.
    ASSERT_EQUAL(probeData_t1[probeCol["umbTotalAllPieces_both_basal"]],
                 probeDataInt[probeCol["umbBasal_rate_m1"]],
                 1.0e-2, __FILE__, __LINE__,
        "Umberger2010: error in reporting components of metabolic probe.");

    ASSERT_EQUAL(probeData_t1[probeCol["umbTotalAllPieces_both_muscle1"]],
                 probeData_t1[probeCol["umbTotal_m1"]]
                 - probeDataInt[probeCol["umbBasal_rate_m1"]],
                 1.0e-2, __FILE__, __LINE__,
        "Umberger2010: error in reporting components of metabolic probe.");

    ASSERT_EQUAL(probeData_t1[probeCol["umbTotalAllPieces_both_muscle2"]],
                 probeData_t1[probeCol["umbTotal_m2"]]
                 - probeDataInt[probeCol["umbBasal_rate_m1"]],
                 1.0e-2, __FILE__, __LINE__,
        "Umberger2010: error in reporting components of metabolic probe.");

    // Check reporting of metabolic probe components: Bhargava2004.
    ASSERT_EQUAL(probeData_t1[probeCol["bhaTotalAllPieces_both_basal"]],
                 probeDataInt[probeCol["bhaBasal_rate_m1"]],
                 1.0e-2, __FILE__, __LINE__,
        "Bhargava2004: error in reporting components of metabolic probe.");

    ASSERT_EQUAL(probeData_t1[probeCol["bhaTotalAllPieces_both_muscle1"]],
                 probeData_t1[probeCol["bhaTotal_m1"]]
                 - probeDataInt[probeCol["bhaBasal_rate_m1"]],
                 1.0e-2, __FILE__, __LINE__,
        "Bhargava2004: error in reporting components of metabolic probe.");

    ASSERT_EQUAL(probeData_t1[probeCol["bhaTotalAllPieces_both_muscle2"]],
                 probeData_t1[probeCol["bhaTotal_m2"]]
                 - probeDataInt[probeCol["bhaBasal_rate_m1"]],
                 1.0e-2, __FILE__, __LINE__,
        "Bhargava2004: error in reporting components of metabolic probe.");

    // Check reporting for multiple muscles.
    //   Total energy for muscle1      = basal + heat1 + work1
    //   Total energy for muscle2      = basal + heat2 + work2
    //   Total energy for both muscles = basal + heat1 + heat2 + work1 + work2
    ASSERT_EQUAL(probeData_t1[probeCol["umbTotal_both"]],
                 probeData_t1[probeCol["umbTotal_m1"]]
                 + probeData_t1[probeCol["umbTotal_m2"]]
                 - probeDataInt[probeCol["umbBasal_rate_m1"]],
                 1.0e-2, __FILE__, __LINE__,
        "Umberger2010: error in reporting data for multiple muscles.");

    ASSERT_EQUAL(probeData_t1[probeCol["bhaTotal_both"]],
                 probeData_t1[probeCol["bhaTotal_m1"]]
                 + probeData_t1[probeCol["bhaTotal_m2"]]
                 - probeDataInt[probeCol["bhaBasal_rate_m1"]],
                 1.0e-2, __FILE__, __LINE__,
        "Bhargava2004: error in reporting data for multiple muscles.");

    //--------------------------------------------------------------------------
    // Run simulation with lower activation and ensure less energy is liberated.
    //--------------------------------------------------------------------------
    cout << "- reconfiguring with lower activation and fewer probes" << endl;
    desiredActivation = 0.5;
    muscle1->setDefaultActivation(desiredActivation);
    muscle2->setDefaultActivation(desiredActivation);
    controller->setConstantExcitation(desiredActivation);

    // Retain only the probes that report the total energy liberated.
    model.removeAnalysis(probeReporter);
    model.removeAnalysis(muscleAnalysis);
    int idx = 0;
    while (idx < model.getProbeSet().getSize()) {
        std::string thisName = model.getProbeSet().get(idx).getName();
        if (thisName=="umbergerTotal_m1" || thisName=="umbergerTotal_m2" ||
            thisName=="bhargavaTotal_m1" || thisName=="bhargavaTotal_m2")
            idx++;
        else
            model.removeProbe(&model.getProbeSet().get(idx));
    }
    ProbeReporter* probeReporter2 = new ProbeReporter(&model);
    model.addAnalysis(probeReporter2);

    // Print the model.
    model.finalizeFromProperties();
    printf("\n"); model.printBasicInfo(); printf("\n");
    if (OUTPUT_FILES) {
        const std::string fname = baseFilename + "Model2.osim";
        model.print(fname);
        cout << "+ saved model file: " << fname << endl;
    }

    // Simulate.
    model.setup();
    Storage stateStorage2 = simulateModel(model, t0, t1);

    if (OUTPUT_FILES) {
        std::string fname = baseFilename + "_states2.sto";
        stateStorage2.print(fname);
        cout << "+ saved state storage file: " << fname << endl;

        fname = baseFilename + "_probes2.sto";
        probeReporter2->getProbeStorage().print(fname);
        cout << "+ saved probe storage file: " << fname << endl;
    }

    // Total energy liberated must be less than that liberated in the first
    // simulation, where muscle activation was higher.
    cout << "- checking total energy liberation results" << endl;

    const Storage& probeStorage2 = probeReporter2->getProbeStorage();
    const int numProbeOutputs2 = probeStorage2.getColumnLabels().getSize()-1;
    ASSERT(numProbeOutputs2 == 4, __FILE__, __LINE__,
        "Incorrect number of columns in probe storage.");

    Array<double> probeData2_t1;
    probeData2_t1.setSize(numProbeOutputs2);
    probeStorage2.getDataAtTime(t1, numProbeOutputs2, probeData2_t1);

    std::map<std::string, int> probeCol2;
    probeCol2["umbTotal_m1"] = probeStorage2
      .getColumnIndicesForIdentifier("umbergerTotal_m1_TOTAL")[0]-1;
    probeCol2["umbTotal_m2"] = probeStorage2
      .getColumnIndicesForIdentifier("umbergerTotal_m2_TOTAL")[0]-1;
    probeCol2["bhaTotal_m1"] = probeStorage2
      .getColumnIndicesForIdentifier("bhargavaTotal_m1_TOTAL")[0]-1;
    probeCol2["bhaTotal_m2"] = probeStorage2
      .getColumnIndicesForIdentifier("bhargavaTotal_m2_TOTAL")[0]-1;

    ASSERT(probeData_t1[probeCol["umbTotal_m1"]] >
           probeData2_t1[probeCol2["umbTotal_m1"]], __FILE__, __LINE__,
           "Umberger2010: total energy must decrease with lower activation.");
    ASSERT(probeData_t1[probeCol["umbTotal_m2"]] >
           probeData2_t1[probeCol2["umbTotal_m2"]], __FILE__, __LINE__,
           "Umberger2010: total energy must decrease with lower activation.");
    ASSERT(probeData_t1[probeCol["bhaTotal_m1"]] >
           probeData2_t1[probeCol2["bhaTotal_m1"]], __FILE__, __LINE__,
           "Bhargava2004: total energy must decrease with lower activation.");
    ASSERT(probeData_t1[probeCol["bhaTotal_m2"]] >
           probeData2_t1[probeCol2["bhaTotal_m2"]], __FILE__, __LINE__,
           "Bhargava2004: total energy must decrease with lower activation.");
}


//==============================================================================
//                                     MAIN
//==============================================================================
void horizontalRule() { for(int i=0;i<80;++i) cout<<"*"; cout<<endl; }
int main()
{
    SimTK::Array_<std::string> failures;

    printf("\n"); horizontalRule();
    cout << "Comparing Umberger2010 probe output to published results" << endl;
    horizontalRule();
    try { compareUmbergerProbeToPublishedResults();
        cout << "\ncompareUmbergerProbeToPublishedResults test passed\n" << endl;
    } catch (const OpenSim::Exception& e) {
        e.print(cerr);
        failures.push_back("compareUmbergerProbeToPublishedResults");
    }

    printf("\n"); horizontalRule();
    cout << "Testing Umberger2010 and Bhargava2004 probes in simulation" << endl;
    horizontalRule();
    try { testProbesUsingMillardMuscleSimulation();
        cout << "\ntestProbesUsingMillardMuscleSimulation test passed\n" << endl;
    } catch (const OpenSim::Exception& e) {
        e.print(cerr);
        failures.push_back("testProbesUsingMillardMuscleSimulation");
    }

    printf("\n"); horizontalRule(); horizontalRule();
    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "testMuscleMetabolicsProbes passed\n" << endl;
    return 0;
}
