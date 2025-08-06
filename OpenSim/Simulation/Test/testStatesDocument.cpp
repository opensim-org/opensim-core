/* -------------------------------------------------------------------------- *
 *                    OpenSim:  testStatesDocument.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2024 Stanford University and the Authors                     *
 * Author(s): F. C. Anderson                                                  *
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
#include <simbody/internal/Force.h>
#include <OpenSim/Simulation/Model/Force.h>

#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/StatesDocument.h>
#include <OpenSim/Simulation/StatesTrajectory.h>
#include <OpenSim/Simulation/StatesTrajectoryReporter.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/PointToPointSpring.h>
#include <catch2/catch_all.hpp>

using namespace SimTK;
using namespace OpenSim;
using std::cout;
using std::endl;
using std::string;
using std::vector;


// Internal static methods and classes.
namespace
{

// Constant used to determine equality tolerances
const double padFactor = 1.0 + SimTK::SignificantReal;


//-----------------------------------------------------------------------------
// Create a force component derived from PointToPointSpring that adds a
// discrete state of each supported type (bool, int, double, Vec2, Vec3,
// Vec4, Vec5, Vec6).
class ExtendedPointToPointSpring : public OpenSim::PointToPointSpring
{
    OpenSim_DECLARE_CONCRETE_OBJECT(ExtendedPointToPointSpring,
        OpenSim::PointToPointSpring);

private:
    // Subsystem index
    SubsystemIndex indexSS;
    // Indexes of discrete variables
    DiscreteVariableIndex indexBool;
    DiscreteVariableIndex indexInt;
    DiscreteVariableIndex indexDbl;
    DiscreteVariableIndex indexVec2;
    DiscreteVariableIndex indexVec3;
    DiscreteVariableIndex indexVec4;
    DiscreteVariableIndex indexVec5;
    DiscreteVariableIndex indexVec6;
    // Names of discrete variables
    string nameBool{"dvBool"};
    string nameInt{"dvInt"};
    string nameDbl{"dvDbl"};
    string nameVec2{"dvVec2"};
    string nameVec3{"dvVec3"};
    string nameVec4{"dvVec4"};
    string nameVec5{"dvVec5"};
    string nameVec6{"dvVec6"};
    // Omit a discrete state altogether
    int omit;

public:

    // Constructor
    // @param which Specify which discrete state name (0 to 7) to append the
    // suffix to.
    // @param suffix String to append to the discrete state name.
    // @param omit Specify the discrete state to omit.
    ExtendedPointToPointSpring(const PhysicalFrame& body1, SimTK::Vec3 point1,
        const PhysicalFrame& body2, SimTK::Vec3 point2,
        double stiffness, double restlength,
        int which = -1, const string& suffix = "", int omit = -1) :
        PointToPointSpring(body1, point1, body2, point2, stiffness, restlength),
        omit(omit)
    {
        switch (which) {
        case(0) :
            nameBool += suffix;
            break;
        case(1) :
            nameInt += suffix;
            break;
        case(2) :
            nameDbl += suffix;
            break;
        case(3) :
            nameVec2 += suffix;
            break;
        case(4) :
            nameVec3 += suffix;
            break;
        case(5) :
            nameVec4 += suffix;
            break;
        case(6) :
            nameVec5 += suffix;
            break;
        case(7) :
            nameVec6 += suffix;
            break;
        }
    }

    void
    extendAddToSystemAfterSubcomponents(SimTK::MultibodySystem& system) const override
    {
        Super::extendAddToSystemAfterSubcomponents(system);

        // Add the discrete state to the list of OpenSim Components
        // For exception testing purposes, the member variable 'omit' is used
        // to omit one state.
        bool allocate = false;
        if(omit!=0) addDiscreteVariable(nameBool, Stage::Position, allocate);
        if(omit!=1) addDiscreteVariable(nameInt, Stage::Position, allocate);
        if(omit!=2) addDiscreteVariable(nameDbl, Stage::Position, allocate);
        if(omit!=3) addDiscreteVariable(nameVec2, Stage::Position, allocate);
        if(omit!=4) addDiscreteVariable(nameVec3, Stage::Position, allocate);
        if(omit!=5) addDiscreteVariable(nameVec4, Stage::Position, allocate);
        if(omit!=6) addDiscreteVariable(nameVec5, Stage::Position, allocate);
        if(omit!=7) addDiscreteVariable(nameVec6, Stage::Position, allocate);
    }

    void
    extendRealizeTopology(SimTK::State& s) const override
    {
        Super::extendRealizeTopology(s);

        // Create a mutableThis
        ExtendedPointToPointSpring* mutableThis =
            const_cast<ExtendedPointToPointSpring*>(this);

        // Get the Subsystem
        const DefaultSystemSubsystem& fsub = getModel().getDefaultSubsystem();
        mutableThis->indexSS = fsub.getMySubsystemIndex();

        // 0 Bool
        if(omit != 0) {
            bool dvBool{false};
            mutableThis->indexBool =
                s.allocateAutoUpdateDiscreteVariable(indexSS,
                Stage::Velocity, new Value<bool>(dvBool), Stage::Dynamics);
            initializeDiscreteVariableIndexes(nameBool, indexSS, indexBool);
        }

        // 1 Int
        if(omit != 1) {
            int dvInt{0};
            mutableThis->indexInt =
                s.allocateAutoUpdateDiscreteVariable(indexSS,
                    Stage::Velocity, new Value<int>(dvInt), Stage::Dynamics);
            initializeDiscreteVariableIndexes(nameInt, indexSS, indexInt);
        }

        // 2 Dbl
        if(omit != 2) {
            double dvDbl{0.0};
            mutableThis->indexDbl =
                s.allocateAutoUpdateDiscreteVariable(indexSS,
                Stage::Velocity, new Value<double>(dvDbl), Stage::Dynamics);
            initializeDiscreteVariableIndexes(nameDbl, indexSS, indexDbl);
        }

        // 3 Vec2
        if(omit != 3) {
            Vec2 dvVec2(0.1, 0.2);
            mutableThis->indexVec2 =
                s.allocateAutoUpdateDiscreteVariable(indexSS,
                Stage::Velocity, new Value<Vec2>(dvVec2), Stage::Dynamics);
            initializeDiscreteVariableIndexes(nameVec2, indexSS, indexVec2);
        }

        // 4 Vec3
        if(omit != 4) {
            Vec3 dvVec3(0.1, 0.2, 0.3);
            mutableThis->indexVec3 =
                s.allocateAutoUpdateDiscreteVariable(indexSS,
                Stage::Velocity, new Value<Vec3>(dvVec3), Stage::Dynamics);
            initializeDiscreteVariableIndexes(nameVec3, indexSS, indexVec3);
        }

        // 5 Vec4
        if(omit != 5) {
            Vec4 dvVec4(0.1, 0.2, 0.3, 0.4);
            mutableThis->indexVec4 =
                s.allocateAutoUpdateDiscreteVariable(indexSS,
                Stage::Velocity, new Value<Vec4>(dvVec4), Stage::Dynamics);
            initializeDiscreteVariableIndexes(nameVec4, indexSS, indexVec4);
        }

        // 6 Vec5
        if(omit != 6) {
            Vec5 dvVec5(0.1, 0.2, 0.3, 0.4, 0.5);
            mutableThis->indexVec5 =
                s.allocateAutoUpdateDiscreteVariable(indexSS,
                Stage::Velocity, new Value<Vec5>(dvVec5), Stage::Dynamics);
            initializeDiscreteVariableIndexes(nameVec5, indexSS, indexVec5);
        }

        // 7 Vec6
        if(omit != 7) {
            Vec6 dvVec6(0.1, 0.2, 0.3, 0.4, 0.5, 0.6);
            mutableThis->indexVec6 =
                s.allocateAutoUpdateDiscreteVariable(indexSS,
                Stage::Velocity, new Value<Vec6>(dvVec6), Stage::Dynamics);
            initializeDiscreteVariableIndexes(nameVec6, indexSS, indexVec6);
        }
    }

    // Set the values of the discrete variables.
    // The actual force calculation is done in SimTK::TwoPointLinearSpring.
    // This method just provides a means of setting the added discrete
    // variables so that they change during the course of a simulation.
    // The discrete variables are just set to the generalized speeds.
    virtual void computeForce(const SimTK::State& state,
        SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
        SimTK::Vector& generalizedForces) const override
    {
        Super::computeForce(state, bodyForces, generalizedForces);

        const SimTK::Vector& u = state.getU();

        // 0 Bool
        if (omit != 0) {
            bool& vBool = SimTK::Value<bool>::downcast(
                state.updDiscreteVarUpdateValue(indexSS, indexBool));
            vBool = u[0];
            state.markDiscreteVarUpdateValueRealized(indexSS, indexBool);
        }

        // 1 Int
        if (omit != 1) {
            SimTK::Value<int>::downcast(
                state.updDiscreteVarUpdateValue(indexSS, indexInt)) = u[0];
            state.markDiscreteVarUpdateValueRealized(indexSS, indexInt);
        }

        // 2 Dbl
        if (omit != 2) {
            SimTK::Value<double>::downcast(
                state.updDiscreteVarUpdateValue(indexSS, indexDbl)) = u[0];
            state.markDiscreteVarUpdateValueRealized(indexSS, indexDbl);
        }

        // 3 Vec2
        if (omit != 3) {
            Vec2& v2 = SimTK::Value<Vec2>::downcast(
                state.updDiscreteVarUpdateValue(indexSS, indexVec2));
            v2[0] = u[0];
            v2[1] = u[1];
            state.markDiscreteVarUpdateValueRealized(indexSS, indexVec2);
        }

        // 4 Vec3
        if (omit != 4) {
            Vec3& v3 = SimTK::Value<Vec3>::downcast(
                state.updDiscreteVarUpdateValue(indexSS, indexVec3));
            v3[0] = u[0];
            v3[1] = u[1];
            v3[2] = u[2];
            state.markDiscreteVarUpdateValueRealized(indexSS, indexVec3);
        }

        // 5 Vec4
        if (omit != 5) {
            Vec4& v4 = SimTK::Value<Vec4>::downcast(
                state.updDiscreteVarUpdateValue(indexSS, indexVec4));
            v4[0] = u[0];
            v4[1] = u[1];
            v4[2] = u[2];
            v4[3] = u[3];
            state.markDiscreteVarUpdateValueRealized(indexSS, indexVec4);
        }

        // 6 Vec5
        if (omit != 6) {
            Vec5& v5 = SimTK::Value<Vec5>::downcast(
                state.updDiscreteVarUpdateValue(indexSS, indexVec5));
            v5[0] = u[0];
            v5[1] = u[1];
            v5[2] = u[2];
            v5[3] = u[3];
            v5[4] = u[4];
            state.markDiscreteVarUpdateValueRealized(indexSS, indexVec5);
        }

        // 7 Vec6
        if (omit != 7) {
            Vec6& v6 = SimTK::Value<Vec6>::downcast(
                state.updDiscreteVarUpdateValue(indexSS, indexVec6));
            v6[0] = u[0];
            v6[1] = u[1];
            v6[2] = u[2];
            v6[3] = u[3];
            v6[4] = u[4];
            v6[5] = u[5];
            state.markDiscreteVarUpdateValueRealized(indexSS, indexVec6);
        }
    }

}; // End of class ExtendedPointToPointSpring


//-----------------------------------------------------------------------------
// Other Local Static Methods
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
Compute the maximum error that can result from rounding a value at a
specified precision. This method assumes a base-10 representation of the value.
@param value Value to be rounded.
@param precision Number of significant figures that will be retained in the
value.
@return Maximum rounding error.

double
computeMaxRoundingError(double value, int precision) {
    if (value == 0) return 0.0;
    int p = clamp(1, precision, SimTK::LosslessNumDigitsReal);
    double leastSigDigit = trunc(log10(fabs(value))-precision);
    double max_eps = 0.5*pow(10.0, leastSigDigit);
    if(max_eps < SimTK::LeastPositiveReal) return SimTK::LeastPositiveReal;
    return max_eps;
}
*/ // No longer used, but might be useful elsewhere, so saving.

//_____________________________________________________________________________
/**
Compute the expected error that will occur as a result of rounding a value at
a specified precision.
@param value Value to be rounded.
@param precision Number of significant figures that will be retained in the
value.
@return Expected rounding error.
*/
double
computeRoundingError(const double& value, int precision) {
    int p = clamp(1, precision, SimTK::LosslessNumDigitsReal);
    SimTK::String valueStr(value, precision);
    double valueDbl;
    if(!valueStr.tryConvertToDouble(valueDbl))
        cout << "Conversion to double failed" << endl;
    return fabs(valueDbl - value);
}

//_____________________________________________________________________________
// Test for equality of the continuous variables in two state trajectories.
void
testEqualityForContinuousVariables(const Model& model,
    const Array_<State>& trajA, const Array_<State>& trajB, int precision)
{
    // Continuous variables are gathered efficiently without using any
    // OpenSim::Component methods by using state.getQ(), state.getU(), and
    // state.getZ().
    double tol;
    double tA, tB;
    const State* stateA = trajA.cbegin();
    const State* stateB = trajB.cbegin();

    // Loop over time
    for(int iTime=0; stateA!=trajA.cend(); ++iTime, ++stateA, ++stateB) {

        // Check subsystem consistency
        // This checks that basic parameters like number of subystem, nq, nu,
        // and nz are the same for two state objects.
        REQUIRE(stateA->isConsistent(*stateB));

        // Get time
        tA = stateA->getTime();
        tB = stateB->getTime();
        tol = padFactor * computeRoundingError(tA, precision);
        CHECK_THAT(tB, Catch::Matchers::WithinAbs(tA, tol));

        // Check the number of subsystesm
        int nsubA = stateA->getNumSubsystems();
        int nsubB = stateB->getNumSubsystems();
        REQUIRE(nsubA == nsubB);

        // Q
        double diff;
        const Vector& qA = stateA->getQ();
        const Vector& qB = stateB->getQ();
        int nq = qA.size();
        for (int i = 0; i < nq; ++i) {
            tol = padFactor * computeRoundingError(qA[i], precision);
            CHECK_THAT(qB[i], Catch::Matchers::WithinAbs(qA[i], tol));
        }
        // U
        const Vector& uA = stateA->getU();
        const Vector& uB = stateB->getU();
        int nu = uA.size();
        for (int i = 0; i < nu; ++i) {
            tol = padFactor * computeRoundingError(uA[i], precision);
            CHECK_THAT(uB[i], Catch::Matchers::WithinAbs(uA[i], tol));
        }
        // Z
        const Vector& zA = stateA->getZ();
        const Vector& zB = stateB->getZ();
        int nz = zA.size();
        for (int i = 0; i < nz; ++i) {
            tol = padFactor * computeRoundingError(zA[i], precision);
            CHECK_THAT(zB[i], Catch::Matchers::WithinAbs(zA[i], tol));
        }
    }
}

//_____________________________________________________________________________
// Test for equality of a scalar variable in two state trajectories.
template <class T>
void
checkScalar(const Array_<T>& a, const Array_<T>& b, int precision)
{
    double tol;
    Array_<T> dvA, dvB;
    for (size_t i = 0; i < (size_t)dvA.size(); ++i) {
        tol = padFactor*computeRoundingError(a[i], precision);
        CHECK_THAT(b[i], Catch::Matchers::WithinAbs(a[i], tol));
    }
}

//_____________________________________________________________________________
// Test for equality of a Vector variable in two state trajectories.
template <class T>
void
checkVector(const Array_<T>& a, const Array_<T>& b, int precision)
{
    double tol;
    for (size_t i = 0; i < (size_t)a.size(); ++i) {
        for (size_t j = 0; j < (size_t)a[i].size(); ++j) {
            tol = padFactor*computeRoundingError(a[i][j], precision);
            CHECK_THAT(b[i][j], Catch::Matchers::WithinAbs(a[i][j], tol));
        }
    }
}

//_____________________________________________________________________________
// Test the equality of the discrete variables.
//
// The SimTK API does not allow an exhaustive, low-level comparison of
// discrete variables on the SimTK side.
//
// The comparision is done only for the discrete variables registered
// in the OpenSim Component heirarchy. Any discrete variable that is
// not registered in OpenSim will not be serialized, deserialized, or
// compared in this unit test.
void
testEqualityForDiscreteVariables(const Model& model,
    const Array_<State>& trajA, const Array_<State>& trajB, int precision)
{
    // Loop over the named variables
    OpenSim::Array<std::string> paths = model.getDiscreteVariableNames();
    int nPaths = paths.getSize();
    for (int i = 0; i < nPaths; ++i) {

        // Get one variable so that its type can be ascertained.
        const AbstractValue& abstractVal =
            model.getDiscreteVariableAbstractValue(trajA[i],paths[i]);

        // Get the trajectory for the discrete variable
        if (SimTK::Value<bool>::isA(abstractVal)) {
            Array_<bool> dvA, dvB;
            model.getDiscreteVariableTrajectory<bool>(paths[i], trajA, dvA);
            model.getDiscreteVariableTrajectory<bool>(paths[i], trajB, dvB);
            for (size_t j = 0; j < (size_t)dvA.size(); ++j)
                CHECK(dvB[j] == dvA[j]);
        }
        else if (SimTK::Value<int>::isA(abstractVal)) {
            Array_<int> dvA, dvB;
            model.getDiscreteVariableTrajectory<int>(paths[i], trajA, dvA);
            model.getDiscreteVariableTrajectory<int>(paths[i], trajB, dvB);
            for (size_t j = 0; j < (size_t)dvA.size(); ++j)
                CHECK(dvB[j] == dvA[j]);
        }
        else if (SimTK::Value<float>::isA(abstractVal)) {
            Array_<float> dvA, dvB;
            model.getDiscreteVariableTrajectory<float>(paths[i], trajA, dvA);
            model.getDiscreteVariableTrajectory<float>(paths[i], trajB, dvB);
            checkScalar<float>(dvA, dvB, precision);
        }
        else if (SimTK::Value<double>::isA(abstractVal)) {
            Array_<double> dvA, dvB;
            model.getDiscreteVariableTrajectory<double>(paths[i], trajA, dvA);
            model.getDiscreteVariableTrajectory<double>(paths[i], trajB, dvB);
            checkScalar<double>(dvA, dvB, precision);
        }
        else if (SimTK::Value<Vec2>::isA(abstractVal)) {
            Array_<Vec2> dvA, dvB;
            model.getDiscreteVariableTrajectory<Vec2>(paths[i], trajA, dvA);
            model.getDiscreteVariableTrajectory<Vec2>(paths[i], trajB, dvB);
            checkVector<Vec2>(dvA, dvB, precision);
        }
        else if (SimTK::Value<Vec3>::isA(abstractVal)) {
            Array_<Vec3> dvA, dvB;
            model.getDiscreteVariableTrajectory<Vec3>(paths[i], trajA, dvA);
            model.getDiscreteVariableTrajectory<Vec3>(paths[i], trajB, dvB);
            checkVector<Vec3>(dvA, dvB, precision);
        }
        else if (SimTK::Value<Vec4>::isA(abstractVal)) {
            Array_<Vec4> dvA, dvB;
            model.getDiscreteVariableTrajectory<Vec4>(paths[i], trajA, dvA);
            model.getDiscreteVariableTrajectory<Vec4>(paths[i], trajB, dvB);
            checkVector<Vec4>(dvA, dvB, precision);
        }
        else if (SimTK::Value<Vec5>::isA(abstractVal)) {
            Array_<Vec5> dvA, dvB;
            model.getDiscreteVariableTrajectory<Vec5>(paths[i], trajA, dvA);
            model.getDiscreteVariableTrajectory<Vec5>(paths[i], trajB, dvB);
            checkVector<Vec5>(dvA, dvB, precision);
        }
        else if (SimTK::Value<Vec6>::isA(abstractVal)) {
            Array_<Vec6> dvA, dvB;
            model.getDiscreteVariableTrajectory<Vec6>(paths[i], trajA, dvA);
            model.getDiscreteVariableTrajectory<Vec6>(paths[i], trajB, dvB);
            checkVector<Vec6>(dvA, dvB, precision);
        }
        else {
            String msg = "Unrecognized type: " + abstractVal.getTypeName();
            SimTK_ASSERT(false, msg.c_str());
        }
    }
}

//_____________________________________________________________________________
// Test the equality of the modeling options.
//
// The SimTK API does not allow an exhaustive, low-level comparison of
// modeling options on the SimTK side.
//
// The comparision is done only for the modeling options registered
// in the OpenSim Component heirarchy. Any modeling option that is
// not registered in the OpenSim Component hierarchy will not be serialized,
// deserialized, or compared.
void
testEqualityForModelingOptions(const Model& model,
    const Array_<State>& trajA, const Array_<State>& trajB, int precision)
{
    // Loop over the named variables
    OpenSim::Array<std::string> paths = model.getModelingOptionNames();
    int nPaths = paths.getSize();
    for (int i = 0; i < nPaths; ++i) {
        Array_<int> moA, moB;
        model.getModelingOptionTrajectory<int>(paths[i], trajA, moA);
        model.getModelingOptionTrajectory<int>(paths[i], trajB, moB);
        for (size_t j = 0; j<(size_t)moA.size(); ++j) CHECK(moB[j] == moA[j]);
    }
}

//_____________________________________________________________________________
// Test for equality of two state trajectories.
// If a state variable fails an equality test, it is likely that that
// variable has not been added to OpenSim's Component heirarchy and therefore
// has not been serialized.
void
testStateEquality(const Model& model,
    const Array_<State>& trajA, const Array_<State>& trajB, int precision)
{
    testEqualityForContinuousVariables(model, trajA, trajB, precision);
    testEqualityForDiscreteVariables(model, trajA, trajB, precision);
    testEqualityForModelingOptions(model, trajA, trajB, precision);
}

//_____________________________________________________________________________
// Build the model
Model*
buildModel(int whichDiscreteState = -1,
    const string& discreteStateSuffix = "", int omit = -1) {

    // Create an empty model
    Model* model = new Model();
    Vec3 gravity(0.0, -10.0, 0.0);
    model->setGravity(gravity);
    model->setName("BlockOnASpringFreeJoint");

    // Add bodies and joints
    OpenSim::Ground& ground = model->updGround();

    // Body
    std::string name = "block";
    OpenSim::Body* block = new OpenSim::Body();
    double mass = 10.0;
    block->setName(name);
    block->set_mass(mass);
    block->set_mass_center(Vec3(0));
    block->setInertia(Inertia(1.0));
    model->addBody(block);

    // Joint
    name = "free";
    FreeJoint *free = new
        FreeJoint(name, ground, Vec3(0), Vec3(0), *block, Vec3(0), Vec3(0));
    model->addJoint(free);

    // Point-To-Point Spring
    // This actuator connects the origin of the block to the orgin of the
    // coordinate frame.
    double kp = 1000.0; // Stiffness
    double kv = 100.0;  // Viscosity (under-damped)
    double restlength = 0.0;
    Vec3 origin(0.0);
    Vec3 insertion(0.1, 0.1, 0.025);
    ExtendedPointToPointSpring* spring = new ExtendedPointToPointSpring(
        ground, origin, *block, insertion, kp, restlength,
        whichDiscreteState, discreteStateSuffix, omit);
    model->addForce(spring);

    return model;
}

//_____________________________________________________________________________
// Simulate
SimTK::Array_<SimTK::State>
simulate(Model* model) {

    // Add a StatesTrajectoryReporter
    // The reporter records the SimTK::State in a SimTK::Array_<> at a
    // specified time interval.
    OpenSim::StatesTrajectoryReporter* reporter =
        new StatesTrajectoryReporter();
    reporter->setName("states_reporter");
    double interval = 0.1;
    reporter->set_report_time_interval(interval);
    model->addComponent(reporter);

    // Build the system
    model->buildSystem();
    SimTK::State& state = model->initializeState();

    // Integrate
    Manager manager(*model);
    manager.getIntegrator().setMaximumStepSize(0.01);
    manager.setIntegratorAccuracy(1.0e-5);
    double ti = 0.0;
    double tf = 5.0;
    state.setTime(ti);
    manager.initialize(state);
    state = manager.integrate(tf);

    // Return a copy of the underlying state array, after repackaging it
    // as a SimTK::Array_<State>.
    const vector<State>& trajectory = reporter->getVectorOfStateObjects();
    const Array_<State> traj(trajectory);
    return traj;
}

} // End anonymous namespace


TEST_CASE("Serialization and Deserialization")
{
    // Build the model and run a simulation.
    // The output of simulate() is the state trajectory.
    // Note that a copy of the state trajectory is returned, so we don't have
    // to worry about the reporter (or any other object) going out of scope
    // or being deleted.
    Model *model = buildModel();
    Array_<State> traj = simulate(model);

    // Serialize
    SimTK::String filename = "BlockOnASpring.ostates";
    SimTK::String note = "Output from `testStatesDocument.cpp`.";
    for (int p = 1; p < 22; ++p) {
        cout << "Testing for precision = " << p << endl;

        StatesDocument doc(*model, traj, note, p);
        doc.serialize(filename);

        // (A) Deserialize
        StatesDocument docA(filename);
        Array_<State> trajA;
        docA.deserialize(*model, trajA);

        // Check the note and the precision.
        CHECK(docA.getNote() == doc.getNote());
        CHECK(docA.getPrecision() == doc.getPrecision());

        // Check size
        REQUIRE(traj.size() == traj.size());

        // Realize both state trajectories to Stage::Report
        for (size_t i = 0; i < (size_t)trajA.size(); ++i) {
            model->getSystem().realize(traj[i], Stage::Report);
            model->getSystem().realize(trajA[i], Stage::Report);
        }

        // Are state trajectories A and B the same?
        testStateEquality(*model, traj, trajA, p);
    }

    delete model;
}

TEST_CASE("Exceptions")
{
    // Build the default model and run a simulation
    Model *model = buildModel();
    Array_<State> traj = simulate(model);

    // Serialize the default model
    SimTK::String filename = "BlockOnASpring.ostates";
    SimTK::String note = "Output from `testStatesDocument.cpp`.";
    int precision = 6;
    StatesDocument doc(*model, traj, note, precision);
    doc.serialize(filename);

    // (A) Model names don't match
    const string& name = model->getName();
    model->setName(name + "_diff");
    StatesDocument docA(filename);
    Array_<State> trajA;
    CHECK_THROWS(docA.deserialize(*model, trajA),
        "Model names should not match.");
    model->setName(name); // return the original name

    // (B) A discrete state is not found because no name matches
    // In each model, the name of one discrete state is changed.
    string suffix{"_ShouldNotBeFound"};
    for (int which = 0; which < 8; ++which) {
        cout << "Changing the name of discrete state " << which << endl;

        // Build a model that is different only with respect to one name of a
        // specified discrete state.
        Model* modelB = buildModel(which, suffix);
        Array_<State> trajDoNotNeed = simulate(modelB);

        // Deserialize using modelB
        // This should fail when name of a discrete state has been changed.
        StatesDocument docB(filename);
        Array_<State> trajB;
        CHECK_THROWS(docB.deserialize(*modelB, trajB),
            "Discrete state should not be found");

        delete modelB;
    }

    // (C) A discrete state is not found because the state doesn't exist.
    // An exception should be thrown because the number of states don't match.
    for (int which = -1, omit = 0; omit < 8; ++omit) {
        cout << "Omitting discrete state " << omit << endl;

        // Build a model that is different only in that one discrete state
        // is omitted.
        Model* modelC = buildModel(which, suffix, omit);
        Array_<State> trajDoNotNeed = simulate(modelC);

        // Deserialize using modelC
        StatesDocument docC(filename);
        Array_<State> trajC;
        CHECK_THROWS(docC.deserialize(*modelC, trajC),
            "Expected number of discrete states should be wrong");

        delete modelC;
    }

    delete model;
}