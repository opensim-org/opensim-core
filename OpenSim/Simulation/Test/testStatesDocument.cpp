/* -------------------------------------------------------------------------- *
*                OpenSim:  testComponentInterface.cpp                        *
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


// Internal static methods and classes.
namespace
{

// Constant used to determine equality tolerances
const double padFactor = 1.0 + SimTK::SignificantReal;

//=============================================================================
// Extension of the underlying Simbody TwoPointLinearSpring force class.
// Discrete variables of all types supported by OpenSim are added to the
// SimTK::State and given values in an overriding SimTK::relalizePosition()
// implementation.
class ExtendedTwoPointLinearSpring : public SimTK::Force::TwoPointLinearSpring {

public:
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

    // Constructor
    ExtendedTwoPointLinearSpring(SimTK::GeneralForceSubsystem& subsystem,
        const MobilizedBody& body1, const Vec3 point1,
        const MobilizedBody& body2, const Vec3 point2,
        double stiffness, double restlength) :
        TwoPointLinearSpring(subsystem, body1, point1, body2, point2,
                stiffness, restlength) {}

    // Allocate the discrete variables
    void realizeTopology(SimTK::State& state) const {
        // Create a mutableThis
        ExtendedTwoPointLinearSpring* mutableThis =
            const_cast<ExtendedTwoPointLinearSpring*>(this);

        // Get the GeneralForceSubsystem
        const GeneralForceSubsystem& fsub = getForceSubsystem();
        mutableThis->indexSS = fsub.getMySubsystemIndex();

        // Bool
        bool dvBool{false};
        mutableThis->indexBool = fsub.allocateAutoUpdateDiscreteVariable(state,
            Stage::Position, new Value<bool>(dvBool), Stage::Position);

        // Int
        int dvInt{0};
        mutableThis->indexInt = fsub.allocateAutoUpdateDiscreteVariable(state,
            Stage::Position, new Value<int>(dvInt), Stage::Position);

        // Dbl
        double dvDbl{0.0};
        mutableThis->indexDbl = fsub.allocateAutoUpdateDiscreteVariable(state,
            Stage::Position, new Value<double>(dvDbl), Stage::Position);

        // Vec2
        Vec2 dvVec2(0.1, 0.2);
        mutableThis->indexVec2 = fsub.allocateAutoUpdateDiscreteVariable(state,
            Stage::Position, new Value<Vec2>(dvVec2), Stage::Position);

        // Vec3
        Vec3 dvVec3(0.1, 0.2, 0.3);
        mutableThis->indexVec3 = fsub.allocateAutoUpdateDiscreteVariable(state,
            Stage::Position, new Value<Vec3>(dvVec3), Stage::Position);

        // Vec4
        Vec4 dvVec4(0.1, 0.2, 0.3, 0.4);
        mutableThis->indexVec4 = fsub.allocateAutoUpdateDiscreteVariable(state,
            Stage::Position, new Value<Vec4>(dvVec4), Stage::Position);

        // Vec5
        Vec5 dvVec5(0.1, 0.2, 0.3, 0.4, 0.5);
        mutableThis->indexVec5 = fsub.allocateAutoUpdateDiscreteVariable(state,
            Stage::Position, new Value<Vec5>(dvVec5), Stage::Position);

        // Vec6
        Vec6 dvVec6(0.1, 0.2, 0.3, 0.4, 0.5, 0.6);
        mutableThis->indexVec6 = fsub.allocateAutoUpdateDiscreteVariable(state,
            Stage::Position, new Value<Vec6>(dvVec6), Stage::Position);
    }

    // Set values of the discrete variables
    // These values don't mean anything. They just provided a means of
    // validating that the discrete variables are changing during a simulation
    // and are being de/serialized correctly.
    void realizePosition(const State& state) const {

        SimTK::GeneralForceSubsystem& fsub = SimTK::GeneralForceSubsystem();
        const SimTK::Vector& u = state.getU();

        // Bool
        SimTK::Value<bool>::downcast(
            fsub.updDiscreteVarUpdateValue(state, indexBool)) = u[0];
        fsub.markDiscreteVarUpdateValueRealized(state, indexBool);

        // Int
        SimTK::Value<int>::downcast(
            fsub.updDiscreteVarUpdateValue(state, indexInt)) = u[0];
        fsub.markDiscreteVarUpdateValueRealized(state, indexInt);

        // Dbl
        SimTK::Value<double>::downcast(
            fsub.updDiscreteVarUpdateValue(state, indexDbl)) = u[0];
        fsub.markDiscreteVarUpdateValueRealized(state, indexDbl);

        // Vec2
        Vec2& v2 = SimTK::Value<Vec2>::downcast(
            fsub.updDiscreteVarUpdateValue(state, indexVec2));
        v2[0] = u[0];
        v2[1] = u[1];
        fsub.markDiscreteVarUpdateValueRealized(state, indexVec2);

        // Vec3
        Vec3& v3 = SimTK::Value<Vec3>::downcast(
            fsub.updDiscreteVarUpdateValue(state, indexVec3));
        v3[0] = u[0];
        v3[1] = u[1];
        v3[2] = u[2];
        fsub.markDiscreteVarUpdateValueRealized(state, indexVec3);

        // Vec4
        Vec4& v4 = SimTK::Value<Vec4>::downcast(
            fsub.updDiscreteVarUpdateValue(state, indexVec4));
        v4[0] = u[0];
        v4[1] = u[1];
        v4[2] = u[2];
        v4[3] = u[3];
        fsub.markDiscreteVarUpdateValueRealized(state, indexVec4);

        // Vec5
        Vec5& v5 = SimTK::Value<Vec5>::downcast(
            fsub.updDiscreteVarUpdateValue(state, indexVec5));
        v5[0] = u[0];
        v5[1] = u[1];
        v5[2] = u[2];
        v5[3] = u[3];
        v5[4] = u[4];
        fsub.markDiscreteVarUpdateValueRealized(state, indexVec5);

        // Vec6
        Vec6& v6 = SimTK::Value<Vec6>::downcast(
            fsub.updDiscreteVarUpdateValue(state, indexVec6));
        v6[0] = u[0];
        v6[1] = u[1];
        v6[2] = u[2];
        v6[3] = u[3];
        v6[4] = u[4];
        v6[5] = u[5];
        fsub.markDiscreteVarUpdateValueRealized(state, indexVec6);
    }

};


//-----------------------------------------------------------------------------
// Create a Component class for the purpose of adding discrete variables of
// all supported types (bool, int, double, Vec2, Vec3, Vec4, Vec5, Vec6.
class ExtendedPointToPointSpring : public OpenSim::Force {
    OpenSim_DECLARE_CONCRETE_OBJECT(ExtendedPointToPointSpring, OpenSim::Force);

public:

    ExtendedTwoPointLinearSpring* sbSpring{nullptr};

    // Properties
    OpenSim_DECLARE_PROPERTY(point1, SimTK::Vec3,
        "Spring attachment point on body1.");
    OpenSim_DECLARE_PROPERTY(point2, SimTK::Vec3,
        "Spring attachment point on body2.");
    OpenSim_DECLARE_PROPERTY(stiffness, double,
        "Spring stiffness (N/m).");
    OpenSim_DECLARE_PROPERTY(rest_length, double,
        "Spring resting length (m).");

    // Sockets
    OpenSim_DECLARE_SOCKET(body1, PhysicalFrame,
        "A frame on the first body that this spring connects to.");
    OpenSim_DECLARE_SOCKET(body2, PhysicalFrame,
        "A frame on the second body that this spring connects to.");

    // Constructor
    ExtendedPointToPointSpring(const PhysicalFrame& body1, SimTK::Vec3 point1,
        const PhysicalFrame& body2, SimTK::Vec3 point2,
        double stiffness, double restlength)
    {
        setNull();
        constructProperties();

        // Set properties to the passed-in values.
        setBody1(body1);
        setBody2(body2);

        setPoint1(point1);
        setPoint2(point2);

        setStiffness(stiffness);
        setRestlength(restlength);

    }

    // Body accessors
    void setBody1(const PhysicalFrame& body) {
        connectSocket_body1(body); }
    void setBody2(const PhysicalFrame& body) {
        connectSocket_body2(body); }
    const PhysicalFrame& getBody1() const {
        return getConnectee<PhysicalFrame>("body1"); }
    const PhysicalFrame& getBody2() const {
        return getConnectee<PhysicalFrame>("body2"); }
    // Point accessors
    void setPoint1(SimTK::Vec3 aPosition) { set_point1(aPosition); }
    const SimTK::Vec3& getPoint1() const { return get_point1(); }
    void setPoint2(SimTK::Vec3 aPosition) { set_point2(aPosition); }
    const SimTK::Vec3& getPoint2() const { return get_point2(); }
    // Consitutive accessors
    void setStiffness(double stiffness) {set_stiffness(stiffness);}
    double getStiffness() const {return get_stiffness();}
    void setRestlength(double restLength) {set_rest_length(restLength);}
    double getRestlength() const {return get_rest_length();}


    void
    extendAddToSystem(SimTK::MultibodySystem& system) const {
        Super::extendAddToSystem(system);


    }


    void
    extendAddToSystemAfterSubcomponents(SimTK::MultibodySystem& system) const {
        Super::extendAddToSystemAfterSubcomponents(system);

        const PhysicalFrame& body1 = getBody1();
        const PhysicalFrame& body2 = getBody2();

        // Get underlying mobilized bodies
        const SimTK::MobilizedBody& b1 = body1.getMobilizedBody();
        const SimTK::MobilizedBody& b2 = body2.getMobilizedBody();

        // Now create a Simbody Force::ExtendedTwoPointLinearSpring
        ExtendedPointToPointSpring* mutableThis =
            const_cast<ExtendedPointToPointSpring *>(this);
        mutableThis->sbSpring = new ExtendedTwoPointLinearSpring(
                _model->updForceSubsystem(),
                b1, getPoint1(), b2, getPoint2(),
                getStiffness(), getRestlength());

        // Beyond the const Component get the index so we can access the
        // SimTK::Force later.
        mutableThis->_index = mutableThis->sbSpring->getForceIndex();

        // Initialize the indexes
        bool allocate = false;
        addDiscreteVariable(sbSpring->nameBool, Stage::Position, allocate);
        addDiscreteVariable(sbSpring->nameInt, Stage::Position, allocate);
        addDiscreteVariable(sbSpring->nameDbl, Stage::Position, allocate);
        addDiscreteVariable(sbSpring->nameVec2, Stage::Position, allocate);
        addDiscreteVariable(sbSpring->nameVec3, Stage::Position, allocate);
        addDiscreteVariable(sbSpring->nameVec4, Stage::Position, allocate);
        addDiscreteVariable(sbSpring->nameVec5, Stage::Position, allocate);
        addDiscreteVariable(sbSpring->nameVec6, Stage::Position, allocate);
    }

    void extendConnectToModel(Model& model) override {
        // validate that the spring is attached to two different base
        // frames; otherwise, unusual simulation behavior may occur
        // (#3485)
        auto const& pf1 = getConnectee<PhysicalFrame>("body1");
        auto const& pf2 = getConnectee<PhysicalFrame>("body2");
        OpenSim::Frame const& pf1Base = pf1.findBaseFrame();

        if (&pf1Base == &pf2.findBaseFrame()) {
            std::stringstream ss;
            ss << " body1 (" << pf1.getAbsolutePathString() << ") and body2 (" << pf2.getAbsolutePathString() << ") have the same base frame (" << pf1Base.getAbsolutePathString() << "), this is not permitted.";
            OPENSIM_THROW_FRMOBJ(OpenSim::Exception, std::move(ss).str());
        }
    }

    void
    extendRealizeTopology(SimTK::State& state) const override {
        Super::extendRealizeTopology(state);
        initializeDiscreteVariableIndexes(sbSpring->nameBool,
            sbSpring->indexSS, sbSpring->indexBool);
        initializeDiscreteVariableIndexes(sbSpring->nameInt,
            sbSpring->indexSS, sbSpring->indexInt);
        initializeDiscreteVariableIndexes(sbSpring->nameDbl,
            sbSpring->indexSS, sbSpring->indexDbl);
        initializeDiscreteVariableIndexes(sbSpring->nameVec2,
            sbSpring->indexSS, sbSpring->indexVec2);
        initializeDiscreteVariableIndexes(sbSpring->nameVec3,
            sbSpring->indexSS, sbSpring->indexVec3);
        initializeDiscreteVariableIndexes(sbSpring->nameVec4,
            sbSpring->indexSS, sbSpring->indexVec4);
        initializeDiscreteVariableIndexes(sbSpring->nameVec5,
            sbSpring->indexSS, sbSpring->indexVec5);
        initializeDiscreteVariableIndexes(sbSpring->nameVec6,
            sbSpring->indexSS, sbSpring->indexVec6);
    }

private:
    void setNull() { }
    void constructProperties() {
        const SimTK::Vec3 bodyOrigin(0.0, 0.0, 0.0);
        constructProperty_point1(bodyOrigin);
        constructProperty_point2(bodyOrigin);

        constructProperty_stiffness(1.0);
        constructProperty_rest_length(0.0);
    };

}; // End of class ExtendedPointToPointSpring


//-----------------------------------------------------------------------------
// Other Local Static Methods
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
// Sample internal method
double
customSquare(double x)
{
    return(x*x);
}

//_____________________________________________________________________________
/**
Compute the maximum error that can result from rounding a value at a
specified precision. This method assumes a base-10 representation of the value.
@param value Value to be rounded.
@param precision Number of significant figures that will be retained in the
value.
@return Maximum rounding error.
*/
double
computeMaxRoundingError(double value, int precision) {
    if (value == 0) return 0.0;
    int p = clamp(1, precision, SimTK::LosslessNumDigitsReal);
    double leastSigDigit = trunc(log10(fabs(value))-precision);
    double max_eps = 0.5*pow(10.0, leastSigDigit);
    if(max_eps < SimTK::LeastPositiveReal) return SimTK::LeastPositiveReal;
    return max_eps;
}

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
    for (int i = 0; i < dvA.size(); ++i) {
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
    for (int i = 0; i < a.size(); ++i) {
        for (int j = 0; j < a[i].size(); ++j) {
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
            for (int j = 0; j < dvA.size(); ++j) CHECK(dvB[j] == dvA[j]);
        }
        else if (SimTK::Value<int>::isA(abstractVal)) {
            Array_<int> dvA, dvB;
            model.getDiscreteVariableTrajectory<int>(paths[i], trajA, dvA);
            model.getDiscreteVariableTrajectory<int>(paths[i], trajB, dvB);
            for (int j = 0; j < dvA.size(); ++j) CHECK(dvB[j] == dvA[j]);
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
    double tol;

    // Loop over the named variables
    OpenSim::Array<std::string> paths = model.getModelingOptionNames();
    int nPaths = paths.getSize();
    for (int i = 0; i < nPaths; ++i) {
        Array_<int> moA, moB;
        model.getModelingOptionTrajectory<int>(paths[i], trajA, moA);
        model.getModelingOptionTrajectory<int>(paths[i], trajB, moB);
        for (int j = 0; j < moA.size(); ++j) CHECK(moB[j] == moA[j]);
    }
}

//_____________________________________________________________________________
// Test for equality of two state trajectories.
// If a state variable fails an equality test, it is likely that that
// variable has not been added to OpenSim's Component heirarchy and therefore
// has not been serialized.
void
testEquality(const Model& model,
    const Array_<State>& trajA, const Array_<State>& trajB, int precision)
{
    testEqualityForContinuousVariables(model, trajA, trajB, precision);
    testEqualityForDiscreteVariables(model, trajA, trajB, precision);
    testEqualityForModelingOptions(model, trajA, trajB, precision);
}

//_____________________________________________________________________________
// Build the model
Model*
buildModel() {

    // Create an empty model
    Model* model = new Model();
    Vec3 gravity(0.0, -10.0, 0.0);
    model->setGravity(gravity);
    model->setName("BlockOnASpring");

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
        ground, origin, *block, insertion, kp, restlength);
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

    // Return a copy of the state trajectory
    return reporter->getStates().getUnderlyingStateArray();
}

} // End anonymous namespace


TEST_CASE("Getting Started")
{
    double x = 2.0;
    double square = customSquare(x);
    REQUIRE(square == x*x);
}


TEST_CASE("Serialization and Deserialization")
{
    // Build the model and run a simulation
    // The output of simulate() is the state trajectory.
    // Note that a copy of the state trajectory is returned, so we don't have
    // to worry about the reporter (or any other object) going out of scope
    // or being deleted.
    Model *model = buildModel();
    Array_<State> trajA = simulate(model);

    // Serialize (A)
    int precision = 6;
    SimTK::String filename = "BlockOnASpring.ostates";
    StatesDocument docA(*model, trajA, precision);
    docA.serialize(filename);

    // Deserialize (B)
    StatesDocument docB(filename);
    Array_<State> trajB;
    docB.deserialize(*model, trajB);

    // Check size
    REQUIRE(trajA.size() == trajB.size());

    // Realize both state trajectories to Stage::Report
    for (int i = 0; i < trajA.size(); ++i) {
        model->getSystem().realize(trajA[i], Stage::Report);
        model->getSystem().realize(trajB[i], Stage::Report);
    }

    // Does A == B?
    testEquality(*model, trajA, trajB, precision);

    REQUIRE(1 == 1);

    delete model;
}
