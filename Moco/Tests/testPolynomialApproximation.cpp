/* -------------------------------------------------------------------------- *
 * OpenSim Moco: testPolynomialApproximation.cpp                              *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017-19 Stanford University and the Authors                  *
 *                                                                            *
 * Author(s): Antoine Falisse                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#define CATCH_CONFIG_MAIN
#include "Testing.h"

#include <Moco/osimMoco.h>

using namespace OpenSim;

Model createGait2D() {

    ///////////////////////////////////////////////////////////////////////////
    // Create model
    ///////////////////////////////////////////////////////////////////////////
    Model model;
    model.setName("gait_2D");
    using SimTK::Vec3;
    using SimTK::Inertia;

    ///////////////////////////////////////////////////////////////////////////
    // Add bodies
    ///////////////////////////////////////////////////////////////////////////
    auto* pelvis = new OpenSim::Body("pelvis", 9.7143336091724,
        Vec3(-0.0682778,0,0), Inertia(0.0814928846050306,0.0814928846050306,
        0.0445427591530667,0,0,0));
    model.addBody(pelvis);
    auto* femur_l = new OpenSim::Body("femur_l", 7.67231915023828,
        Vec3(0,-0.170467,0), Inertia(0.111055472890139,0.0291116288158616,
        0.117110028170931,0,0,0));
    model.addBody(femur_l);
    auto* femur_r = new OpenSim::Body("femur_r", 7.67231915023828,
        Vec3(0,-0.170467,0), Inertia(0.111055472890139,0.0291116288158616,
        0.117110028170931,0,0,0));
    model.addBody(femur_r);
    auto* tibia_l = new OpenSim::Body("tibia_l", 3.05815503574821,
        Vec3(0,-0.180489,0), Inertia(0.0388526996597354,0.00393152317985418,
        0.0393923204883429,0,0,0));
    model.addBody(tibia_l);
    auto* tibia_r = new OpenSim::Body("tibia_r", 3.05815503574821,
        Vec3(0,-0.180489,0), Inertia(0.0388526996597354,0.00393152317985418,
        0.0393923204883429,0,0,0));
    model.addBody(tibia_r);
    auto* talus_l = new OpenSim::Body("talus_l", 0.082485638186061,
        Vec3(0), Inertia(0.000688967700910182,0.000688967700910182,
        0.000688967700910182,0,0,0));
    model.addBody(talus_l);
    auto* talus_r = new OpenSim::Body("talus_r", 0.082485638186061,
        Vec3(0), Inertia(0.000688967700910182,0.000688967700910182,
        0.000688967700910182,0,0,0));
    model.addBody(talus_r);
    auto* calcn_l = new OpenSim::Body("calcn_l", 1.03107047732576,
        Vec3(0.0913924,0.0274177,0), Inertia(0.000964554781274254,
        0.00268697403354971,0.00282476757373175,0,0,0));
    model.addBody(calcn_l);
    auto* calcn_r = new OpenSim::Body("calcn_r", 1.03107047732576,
        Vec3(0.0913924,0.0274177,0), Inertia(0.000964554781274254,
        0.00268697403354971,0.00282476757373175,0,0,0));
    model.addBody(calcn_r);
    auto* toes_l = new OpenSim::Body("toes_l", 0.178663892311008,
        Vec3(0.0316218,0.00548355,0.0159937), Inertia(6.88967700910182e-005,
        0.000137793540182036,6.88967700910182e-005,0,0,0));
    model.addBody(toes_l);
    auto* toes_r = new OpenSim::Body("toes_r", 0.178663892311008,
        Vec3(0.0316218,0.00548355,-0.0159937), Inertia(6.88967700910182e-005,
        0.000137793540182036,6.88967700910182e-005,0,0,0));
    model.addBody(toes_r);
    auto* torso = new OpenSim::Body("torso", 28.240278003209,
        Vec3(-0.0289722,0.309037,0), Inertia(1.14043571182129,
        0.593400919285897,1.14043571182129,0,0,0));
    model.addBody(torso);

    ///////////////////////////////////////////////////////////////////////////
    // Add joints
    ///////////////////////////////////////////////////////////////////////////
    // Ground pelvis
    auto* groundPelvis = new PlanarJoint("groundPelvis", model.getGround(),
        Vec3(0), Vec3(0), *pelvis, Vec3(0), Vec3(0));
    auto& groundPelvis_q_rz =
        groundPelvis->updCoordinate(PlanarJoint::Coord::RotationZ);
    groundPelvis_q_rz.setName("groundPelvis_q_rz");
    auto& groundPelvis_q_tx =
        groundPelvis->updCoordinate(PlanarJoint::Coord::TranslationX);
    groundPelvis_q_tx.setName("groundPelvis_q_tx");
    auto& groundPelvis_q_ty =
        groundPelvis->updCoordinate(PlanarJoint::Coord::TranslationY);
    groundPelvis_q_ty.setName("groundPelvis_q_ty");
    model.addJoint(groundPelvis);

    // Hip left
    auto* hip_l = new PinJoint("hip_l", *pelvis, Vec3(-0.0682778001711179,
        -0.0638353973311301,-0.0823306940058688), Vec3(0), *femur_l, Vec3(0),
        Vec3(0));
    auto& hip_q_l = hip_l->updCoordinate();
    hip_q_l.setName("hip_q_l");
    model.addJoint(hip_l);

    // Hip right
    auto* hip_r = new PinJoint("hip_r", *pelvis, Vec3(-0.0682778001711179,
        -0.0638353973311301, 0.0823306940058688), Vec3(0), *femur_r, Vec3(0),
        Vec3(0));
    auto& hip_q_r = hip_r->updCoordinate();
    hip_q_r.setName("hip_q_r");
    model.addJoint(hip_r);

    // Knee left
    auto* knee_l = new PinJoint("knee_l", *femur_l, Vec3(-0.00451221232146798,
        -0.396907245921447, 0), Vec3(0), *tibia_l, Vec3(0), Vec3(0));
    auto& knee_q_l = knee_l->updCoordinate();
    knee_q_l.setName("knee_q_l");
    model.addJoint(knee_l);

    // Knee right
    auto* knee_r = new PinJoint("knee_r", *femur_r, Vec3(-0.00451221232146798,
        -0.396907245921447, 0), Vec3(0), *tibia_r, Vec3(0), Vec3(0));
    auto& knee_q_r = knee_r->updCoordinate();
    knee_q_r.setName("knee_q_r");
    model.addJoint(knee_r);

    // Ankle left
    auto* ankle_l = new PinJoint("ankle_l", *tibia_l,
        Vec3(0, -0.415694825374905, 0), Vec3(0), *talus_l, Vec3(0), Vec3(0));
    auto& ankle_q_l = ankle_l->updCoordinate();
    ankle_q_l.setName("ankle_q_l");
    model.addJoint(ankle_l);
    // Ankle right
    auto* ankle_r = new PinJoint("ankle_r", *tibia_r,
        Vec3(0, -0.415694825374905, 0), Vec3(0), *talus_r, Vec3(0), Vec3(0));
    auto& ankle_q_r = ankle_r->updCoordinate();
    ankle_q_r.setName("ankle_q_r");
    model.addJoint(ankle_r);
    // Subtalar left
    auto* subtalar_l = new WeldJoint("subtalar_l", *talus_l,
        Vec3(-0.0445720919117321, -0.0383391276542374, -0.00723828107321956),
        Vec3(0), *calcn_l, Vec3(0), Vec3(0));
    model.addJoint(subtalar_l);
    // Subtalar right
    auto* subtalar_r = new WeldJoint("subtalar_r", *talus_r,
        Vec3(-0.0445720919117321, -0.0383391276542374, 0.00723828107321956),
        Vec3(0), *calcn_r, Vec3(0), Vec3(0));
    model.addJoint(subtalar_r);
    // MTP left
    auto* mtp_l = new WeldJoint("mtp_l", *calcn_l,
        Vec3(0.163409678774199, -0.00182784875586352, -0.000987038328166303),
        Vec3(0), *toes_l, Vec3(0), Vec3(0));
    model.addJoint(mtp_l);
     // MTP right
    auto* mtp_r = new WeldJoint("mtp_r", *calcn_r,
        Vec3(0.163409678774199, -0.00182784875586352, 0.000987038328166303),
        Vec3(0), *toes_r, Vec3(0), Vec3(0));
    model.addJoint(mtp_r);
    // Lumbar
    auto* lumbar = new PinJoint("lumbar", *pelvis,
        Vec3(-0.0972499926058214, 0.0787077894476112, 0), Vec3(0),
        *torso, Vec3(0), Vec3(0));
    auto& lumbar_q = lumbar->updCoordinate();
    lumbar_q.setName("lumbar_q");
    model.addJoint(lumbar);

    ///////////////////////////////////////////////////////////////////////////
    // Add muscles
    // Coefficients for polynomial approximations computed with MATLAB code.
    ///////////////////////////////////////////////////////////////////////////

    int dimHamstrings = 2;
    int orderHamstrings = 3;
    const int nCoeffHamstrings = 10;
    double coeffHamstrings[nCoeffHamstrings] = {0.420495248805280,
        0.028755254940480, 0.018300167519638, 0.006995443639179,
        0.053690588126276, 0.002092858497112, 0.001164874424056,
        0.016799961996946, 1.886801073680048e-04, 0.009576337870147};

    int dimBFSH = 1;
    int orderBFSH = 3;
    const int nCoeffBFSH = 4;
    double coeffBFSH[nCoeffBFSH] = {0.244916441634551, 0.021846865400467,
        -0.020950072312087, -0.006400711591235};

    int dimGLU = 1;
    int orderGLU = 4;
    const int nCoeffGLU = 5;
    double coeffGLU[nCoeffGLU] = {0.201384070841096, 0.056529803505131,
        -0.001007247819865, -0.013044763452684, 0.004357279476748};

    int dimIlioPsoas = 1;
    int orderIlioPsoas = 3;
    const int nCoeffIlioPsoas = 4;
    double coeffIlioPsoas[nCoeffIlioPsoas] = {0.254156701045543,
        -0.038230025226882, 0.005210532284289, 0.002534168005785};

    int dimRF = 2;
    int orderRF = 5;
    const int nCoeffRF = 21;
    double coeffRF[nCoeffRF] = {0.411041216316924, -0.041901285727185,
        0.015455075788113, 0.035343831737396, 0.019052568915166,
        0.003490516739746, -0.041198556780086, -0.002495169893594,
        -0.001467645187366, -8.011395917540590e-04, -2.062908241814378e-04,
        -0.014671630141187, 0.001472708873289, 4.702571981803344e-04,
        2.278861920386511e-05, 0.005544930296365, 2.921882702941324e-04,
        6.068963635841811e-05, 0.001214860648500, -4.506103418022895e-05,
        -1.110736612700566e-04};

    int dimVAS = 1;
    int orderVAS = 3;
    const int nCoeffVAS = 4;
    double coeffVAS[nCoeffVAS] = {0.184215306369091, -0.040084488603560,
        -0.006524542229607, 3.197777963362615e-04};

    int dimGAS = 2;
    int orderGAS = 3;
    const int nCoeffGAS = 10;
    double coeffGAS[nCoeffGAS] = {0.434930944648804, 0.043988148379275,
        -0.005155011706555, -0.006729667224386, 0.025377138303286,
        3.450897747676789e-04, 7.482682035176111e-04, 0.007819367321315,
        -3.278790906939024e-05, 6.958181391218760e-05};

    int dimSOL = 1;
    int orderSOL = 3;
    const int nCoeffSOL = 4;
    double coeffSOL[nCoeffSOL] = {0.280659364908914, 0.042682272678384,
        -0.007031377756060, -0.006261791606827};

    int dimTA = 1;
    int orderTA = 3;
    const int nCoeffTA = 4;
    double coeffTA[nCoeffTA] = {0.290611643802287, -0.043920949896278,
        -0.004349864686700, 0.003846471985262};

    auto* hamstrings_f = new MultivariatePolynomialFunction();
    hamstrings_f->setDimension(dimHamstrings);
    hamstrings_f->setOrder(orderHamstrings);
    hamstrings_f->setCoefficients(
            SimTK::Vector(nCoeffHamstrings,coeffHamstrings));

    auto* BFSH_f = new MultivariatePolynomialFunction();
    BFSH_f->setDimension(dimBFSH);
    BFSH_f->setOrder(orderBFSH);
    BFSH_f->setCoefficients(
            SimTK::Vector(nCoeffBFSH,coeffBFSH));

    auto* GLU_f = new MultivariatePolynomialFunction();
    GLU_f->setDimension(dimGLU);
    GLU_f->setOrder(orderGLU);
    GLU_f->setCoefficients(
            SimTK::Vector(nCoeffGLU,coeffGLU));

    auto* IlioPsoas_f = new MultivariatePolynomialFunction();
    IlioPsoas_f->setDimension(dimIlioPsoas);
    IlioPsoas_f->setOrder(orderIlioPsoas);
    IlioPsoas_f->setCoefficients(
            SimTK::Vector(nCoeffIlioPsoas,coeffIlioPsoas));

    auto* RF_f = new MultivariatePolynomialFunction();
    RF_f->setDimension(dimRF);
    RF_f->setOrder(orderRF);
    RF_f->setCoefficients(
            SimTK::Vector(nCoeffRF,coeffRF));

    auto* VAS_f = new MultivariatePolynomialFunction();
    VAS_f->setDimension(dimVAS);
    VAS_f->setOrder(orderVAS);
    VAS_f->setCoefficients(
            SimTK::Vector(nCoeffVAS,coeffVAS));

    auto* GAS_f = new MultivariatePolynomialFunction();
    GAS_f->setDimension(dimGAS);
    GAS_f->setOrder(orderGAS);
    GAS_f->setCoefficients(
            SimTK::Vector(nCoeffGAS,coeffGAS));

    auto* SOL_f = new MultivariatePolynomialFunction();
    SOL_f->setDimension(dimSOL);
    SOL_f->setOrder(orderSOL);
    SOL_f->setCoefficients(
            SimTK::Vector(nCoeffSOL,coeffSOL));

    auto* TA_f = new MultivariatePolynomialFunction();
    TA_f->setDimension(dimTA);
    TA_f->setOrder(orderTA);
    TA_f->setCoefficients(
            SimTK::Vector(nCoeffTA,coeffTA));

    PolynomialActuators hamstrings_p_r;
    hamstrings_p_r.set_function(*hamstrings_f);
    hamstrings_p_r.append_coordinate_list("/jointset/hip_r/hip_q_r");
    hamstrings_p_r.append_coordinate_list("/jointset/knee_r/knee_q_r");
    auto* hamstrings_r = new DeGrooteFregly2016Muscle();
    hamstrings_r->set_GeometryPath(hamstrings_p_r);
    hamstrings_r->set_ignore_tendon_compliance(true);
    hamstrings_r->setName("hamstrings_r");
    model.addComponent(hamstrings_r);

    PolynomialActuators BFSH_p_r;
    BFSH_p_r.set_function(*BFSH_f);
    BFSH_p_r.append_coordinate_list("/jointset/knee_r/knee_q_r");
    auto* BFSH_r = new DeGrooteFregly2016Muscle();
    BFSH_r->set_GeometryPath(BFSH_p_r);
    BFSH_r->set_ignore_tendon_compliance(true);
    BFSH_r->setName("BFSH_r");
    model.addComponent(BFSH_r);

    PolynomialActuators GLU_p_r;
    GLU_p_r.set_function(*GLU_f);
    GLU_p_r.append_coordinate_list("/jointset/hip_r/hip_q_r");
    auto* GLU_r = new DeGrooteFregly2016Muscle();
    GLU_r->set_GeometryPath(GLU_p_r);
    GLU_r->set_ignore_tendon_compliance(true);
    GLU_r->setName("GLU_r");
    model.addComponent(GLU_r);

    PolynomialActuators IlioPsoas_p_r;
    IlioPsoas_p_r.set_function(*IlioPsoas_f);
    IlioPsoas_p_r.append_coordinate_list("/jointset/hip_r/hip_q_r");
    auto* IlioPsoas_r = new DeGrooteFregly2016Muscle();
    IlioPsoas_r->set_GeometryPath(IlioPsoas_p_r);
    IlioPsoas_r->set_ignore_tendon_compliance(true);
    IlioPsoas_r->setName("IlioPsoas_r");
    model.addComponent(IlioPsoas_r);

    PolynomialActuators RF_p_r;
    RF_p_r.set_function(*RF_f);
    RF_p_r.append_coordinate_list("/jointset/hip_r/hip_q_r");
    RF_p_r.append_coordinate_list("/jointset/knee_r/knee_q_r");
    auto* RF_r = new DeGrooteFregly2016Muscle();
    RF_r->set_GeometryPath(RF_p_r);
    RF_r->set_ignore_tendon_compliance(true);
    RF_r->setName("RF_r");
    model.addComponent(RF_r);

    PolynomialActuators VAS_p_r;
    VAS_p_r.set_function(*VAS_f);
    VAS_p_r.append_coordinate_list("/jointset/knee_r/knee_q_r");
    auto* VAS_r = new DeGrooteFregly2016Muscle();
    VAS_r->set_GeometryPath(VAS_p_r);
    VAS_r->set_ignore_tendon_compliance(true);
    VAS_r->setName("VAS_r");
    model.addComponent(VAS_r);

    PolynomialActuators GAS_p_r;
    GAS_p_r.set_function(*GAS_f);
    GAS_p_r.append_coordinate_list("/jointset/knee_r/knee_q_r");
    GAS_p_r.append_coordinate_list("/jointset/ankle_r/ankle_q_r");
    auto* GAS_r = new DeGrooteFregly2016Muscle();
    GAS_r->set_GeometryPath(GAS_p_r);
    GAS_r->set_ignore_tendon_compliance(true);
    GAS_r->setName("GAS_r");
    model.addComponent(GAS_r);

    PolynomialActuators SOL_p_r;
    SOL_p_r.set_function(*SOL_f);
    SOL_p_r.append_coordinate_list("/jointset/ankle_r/ankle_q_r");
    auto* SOL_r = new DeGrooteFregly2016Muscle();
    SOL_r->set_GeometryPath(SOL_p_r);
    SOL_r->set_ignore_tendon_compliance(true);
    SOL_r->setName("SOL_r");
    model.addComponent(SOL_r);

    PolynomialActuators TA_p_r;
    TA_p_r.set_function(*TA_f);
    TA_p_r.append_coordinate_list("/jointset/ankle_r/ankle_q_r");
    auto* TA_r = new DeGrooteFregly2016Muscle();
    TA_r->set_GeometryPath(TA_p_r);
    TA_r->set_ignore_tendon_compliance(true);
    TA_r->setName("TA_r");
    model.addComponent(TA_r);

    model.finalizeConnections();

    return model;
}

// Test comparing muscle-tendon lengths and lengthening speeds obtained with
// original MATLAB code and current implementation.
void testPolynomialApproximation() {

    Model model(createGait2D());
    SimTK::State& state = model.initSystem();

    int nStates = model.getNumStateVariables();
    SimTK::Vector stateValues(nStates);
    stateValues.setTo(0);

    // Set values for speeds to have non-null muscle-tendon lengthening speeds
    stateValues[18] = -1;
    stateValues[22] = 1;
    stateValues[26] = 0.5;

    model.setStateVariableValues(state, stateValues);
    model.realizeVelocity(state);

    double hamstrings_lmt_r = model.getComponent<PathActuator>(
                "hamstrings_r").getGeometryPath().getLength(state);
    double hamstrings_vmt_r = model.getComponent<PathActuator>(
                "hamstrings_r").getGeometryPath().getLengtheningSpeed(state);
    CHECK(0.420495248805280 == hamstrings_lmt_r);
    CHECK(-0.024935333185797 == Approx(hamstrings_vmt_r).margin(1e-10));

    double BFSH_lmt_r = model.getComponent<PathActuator>(
                "BFSH_r").getGeometryPath().getLength(state);
    double BFSH_vmt_r = model.getComponent<PathActuator>(
                "BFSH_r").getGeometryPath().getLengtheningSpeed(state);
    CHECK(0.244916441634551 == BFSH_lmt_r);
    CHECK(0.021846865400467 == BFSH_vmt_r);

    double GLU_lmt_r = model.getComponent<PathActuator>(
                "GLU_r").getGeometryPath().getLength(state);
    double GLU_vmt_r = model.getComponent<PathActuator>(
                "GLU_r").getGeometryPath().getLengtheningSpeed(state);
    CHECK(0.201384070841096 == GLU_lmt_r);
    CHECK(-0.056529803505131 == GLU_vmt_r);

    double IlioPsoas_lmt_r = model.getComponent<PathActuator>(
                "IlioPsoas_r").getGeometryPath().getLength(state);
    double IlioPsoas_vmt_r = model.getComponent<PathActuator>(
                "IlioPsoas_r").getGeometryPath().getLengtheningSpeed(state);
    CHECK(0.254156701045543 == IlioPsoas_lmt_r);
    CHECK(0.038230025226882 == IlioPsoas_vmt_r);

    double RF_lmt_r = model.getComponent<PathActuator>(
                "RF_r").getGeometryPath().getLength(state);
    double RF_vmt_r = model.getComponent<PathActuator>(
                "RF_r").getGeometryPath().getLengtheningSpeed(state);
    CHECK(0.411041216316924 == RF_lmt_r);
    CHECK(-7.027289470986353e-04 == Approx(RF_vmt_r).margin(1e-10));

    double VAS_lmt_r = model.getComponent<PathActuator>(
                "VAS_r").getGeometryPath().getLength(state);
    double VAS_vmt_r = model.getComponent<PathActuator>(
                "VAS_r").getGeometryPath().getLengtheningSpeed(state);
    CHECK(0.184215306369091 == VAS_lmt_r);
    CHECK(-0.040084488603560 == VAS_vmt_r);

    double GAS_lmt_r = model.getComponent<PathActuator>(
                "GAS_r").getGeometryPath().getLength(state);
    double GAS_vmt_r = model.getComponent<PathActuator>(
                "GAS_r").getGeometryPath().getLengtheningSpeed(state);
    CHECK(0.434930944648804 == GAS_lmt_r);
    CHECK(0.047371212492924 == Approx(GAS_vmt_r).margin(1e-10));

    double SOL_lmt_r = model.getComponent<PathActuator>(
                "SOL_r").getGeometryPath().getLength(state);
    double SOL_vmt_r = model.getComponent<PathActuator>(
                "SOL_r").getGeometryPath().getLengtheningSpeed(state);
    CHECK(0.280659364908914 == SOL_lmt_r);
    CHECK(0.021341136339192 == SOL_vmt_r);

    double TA_lmt_r = model.getComponent<PathActuator>(
                "TA_r").getGeometryPath().getLength(state);
    double TA_vmt_r = model.getComponent<PathActuator>(
                "TA_r").getGeometryPath().getLengtheningSpeed(state);
    CHECK(0.290611643802287 == TA_lmt_r);
    CHECK(-0.021960474948139 == TA_vmt_r);

}

TEST_CASE("testPolynomialApproximation") {
    testPolynomialApproximation();
}
