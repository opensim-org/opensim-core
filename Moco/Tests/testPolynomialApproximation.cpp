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

Model createGait2D_PolynomialPath() {

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
    auto* ground_pelvis = new PlanarJoint("ground_pelvis", model.getGround(),
        Vec3(0), Vec3(0), *pelvis, Vec3(0), Vec3(0));
    auto& pelvis_tilt =
        ground_pelvis->updCoordinate(PlanarJoint::Coord::RotationZ);
    pelvis_tilt.setRangeMin(-1.570796327);
    pelvis_tilt.setRangeMax(1.570796327);
    pelvis_tilt.setName("pelvis_tilt");
    auto& pelvis_tx =
        ground_pelvis->updCoordinate(PlanarJoint::Coord::TranslationX);
    pelvis_tx.setRangeMin(-5);
    pelvis_tx.setRangeMax(5);
    pelvis_tx.setName("pelvis_tx");
    auto& pelvis_ty =
        ground_pelvis->updCoordinate(PlanarJoint::Coord::TranslationY);
    pelvis_ty.setRangeMin(-1);
    pelvis_ty.setRangeMax(2);
    pelvis_ty.setName("pelvis_ty");
    model.addJoint(ground_pelvis);

    // Hip left
    auto* hip_l = new PinJoint("hip_l", *pelvis, Vec3(-0.0682778001711179,
        -0.0638353973311301,-0.0823306940058688), Vec3(0), *femur_l, Vec3(0),
        Vec3(0));
    auto& hip_flexion_l = hip_l->updCoordinate();
    hip_flexion_l.setRangeMin(-1.221730476);
    hip_flexion_l.setRangeMax(1.570796327);
    hip_flexion_l.setName("hip_flexion_l");
    model.addJoint(hip_l);

    // Hip right
    auto* hip_r = new PinJoint("hip_r", *pelvis, Vec3(-0.0682778001711179,
        -0.0638353973311301, 0.0823306940058688), Vec3(0), *femur_r, Vec3(0),
        Vec3(0));
    auto& hip_flexion_r = hip_r->updCoordinate();
    hip_flexion_r.setRangeMin(-1.221730476);
    hip_flexion_r.setRangeMax(1.570796327);
    hip_flexion_r.setName("hip_flexion_r");
    model.addJoint(hip_r);

    // Knee left
    auto* knee_l = new PinJoint("knee_l", *femur_l, Vec3(-0.00451221232146798,
        -0.396907245921447, 0), Vec3(0), *tibia_l, Vec3(0), Vec3(0));
    auto& knee_angle_l = knee_l->updCoordinate();
    knee_angle_l.setRangeMin(-2.094395102);
    knee_angle_l.setRangeMax(0.174532925);
    knee_angle_l.setName("knee_angle_l");
    model.addJoint(knee_l);

    // Knee right
    auto* knee_r = new PinJoint("knee_r", *femur_r, Vec3(-0.00451221232146798,
        -0.396907245921447, 0), Vec3(0), *tibia_r, Vec3(0), Vec3(0));
    auto& knee_angle_r = knee_r->updCoordinate();
    knee_angle_r.setRangeMin(-2.094395102);
    knee_angle_r.setRangeMax(0.174532925);
    knee_angle_r.setName("knee_angle_r");
    model.addJoint(knee_r);

    // Ankle left
    auto* ankle_l = new PinJoint("ankle_l", *tibia_l,
        Vec3(0, -0.415694825374905, 0), Vec3(0), *talus_l, Vec3(0), Vec3(0));
    auto& ankle_angle_l = ankle_l->updCoordinate();
    ankle_angle_l.setRangeMin(-0.872664626);
    ankle_angle_l.setRangeMax(0.698131701);
    ankle_angle_l.setName("ankle_angle_l");
    model.addJoint(ankle_l);
    // Ankle right
    auto* ankle_r = new PinJoint("ankle_r", *tibia_r,
        Vec3(0, -0.415694825374905, 0), Vec3(0), *talus_r, Vec3(0), Vec3(0));
    auto& ankle_angle_r = ankle_r->updCoordinate();
    ankle_angle_r.setRangeMin(-0.872664626);
    ankle_angle_r.setRangeMax(0.698131701);
    ankle_angle_r.setName("ankle_angle_r");
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
    auto& lumbar_extension = lumbar->updCoordinate();
    lumbar_extension.setName("lumbar_extension");
    model.addJoint(lumbar);

    ///////////////////////////////////////////////////////////////////////////
    // Add muscles
    // Coefficients for polynomial approximations computed with MATLAB code.
    ///////////////////////////////////////////////////////////////////////////

    int dimHamstrings = 2;
    int orderHamstrings = 3;
    const int nCoeffHamstrings = 10;
    double coeffHamstrings[nCoeffHamstrings] = {0.420495248805280,
        0.028755254940480, -0.018300167519638, -0.006995443639179,
        0.053690588126276, 0.002092858497112, 0.001164874424056,
        0.016799961996946, 1.886801073680048e-04, -0.009576337870147};

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
        -0.038230025226882, -0.005210532284289, 0.002534168005785};

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

    auto* hamstrings_r = new DeGrooteFregly2016Muscle();
    auto& hamstrings_p_r = hamstrings_r->updGeometryPath();
    hamstrings_p_r.set_use_approximation(true);
    hamstrings_p_r.append_length_approximation(*hamstrings_f);
    hamstrings_p_r.append_approximation_coordinates("/jointset/hip_r/hip_flexion_r");
    hamstrings_p_r.append_approximation_coordinates("/jointset/knee_r/knee_angle_r");
    hamstrings_r->set_ignore_tendon_compliance(true);
    hamstrings_r->addNewPathPoint("hamstrings_r-P1", *pelvis, Vec3(-0.121645, -0.0990559, 0.0684676));
    hamstrings_r->addNewPathPoint("hamstrings_r-P2", *tibia_r, Vec3(-0.0290986, -0.0348024, 0.0284509));
    hamstrings_r->addNewPathPoint("hamstrings_r-P3", *tibia_r, Vec3(-0.0226215, -0.054427, 0.0331589));
    hamstrings_r->setName("hamstrings_r");
    model.addComponent(hamstrings_r);

    auto* BFSH_r = new DeGrooteFregly2016Muscle();
    auto& BFSH_p_r = BFSH_r->updGeometryPath();
    BFSH_p_r.set_use_approximation(true);
    BFSH_p_r.append_length_approximation(*BFSH_f);
    BFSH_p_r.append_approximation_coordinates("/jointset/knee_r/knee_angle_r");
    BFSH_r->set_ignore_tendon_compliance(true);
    BFSH_r->addNewPathPoint("bifemsh_r-P1", *femur_r, Vec3(0.00501373, -0.211679, 0.0234642));
    BFSH_r->addNewPathPoint("bifemsh_r-P2", *tibia_r, Vec3(-0.0290986, -0.0348024, 0.0284509));
    BFSH_r->addNewPathPoint("bifemsh_r-P3", *tibia_r, Vec3(-0.0226215, -0.054427, 0.0331589));
    BFSH_r->setName("BFSH_r");
    model.addComponent(BFSH_r);

    auto* GLU_r = new DeGrooteFregly2016Muscle();
    auto& GLU_p_r = GLU_r->updGeometryPath();
    GLU_p_r.set_use_approximation(true);
    GLU_p_r.append_length_approximation(*GLU_f);
    GLU_p_r.append_approximation_coordinates("/jointset/hip_r/hip_flexion_r");
    GLU_r->set_ignore_tendon_compliance(true);
    GLU_r->addNewPathPoint("glut_max_r-P1", *pelvis, Vec3(-0.127188, 0.00840194, 0.045553));
    GLU_r->addNewPathPoint("glut_max_r-P2", *pelvis, Vec3(-0.129795, -0.0588136, 0.0801615));
    GLU_r->addNewPathPoint("glut_max_r-P3", *femur_r, Vec3(-0.0451235, -0.0585603, 0.0252692));
    GLU_r->addNewPathPoint("glut_max_r-P4", *femur_r, Vec3(-0.0156428, -0.101879, 0.042015));
    GLU_r->setName("GLU_r");
    model.addComponent(GLU_r);

    auto* IlioPsoas_r = new DeGrooteFregly2016Muscle();
    auto& IlioPsoas_p_r = IlioPsoas_r->updGeometryPath();
    IlioPsoas_p_r.set_use_approximation(true);
    IlioPsoas_p_r.append_length_approximation(*IlioPsoas_f);
    IlioPsoas_p_r.append_approximation_coordinates("/jointset/hip_r/hip_flexion_r");
    IlioPsoas_r->set_ignore_tendon_compliance(true);
    IlioPsoas_r->addNewPathPoint("IlioPsoas_r-P1", *pelvis, Vec3(-0.0624834, 0.0856611, 0.0284953));
    IlioPsoas_r->addNewPathPoint("IlioPsoas_r-P2", *pelvis, Vec3(-0.0229846, -0.0550472, 0.0748371));
    ConditionalPathPoint* IlioPsoas_P3_r = new ConditionalPathPoint();
    IlioPsoas_P3_r->setName("IlioPsoas_r-P3");
    IlioPsoas_P3_r->setCoordinate(hip_flexion_r);
    IlioPsoas_P3_r->setBody(*pelvis);
    IlioPsoas_P3_r->setLocation(Vec3(-0.0278133, -0.077742, 0.0804573));
    IlioPsoas_P3_r->setRangeMin(-1.5708);
    IlioPsoas_P3_r->setRangeMax(0.785398);
    IlioPsoas_r->updGeometryPath().updPathPointSet().adoptAndAppend(IlioPsoas_P3_r);
    IlioPsoas_r->addNewPathPoint("IlioPsoas_r-P4", *femur_r, Vec3(0.00160439, -0.0508392, 0.00381043));
    IlioPsoas_r->addNewPathPoint("IlioPsoas_r-P5", *femur_r, Vec3(-0.0188516, -0.0598639, 0.0104285));
    IlioPsoas_r->setName("IlioPsoas_r");
    model.addComponent(IlioPsoas_r);

    auto* RF_r = new DeGrooteFregly2016Muscle();
    auto& RF_p_r = RF_r->updGeometryPath();
    RF_p_r.set_use_approximation(true);
    RF_p_r.append_length_approximation(*RF_f);
    RF_p_r.append_approximation_coordinates("/jointset/hip_r/hip_flexion_r");
    RF_p_r.append_approximation_coordinates("/jointset/knee_r/knee_angle_r");
    RF_r->set_ignore_tendon_compliance(true);
    RF_r->addNewPathPoint("RF_r-P1", *pelvis, Vec3(-0.0284893, -0.0300345, 0.0954444));
    ConditionalPathPoint* RF_P2_r = new ConditionalPathPoint();
    RF_P2_r->setName("RF_r-P2");
    RF_P2_r->setCoordinate(knee_angle_r);
    RF_P2_r->setBody(*femur_r);
    RF_P2_r->setLocation(Vec3(0.0334917, -0.404106, 0.00190522));
    RF_P2_r->setRangeMin(-2.61799);
    RF_P2_r->setRangeMax(-1.45997);
    RF_r->updGeometryPath().updPathPointSet().adoptAndAppend(RF_P2_r);
    OpenSim::MovingPathPoint* RF_P3_r = new MovingPathPoint();
    RF_P3_r->setName("RF_r-P3");
    RF_P3_r->setBody(*tibia_r);
    double scale = 0.966732152034662;
    int np_rfX_r = 17;
    double rfX_r_x[] = { -2.0944, -1.99997, -1.5708, -1.45752, -1.39626, -1.0472, -0.698132, -0.526391, -0.349066, -0.174533, 0, 0.00017453, 0.00034907, 0.0279253, 0.0872665, 0.174533, 2.0944 };
    double rfX_r_y[] = { 0.0155805, 0.0179938, 0.0275081, 0.0296564, 0.0307615, 0.0365695, 0.0422074, 0.0450902, 0.048391, 0.0534299, 0.0617576, 0.0617669, 0.0617762, 0.0633083, 0.066994, 0.0733035, 0.0573481 };
    SimTK::Vector rfX_r_y_s(np_rfX_r,&rfX_r_y[0]);
    rfX_r_y_s = scale * rfX_r_y_s;
    //rfX_r_y_s.scalarMultiplyInPlace(scale);
    SimmSpline SS_rf_r_x(np_rfX_r, rfX_r_x, &rfX_r_y_s[0]);
    RF_P3_r->set_x_location(SS_rf_r_x);
    RF_P3_r->setXCoordinate(knee_angle_r);
    int np_rfY_r = 17;
    double rfY_r_x[] = { -2.0944, -1.99997, -1.5708, -1.45752, -1.39626, -1.0472, -0.698132, -0.526391, -0.349066, -0.174533, 0, 0.00017453, 0.00034907, 0.0279253, 0.0872665, 0.174533, 2.0944 };
    double rfY_r_y[] = { 0.0234116, 0.0237613, 0.0251141, 0.0252795, 0.0253146, 0.0249184, 0.0242373, 0.0238447, 0.0234197, 0.0227644, 0.020984, 0.0209814, 0.0209788, 0.0205225, 0.0191754, 0.0159554, -0.0673774 };
    SimTK::Vector rfY_r_y_s(np_rfY_r,&rfY_r_y[0]);
    rfY_r_y_s = scale * rfY_r_y_s;
    //rfY_r_y_s.scalarMultiplyInPlace(scale);
    SimmSpline SS_rf_r_y(np_rfY_r, rfY_r_x, &rfY_r_y_s[0]);
    RF_P3_r->set_y_location(SS_rf_r_y);
    RF_P3_r->setYCoordinate(knee_angle_r);
    int np_rfZ_r = 2;
    double rfZ_r_x[] = { -2.0944, 0.1745 };
    double rfZ_r_y[] = { 0.0014, 0.0014 };
    SimTK::Vector rfZ_r_y_s(np_rfZ_r,&rfZ_r_y[0]);
    rfZ_r_y_s = scale * rfZ_r_y_s;
    //rfZ_r_y_s.scalarMultiplyInPlace(scale);
    SimmSpline SS_rf_r_z(np_rfZ_r, rfZ_r_x, &rfZ_r_y_s[0]);
    RF_P3_r->set_z_location(SS_rf_r_z);
    RF_P3_r->setZCoordinate(knee_angle_r);
    RF_r->updGeometryPath().updPathPointSet().adoptAndAppend(RF_P3_r);
    RF_r->setName("RF_r");
    model.addComponent(RF_r);

    auto* VAS_r = new DeGrooteFregly2016Muscle();
    auto& VAS_p_r = VAS_r->updGeometryPath();
    VAS_p_r.set_use_approximation(true);
    VAS_p_r.append_length_approximation(*VAS_f);
    VAS_p_r.append_approximation_coordinates("/jointset/knee_r/knee_angle_r");
    VAS_r->set_ignore_tendon_compliance(true);
    VAS_r->addNewPathPoint("VAS_r-P1", *femur_r, Vec3(0.0290796, -0.192928, 0.0310851));
    VAS_r->addNewPathPoint("VAS_r-P2", *femur_r, Vec3(0.033592, -0.208972, 0.0285782));
    ConditionalPathPoint* VAS_P3_r = new ConditionalPathPoint();
    VAS_P3_r->setName("VAS_r-P3");
    VAS_P3_r->setCoordinate(knee_angle_r);
    VAS_P3_r->setBody(*femur_r);
    VAS_P3_r->setLocation(Vec3(0.0343942, -0.404106, 0.0055151));
    VAS_P3_r->setRangeMin(-2.61799);
    VAS_P3_r->setRangeMax(-1.42);
    VAS_r->updGeometryPath().updPathPointSet().adoptAndAppend(VAS_P3_r);
    OpenSim::MovingPathPoint* VAS_P4_r = new MovingPathPoint();
    VAS_P4_r->setName("VAS_P4_r");
    VAS_P4_r->setBody(*tibia_r);
    int np_viX_r = 17;
    double viX_r_x[] = { -2.0944, -1.99997, -1.5708, -1.45752, -1.39626, -1.0472, -0.698132, -0.526391, -0.349066, -0.174533, 0, 0.00017453, 0.00034907, 0.0279253, 0.0872665, 0.174533, 2.0944 };
    double viX_r_y[] = { 0.0082733, 0.0106866, 0.0202042, 0.022353, 0.0234583, 0.0292715, 0.0349465, 0.037871, 0.0412569, 0.0465287, 0.0554632, 0.0554735, 0.0554837, 0.0571717, 0.061272, 0.0684368, 0.0648818 };
    SimTK::Vector viX_r_y_s(np_viX_r,&viX_r_y[0]);
    viX_r_y_s = scale * viX_r_y_s;
    //viX_r_y_s.scalarMultiplyInPlace(scale);
    SimmSpline SS_vi_r_x(np_viX_r, viX_r_x, &viX_r_y_s[0]);
    VAS_P4_r->set_x_location(SS_vi_r_x);
    VAS_P4_r->setXCoordinate(knee_angle_r);
    int np_viY_r = 17;
    double viY_r_x[] = { -2.0944, -1.99997, -1.5708, -1.45752, -1.39626, -1.0472, -0.698132, -0.526391, -0.349066, -0.174533, 0, 0.00017453, 0.00034907, 0.0279253, 0.0872665, 0.174533, 2.0944 };
    double viY_r_y[] = { 0.025599, 0.0259487, 0.0273124, 0.0274796, 0.0275151, 0.0271363, 0.0265737, 0.0263073, 0.0261187, 0.0260129, 0.0252923, 0.0252911, 0.0252898, 0.0250526, 0.0242191, 0.0218288, -0.0685706 };
    SimTK::Vector viY_r_y_s(np_viY_r,&viY_r_y[0]);
    viY_r_y_s = scale * viY_r_y_s;
    //viY_r_y_s.scalarMultiplyInPlace(scale);
    SimmSpline SS_vi_r_y(np_viY_r, viY_r_x, &viY_r_y_s[0]);
    VAS_P4_r->set_y_location(SS_vi_r_y);
    VAS_P4_r->setYCoordinate(knee_angle_r);
    int np_viZ_r = 2;
    double viZ_r_x[] = { -2.0944, 2.0944 };
    double viZ_r_y[] = { 0.0018, 0.0018 };
    SimTK::Vector viZ_r_y_s(np_viZ_r,&viZ_r_y[0]);
    viZ_r_y_s = scale * viZ_r_y_s;
    //viZ_r_y_s.scalarMultiplyInPlace(scale);
    SimmSpline SS_vi_r_z(np_viZ_r, viZ_r_x, &viZ_r_y_s[0]);
    VAS_P4_r->set_z_location(SS_vi_r_z);
    VAS_P4_r->setZCoordinate(knee_angle_r);
    VAS_r->updGeometryPath().updPathPointSet().adoptAndAppend(VAS_P4_r);
    VAS_r->setName("VAS_r");
    model.addComponent(VAS_r);

    auto* GAS_r = new DeGrooteFregly2016Muscle();
    auto& GAS_p_r = GAS_r->updGeometryPath();
    GAS_p_r.set_use_approximation(true);
    GAS_p_r.append_length_approximation(*GAS_f);
    GAS_p_r.append_approximation_coordinates("/jointset/knee_r/knee_angle_r");
    GAS_p_r.append_approximation_coordinates("/jointset/ankle_r/ankle_angle_r");
    GAS_r->set_ignore_tendon_compliance(true);
    GAS_r->addNewPathPoint("GAS_r-P1", *femur_r, Vec3(-0.0190522, -0.393979, -0.0235645));

    ConditionalPathPoint* GAS_P2_r = new ConditionalPathPoint();
    GAS_P2_r->setName("GAS_r-P2");
    GAS_P2_r->setCoordinate(knee_angle_r);
    GAS_P2_r->setBody(*femur_r);
    GAS_P2_r->setLocation(Vec3(-0.0300824, -0.403304, -0.0258708));
    GAS_P2_r->setRangeMin(-0.785398);
    GAS_P2_r->setRangeMax(0.174533);
    GAS_r->updGeometryPath().updPathPointSet().adoptAndAppend(GAS_P2_r);
    GAS_r->addNewPathPoint("GAS_r-P3", *calcn_r, Vec3(0, 0.0283317, -0.0048438));
    GAS_r->setName("GAS_r");
    model.addComponent(GAS_r);

    auto* SOL_r = new DeGrooteFregly2016Muscle();
    auto& SOL_p_r = SOL_r->updGeometryPath();
    SOL_p_r.set_use_approximation(true);
    SOL_p_r.append_length_approximation(*SOL_f);
    SOL_p_r.append_approximation_coordinates("/jointset/ankle_r/ankle_angle_r");
    SOL_r->set_ignore_tendon_compliance(true);
    SOL_r->addNewPathPoint("SOL_r-P1", *tibia_r, Vec3(-0.00232016, -0.1482, 0.0068638));
    SOL_r->addNewPathPoint("SOL_r-P2", *calcn_r, Vec3(0, 0.0283317, -0.0048438));
    SOL_r->setName("SOL_r");
    model.addComponent(SOL_r);

    auto* TA_r = new DeGrooteFregly2016Muscle();
    auto& TA_p_r = TA_r->updGeometryPath();
    TA_p_r.set_use_approximation(true);
    TA_p_r.append_length_approximation(*TA_f);
    TA_p_r.append_approximation_coordinates("/jointset/ankle_r/ankle_angle_r");
    TA_r->set_ignore_tendon_compliance(true);
    TA_r->addNewPathPoint("TA_r-P1", *tibia_r, Vec3(0.0173045, -0.156997, 0.0111174));
    TA_r->addNewPathPoint("TA_r-P2", *tibia_r, Vec3(0.0318055, -0.381956, -0.0171112));
    TA_r->addNewPathPoint("TA_r-P3", *calcn_r, Vec3(0.106564, 0.0162679, -0.0278747));
    TA_r->setName("TA_r");
    model.addComponent(TA_r);

    model.finalizeConnections();

    return model;
}

// Test comparing muscle-tendon lengths and lengthening speeds obtained with
// original MATLAB code and current implementation.
void testPolynomialApproximationImpl() {

    Model model(createGait2D_PolynomialPath());
    SimTK::State& state = model.initSystem();

    int nStates = model.getNumStateVariables();
    SimTK::Vector stateValues(nStates);
    stateValues.setTo(0);

    // Set values for speeds to have non-null muscle-tendon lengthening speeds

    stateValues[18] = -1; // hip_flexion_r speed
    stateValues[22] = 1; // knee_angle_r speed
    stateValues[26] = 0.5; // ankle_angle_r speed

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

void testPolynomialApproximation() {

    Model model(createGait2D_PolynomialPath());
    SimTK::State& state = model.initSystem();

    // Lengths
    auto& hamstringsPath_r = model.getComponent<PathActuator>(
            "hamstrings_r").getGeometryPath();
    double hamstrings_lengthError_r =
            hamstringsPath_r.computeApproximationErrorOnGrid(40, "length");
    auto& BFSHPath_r = model.getComponent<PathActuator>(
            "BFSH_r").getGeometryPath();
    double BFSH_lengthError_r =
            BFSHPath_r.computeApproximationErrorOnGrid(40, "length");
    auto& GLUPath_r = model.getComponent<PathActuator>(
            "GLU_r").getGeometryPath();
    double GLU_lengthError_r =
            GLUPath_r.computeApproximationErrorOnGrid(40, "length");
    auto& IlioPsoasPath_r = model.getComponent<PathActuator>(
            "IlioPsoas_r").getGeometryPath();
    double IlioPsoas_lengthError_r =
            IlioPsoasPath_r.computeApproximationErrorOnGrid(40, "length");
    auto& RFPath_r = model.getComponent<PathActuator>(
            "RF_r").getGeometryPath();
    double RF_lengthError_r =
            RFPath_r.computeApproximationErrorOnGrid(40, "length");
    auto& VASPath_r = model.getComponent<PathActuator>(
            "VAS_r").getGeometryPath();
    double VAS_lengthError_r =
            VASPath_r.computeApproximationErrorOnGrid(40, "length");
    auto& GASPath_r = model.getComponent<PathActuator>(
            "GAS_r").getGeometryPath();
    double GAS_lengthError_r =
            GASPath_r.computeApproximationErrorOnGrid(40, "length");
    auto& SOLPath_r = model.getComponent<PathActuator>(
            "SOL_r").getGeometryPath();
    double SOL_lengthError_r =
            SOLPath_r.computeApproximationErrorOnGrid(40, "length");
    auto& TAPath_r = model.getComponent<PathActuator>(
            "TA_r").getGeometryPath();
    double TA_lengthError_r =
            TAPath_r.computeApproximationErrorOnGrid(40, "length");

    // Lengthening speeds
    double hamstrings_lengtheningSpeedError_r =
            hamstringsPath_r.computeApproximationErrorOnGrid(40,
                    "lengthening_speed");
    double BFSH_lengtheningSpeedError_r =
            BFSHPath_r.computeApproximationErrorOnGrid(40,
                    "lengthening_speed");
    double GLU_lengtheningSpeedError_r =
            GLUPath_r.computeApproximationErrorOnGrid(40, "lengthening_speed");
    double IlioPsoas_lengtheningSpeedError_r =
            IlioPsoasPath_r.computeApproximationErrorOnGrid(40,
                    "lengthening_speed");
    double RF_lengtheningSpeedError_r =
            RFPath_r.computeApproximationErrorOnGrid(40, "lengthening_speed");
    double VAS_lengtheningSpeedError_r =
            VASPath_r.computeApproximationErrorOnGrid(40, "lengthening_speed");
    double GAS_lengtheningSpeedError_r =
            GASPath_r.computeApproximationErrorOnGrid(40, "lengthening_speed");
    double SOL_lengtheningSpeedError_r =
            SOLPath_r.computeApproximationErrorOnGrid(40, "lengthening_speed");
    double TA_lengtheningSpeedError_r =
            TAPath_r.computeApproximationErrorOnGrid(40, "lengthening_speed");

    // Moment arms

    const auto& hip_flexion_r = model.getCoordinateSet().get("hip_flexion_r");
    const auto& knee_angle_r = model.getCoordinateSet().get("knee_angle_r");
    const auto& ankle_angle_r = model.getCoordinateSet().get("ankle_angle_r");

    double hamstrings_momArmErrorHip_r =
            hamstringsPath_r.computeApproximationErrorOnGrid(40,
                    "moment_arm", &hip_flexion_r);
    double hamstrings_momArmErrorKnee_r =
            hamstringsPath_r.computeApproximationErrorOnGrid(40,
                    "moment_arm", &knee_angle_r);
    double BFSH_momArmError_r =
            BFSHPath_r.computeApproximationErrorOnGrid(40,"moment_arm", &knee_angle_r);
    double GLU_momArmError_r =
            GLUPath_r.computeApproximationErrorOnGrid(40,"moment_arm", &hip_flexion_r);
    double IlioPsoas_momArmError_r =
            IlioPsoasPath_r.computeApproximationErrorOnGrid(40,"moment_arm", &hip_flexion_r);
    double RF_momArmErrorHip_r =
            RFPath_r.computeApproximationErrorOnGrid(40,"moment_arm",
                    &hip_flexion_r);
    double RF_momArmErrorKnee_r =
            RFPath_r.computeApproximationErrorOnGrid(40,"moment_arm",
                    &knee_angle_r);
    double VAS_momArmError_r =
            VASPath_r.computeApproximationErrorOnGrid(40,"moment_arm", &knee_angle_r);
    double GAS_momArmErrorKnee_r =
            GASPath_r.computeApproximationErrorOnGrid(40,"moment_arm",
                    &knee_angle_r);
    double GAS_momArmErrorAnkle_r =
            GASPath_r.computeApproximationErrorOnGrid(40,"moment_arm",
                    &ankle_angle_r);
    double SOL_momArmError_r =
            SOLPath_r.computeApproximationErrorOnGrid(40,"moment_arm", &ankle_angle_r);
    double TA_momArmError_r =
            TAPath_r.computeApproximationErrorOnGrid(40,"moment_arm", &ankle_angle_r);

     CHECK(hamstrings_lengthError_r < 3e-3);
     CHECK(BFSH_lengthError_r < 3e-3);
     CHECK(GLU_lengthError_r < 3e-3);
     CHECK(IlioPsoas_lengthError_r < 3e-3);
     CHECK(RF_lengthError_r < 3e-3);
     CHECK(VAS_lengthError_r < 3e-3);
     CHECK(GAS_lengthError_r < 3e-3);
     CHECK(SOL_lengthError_r < 3e-3);
     CHECK(TA_lengthError_r < 3e-3);

     CHECK(hamstrings_lengtheningSpeedError_r < 3e-3);
     CHECK(BFSH_lengtheningSpeedError_r < 3e-3);
     CHECK(GLU_lengtheningSpeedError_r < 3e-3);
     CHECK(IlioPsoas_lengtheningSpeedError_r < 3e-3);
     CHECK(RF_lengtheningSpeedError_r < 3e-3);
     CHECK(VAS_lengtheningSpeedError_r < 3e-3);
     CHECK(GAS_lengtheningSpeedError_r < 3e-3);
     CHECK(SOL_lengtheningSpeedError_r < 3e-3);
     CHECK(TA_lengtheningSpeedError_r < 3e-3);

     CHECK(hamstrings_momArmErrorHip_r < 3e-3);
     CHECK(hamstrings_momArmErrorKnee_r < 3e-3);
     CHECK(BFSH_momArmError_r < 3e-3);
     CHECK(GLU_momArmError_r < 3e-3);
     CHECK(IlioPsoas_momArmError_r < 3e-3);
     CHECK(RF_momArmErrorHip_r < 3e-3);
     CHECK(RF_momArmErrorKnee_r < 3e-3);
     CHECK(VAS_momArmError_r < 3e-3);
     CHECK(GAS_momArmErrorKnee_r < 3e-3);
     CHECK(GAS_momArmErrorAnkle_r < 3e-3);
     CHECK(SOL_momArmError_r < 3e-3);
     CHECK(TA_momArmError_r < 3e-3);



}

TEST_CASE("testPolynomialApproximation") {

    testPolynomialApproximationImpl();
    testPolynomialApproximation();
}
