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
#include <OpenSim/Common/LogManager.h>
#include <OpenSim/Simulation/Manager/Manager.h>

using namespace OpenSim;

Model createGait2D() {

    ///////////////////////////////////////////////////////////////////////////
    // Create model
    ///////////////////////////////////////////////////////////////////////////
    Model model;
    model.setName("gait_2D");
    using SimTK::Vec3;
    using SimTK::Inertia;
    using SimTK::Transform;

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
    ///////////////////////////////////////////////////////////////////////////

    int dimHamstrings = 1;
    int orderHamstrings = 3;
    const int nCoeffHamstrings = 10;
    double coeffHamstrings[nCoeffHamstrings] = {0.420495248805280,
        0.028755254940480, 0.018300167519638, 0.006995443639179,
        0.053690588126276, 0.002092858497112, 0.001164874424056,
        0.016799961996946, 1.886801073680048e-04, 0.009576337870147};

    //int dimBFSH = 2;
    //int orderBFSH = 3;
    //const int nCoeffBFSH = 10;
    //double coeffBFSH[nCoeffBFSH] = {0.4205, 0.0288, -0.0183,
    //        -0.0070, 0.0537, 0.0021, 0.0012, 0.0168, 1.8868e-04, -0.0096};




    auto* hamstrings_f = new MultivariatePolynomialFunction();
    hamstrings_f->setDimension(dimHamstrings);
    hamstrings_f->setOrder(orderHamstrings);
    hamstrings_f->setCoefficients(
            SimTK::Vector(nCoeffHamstrings,coeffHamstrings));

    PolynomialActuators hamstrings_p_r;;
    hamstrings_p_r.set_function(*hamstrings_f);

    //hamstrings_p_r.appendNewPathPoint("P1", *tibia_r, Vec3(0));
    //hamstrings_p_r.appendNewPathPoint("P2", *calcn_r, Vec3(0));

    hamstrings_p_r.append_coordinate_list("/jointset/hip_r/hip_q_r");
    hamstrings_p_r.append_coordinate_list("/jointset/knee_r/knee_q_r");

    auto* hamstrings_r = new DeGrooteFregly2016Muscle();
    hamstrings_r->set_GeometryPath(hamstrings_p_r);
    hamstrings_r->set_ignore_tendon_compliance(true);
    hamstrings_r->setName("hamstrings_r");
    model.addComponent(hamstrings_r);

    model.finalizeConnections();

    return model;
}

void testPolynomialApproximation() {

    Model model(createGait2D());
    SimTK::State& state = model.initSystem();

    int ndof = model.getNumStateVariables()/2;
    SimTK::Vector QsUs(2*ndof);
    QsUs.setTo(0);
    QsUs[1] = -1;
    QsUs[3] = 1;
    QsUs[5] = 0.5;


    model.setStateVariableValues(state, QsUs);
    model.realizeVelocity(state);

    double hamstrings_lmt_r = model.getComponent<PathActuator>(
                "hamstrings_r").getGeometryPath().getLength(state);
    double hamstrings_vmt_r = model.getComponent<PathActuator>(
                "hamstrings_r").getGeometryPath().getLengtheningSpeed(state);

    CAPTURE(hamstrings_lmt_r);

    CHECK(0.420495248805280 == hamstrings_lmt_r);

    //CHECK(0.244916441634551
    //CHECK(0.201384070841096
    //CHECK(0.254156701045543
    //CHECK(0.411041216316924
    //CHECK(0.184215306369091
    //CHECK(0.434930944648804
    //CHECK(0.280659364908914
    //CHECK(0.290611643802287

    CAPTURE(hamstrings_vmt_r);
    CHECK(-0.024935333185797 == hamstrings_vmt_r);
    //CHECK(0.021846865400467
    //CHECK(-0.056529803505131
    //CHECK(0.038230025226882
    //CHECK(-7.027289470986353e-04
    //CHECK(-0.040084488603560
    //CHECK(0.047371212492924
    //CHECK(0.021341136339192
    //CHECK(-0.021960474948139





    ////model.getComponent("hamstrings_r").

    //[0.420495, 0.244916, 0.201384, 0.254157, 0.411041, 0.184215, 0.434931, 0.280659, 0.290612]

    ////[-0.0249353, 0.0218469, -0.0565298, 0.03823, -0.000702729, -0.0400845, 0.0473712, 0.0213411, -0.0219605]

    //[lMTk_l,vMTk_l,MA_l] = f_lMT_vMT_dM([0,0,0],[-1,1,0.5]);

}



TEST_CASE("testPolynomialApproximation") {
    testPolynomialApproximation();
}
