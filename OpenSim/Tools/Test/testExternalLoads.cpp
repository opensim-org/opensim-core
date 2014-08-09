/* -------------------------------------------------------------------------- *
 *                      OpenSim:  testExternalLoads.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

#include <iostream>
#include <OpenSim/OpenSim.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

void testExternalLoad();

int main()
{
    try {
        testExternalLoad();
    }
    catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}


void addLoadToStorage(Storage &forceStore, SimTK::Vec3 force, SimTK::Vec3 point, SimTK::Vec3 torque)
{
    int nLoads = forceStore.getColumnLabels().getSize()/9;
    string labels[9] = { "forceX", "forceY", "forceZ", "pointX", "pointY", "pointZ","torqueX", "torqueY", "torqueZ"};
    char suffix[2];
    sprintf(suffix, "%d", nLoads);

    Array<string> col_labels;
    col_labels.append("time");
    StateVector dataRow;
    dataRow.setTime(0);
    double data[9];
    for(int i = 0; i<9; i++) {
        col_labels.append(labels[i]);
        if(i<3) {
            data[i] = force[i];
            continue;
        }
        else if(i<6) {
            data[i] = point[i-3];
            continue;
        }
        else
            data[i] = torque[i-6];
    }

    dataRow.setStates(0, 9, data);

    Storage *forces = NULL;
    Storage tempStore;

    if(nLoads == 0)
        forces = &forceStore;
    else if (nLoads > 0)
        forces = &tempStore;
    else
        throw OpenSim::Exception("addLoadToStorage: ERROR");

    forces->setColumnLabels(col_labels);
    forces->append(dataRow);
    dataRow.setTime(1.0);
    forces->append(dataRow);
    dataRow.setTime(2.0);
    forces->append(dataRow);

    if (nLoads > 0)
        forces->addToRdStorage(forceStore, 0.0, 1.0);
}

void testExternalLoad()
{
    using namespace SimTK;

    Model model("Pendulum.osim");
    State &s = model.initSystem();

    // Simulate gravity
    double init_t =-1e-8;
    double final_t = 2.0;
    int nsteps = 10;
    double dt = final_t/nsteps;

    //initial state
    double q_init = Pi/4;
    model.updCoordinateSet()[0].setValue(s, q_init);

    Vector_<double> q_grav(nsteps+1);

    // Integrator and integration manager
    double integ_accuracy = 1e-6;
    RungeKuttaMersonIntegrator integrator(model.getMultibodySystem() );
    integrator.setAccuracy(integ_accuracy);
    Manager manager(model,  integrator);
    manager.setInitialTime(init_t);

    for(int i = 0; i < nsteps+1; i++) {
        manager.setFinalTime(dt*i);
        manager.integrate(s);
        q_grav[i] = model.updCoordinateSet()[0].getValue(s);
        manager.setInitialTime(dt*i);
    }

    //q_grav.dump("Coords due to gravity.");

    /***************************** CASE 1 ************************************/
    // Simulate the same system without gravity but with an equivalent external load
    OpenSim::Body &pendulum = model.getBodySet().get(model.getNumBodies()-1);
    string pendBodyName = pendulum.getName();
    Vec3 comInB = pendulum.getMassCenter();

    Storage forceStore;
    addLoadToStorage(forceStore,  pendulum.getMass()*model.getGravity(),  comInB, Vec3(0, 0, 0));
    forceStore.setName("test_external_loads.sto");
    forceStore.print(forceStore.getName());

    // Apply external force with force in ground, point in body, zero torque
    ExternalForce xf(forceStore, "force", "point", "torque", pendBodyName, "ground", pendBodyName);
    xf.setName("grav");

    ExternalLoads* extLoads = new ExternalLoads(model);
    extLoads->adoptAndAppend(&xf);

    extLoads->print("ExternalLoads_test.xml");

    for(int i=0; i<extLoads->getSize(); i++)
        model.addForce(&(*extLoads)[i]);

    // Create the force reporter
    ForceReporter* reporter = new ForceReporter();
    model.addAnalysis(reporter);

    Kinematics* kin = new Kinematics();
    kin->setInDegrees(false);
    model.addAnalysis(kin);

    PointKinematics* pKin = new PointKinematics();
    pKin->setBody(&pendulum);
    pKin->setPoint(comInB);
    pKin->setPointName(pendulum.getName()+"_com_p");
    model.addAnalysis(pKin);

    SimTK::State& s2 = model.initSystem();

    // Turn-off gravity in the model
    model.updGravityForce().disable(s2);

    // initial position
    model.updCoordinateSet()[0].setValue(s2, q_init);

    RungeKuttaMersonIntegrator integrator2(model.getMultibodySystem() );
    integrator2.setAccuracy(integ_accuracy);
    Manager manager2(model,  integrator2);
    manager2.setInitialTime(init_t);

    // Simulate with the external force applied instead of gravity
    Vector_<double> q_xf(nsteps+1);
    Vector_<Vec3> pcom_xf(nsteps+1);

    for(int i = 0; i < nsteps+1; i++) {
        manager2.setFinalTime(dt*i);
        manager2.integrate(s2);
        q_xf[i] = model.updCoordinateSet()[0].getValue(s2);
        manager2.setInitialTime(dt*i);
    }

    //q_xf.dump("Coords due to external force point expressed in pendulum.");

    Vector err = q_xf-q_grav;
    double norm_err = err.norm();

    // kinematics should match to within integ accuracy
    ASSERT_EQUAL(0.0, norm_err, integ_accuracy);

    /***************************** CASE 2 ************************************/
    // Simulate the same system without gravity but with an equivalent external
    // force but this time with the point expressed in  ground and using
    // previous kinematics to transform point to pendulum.

    // Construct a new Storage for ExternalForce data source with point
    // described in ground
    Storage forceStore2 = reporter->getForceStorage();
    forceStore2.print("ForcesTest.sto");
    Storage *pStore = pKin->getPositionStorage();
    pStore->print("PointInGroundTest.sto");
    pStore->addToRdStorage(forceStore2, init_t, final_t);

    forceStore2.setName("ExternalForcePointInGround.sto");
    forceStore2.print(forceStore2.getName());

    Storage *qStore = kin->getPositionStorage();
    qStore->print("LoadKinematics.sto");

    string id_base = pendBodyName+"_"+xf.getName();
    string point_id = pKin->getPointName();

    ExternalForce xf2(forceStore2, id_base+"_F", point_id, id_base+"_T", pendBodyName, "ground", "ground");
    xf2.setName("xf_pInG");
    // Empty out existing external forces
    extLoads->setMemoryOwner(false);
    extLoads->setSize(0);
    extLoads->adoptAndAppend(&xf2);

    //Ask external loads to transform point expressed in ground to the applied body
    extLoads->setDataFileName(forceStore2.getName());
    extLoads->invokeConnectToModel(model);
    extLoads->transformPointsExpressedInGroundToAppliedBodies(*qStore);

    // remove previous external force from the model too
    model.disownAllComponents();
    model.updForceSet().setSize(0);

    // after external loads has transformed the point of the force, then add it the model
    for(int i=0; i<extLoads->getSize(); i++)
        model.addForce(&(*extLoads)[i]);

    // recreate dynamical system to reflect new force
    SimTK::State &s3 = model.initSystem();

    // Turn-off gravity in the model
    model.updGravityForce().disable(s3);

    // initial position
    model.updCoordinateSet()[0].setValue(s3, q_init);

    RungeKuttaMersonIntegrator integrator3(model.getMultibodySystem() );
    integrator3.setAccuracy(integ_accuracy);
    Manager manager3(model,  integrator3);
    manager3.setInitialTime(init_t);

    // Simulate with the external force applied instead of gravity
    Vector_<double> q_xf2(nsteps+1);

    for(int i = 0; i < nsteps+1; i++) {
        manager3.setFinalTime(dt*i);
        manager3.integrate(s3);
        q_xf2[i] = model.updCoordinateSet()[0].getValue(s3);
        manager3.setInitialTime(dt*i);
    }

    //q_xf2.dump("Coords due to external force point expressed in ground.");
    err = q_xf2-q_grav;
    //err.dump("Coordinate error after transforming point.");
    norm_err = err.norm();

    // kinematics should match to within integ accuracy
    ASSERT_EQUAL(0.0, norm_err, integ_accuracy);
}