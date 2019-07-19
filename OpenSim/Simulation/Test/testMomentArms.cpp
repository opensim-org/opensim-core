/* -------------------------------------------------------------------------- *
 *                        OpenSim:  testMomentArms.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

//=============================================================================
// testMomentArms loads various OpenSim models to compute and test moment arms
// results from these models to the definition r*f = Tau , where r is the 
// moment-arm about a coordinate, f is the scaler magnitude of the Force and 
// Tau is the resulting generalized force. 
//
//  Tests Include:
//      1. ECU muscle from Tutorial 2
//      2. Vasti from gait23 models with and without a patella
//      
//     Add more test cases to address specific problems with moment-arms
//
//=============================================================================
#include "SimulationComponentsForTesting.h"

#include <OpenSim/Actuators/Thelen2003Muscle.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Common/PolynomialFunction.h>
#include <OpenSim/Common/Sine.h>
#include <OpenSim/Simulation/osimSimulation.h>

using namespace OpenSim;
using namespace std;

//==============================================================================
// Common Parameters for the simulations are just global.
const static double integ_accuracy = 1.0e-3;

void testMomentArmDefinitionForModel(const string &filename, 
                                     const string &coordName = "", 
                                     const string &muscleName = "",
                                     SimTK::Vec2 rom = SimTK::Vec2(-SimTK::Pi/2,0),
                                     double mass = -1.0, string errorMessage = "");

void testMomentArmsAcrossCompoundJoint();

void testGeometryPathApproximation();

int main()
{
    clock_t startTime = clock();
    LoadOpenSimLibrary("osimActuators");
    Object::registerType(CompoundJoint());

    try {

        testMomentArmsAcrossCompoundJoint();
        cout << "Joint composed of more than one mobilized body: PASSED\n" << endl;

        testMomentArmDefinitionForModel("BothLegs22.osim", "r_knee_angle", "VASINT", 
            SimTK::Vec2(-2*SimTK::Pi/3, SimTK::Pi/18), 0.0, 
            "VASINT of BothLegs with no mass: FAILED");
        cout << "VASINT of BothLegs with no mass: PASSED\n" << endl;

        testMomentArmDefinitionForModel("testMomentArmsConstraintB.osim", 
            "hip_flexion_r", "rect_fem_r", SimTK::Vec2(-SimTK::Pi/3, SimTK::Pi/3),
            -1.0, "Rectus Femoris at hip with muscle attachment on patella defined w.r.t Femur: FAILED");
        cout << "Rectus Femoris at hip with muscle attachment on patella defined w.r.t Femur: PASSED\n" << endl;

        testMomentArmDefinitionForModel("testMomentArmsConstraintB.osim", "knee_angle_r", "rect_fem_r", SimTK::Vec2(-2*SimTK::Pi/3, SimTK::Pi/18), -1.0, "Rectus Femoris with muscle attachment on patella defined w.r.t Femur: FAILED");
        cout << "Rectus Femoris with muscle attachment on patella defined w.r.t Femur: PASSED\n" << endl;

        testMomentArmDefinitionForModel("testMomentArmsConstraintB.osim", "knee_angle_r", "vas_int_r", SimTK::Vec2(-2*SimTK::Pi/3, SimTK::Pi/18), -1.0, "Knee with Vasti attachment on patella defined w.r.t Femur: FAILED");
        cout << "Knee with Vasti attachment on patella defined w.r.t Femur: PASSED\n" << endl;

        testMomentArmDefinitionForModel("testMomentArmsConstraintA.osim", "knee_angle_r", "vas_int_r", SimTK::Vec2(-2*SimTK::Pi/3, SimTK::Pi/18), -1.0, "Knee with Vasti attachment on patella w.r.t Tibia: FAILED");
        cout << "Knee with Vasti attachment on patella w.r.t Tibia: PASSED\n" << endl;

        testMomentArmDefinitionForModel("MovingPathPointMomentArmTest.osim", "", "", SimTK::Vec2(-2*SimTK::Pi/3, SimTK::Pi/18), -1.0, "Moving path point across PinJoint: FAILED");
        cout << "Moving path point across PinJoint: PASSED\n" << endl;

        testMomentArmDefinitionForModel("gait2354_simbody.osim", "knee_angle_r", "vas_int_r", SimTK::Vec2(-119*SimTK::Pi/180, 9*SimTK::Pi/180), -1.0, "Knee with moving muscle point (no patella): FAILED");
        cout << "Knee with moving muscle point (no patella): PASSED\n" << endl;
        
        //massless should not break moment-arm solver
        testMomentArmDefinitionForModel("wrist_mass.osim", "flexion", "ECU_post-surgery", SimTK::Vec2(-SimTK::Pi/3, SimTK::Pi/3), 0.0, "WRIST ECU TEST with MASSLESS BODIES: FAILED");
        cout << "WRIST ECU TEST with MASSLESS BODIES: PASSED\n" << endl;

        testMomentArmDefinitionForModel("wrist_mass.osim", "flexion", "ECU_post-surgery", SimTK::Vec2(-SimTK::Pi/3, SimTK::Pi/3), 1.0, "WRIST ECU TEST with MASS  = 1.0 : FAILED");
        cout << "WRIST ECU TEST with MASS  = 1.0 : PASSED\n" << endl;

        testMomentArmDefinitionForModel("wrist_mass.osim", "flexion", "ECU_post-surgery", SimTK::Vec2(-SimTK::Pi/3, SimTK::Pi/3), 100.0, "WRIST ECU TEST with MASS  = 100.0 : FAILED");
        cout << "WRIST ECU TEST with MASS  = 100.0 : PASSED\n" << endl;

        testMomentArmDefinitionForModel("P2PBallJointMomentArmTest.osim", "", "", SimTK::Vec2(-SimTK::Pi/2,0), -1.0, "Point to point muscle across BallJoint: FAILED");
        cout << "Point to point muscle across BallJoint: PASSED\n" << endl;

        testMomentArmDefinitionForModel("P2PBallCustomJointMomentArmTest.osim", "", "", SimTK::Vec2(-SimTK::Pi/2,0), -1.0, "Point to point muscle across a ball implemented by CustomJoint: FAILED");
        cout << "Point to point muscle across a ball implemented by CustomJoint: PASSED\n" << endl;

        testMomentArmDefinitionForModel("P2PCustomJointMomentArmTest.osim", "", "", SimTK::Vec2(-SimTK::Pi/2,0), -1.0, "Point to point muscle across CustomJoint: FAILED");
        cout << "Point to point muscle across CustomJoint: PASSED\n" << endl;

        testMomentArmDefinitionForModel("MovingPointCustomJointMomentArmTest.osim", "", "", SimTK::Vec2(-SimTK::Pi/2,0), -1.0, "Moving path point across CustomJoint: FAILED");
        cout << "Moving path point across CustomJoint: PASSED\n" << endl;

        testMomentArmDefinitionForModel("WrapPathCustomJointMomentArmTest.osim", "", "", SimTK::Vec2(-SimTK::Pi/2,0), -1.0, "Path with wrapping across CustomJoint: FAILED");
        cout << "Path with wrapping across CustomJoint: PASSED\n" << endl;
        
        testMomentArmDefinitionForModel("PathOnConstrainedBodyMomentArmTest.osim", "", "", SimTK::Vec2(-SimTK::Pi/2,0), -1.0, "Path on constrained body across CustomJoint: FAILED");
        cout << "Path on constrained body across CustomJoint: PASSED\n" << endl;
    
        testMomentArmDefinitionForModel("MultipleMPPsMomentArmTest.osim", "knee_angle_1", "vas_int_r", SimTK::Vec2(-1.99*SimTK::Pi/3, SimTK::Pi/18), -1.0, "Multiple moving path points: FAILED");
        cout << "Multiple moving path points test 1: PASSED\n" << endl;

        testMomentArmDefinitionForModel("MultipleMPPsMomentArmTest.osim", "knee_angle_2", "vas_int_r", SimTK::Vec2(-1.99*SimTK::Pi/3, SimTK::Pi/18), -1.0, "Multiple moving path points: FAILED");
        cout << "Multiple moving path points test 2: PASSED\n" << endl;

        testMomentArmDefinitionForModel("CoupledCoordinatesMPPsMomentArmTest.osim", "foot_angle", "vas_int_r", SimTK::Vec2(-2*SimTK::Pi/3, SimTK::Pi/18), -1.0, "Multiple moving path points: FAILED");
        cout << "Multiple moving path points coupled coordinates test: PASSED\n" << endl;

        testGeometryPathApproximation();
        cout << "Geometry path approximation: PASSED\n" << endl;
    }
    catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    cout << "Moment-arm test time: " << 1.0e3*(std::clock()-startTime)/CLOCKS_PER_SEC << "ms" << endl;

    return 0;
}

void testMomentArmsAcrossCompoundJoint()
{
    Model model;

    Body* leg = new Body("leg", 10., SimTK::Vec3(0,1,0), SimTK::Inertia(1,1,1));
    model.addComponent(leg);

    CompoundJoint* hip = new CompoundJoint("hip",
        model.getGround(), SimTK::Vec3(0), SimTK::Vec3(0),
        *leg, SimTK::Vec3(0, 0.5, 0), SimTK::Vec3(0));
    model.addComponent(hip);

    Thelen2003Muscle* musc = new Thelen2003Muscle("muscle", 10., 0.1, 0.2, 0.);
    musc->addNewPathPoint("p1", model.updGround(), SimTK::Vec3(0.05, 0, 0));
    musc->addNewPathPoint("p2", *leg, SimTK::Vec3(0.05, 0.25, 0.01));
    model.addForce(musc);

    SimTK::State& s = model.initSystem();

    const Coordinate& c = model.getCoordinateSet()[0];

    model.print("testMomentArmsAcrossCompoundJoint.osim");
    testMomentArmDefinitionForModel("testMomentArmsAcrossCompoundJoint.osim",
        c.getName(), "muscle", SimTK::Vec2(-2 * SimTK::Pi / 3, SimTK::Pi / 18),
        0.0, "testMomentArmsAcrossCompoundJoint: FAILED");
}

//==========================================================================================================
// moment_arm = dl/dtheta, definition using inexact perturbation technique
//==========================================================================================================
double computeMomentArmFromDefinition(const SimTK::State &s, const GeometryPath &path, const Coordinate &coord)
{
    using namespace SimTK;

    //Compute r = dl/dtheta
    SimTK::State s_ma = s;
    coord.setClamped(s_ma, false);
    coord.setLocked(s_ma, false);
    double theta = coord.getValue(s);
    double dtheta = 0.1*integ_accuracy;
    
    // Compute length 1 
    coord.setValue(s_ma, theta-dtheta, false);

    // satisfy constraints using project since we are close to the solution
    coord.getModel().getMultibodySystem().realize(s_ma, SimTK::Stage::Position);
    coord.getModel().getMultibodySystem().projectQ(s_ma, 1e-8);

    double theta1 = coord.getValue(s_ma);
    coord.getModel().getMultibodySystem().realize(s_ma, SimTK::Stage::Position);

    double len1 = path.getLength(s_ma);

    // Compute length 2
    coord.setValue(s_ma, theta+dtheta, false);

    // satisfy constraints using project since we are close to the solution
    coord.getModel().getMultibodySystem().realize(s_ma, SimTK::Stage::Position);
    coord.getModel().getMultibodySystem().projectQ(s_ma, 1e-8);

    double theta2 = coord.getValue(s_ma);
    coord.getModel().getMultibodySystem().realize(s_ma, SimTK::Stage::Position);

    double len2 = path.getLength(s_ma);

    return (len1-len2)/(theta2-theta1);
}


SimTK::Vector computeGenForceScaling(const Model &osimModel, const SimTK::State &s, const Coordinate &coord, 
                              const Array<string> &coupledCoords)
{
    using namespace SimTK;

    //Local modifiable copy of the state
    State s_ma = s;

    osimModel.getMultibodySystem().realize(s_ma, SimTK::Stage::Instance);

    // Calculate coupling matrix C to determine the influence of other coordinates 
    // (mobilities) on the coordinate of interest due to constraints

    s_ma.updU() = 0;
    // Light-up speed of coordinate of interest and see how other coordinates
    // affected by constraints respond
    coord.setSpeedValue(s_ma, 1);

    // Satisfy velocity constraints. Note that the speed we just set may 
    // change here too so be sure to retrieve the modified value.
    osimModel.getMultibodySystem().realize(s_ma, SimTK::Stage::Velocity);
    osimModel.getMultibodySystem().projectU(s_ma, 1e-10);
    
    // Now calculate C. by checking how speeds of other coordinates change
    // normalized by how much the speed of the coordinate of interest changed. 
    const Vector C = s_ma.getU() / coord.getSpeedValue(s_ma); 
    
    // Compute the scaling matrix for converting gen_forces to torques
    // Unlike C, ignore all coupling that are not explicit coordinate
    // coupling that defines theta = sum(q_i) or q_i = w_i*theta.
    // Also do not consider coupled torques for coordinates not spanned by 
    // the path of interest.
    Vector W(osimModel.getNumSpeeds(), 0.0);

    for(int i=0; i< osimModel.getCoordinateSet().getSize(); i++){
        Coordinate &ac = osimModel.getCoordinateSet()[i];
        //If a coordinate is kinematically coupled  (ac.getName() == coord.getName()) || 
        bool found = ((ac.getName() == coord.getName()) || (coupledCoords.findIndex(ac.getName()) > -1));

        // and not translational (cannot contribute to torque)
        if(found && (ac.getMotionType() != Coordinate::Translational) 
                && (ac.getJoint().getName() != "tib_pat_r") ){
            MobilizedBodyIndex modbodIndex = ac.getBodyIndex();
            const MobilizedBody& mobod = osimModel.getMatterSubsystem().getMobilizedBody(modbodIndex);
            SpatialVec Hcol = mobod.getHCol(s, SimTK::MobilizerUIndex(0)); //ac.getMobilizerQIndex())); // get n�th column of H

            /*double thetaScale = */Hcol[0].norm(); // magnitude of the rotational part of this column of H
            
            double Ci = C[mobod.getFirstUIndex(s)+ac.getMobilizerQIndex()];
            // double Wi = 1.0/thetaScale;
            //if(thetaScale)
                W[mobod.getFirstUIndex(s)+ac.getMobilizerQIndex()] = Ci; 
        }
    }

    return W;
}

//==========================================================================================================
// Main test driver can be used on any model so test cases should be very easy to add
//==========================================================================================================
void testMomentArmDefinitionForModel(const string &filename, const string &coordName, 
                                    const string &muscleName, SimTK::Vec2 rom,
                                    double mass, string errorMessage)
{
    using namespace SimTK;

    bool passesDefinition = true;
    bool passesDynamicConsistency = true;

    // Load OpenSim model
    Model osimModel(filename);
    osimModel.initSystem();

    // Create the moment-arm solver to solve for moment-arms
    MomentArmSolver maSolver(osimModel);

    Coordinate &coord = (coordName != "") ? osimModel.updCoordinateSet().get(coordName) :
        osimModel.updCoordinateSet()[0];

    // Consider one force, which is the muscle of interest
    Muscle &muscle = (muscleName != "") ? dynamic_cast<Muscle&>((osimModel.updMuscles().get(muscleName))) :
        dynamic_cast<Muscle&>(osimModel.updMuscles()[0]);

    if( mass >= 0.0){
        for(int i=0; i<osimModel.updBodySet().getSize(); i++){
            osimModel.updBodySet()[i].setMass(mass);
            Inertia inertia(mass);
            osimModel.updBodySet()[i].setInertia(inertia);
        }
    }

    SimTK::State &s = osimModel.initSystem();

    Array<string> coupledCoordNames;
    for(int i=0; i<osimModel.getConstraintSet().getSize(); i++){
        OpenSim::Constraint& aConstraint = osimModel.getConstraintSet().get(i);
        if(aConstraint.getConcreteClassName() == "CoordinateCouplerConstraint"){
            CoordinateCouplerConstraint& coupler = dynamic_cast<CoordinateCouplerConstraint&>(aConstraint);
            Array<string> coordNames = coupler.getIndependentCoordinateNames();
            coordNames.append(coupler.getDependentCoordinateName());

            int ind = coordNames.findIndex(coord.getName());
            if (ind > -1){
                for(int j=0; j<coordNames.getSize(); j++){
                    if(j!=ind)
                        coupledCoordNames.append(coordNames[j]);
                }
            }
        }
    }

    // Reset all speeds to zero
    s.updU() = 0;

    // Disable all forces
    for(int i=0; i<osimModel.updForceSet().getSize(); i++){
        osimModel.updForceSet()[i].setAppliesForce(s, false);
    }
    // Also disable gravity
    osimModel.getGravityForce().disable(s);

    // Enable just muscle we are interested in.
    muscle.setAppliesForce(s, true);

    coord.setClamped(s, false);
    coord.setLocked(s, false);

    double q = rom[0];
    int nsteps = 10;
    double dq = (rom[1]-rom[0])/nsteps;
    
    cout << "___________________________________________________________________________________" << endl;
    cout << "MA  genforce/fm::dl/dtheta  joint angle       IDTorq :: EquiTorq  MA::MA_dl/dtheta" << endl;
    cout << "===================================================================================" << endl;
    for(int i = 0; i <=nsteps; i++){
        coord.setValue(s, q, true);
        // double angle = coord.getValue(s);

        //cout << "muscle  force: " << muscle.getForce(s) << endl;
        //double ma = muscle.computeMomentArm(s, coord);
        double ma = maSolver.solve(s, coord, muscle.getGeometryPath());
        double ma_dldtheta = computeMomentArmFromDefinition(s, muscle.getGeometryPath(), coord);

        cout << "r's = " << ma << "::" << ma_dldtheta <<"  at q = " << coord.getValue(s)*180/Pi; 

        try {
            // Verify that the definition of the moment-arm is satisfied
            ASSERT_EQUAL(ma, ma_dldtheta, integ_accuracy);
        }
        catch (const OpenSim::Exception&) {
            passesDefinition = false;
        }

        // Verify that the moment-arm calculated is dynamically consistent with moment generated
        if (mass!=0 ) {
            muscle.overrideActuation(s, true);
            muscle.setOverrideActuation(s, 10);
            osimModel.getMultibodySystem().realize(s, Stage::Acceleration);

            double force = muscle.getActuation(s);
        
            // Get muscle's applied body forces 
            const Vector_<SpatialVec>& appliedBodyForces = osimModel.getMultibodySystem().getRigidBodyForces(s, Stage::Dynamics);
            //appliedBodyForces.dump("Applied Body Force resulting from muscle");

            // And any applied mobility (gen) forces due to gearing (moving path point)
            const Vector& appliedGenForce = osimModel.getMultibodySystem().getMobilityForces(s, Stage::Dynamics);       

            // Get current system accelerations
            const Vector& knownUDots = s.getUDot();
            //knownUDots.dump("Acceleration due to ECU muscle:");

            // Convert body forces to equivalent mobility forces (joint torques)
            Vector equivalentGenForce(s.getNU(), 0.0);
            osimModel.getMultibodySystem().getMatterSubsystem().calcTreeEquivalentMobilityForces(s, 
                appliedBodyForces, equivalentGenForce);
            if(s.getSystemStage() < SimTK::Stage::Dynamics)
                osimModel.getMultibodySystem().realize(s,SimTK::Stage::Dynamics);

            // include any directly applied gen force from the path (muscle) tension
            equivalentGenForce += appliedGenForce;

            // Determine the contribution of constraints (if any) to the effective torque
            Vector_<SimTK::SpatialVec> constraintForcesInParent;
            Vector constraintMobilityForces;

            // Get all forces applied to model by constraints
            osimModel.getMultibodySystem().getMatterSubsystem().calcConstraintForcesFromMultipliers(s, -s.getMultipliers(), 
                constraintForcesInParent, constraintMobilityForces);
        
            // Perform inverse dynamics
            Vector ivdGenForces;
            osimModel.getMultibodySystem().getMatterSubsystem().calcResidualForceIgnoringConstraints(s,
                constraintMobilityForces, constraintForcesInParent, knownUDots, ivdGenForces);
            
            //constraintForcesInParent.dump("Constraint Body Forces");
            //constraintMobilityForces.dump("Constraint Mobility Forces");

            Vector W = computeGenForceScaling(osimModel, s, coord, coupledCoordNames);

            double equivalentMuscleTorque = ~W*equivalentGenForce;
            double equivalentIvdMuscleTorque = ~W*(ivdGenForces); //+constraintMobilityForces);

            cout << "  Tau = " << equivalentIvdMuscleTorque <<"::" << equivalentMuscleTorque 
                 << "  r*fm = " << ma*force <<"::" << ma_dldtheta*force << endl;


            try {   
                // Resulting torque from ID (no constraints) + constraints = equivalent applied torque 
                ASSERT_EQUAL(0.0, (equivalentIvdMuscleTorque-equivalentMuscleTorque)/equivalentIvdMuscleTorque, integ_accuracy);
                // verify that equivalent torque is in fact moment-arm*force
                ASSERT_EQUAL(0.0, (ma*force-equivalentMuscleTorque)/equivalentMuscleTorque, integ_accuracy);
            }
            catch (const OpenSim::Exception&) {
                passesDynamicConsistency = false;
            }
        } else {
            cout << endl;
        }

        // Increment the joint angle
        q += dq;
    }

    if(!passesDefinition)
        cout << "WARNING: Moment arm did not satisfy dL/dTheta equivalence." << endl;
    if(!passesDynamicConsistency)
        cout << "WARNING: Moment arm * force did not satisfy Torque equivalence." << endl;

    // Minimum requirement to pass is that calculated moment-arm satisfies either
    // dL/dTheta definition or is at least dynamically consistent, in which dL/dTheta is not
    ASSERT(passesDefinition || passesDynamicConsistency, __FILE__, __LINE__, errorMessage);
}

void testGeometryPathApproximation() {
    using namespace SimTK;


    class FitLength : public OptimizerSystem {
    public:
        FitLength(int numParams, double minQ, double maxQ, Model& model, State& state,
                GeometryPath& path, const Coordinate& coord)
                : OptimizerSystem(numParams), m_model(model), m_state(state),
                  m_path(path), m_coord(coord) {

            int N = 40;
            m_q.resize(N);
            for (int i = 0; i < N; ++i) {
                m_q[i] = minQ + i * (maxQ - minQ) / (N - 1);
            }
            m_lengthWithPoints.resize(N);
            m_momArmWithPoints.resize(N);
            m_lengthWithApprox.resize(N);
            m_momArmWithApprox.resize(N);
        }
        int objectiveFunc(const Vector& params, bool, Real& f) const override {
            m_path.set_length_approximation(PolynomialFunction(params));

            m_path.setUseApproximation(m_state, false);
            for (int i = 0; i < m_q.size(); ++i) {
                m_state.updQ()[0] = m_q[i];
                m_model.realizePosition(m_state);
                m_lengthWithPoints[i] = m_path.getLength(m_state);
                m_momArmWithPoints[i] = m_path.computeMomentArm(m_state, m_coord);
            }

            m_path.setUseApproximation(m_state, true);
            for (int i = 0; i < m_q.size(); ++i) {
                m_state.updQ()[0] = m_q[i];
                m_model.realizePosition(m_state);
                m_lengthWithApprox[i] = m_path.getLength(m_state);
                m_momArmWithApprox[i] = m_path.computeMomentArm(m_state, m_coord);
            }
            f = (m_lengthWithPoints - m_lengthWithApprox).normSqr() +
                    (m_momArmWithPoints - m_momArmWithApprox).normSqr();
            return 0;
        }

    private:
        Model& m_model;
        State& m_state;
        GeometryPath& m_path;
        const Coordinate& m_coord;
        SimTK::Vector m_q;
        mutable SimTK::Vector m_lengthWithPoints;
        mutable SimTK::Vector m_lengthWithApprox;
        mutable SimTK::Vector m_momArmWithPoints;
        mutable SimTK::Vector m_momArmWithApprox;
    };

    // Create a point mass sliding along a sinusoid (1-DOF).
    Model model;
    model.updGround().attachGeometry(new Sphere(0.05));
    auto* body = new OpenSim::Body("body", 1, Vec3(0), Inertia(1));
    body->attachGeometry(new Sphere(0.05));

    SpatialTransform transform;
    TransformAxis& tx = transform.updTransformAxis(3);
    tx.append_coordinates("q");
    tx.setFunction(LinearFunction(1, 0));

    TransformAxis& ty = transform.updTransformAxis(4);
    ty.append_coordinates("q");
    ty.setFunction(Sine(0.3, 2 * SimTK::Pi, 0.5 * SimTK::Pi));

    auto* joint = new CustomJoint("joint", model.getGround(), *body, transform);

    model.addBody(body);
    model.addJoint(joint);

    auto* actu = new PathActuator();
    actu->addNewPathPoint("origin", model.getGround(), Vec3(0));
    actu->addNewPathPoint("insertion", *body, Vec3(0));
    model.addForce(actu);

    // model.setUseVisualizer(true);
    // model.setGravity(Vec3(2.0, 0, 0));

    auto state = model.initSystem();

    double minQ = -0.5;
    double maxQ = 0.5;
    joint->upd_coordinates(0).setRangeMin(minQ);
    joint->upd_coordinates(0).setRangeMax(maxQ);

    // Manager manager(model, state);
    // state = manager.integrate(10.0);
    // model.getVisualizer().show(state);

    int nParams = 12;
    auto& path = actu->updGeometryPath();
    path.set_length_approximation(PolynomialFunction(Vector(nParams)));
    path.append_approximation_coordinates("/jointset/joint/q");
    model.initSystem();
    const auto& coord = joint->get_coordinates(0);
    FitLength sys(nParams, minQ, maxQ, model, state, path, coord);
    Optimizer opt(sys);
    opt.useNumericalGradient(true);
    Vector params(nParams);
    params = 0.2;
    double obj = opt.optimize(params);
    SimTK_TEST(obj < 0.10);
    // If obj is zero, that probably means we are not using the approximation.
    SimTK_TEST(obj > 1e-5);

    double lengthError = path.computeApproximationErrorOnGrid(40, "length");
    SimTK_TEST(lengthError < 0.01);
    // Something is wrong if the error is too small:
    SimTK_TEST(lengthError > 1e-10);

    // Check the moment arm.
    double momArmError = path.computeApproximationErrorOnGrid(
            40, "moment_arm", &coord);
    SimTK_TEST(momArmError < 0.05);
    // Something is wrong if the error is too small:
    SimTK_TEST(momArmError > 1e-10);

    // The lengthening speed of the approximation is close to that using points.
    double speedError = path.computeApproximationErrorOnGrid(40,
            "lengthening_speed");
    SimTK_TEST(speedError < 0.05);
    SimTK_TEST(speedError > 1e-10);


    // Cannot use the approximation if a function is not set.
    {
        path.updProperty_length_approximation().clear();
        SimTK_TEST_MUST_THROW(path.setUseApproximation(state, true));
    }

    // The number of arguments for the function must match the number of
    // provided coordinates.
    {
        path.set_length_approximation(PolynomialFunction());
        path.append_approximation_coordinates("dummy");
        SimTK_TEST_MUST_THROW(path.finalizeFromProperties());
    }

    // Ensure the N-dimensional grid iteration is correct.
    class My3DFunction : public OpenSim::Function {
        OpenSim_DECLARE_CONCRETE_OBJECT(My3DFunction, Function);
    public:
        double calcValue(const SimTK::Vector& x) const override {
            ++numEvals;
            return x.sum();
        }
        int getArgumentSize() const override { return 3; }
        int getMaxDerivativeOrder() const override { return 0; }
        SimTK::Function* createSimTKFunction() const override {
            throw OpenSim::Exception();
        }
        mutable int numEvals = 0;
    };
    {
        path.updProperty_length_approximation().clear();
        path.updProperty_approximation_coordinates().clear();
        model.updJointSet().clearAndDestroy();
        auto* joint = new PlanarJoint("joint", model.getGround(), *body);
        joint->upd_coordinates(0).setName("q1");
        joint->upd_coordinates(1).setName("q2");
        joint->upd_coordinates(2).setName("q3");
        model.addJoint(joint);
        model.initSystem();
        SimTK_TEST(model.getCoordinateSet().getSize() == 3);
        path.set_length_approximation(My3DFunction());
        path.append_approximation_coordinates("/jointset/joint/q1");
        path.append_approximation_coordinates("/jointset/joint/q2");
        path.append_approximation_coordinates("/jointset/joint/q3");
        model.initSystem();
        path.computeApproximationErrorOnGrid(3);
        const auto& func = static_cast<const My3DFunction&>(
                        path.get_length_approximation());
        SimTK_TEST(func.numEvals == 27);
    }
}
