/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxSandbox.cpp                                           *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
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

// This file provides a way to easily prototype or test temporary snippets of
// code during development.

#include <OpenSim/Moco/osimMoco.h>
#include <casadi/casadi.hpp>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Simulation/VisualizerUtilities.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>


using namespace OpenSim;
using namespace casadi;


using ContactForceRefs = std::vector<SimTK::ReferencePtr<const SmoothSphereHalfSpaceForce>>;

struct ContactBodyInfo {
    std::string path;
    std::string force_indentifier;
    std::vector<std::string> contact_force_paths;
};
using ContactBodyInfos = std::vector<ContactBodyInfo>;


void addGroundToFootJoint(Model& model, const std::string& footBodyName) {
    SpatialTransform transform;
    transform[0].setCoordinateNames(OpenSim::Array<std::string>(
            fmt::format("{}_rx", footBodyName), 1, 1));
    transform[0].setFunction(new LinearFunction());
    transform[0].setAxis(SimTK::Vec3(0, 0, 1));

    transform[1].setCoordinateNames(OpenSim::Array<std::string>(
            fmt::format("{}_ry", footBodyName), 1, 1));
    transform[1].setFunction(new LinearFunction());
    transform[1].setAxis(SimTK::Vec3(1, 0, 0));

    transform[2].setCoordinateNames(OpenSim::Array<std::string>(
            fmt::format("{}_rz", footBodyName), 1, 1));
    transform[2].setFunction(new LinearFunction());
    transform[2].setAxis(SimTK::Vec3(0, 1, 0));

    transform[3].setCoordinateNames(OpenSim::Array<std::string>(
            fmt::format("{}_tx", footBodyName), 1, 1));
    transform[3].setFunction(new LinearFunction());
    transform[3].setAxis(SimTK::Vec3(1, 0, 0));

    transform[4].setCoordinateNames(OpenSim::Array<std::string>(
            fmt::format("{}_ty", footBodyName), 1, 1));
    transform[4].setFunction(new LinearFunction());
    transform[4].setAxis(SimTK::Vec3(0, 1, 0));

    transform[5].setCoordinateNames(OpenSim::Array<std::string>(
            fmt::format("{}_tz", footBodyName), 1, 1));
    transform[5].setFunction(new LinearFunction());
    transform[5].setAxis(SimTK::Vec3(0, 0, 1));

    CustomJoint* joint = new CustomJoint(
            fmt::format("ground_{}", footBodyName), 
            model.getGround(), 
            model.getComponent<Body>(fmt::format("/bodyset/{}", footBodyName)), 
            transform);
    model.addJoint(joint);
}

void copyMarkersForBody(const Model& model, Model& contactModel, 
        const std::string& bodyPath) {
    for (const auto& marker : model.getComponentList<Marker>()) {
        if (marker.getSocket<PhysicalFrame>("parent_frame").getConnecteePath()
                == bodyPath) {
            contactModel.addMarker(marker.clone());
        }
    }
}

void copySmoothSphereHalfSpaceForceForContactGeometry(const Model& model, 
        Model& contactModel, const ContactGeometry& geometry,
        ContactBodyInfo& contactBodyInfo) {
    for (const auto& force : model.getComponentList<SmoothSphereHalfSpaceForce>()) {
        if (force.getSocket<ContactGeometry>("sphere").getConnecteePath()
                == geometry.getAbsolutePathString()) {
            
            // Add in the contact force and the half space geometry.
            const ContactGeometry& halfSpace = 
                    force.getSocket<ContactGeometry>("half_space").getConnectee();
            const std::string& halfSpacePath = halfSpace.getAbsolutePathString();
            if (!contactModel.hasComponent<ContactGeometry>(halfSpacePath)) {
                contactModel.addContactGeometry(halfSpace.clone());
            }

            const std::string& forcePath = force.getAbsolutePathString();
            if (forcePath.find("/forceset/") != std::string::npos) {
                contactModel.addForce(force.clone());
            } else {
                contactModel.addComponent(force.clone());
            }

            contactBodyInfo.contact_force_paths.push_back(
                    force.getAbsolutePathString());
        }
    }
}

void copyContactGeometryForBody(const Model& model, const std::string& bodyPath,
         Model& contactModel, ContactBodyInfo& contactBodyInfo) {
    for (const auto& geometry : model.getComponentList<ContactGeometry>()) {
        if (geometry.getSocket<PhysicalFrame>("frame").getConnecteePath()
                == bodyPath) {
            contactModel.addContactGeometry(geometry.clone());
            copySmoothSphereHalfSpaceForceForContactGeometry(model, 
                    contactModel, geometry, contactBodyInfo);
        }
    }
}

void copyDependentBodiesAndJoints(const Model& model, 
        const std::string& bodyPath, Model& contactModel,
        ContactBodyInfo& contactBodyInfo) {
    for (const auto& joint : model.getComponentList<Joint>()) {
        const Frame& parent = joint.getParentFrame().findBaseFrame();
        const std::string& parentPath = parent.getAbsolutePathString();

        // If the parent of the joint matches the body, copy the joint and the
        // child body of the joint.
        if (parentPath == bodyPath) {
            const Frame& child = joint.getChildFrame().findBaseFrame();
            const Body& body = model.getComponent<Body>(
                    child.getAbsolutePathString());

            // The body must be added before the joint.
            contactModel.addBody(body.clone());
            contactModel.addJoint(joint.clone());

            copyMarkersForBody(model, contactModel, 
                    body.getAbsolutePathString());
            copyContactGeometryForBody(model, body.getAbsolutePathString(), 
                    contactModel, contactBodyInfo);

            // Recursively copy the children of the child body.
            copyDependentBodiesAndJoints(model, body.getAbsolutePathString(), 
                   contactModel, contactBodyInfo);
        }
    }
}

void createContactModel(const Model& model, ContactBodyInfos& contactBodyInfos) {
    
    Model contactModel;
    for (auto& info : contactBodyInfos) {
        Body* body = model.getComponent<Body>(info.path).clone();
        contactModel.addBody(body);
        copyMarkersForBody(model, contactModel, info.path);
        copyContactGeometryForBody(model, info.path, contactModel, info);
        copyDependentBodiesAndJoints(model, info.path, contactModel, info);
        addGroundToFootJoint(contactModel, body->getName());
    }

    contactModel.finalizeConnections();
    contactModel.print(fmt::format("{}_contact_bodies.osim", model.getName()));
}

void createFeetCoordinatesReference(Model model, 
        const std::string& markersFile, const std::string& ikTasksFile) {
    // Run inverse kinematics on the foot model.
    InverseKinematicsTool iktool;
    iktool.setModel(model);
    iktool.setMarkerDataFileName(markersFile);
    iktool.set_IKTaskSet(IKTaskSet(ikTasksFile));
    iktool.setOutputMotionFileName("contact_bodies_ik.sto");    
    iktool.run();

    // Update the IK solution to something that we can visualize and use 
    // in Moco.
    TableProcessor tableProcessor = 
            TableProcessor("contact_bodies_ik.sto") |
            TabOpUseAbsoluteStateNames() |
            TabOpConvertDegreesToRadians() |
            TabOpLowPassFilter(6) |
            TabOpAppendCoordinateValueDerivativesAsSpeeds();

    TimeSeriesTable contactStatesReference = 
            tableProcessor.processAndConvertToRadians("", model);
    STOFileAdapter::write(contactStatesReference, 
            "contact_states_reference.sto");
}

class ContactTrackingObjective : public Callback {
public:
    ContactTrackingObjective(Model model,
            TimeSeriesTable grfs, TimeSeriesTable states,
            const ContactBodyInfos& contactBodyInfos) {

        // Model.
        m_model = std::move(model);
        m_state = m_model.initSystem();
        
        // Contact forces.
        for (int ibody = 0; ibody < (int)contactBodyInfos.size(); ++ibody) {
            const auto& info = contactBodyInfos[ibody];
            for (const auto& contactForcePath : info.contact_force_paths) {
                m_contactForceRefs.emplace_back( 
                        &m_model.getComponent<SmoothSphereHalfSpaceForce>(
                                contactForcePath));
                m_bodyIndexes.push_back(ibody);
            }
            numContactBodies++;
        }

        // Ground reaction forces.
        m_grfs = GCVSplineSet(grfs);
        std::vector<std::string> suffixes = {"x", "y", "z"};
        TimeSeriesTableVec3 grfsPacked = grfs.pack<SimTK::Vec3>(suffixes);
        std::vector<double> peakMagnitudes(grfsPacked.getNumColumns(), 0.0);
        for (int itime = 0; itime < (int)grfsPacked.getNumRows(); ++itime) {
            const auto row = grfsPacked.getRowAtIndex(itime);
            for (int igrf = 0; igrf < row.size(); ++igrf) {
                peakMagnitudes[igrf] = std::max(peakMagnitudes[igrf], 
                        row[igrf].normSqr());
            }
        }
        for (const auto& mag : peakMagnitudes) {
            m_forceScales.push_back(1.0 / mag);
        }

        // States.
        m_states = GCVSplineSet(states);

        casadi::Dict opts;
        opts["enable_fd"] = true;
        this->construct("objective", opts);
    }

    void setTime(double time) {
        m_time = SimTK::Vector(1, time);
    }

    void setForceWeight(double weight) {
        m_forceWeight = weight;
    }

    void setCoordinateWeight(double weight) {
        m_coordinateWeight = weight;
    }

    void setSpeedWeight(double weight) {
        m_speedWeight = weight;
    }

    const ContactForceRefs& getContactForceRefs() const {
        return m_contactForceRefs;
    }

    void calcContactForces(const Model& model, SimTK::State& state, 
            const SimTK::Vector& q, const SimTK::Vector& u,
            SimTK::Vector_<SimTK::Vec3>& forces) const {
        state.updQ() = q;
        state.updU() = u;
        model.realizeVelocity(state);
        int iforce = 0;
        for (const auto& contactForce : m_contactForceRefs) {
            forces[m_bodyIndexes[iforce++]] += 
                    contactForce->getSphereForce(state)[1];
        }
    }

    // Number of inputs and outputs
    casadi_int get_n_in() override { return 2;}
    casadi_int get_n_out() override { return 1;}

    Sparsity get_sparsity_in(casadi_int i) override {
        if (i == 0) {
            return Sparsity::dense(m_state.getNQ(), 1);
        } else if (i == 1) {
            return Sparsity::dense(m_state.getNU(), 1);
        } else {
            return Sparsity::dense(0, 0);
        }
    }

    Sparsity get_sparsity_out(casadi_int i) override {
        if (i == 0) {
            return Sparsity::dense(1, 1);
        } else {
            return Sparsity::dense(0, 0);
        }
    }

    // Evaluate numerically
    std::vector<DM> eval(const std::vector<DM>& arg) const override {        
        SimTK::Vector q(m_state.getNQ(), arg.at(0)->data(), true);
        SimTK::Vector u(m_state.getNU(), arg.at(1)->data(), true);

        SimTK::Vector_<SimTK::Vec3> errors(numContactBodies, SimTK::Vec3(0.0));
        calcContactForces(m_model, m_state, q, u, errors);

        // Force errors.
        int igrf = 0;
        double forceError = 0;
        for (int ibody = 0; ibody < numContactBodies; ++ibody) {
            for (int ixyz = 0; ixyz < 3; ++ixyz) {
                errors[ibody][ixyz] -= m_grfs.get(igrf++).calcValue(m_time);
            }
            forceError += m_forceScales[ibody] * errors[ibody].normSqr();
        }

        // Coordinate errors.
        double coordinateError = 0.0;
        double speedError = 0.0;
        int istate = 0;
        for (int iq = 0; iq < m_state.getNQ(); ++iq) {
            double error = q[iq] - m_states.get(istate).calcValue(m_time);
            coordinateError += error * error;
            istate++;
        }
        for (int iu = 0; iu < m_state.getNU(); ++iu) {
            double error = u[iu] - m_states.get(istate).calcValue(m_time);
            speedError += error * error;
            istate++;
        }
           
        DM f = m_forceWeight      * forceError + 
               m_coordinateWeight * (coordinateError / m_state.getNQ()) +
               m_speedWeight      * (speedError / m_state.getNU());
        return {f};
    }

private:
    Model m_model;
    mutable SimTK::State m_state;
    SimTK::Vector m_time;

    GCVSplineSet m_grfs;
    GCVSplineSet m_states;

    int numContactBodies = 0;
    ContactForceRefs m_contactForceRefs;
    std::vector<int> m_bodyIndexes;

    std::vector<double> m_forceScales;
    double m_forceWeight = 1.0;
    double m_coordinateWeight = 1.0;
    double m_speedWeight = 1.0;

};

SimTK::RowVector solveContactTrackingProblem(ContactTrackingObjective& objfun, 
        double time, const TimeSeriesTable& states) {

    // The number of states. For each foot, we have a 6 DOF joint between the 
    // ground and the calcaneus, and a 1 DOF joint between the calcaneus and
    // the toes.
    int nq = 14;      // number of generalized coordinates
    int nu = nq;      // number of generalized speeds
    int ns = nq + nu; // number of states

    // Define the optimization variables.
    MX q = MX::sym("q", nq, 1); // coordinates
    MX u = MX::sym("u", nu, 1); // speeds
    MX x = vertcat(q, u);       // states

    // Objective function
    objfun.setTime(time);
    objfun.setForceWeight(0.1);
    objfun.setCoordinateWeight(0.01);
    objfun.setSpeedWeight(0.01);

    MXVector costOut;
    objfun.call({q, u}, costOut);
    MX f = MX::sum1(costOut.at(0));

    // Create the NLP.
    MXDict nlp;
    nlp.emplace(std::make_pair("x", x));
    nlp.emplace(std::make_pair("f", f));
    casadi::Dict opts;
    casadi::Dict optsIpopt;
    double tol = 1e-3;
    optsIpopt["tol"] = tol;
    optsIpopt["dual_inf_tol"] = tol;
    optsIpopt["compl_inf_tol"] = tol;
    optsIpopt["acceptable_tol"] = tol;
    optsIpopt["acceptable_dual_inf_tol"] = tol;
    optsIpopt["acceptable_compl_inf_tol"] = tol;
    opts["ipopt"] = optsIpopt;
    casadi::Function solver = nlpsol("solver", "ipopt", nlp, opts);

    // Set the bounds and initial guess.
    SimTK::RowVector row = states.getNearestRow(time);
    std::vector<double> x0;
    std::vector<double> lbx;
    std::vector<double> ubx;
    for (int i = 0; i < row.size(); ++i) {
        x0.push_back(row[i]);
        lbx.push_back(-std::numeric_limits<double>::infinity());
        ubx.push_back(std::numeric_limits<double>::infinity());
        // lbx.push_back(row[i] - 0.25*std::abs(row[i]));
        // ubx.push_back(row[i] + 0.25*std::abs(row[i]));
    }

    // Solve the problem
    DMDict input = {{"lbx", lbx},
                    {"ubx", ubx},
                    {"x0",  x0}};
    DMDict output = solver(input);

    // Print the optimal solution
    std::vector<double> xopt(output.at("x"));
    SimTK::RowVector solution(nq+nu, xopt.data());

    return solution;
}

void runContactTracking(Model model, 
        const ContactBodyInfos& contactBodyInfos,
        const TimeSeriesTable& grfData, 
        const TimeSeriesTable& statesReference) {

    std::vector<std::string> grfLabelsInOrder;
    for (const auto& info : contactBodyInfos) {
        grfLabelsInOrder.push_back(info.force_indentifier + "x");
        grfLabelsInOrder.push_back(info.force_indentifier + "y");
        grfLabelsInOrder.push_back(info.force_indentifier + "z");
    }

    TimeSeriesTable grfs(grfData.getIndependentColumn());
    for (const auto& label : grfLabelsInOrder) {
        grfs.appendColumn(label, grfData.getDependentColumn(label));
    }

    std::vector<std::string> stateLabelsInOrder;
    model.initSystem();
    auto coordinateRefs = model.getCoordinatesInMultibodyTreeOrder();
    for (int i = 0; i < (int)coordinateRefs.size(); ++i) {
        std::string valuePath = fmt::format("{}/value", 
                coordinateRefs[i]->getAbsolutePathString());
        stateLabelsInOrder.push_back(valuePath);
    }
    for (int i = 0; i < (int)coordinateRefs.size(); ++i) {
        std::string speedPath = fmt::format("{}/speed", 
                coordinateRefs[i]->getAbsolutePathString());
        stateLabelsInOrder.push_back(speedPath);
    }

    TimeSeriesTable states(statesReference.getIndependentColumn());
    for (const auto& label : stateLabelsInOrder) {
        std::cout << label << std::endl;
        states.appendColumn(label, 
                statesReference.getDependentColumn(label));
    }

    model.printSubcomponentInfo();
    ContactTrackingObjective objfun(model, grfs, states, contactBodyInfos);

    const auto& times = states.getIndependentColumn();
    TimeSeriesTable statesSolutionTable;
    TimeSeriesTable forcesSolutionTable;

    SimTK::State state = model.initSystem();
    for (int itime = 75; itime < 125; ++itime) {
        double time = times[itime];
        SimTK::RowVector row = solveContactTrackingProblem(objfun, time, states);
        statesSolutionTable.appendRow(time, row);

        SimTK::Vector q(state.getNQ(), row.getContiguousScalarData(), true);
        SimTK::Vector u(state.getNU(), 
                row.getContiguousScalarData() + state.getNQ(), true);

        SimTK::Vector_<SimTK::Vec3> forces((int)contactBodyInfos.size(), 
                SimTK::Vec3(0.0));
        objfun.calcContactForces(model, state, q, u, forces);

        SimTK::RowVector forcesRow((int)contactBodyInfos.size()*3);
        for (int i = 0; i < (int)contactBodyInfos.size(); ++i) {
            forcesRow[3*i] = forces[i][0];
            forcesRow[3*i+1] = forces[i][1];
            forcesRow[3*i+2] = forces[i][2];
        }

        forcesSolutionTable.appendRow(time, forcesRow);
    }
    statesSolutionTable.setColumnLabels(stateLabelsInOrder);
    statesSolutionTable.addTableMetaData<std::string>("inDegrees", "no");
    STOFileAdapter::write(statesSolutionTable, 
            "contact_initializer_solution_states.sto");

    forcesSolutionTable.setColumnLabels(grfLabelsInOrder);
    STOFileAdapter::write(forcesSolutionTable, 
            "contact_initializer_solution_forces.sto");
}

int main() {

    // contactTracking();

    // Load original model.
    Model model("subject_walk_scaled.osim");
    model.initSystem();

    // Load external loads.
    ExternalLoads externalLoads("grf_walk.xml", true);

    // Load IK data.
    std::string markersFile = "marker_trajectories.trc";
    std::string ikTasksFile = "ik_tasks.xml";

    // Attach contact geometry and forces to the model.
    // TODO have user add this to the model?
    ContactGeometrySet contactGeometrySet("subject_walk_scaled_ContactGeometrySet.xml");
    for (int i = 0; i < contactGeometrySet.getSize(); ++i) {
        ContactGeometry* contactGeometry = contactGeometrySet.get(i).clone();
        // Raise the ContactSpheres up by 2 cm so that the bottom of the spheres
        // are better aligned with the ground.
        if (contactGeometry->getName() != "floor") {
            SimTK::Vec3& location = contactGeometry->upd_location();
            location[1] += 0.02; 
        }
        model.addContactGeometry(contactGeometry);
    }
    model.finalizeConnections();

    ForceSet contactForceSet("subject_walk_scaled_ContactForceSet.xml");
    for (int i = 0; i < contactForceSet.getSize(); ++i) {
        model.addComponent(contactForceSet.get(i).clone());
    }
    model.finalizeConnections();

    // Detect the contact bodies.
    ContactBodyInfos contactBodyInfos;
    for (int i = 0; i < externalLoads.getSize(); ++i) {
        ContactBodyInfo contactBodyInfo;
        const auto& externalForce = externalLoads.get(i);
        const auto& contactBodyName = externalForce.get_applied_to_body();
        std::string bodysetPath = fmt::format("/bodyset/{}", contactBodyName);
        std::string componentPath = fmt::format("/{}", contactBodyName);

        if (model.hasComponent<Body>(bodysetPath)) {
            contactBodyInfo.path = bodysetPath;
        } else if (model.hasComponent<Body>(componentPath)) {
            contactBodyInfo.path = componentPath;
        } else {
            OPENSIM_THROW(Exception, "Contact body '{}' not found in model.", 
                    contactBodyName);
        }

        contactBodyInfo.force_indentifier = externalForce.get_force_identifier();
        contactBodyInfo.contact_force_paths = {};
        contactBodyInfos.push_back(contactBodyInfo);
    }

    // Initialize the contact body map.

    // TODO check that the bodies with contact geometry are consistent with the 
    // external loads file.

    // input: original model (with contacts), contact body trees
    // output: "feet" model with contacts, markers etc.
    createContactModel(model, contactBodyInfos);

    // Print out contact body infos.
    for (const auto& info : contactBodyInfos) {
        std::cout << "Contact body: " << info.path << std::endl;
        std::cout << "Force identifier: " << info.force_indentifier << std::endl;
        for (const auto& contactForcePath : info.contact_force_paths) {
            std::cout << "Contact force: " << contactForcePath << std::endl;
        }
    }

    // Load the contacts model.
    Model contactModel(fmt::format("{}_contact_bodies.osim", model.getName()));
    model.initSystem();

    // input: "feet" model, original TRC file, IK tasks
    // output: IK solution containing foot coordinates
    createFeetCoordinatesReference(contactModel, markersFile, ikTasksFile);

    // input: "feet" model, external loads file, coordinate reference file
    // output: states file, contact forces file (for debugging) 
    TimeSeriesTable contactStatesReference("contact_states_reference.sto");
    const std::string grfDataFilePath =
            convertRelativeFilePathToAbsoluteFromXMLDocument(
                    externalLoads.getDocumentFileName(),
                    externalLoads.getDataFileName());
    TimeSeriesTable grfData(grfDataFilePath);
    runContactTracking(contactModel, contactBodyInfos, grfData, 
            contactStatesReference);

    // input: "feet" model, states file
    // output: TRC file containing updated marker positions
    computeFootMarkerPositions();

    // input: original model, "feet" TRC file, original TRC file, IK tasks
    // createUpdatedCoordinateReference();



    return EXIT_SUCCESS;
}


